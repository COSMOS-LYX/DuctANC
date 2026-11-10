"""管道 ANC 基础硬件测试上位机（蓝牙 SPP 虚拟串口）。

用法示例：
    python hwtest.py ports
    python hwtest.py -p COM7 ping -n 20
    python hwtest.py -p COM7 tone 200 --amp 0.1
    python hwtest.py -p COM7 level --watch
    python hwtest.py -p COM7 capture --frames 4000 --tag source_on
    python hwtest.py -p COM7 selftest
    python hwtest.py -p COM7 p0test --part couple
    python hwtest.py -p COM7 shell
"""
from __future__ import annotations

import argparse
import datetime as dt
import re
import sys
import threading
import time
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports

import analysis as an

AUDIO_BLK = 128  # 旧固件 128 点块同步干扰的检测窗。0.3 起固件逐点，这个常数不再等于 AUDIO_BLK
CAP_MAX, CAP_MAX_DAC = 16000, 10666  # 同 audio.h：只采麦克风 / 加 DAC 通道的帧数上限

ROOT = Path(__file__).resolve().parent.parent
RECORD_DIR = ROOT / "实验记录"
LVL_RE = re.compile(r"\|\s*(\w+) dc=([-\d.]+) rms=([-\d.]+) mv=([-\d.]+) dbfs=([-\d.]+) "
                    r"min=(\d+) max=(\d+) clip=(\d+)")


class DeviceError(RuntimeError):
    pass


class Device:
    """行协议：命令一行，回复 OK/ERR/数据行；'#' 开头是注释；CAP 回复 #CAP 头 + 二进制 + #END。"""

    def __init__(self, port: str, baud: int = 115200, ser=None):
        self.ser = ser if ser is not None else serial.Serial(port, baud, timeout=0.05)
        self.baud = baud
        self.buf = bytearray()

    def close(self):
        self.ser.close()

    def send(self, line: str):
        self.ser.write((line + "\r\n").encode("ascii"))

    def _fill(self, deadline: float) -> bool:
        chunk = self.ser.read(max(1, getattr(self.ser, "in_waiting", 0) or 1))
        if chunk:
            self.buf += chunk
            return True
        return time.monotonic() < deadline

    def readline(self, timeout: float = 2.0) -> str | None:
        deadline = time.monotonic() + timeout
        while b"\n" not in self.buf:
            if not self._fill(deadline):
                return None
        i = self.buf.index(b"\n")
        line, self.buf = bytes(self.buf[:i]), self.buf[i + 1:]
        return line.decode("utf-8", "replace").strip()

    def read_exact(self, n: int, timeout: float) -> bytes:
        deadline = time.monotonic() + timeout
        while len(self.buf) < n:
            if not self._fill(deadline):
                raise DeviceError(f"binary timeout: got {len(self.buf)}/{n} bytes")
        data, self.buf = bytes(self.buf[:n]), self.buf[n:]
        return data

    def drain(self, quiet: float = 0.3):
        """丢弃残留输出（例如上电横幅、STREAM 行）。"""
        while self.readline(quiet) is not None:
            pass
        self.buf.clear()

    def cmd(self, line: str, prefixes=("OK", "ERR"), timeout: float = 3.0, retries: int = 0) -> str:
        """retries 只用于可重复执行的查询命令（LEVEL/INFO），蓝牙偶尔丢包时重发。"""
        for _ in range(retries + 1):
            self.send(line)
            deadline = time.monotonic() + timeout
            while (r := self.readline(max(0.0, deadline - time.monotonic()))) is not None:
                if r.startswith(prefixes):
                    if r.startswith("ERR"):
                        raise DeviceError(f"{line} -> {r}")
                    return r
            self.drain(0.2)
        raise DeviceError(f"no reply to '{line}'（看板上 D2 是否还在闪、蓝牙是否断开）")

    def ping(self, token: str) -> float:
        t0 = time.perf_counter()
        r = self.cmd(f"PING {token}", prefixes=("PONG",))
        if r.split()[1] != token:
            raise DeviceError(f"token mismatch: {r}")
        return (time.perf_counter() - t0) * 1e3

    def level(self) -> dict:
        r = self.cmd("LEVEL", prefixes=("LVL", "ERR"), retries=2)
        return parse_level(r)

    def info(self) -> dict:
        r = self.cmd("INFO", prefixes=("INFO",), retries=2)
        return dict(kv.split("=", 1) for kv in r.split()[1:] if "=" in kv)

    def capture(self, frames: int, progress=True, dac=False) -> tuple[np.ndarray, int]:
        """返回 (codes[n, nch], fs)；dac=True 时第 3 列是该帧 DAC 正在输出的码值。"""
        self.send(f"CAP {frames}" + (" D" if dac else ""))
        while True:
            r = self.readline(frames / 8000 + 3)
            if r is None:
                raise DeviceError("no #CAP header")
            if r.startswith("ERR"):
                raise DeviceError(r)
            if r.startswith("#CAP"):
                break
        _, n, fs, nch = r.split()
        n, fs, nch = int(n), int(fs), int(nch)
        nbytes = n * nch * 2
        eta = nbytes * 10 / self.baud
        if progress:
            print(f"  receiving {nbytes} bytes, ~{eta:.0f} s at {self.baud} baud ...", flush=True)
        raw = self.read_exact(nbytes, eta * 1.5 + 5)
        end = self.readline(3)
        if not end or not end.startswith("#END"):
            raise DeviceError(f"bad trailer: {end!r}")
        codes = np.frombuffer(raw, dtype="<u2").reshape(n, nch)
        if (int(codes.astype(np.uint64).sum()) & 0xFFFFFFFF) != int(end.split()[1], 16):
            raise DeviceError("checksum mismatch")
        return codes.copy(), fs

    def anc_status(self) -> dict:
        r = self.cmd("ANC STATUS", prefixes=("ANC", "ERR"), timeout=3)
        return dict(kv.split("=", 1) for kv in r.split()[1:] if "=" in kv)

    def xchk(self, codes: np.ndarray) -> np.ndarray:
        """固件用已下发的 Ŝ_eff 重算 x'。先等 OK，再送 uint16 码，DAC 不动。"""
        codes = np.ascontiguousarray(codes, dtype="<u2")
        n = int(codes.shape[0])
        if not 1 <= n <= 400:
            raise DeviceError("XCHK n 要在 1..400")
        self.cmd(f"XCHK {n}", timeout=3)
        self.ser.write(codes.tobytes())
        while True:
            r = self.readline(5)
            if r is None:
                raise DeviceError("no #XCHK header")
            if r.startswith("ERR"):
                raise DeviceError(r)
            if r.startswith("#XCHK"):
                break
        nbytes = n * 4
        raw = self.read_exact(nbytes, 5)
        end = self.readline(3)
        if not end or not end.startswith("#END"):
            raise DeviceError(f"bad xchk trailer: {end!r}")
        if (int(np.frombuffer(raw, dtype=np.uint8).sum()) & 0xFFFFFFFF) != int(end.split()[1], 16):
            raise DeviceError("xchk checksum mismatch")
        return np.frombuffer(raw, dtype="<f4").copy()


def parse_level(line: str) -> dict:
    if line.startswith("ERR"):
        raise DeviceError(line)
    out = {}
    for m in LVL_RE.finditer(line):
        out[m.group(1)] = {
            "dc": float(m.group(2)), "rms": float(m.group(3)), "mv": float(m.group(4)),
            "dbfs": float(m.group(5)), "min": int(m.group(6)), "max": int(m.group(7)),
            "clip": int(m.group(8)),
        }
    return out


def fmt_level(lv: dict) -> str:
    return "  ".join(f"{k}: dc={v['dc']:.0f} rms={v['mv']:.2f} mV ({v['dbfs']:.1f} dBFS) "
                     f"clip={v['clip']}" for k, v in lv.items())


def new_record_dir(tag: str) -> Path:
    d = RECORD_DIR / f"{dt.datetime.now():%Y%m%d_%H%M%S}_{tag}"
    d.mkdir(parents=True, exist_ok=True)
    return d


def save_capture(codes: np.ndarray, fs: int, d: Path, name: str) -> dict:
    """codes 有第 3 列（DAC）时：npz 里另存 dac，并画 DAC→麦克风的冲激响应和相干（<name>_dac.png）。"""
    dac = None
    if codes.shape[1] > 2:
        codes, dac = codes[:, :2], codes[:, 2]
        np.savez(d / f"{name}.npz", codes=codes, fs=fs, dac=dac)
    else:
        np.savez(d / f"{name}.npz", codes=codes, fs=fs)
    an.save_wav(codes, fs, d / f"{name}.wav")
    an.plot_capture(codes, fs, d / f"{name}.png", title=name)
    mv = an.codes_to_mv(codes)
    f, p = an.psd(mv, fs)
    stats = an.basic_stats(codes)
    for c, s in enumerate(stats):
        s["band_100_400_db"] = an.band_level_db(f, p[:, c], 100, 400)
        s["peak_hz"] = float(f[1:][np.argmax(p[1:, c])])
    if codes.shape[1] >= 2:
        fc, cxy = an.coherence(mv[:, 0], mv[:, 1], fs)
        m = (fc >= 100) & (fc <= 400)
        stats[0]["coh_100_400"] = stats[1]["coh_100_400"] = float(np.mean(cxy[m]))
    res = {"f": f, "p": p, "stats": stats}
    if dac is not None:
        res["dac"] = an.dac_check(dac, mv, fs)
        an.plot_dac_check(res["dac"], fs, d / f"{name}_dac.png", title=f"{name}: DAC -> mic")
    return res


def print_stats(res: dict):
    for name, s in zip(an.CH_NAMES, res["stats"]):
        print(f"  {name}: dc={s['dc_mv']:.0f} mV rms={s['rms_mv']:.2f} mV "
              f"band100-400={s['band_100_400_db']:.1f} dB peak={s['peak_hz']:.0f} Hz "
              f"clip={s['clip']}" + (f" coh={s['coh_100_400']:.2f}" if "coh_100_400" in s else ""))


# ---------------------------------------------------------------- 子命令

def do_ports(_args):
    for p in serial.tools.list_ports.comports():
        print(f"{p.device:8s} {p.description}  [{p.hwid}]")
    print("蓝牙 SPP 会出现两个 COM 口，用「传出」(outgoing) 那个。")


def do_ping(dev: Device, args):
    rtts, lost = [], 0
    for i in range(args.n):
        try:
            rtts.append(dev.ping(f"p{i}"))
        except DeviceError as e:
            lost += 1
            print(f"  #{i}: {e}")
            dev.drain(0.2)
    if rtts:
        r = np.array(rtts)
        print(f"PING {args.n} sent, {lost} lost, RTT min/avg/max = "
              f"{r.min():.0f}/{r.mean():.0f}/{r.max():.0f} ms")
    return lost == 0


def do_shell(dev: Device, _args):
    stop = threading.Event()

    def reader():
        while not stop.is_set():
            line = dev.readline(0.2)
            if line is not None:
                print(f"< {line}")

    threading.Thread(target=reader, daemon=True).start()
    print("输入命令（HELP 查看），Ctrl+C 退出")
    try:
        for line in sys.stdin:
            if line.strip():
                dev.send(line.strip())
    except KeyboardInterrupt:
        pass
    stop.set()


def do_level(dev: Device, args):
    while True:
        print(fmt_level(dev.level()))
        if not args.watch:
            return
        time.sleep(0.5)


def do_capture(dev: Device, args):
    limit = CAP_MAX_DAC if args.dac else CAP_MAX
    if not 1 <= args.frames <= limit:
        raise DeviceError(f"--frames 超出范围 1..{limit}" + ("（带 --dac 时 3 通道）" if args.dac else ""))
    codes, fs = dev.capture(args.frames, dac=args.dac)
    d = new_record_dir(args.tag)
    res = save_capture(codes, fs, d, args.tag)
    print_stats(res)
    if "dac" in res:
        k = res["dac"]
        if k["dac_pp"] == 0:
            print("  dac: 恒定值，喇叭没在出声（先用 noise / tone 打开输出）")
        else:
            print(f"  dac: rms={k['dac_rms']:.1f} 码，峰峰 {k['dac_pp']} 码")
            for ch in an.CH_NAMES:
                print(f"  dac→{ch}: 到达 {k[ch]['onset']} 样本（|h| 最大在 {k[ch]['peak_lag']}），"
                      f"相干 200 Hz {k[ch]['coh_200']:.2f}，100–400 Hz 平均 {k[ch]['coh_100_400']:.2f}")
    print(f"saved to {d}")


class Checks:
    """检查项收集与报告输出。状态：PASS / WARN / FAIL，WARN 不算失败。"""

    def __init__(self):
        self.rows = []

    def __call__(self, name, status, detail):
        if isinstance(status, (bool, np.bool_)):
            status = "PASS" if status else "FAIL"
        self.rows.append((name, status, detail))
        print(f"[{status}] {name}: {detail}")

    def ok(self) -> bool:
        return all(r[1] != "FAIL" for r in self.rows)

    def write(self, path: Path, title: str, header: str, images: list[str]):
        with open(path, "w", encoding="utf-8") as fp:
            fp.write(f"# {title} {dt.datetime.now():%Y-%m-%d %H:%M:%S}\n\n{header}\n\n")
            fp.write("| 项目 | 结果 | 说明 |\n| --- | --- | --- |\n")
            for r in self.rows:
                fp.write(f"| {r[0]} | {r[1]} | {r[2]} |\n")
            fp.write("\n" + "\n\n".join(f"![{i}]({i}.png)" for i in images) + "\n")
        print(f"\n报告：{path}")


def prompt(args, msg: str):
    if not args.no_prompt:
        input(f"{msg}，然后按回车 ...")


def check_quiet(dev: Device, args, check: Checks, d: Path, name: str):
    """静音段：偏置、本底噪声、两路独立。返回 (LEVEL 结果, 分析结果)。"""
    time.sleep(0.8)
    lv = dev.level()
    print("采集静音段 ...")
    codes, fs = dev.capture(args.frames)
    res = save_capture(codes, fs, d, name)
    for c, (ch, v) in enumerate(lv.items()):
        mv_dc = v["dc"] * 3300 / 4095
        band_mv = 10 ** (res["stats"][c]["band_100_400_db"] / 20)
        check(f"{ch} 偏置", 1300 < mv_dc < 2000, f"dc={mv_dc:.0f} mV (应约 1650)")
        # 本底只看目标频段：20 Hz 以下的气流/漂移不影响 ANC
        check(f"{ch} 本底 100–400 Hz", band_mv < args.max_quiet_mv and v["clip"] == 0,
              f"{band_mv:.1f} mV (要求 <{args.max_quiet_mv:.0f})，全频带 {v['mv']:.1f} mV，"
              f"主峰 {res['stats'][c]['peak_hz']:.0f} Hz，clip={v['clip']}"
              + ("  → 麦克风饱和：增益调低 / 检查是否有气流或振动直接作用在麦上" if v["clip"] else ""))
    xt = an.crosstalk(codes)
    ghost = abs(xt["corr"]) > 0.95 and xt["lag"] == 0
    check("两路独立", not ghost,
          f"相关系数 {xt['corr']:.3f}，滞后 {xt['lag']} 样本，err≈{xt['slope']:.2f}×ref"
          + ("  → err 通道疑似悬空，读到的是 ref 的 ADC 串扰：查误差麦 OUT→PC1、VCC、GND" if ghost else ""))
    return lv, res


def capture_mv(dev: Device, args, d: Path, name: str):
    codes, fs = dev.capture(args.frames)
    return save_capture(codes, fs, d, name), an.codes_to_mv(codes), fs


def lag_status(lag: int, expect: float, tol: float) -> str:
    if abs(lag - expect) <= tol:
        return "PASS"
    return "WARN" if np.sign(lag) == np.sign(expect) else "FAIL"


def tap_added_mv(dev: Device, target: str, quiet_lv: dict, secs: float = 3.0) -> dict:
    """敲击期间每路新增的 rms（mV，扣除本底功率）。

    用绝对增量而不是相对本底的 dB：本底高的那路 dB 涨幅天然偏小，会误判。
    """
    input(f"按回车后 {secs:.0f} 秒内，用手指反复轻敲【{target} 麦】旁边的管壁 ...")
    peak = {ch: 0.0 for ch in an.CH_NAMES}
    t_end = time.monotonic() + secs
    while time.monotonic() < t_end:
        time.sleep(0.5)
        for ch, v in dev.level().items():
            peak[ch] = max(peak[ch], v["mv"])
    return {ch: float(np.sqrt(max(peak[ch] ** 2 - quiet_lv[ch]["mv"] ** 2, 0))) for ch in peak}


def do_mictest(dev: Device, args):
    """麦克风专项：本底 → 敲击定位 → 扬声器白噪声 → 外部声源（信噪比、削波、相干性、声程差）。"""
    d = new_record_dir("mictest")
    check = Checks()
    c_mm = 343000.0
    dev.cmd("STREAM 0")
    dev.cmd("STOP")

    print("\n== 1/4 本底噪声 ==")
    prompt(args, "请关闭外部声源、保持安静")
    quiet_lv, quiet = check_quiet(dev, args, check, d, "1_quiet")

    if not args.no_prompt:
        print("\n== 2/4 敲击定位 ==")
        for target, other in (("ref", "err"), ("err", "ref")):
            add = tap_added_mv(dev, target, quiet_lv)
            # 敲击声会顺着 PVC 管壁传到另一只麦，只作参考；通道顺序以 GCC-PHAT 声程差为准
            check(f"敲击 {target}", "PASS" if add[target] > 1.4 * add[other] else "WARN",
                  f"新增 rms：ref {add['ref']:.0f} mV，err {add['err']:.0f} mV（被敲的那只应更大）")

    print("\n== 3/4 扬声器白噪声 ==")
    dev.cmd(f"NOISE {args.amp}")
    time.sleep(0.8)
    lv = dev.level()
    spk, mv, fs = capture_mv(dev, args, d, "2_speaker_noise")
    dev.cmd("STOP")
    for c, ch in enumerate(an.CH_NAMES):
        rise = spk["stats"][c]["band_100_400_db"] - quiet["stats"][c]["band_100_400_db"]
        clip = lv[ch]["clip"] + spk["stats"][c]["clip"]
        check(f"扬声器→{ch}", rise >= args.min_rise and clip == 0,
              f"100–400 Hz 比本底 +{rise:.1f} dB (要求 ≥{args.min_rise:.0f}), clip={clip}"
              + (f"  → 喇叭太响把麦打满了，用更小的 --amp（当前 {args.amp}）" if clip else ""))
    lag, _ = an.gcc_phat_lag(mv[:, 0], mv[:, 1], fs)
    exp = (args.d_spk_err - args.d_spk_ref) / c_mm * fs
    check("声程差·扬声器", lag_status(lag, exp, args.lag_tol),
          f"err 相对 ref {lag:+d} 样本（≈{lag * c_mm / fs:+.0f} mm），按几何应 {exp:+.1f}；"
          "负值表示误差麦离喇叭更近，符号反了多半是通道接反")

    print("\n== 4/4 外部声源 ==")
    prompt(args, "请用手机播放 200 Hz 单音（手机固定在支架上、不碰管子，参考麦 rms 约 200–300 mV），等稳定")
    time.sleep(1.5)
    lv = dev.level()
    src, mv, fs = capture_mv(dev, args, d, "3_source")
    for c, ch in enumerate(an.CH_NAMES):
        st = src["stats"][c]
        use = (st["max"] - st["min"]) / 4095
        clip = st["clip"] + lv[ch]["clip"]
        check(f"{ch} 声源不削波", "FAIL" if clip else "WARN" if use > 0.9 or use < 0.05 else "PASS",
              f"占用量程 {use * 100:.0f}%，clip={clip}（5%–90% 合适；过小说明增益偏低）")
        snr = st["band_100_400_db"] - quiet["stats"][c]["band_100_400_db"]
        check(f"{ch} 声源信噪比", "PASS" if snr >= 20 else "WARN" if snr >= 12 else "FAIL",
              f"100–400 Hz 比本底高 {snr:.1f} dB（≥20 理想；要测出 10 dB 降噪至少 ≥12）"
              + ("  → 播放前后几乎没变化：确认第 1 步时外部声源已关闭、第 4 步时确实在播放、音量足够" if snr < 3 else ""))
    coh = src["stats"][0]["coh_100_400"]
    check("相干性 100–400 Hz", "PASS" if coh >= 0.9 else "WARN" if coh >= 0.7 else "FAIL",
          f"平均 γ^2={coh:.2f} → 前馈降噪理论上限约 {-10 * np.log10(1 - min(coh, 0.999)):.1f} dB")
    lag, _ = an.gcc_phat_lag(mv[:, 0], mv[:, 1], fs)
    exp = args.d_ref_err / c_mm * fs
    check("声程差·声源", lag_status(lag, exp, args.lag_tol),
          f"err 比 ref 晚 {lag:+d} 样本（≈{lag * c_mm / fs:.0f} mm），按几何应 {exp:+.1f}")
    m = (src["f"] >= 50) & (src["f"] <= 1000)
    peaks = src["f"][m][np.argsort(src["p"][m, 1])[-5:][::-1]]
    print(f"外部声源误差麦主要频率：{', '.join(f'{p:.0f}' for p in peaks)} Hz")
    prompt(args, "测试结束，可以关闭外部声源")

    check.write(d / "report.md", "麦克风测试",
                f"帧数 {args.frames}（{args.frames / fs:.2f} s），扬声器白噪声 amp {args.amp}；"
                f"几何：ref→err {args.d_ref_err:.0f} mm，喇叭→ref {args.d_spk_ref:.0f} mm，"
                f"喇叭→err {args.d_spk_err:.0f} mm。外部声源误差麦主要频率："
                f"{', '.join(f'{p:.0f}' for p in peaks)} Hz",
                ["1_quiet", "2_speaker_noise", "3_source"])
    return check.ok()


def do_p0test(dev: Device, args):
    """P0 硬件对照：参考麦电串扰（拔喇叭线/关功放）、外部声源音量与低频来源。"""
    d = new_record_dir("p0test")
    check = Checks()
    dev.cmd("STREAM 0")
    dev.cmd("STOP")

    def db(x, y):
        return 20 * np.log10((x + 1e-9) / (y + 1e-9))

    rows, order, images = {}, [], []

    def grab(name, hz, search_hz=0.0):
        """search_hz=0：在已知频率上拟合（喇叭单音与 ADC 同一时钟；对照段里没有单音可找）。"""
        codes, fs = dev.capture(args.frames)
        res = save_capture(codes, fs, d, name)
        mv = an.codes_to_mv(codes)
        st = res["stats"]
        rows[name] = {
            "hz": hz, "tm": an.tone_metrics(mv, fs, hz, search_hz),
            "lf": [an.band_rms_mv(mv[:, c], fs, hi=80) for c in (0, 1)],
            "hf": [an.band_rms_mv(mv[:, c], fs, lo=80) for c in (0, 1)],
            "use": [(st[c]["max"] - st[c]["min"]) / 4095 for c in (0, 1)],
            "clip": (st[0]["clip"], st[1]["clip"]),
            "sync": [an.block_sync_steps(mv[:, c], AUDIO_BLK) for c in (0, 1)],
        }
        order.append(name)
        images.append(name)
        return rows[name], mv, fs

    # 0 本底
    prompt(args, "请关闭外部声源和喇叭、保持安静")
    q, qmv, fs = grab("0_quiet", args.spk_hz)
    if args.part in ("source", "all"):
        extra = dict(q)
        extra["hz"] = args.tone_hz
        extra["tm"] = an.tone_metrics(qmv, fs, args.tone_hz, 0.0)
        rows["0_quiet@" + f"{args.tone_hz:.0f}Hz"] = extra
        order.append("0_quiet@" + f"{args.tone_hz:.0f}Hz")
    floor = {ch: q["tm"][ch]["amp_mv"] for ch in an.CH_NAMES}
    amp_used = "-"

    if args.part in ("couple", "all"):
        # ---- 参考麦电串扰：喇叭单音，拔喇叭线 / 关功放对比
        print(f"\n== 参考麦电串扰（喇叭 {args.spk_hz:.0f} Hz）==")
        amps = [float(s) for s in args.amps.split(",")]
        tried, amp_used, accepted = [], amps[-1], False
        for a in amps:
            dev.cmd(f"TONE {args.spk_hz} {a}")
            time.sleep(0.8)
            lv = dev.level()
            cl = [lv[ch]["clip"] for ch in an.CH_NAMES]
            us = [(lv[ch]["max"] - lv[ch]["min"]) / 4095 for ch in an.CH_NAMES]
            tried.append((a, cl, us))
            if cl[0] == 0 and cl[1] == 0 and us[0] < 0.9 and us[1] < 0.9:
                amp_used, accepted = a, True
                break
        check("喇叭幅度", "PASS" if accepted else "WARN",
              f"选用 amp {amp_used}；" + "；".join(
                  f"amp {a}: ref clip={c[0]} 量程 {u[0] * 100:.0f}%，err clip={c[1]} 量程 {u[1] * 100:.0f}%"
                  for a, c, u in tried))
        print("采集：喇叭接着 ...")
        on, _, _ = grab("1_spk_on", args.spk_hz)
        prompt(args, "拔掉喇叭线（只拔 PAM8403 输出端，功放保持通电）")
        time.sleep(0.8)
        un, _, _ = grab("2_spk_unplugged", args.spk_hz)
        prompt(args, "关闭 PAM8403 电源（喇叭线保持拔掉），DAC 照常输出")
        time.sleep(0.8)
        off, _, _ = grab("3_amp_off", args.spk_hz)
        dev.cmd("STOP")
        prompt(args, "恢复功放供电和喇叭接线")

        a_on = {ch: on["tm"][ch]["amp_mv"] for ch in an.CH_NAMES}
        a_un = {ch: un["tm"][ch]["amp_mv"] for ch in an.CH_NAMES}
        a_off = {ch: off["tm"][ch]["amp_mv"] for ch in an.CH_NAMES}
        r_un_err = db(a_un["err"], a_on["err"])
        check("err 声学路径断开",
              "PASS" if a_un["err"] <= 2 * floor["err"] or r_un_err <= -30 else "WARN",
              f"{args.spk_hz:.0f} Hz：接喇叭 {a_on['err']:.1f} → 拔线 {a_un['err']:.1f} mV"
              f"（{r_un_err:+.1f} dB，本底 {floor['err']:.2f} mV）"
              + ("" if a_un["err"] <= 2 * floor["err"] or r_un_err <= -30
                 else "  → 喇叭线可能没拔干净，或误差麦也有电串扰"))
        ref_pass = {}
        for tag, a_x, label in (("功放通电", a_un["ref"], "拔线"), ("功放断电", a_off["ref"], "断电")):
            r = db(a_x, a_on["ref"])
            status = "PASS" if a_x <= 2 * floor["ref"] or r <= -30 else "WARN" if r <= -15 else "FAIL"
            ref_pass[tag] = status == "PASS"
            check(f"ref 电串扰·{tag}", status,
                  f"{args.spk_hz:.0f} Hz：接喇叭 {a_on['ref']:.1f} mV → {label} {a_x:.1f} mV"
                  f"（{r:+.1f} dB，本底 {floor['ref']:.2f} mV），约 {100 * a_x / (a_on['ref'] + 1e-9):.0f}% 来自电路")
        if not ref_pass["功放通电"] and ref_pass["功放断电"]:
            where = "功放通电时才有：经功放电源/地线或功放输入线耦合，检查单点接地、喇叭线与麦克风线分开走、功放输入加 RC"
        elif not ref_pass["功放通电"] and not ref_pass["功放断电"]:
            where = "功放断电仍有：来自 DAC 线（PA4）或开发板本身，检查 PA4 走线是否靠近参考麦线、ADC 采样时间"
        else:
            where = ("参考麦里没有可测的电串扰；喇叭测试时参考麦削波是声学原因"
                     "（喇叭支管离参考麦近），降低喇叭幅度即可")
        tm_on = on["tm"]
        check("串扰位置", "PASS" if ref_pass["功放通电"] and ref_pass["功放断电"] else "WARN",
              where + f"。接喇叭时 err 相对 ref {tm_on['delay_samples']:+.1f} 样本"
              f"（模 {tm_on['period_samples']:.1f}），按几何应 "
              f"{(args.d_spk_err - args.d_spk_ref) / 343000 * fs:+.1f}")

    if args.part in ("source", "all"):
        # ---- 外部声源：手机单音，支架/贴管/静音文件对比
        print(f"\n== 外部声源（手机 {args.tone_hz:.0f} Hz）==")
        prompt(args, f"把手机固定在支架上、不碰管子，播放 {args.tone_hz:.0f} Hz 单音")
        if not args.no_prompt:
            target = f"{args.ref_target[0]:.0f}–{args.ref_target[1]:.0f} mV"
            while True:
                clip_tot = {"ref": 0, "err": 0}
                for _ in range(6):
                    lv = dev.level()
                    for ch in clip_tot:
                        clip_tot[ch] += lv[ch]["clip"]
                    print("  " + fmt_level(lv) + f"   (目标 ref {target})")
                    time.sleep(0.5)
                if any(clip_tot.values()):
                    print(f"  ！参考麦/误差麦在削波（ref clip 合计 {clip_tot['ref']}，"
                          f"err clip 合计 {clip_tot['err']}），请先调小手机音量")
                    if input("调小后按回车再看一遍；确实要带着削波继续，输入 y：").strip() == "y":
                        break
                elif input(f"参考麦 rms 目标 {target}，调好按回车，需要再看一遍输入 r：").strip() != "r":
                    break
        time.sleep(1.0)
        lv = dev.level()
        st, _, _ = grab("4_stand", args.tone_hz, 20.0)
        tm = st["tm"]
        prompt(args, "保持音量，把手机贴在管壁上（或用手扶着管子）")
        time.sleep(1.0)
        tc, _, _ = grab("5_touch", tm["f"])
        prompt(args, "手机放回支架，改播静音文件（没有的话就暂停）")
        time.sleep(1.0)
        zr, _, _ = grab("6_silent", tm["f"])
        prompt(args, "测试结束，可以关闭外部声源")

        lo_t, hi_t = args.ref_target
        amp_r = tm["ref"]["amp_mv"]
        check("ref 声源电平", "PASS" if lo_t <= amp_r <= hi_t else "WARN",
              f"ref {amp_r:.0f} mV（目标 {lo_t:.0f}–{hi_t:.0f}）"
              + ("" if lo_t <= amp_r <= hi_t
                 else f"  → 调手机音量{'调小' if amp_r > hi_t else '调大'}"))
        for c, ch in enumerate(an.CH_NAMES):
            clip = st["clip"][c] + lv[ch]["clip"]
            check(f"{ch} 不削波", "FAIL" if clip else "PASS",
                  f"占用量程 {st['use'][c] * 100:.0f}%，clip={clip}")
        h3r, h3e = tm["ref"]["h3_db"], tm["err"]["h3_db"]
        check("ref 谐波", "PASS" if h3r <= -30 else "WARN" if h3r <= -20 else "FAIL",
              f"ref 2/3 次 {tm['ref']['h2_db']:.0f}/{h3r:.0f} dB，"
              f"err 2/3 次 {tm['err']['h2_db']:.0f}/{h3e:.0f} dB  "
              + ("两麦都有 → 手机喇叭失真，调小音量" if h3e >= h3r - 6
                 else "只有参考麦明显 → 参考麦接近饱和或在近场，调小音量或让手机离参考麦远一些"))
        ql, sl, tl, zl = q["lf"][1], st["lf"][1], tc["lf"][1], zr["lf"][1]
        ds, dt, dz = db(sl, ql), db(tl, sl), db(sl, zl)
        parts = []
        if dt >= 6:
            parts.append("碰管/手扶会带进明显低频振动 → 手机必须用支架、不碰管")
        if ds >= 6:
            parts.append("低频跟播放内容有关（循环接缝咔哒或手机喇叭低频）→ 换长的无缝文件"
                         if dz >= 6 else
                         "不播放也有 → 与声音无关：支架振动、管口气流或手机位置")
        if not parts:
            parts.append("支架上播放时低频不明显")
        lf_detail = f"err <80 Hz：静音 {ql:.1f} / 支架 {sl:.1f} / 贴管 {tl:.1f} / 静音文件 {zl:.1f} mV。"
        sync_bad = [n for n in ("0_quiet", "4_stand", "5_touch", "6_silent")
                    if any(an.is_block_sync(rows[n]["sync"][c]) for c in (0, 1))]
        if sync_bad:
            check("低频来源", "WARN", lf_detail
                  + f"{', '.join(sync_bad)} 有 MCU 同步干扰，低频来源无法判断，排除干扰后重测")
        else:
            check("低频来源", "WARN" if ds >= 6 or dt >= 6 else "PASS", lf_detail + "；".join(parts))
        print(f"声源 {tm['f']:.1f} Hz，相干 {tm['coh']:.2f}，err 相对 ref "
              f"{tm['delay_samples']:+.1f} 样本（模 {tm['period_samples']:.1f}），"
              f"ref−err {db(tm['ref']['amp_mv'], tm['err']['amp_mv']):+.1f} dB")

    mcu_bad = {}
    for name in order:
        if name.startswith("0_quiet@"):
            continue
        hits = [ch for c, ch in enumerate(an.CH_NAMES) if an.is_block_sync(rows[name]["sync"][c])]
        if hits:
            mcu_bad[name] = hits
    if mcu_bad:
        detail = "；".join(
            f"{name}: " + " ".join(
                f"{ch} 块边界跳变是其余位置的 {rows[name]['sync'][c]['fold']:.1f} 倍"
                f"（{rows[name]['sync'][c]['n']} 个大跳变，{rows[name]['sync'][c]['sync_frac'] * 100:.0f}% "
                f"落在 {AUDIO_BLK} 样本边界）"
                for c, ch in enumerate(an.CH_NAMES) if ch in hits)
            for name, hits in mcu_bad.items())
        detail += ("  → 与固件处理块（16 ms）同步，不是声音：对照 0 段时的功放/喇叭接线和地线；"
                   "手机拿远、喇叭 STOP 时用 capture 复查，再拔喇叭线复查"
                   "（消失 → 喇叭在放 DAC 线上的噪声；仍在 → 地线/接线耦合）")
    else:
        detail = "没有与 128 样本块同步的跳变"
    check("MCU 同步干扰", "FAIL" if mcu_bad else "PASS", detail)

    table = ["| 段 | 单音 Hz | ref 单音 mV | err 单音 mV | ref−err dB | ref 3 次谐波 dB "
             "| err <80 Hz mV | err >80 Hz mV | ref clip | err clip | 同步跳变 |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for name in order:
        r, t = rows[name], rows[name]["tm"]
        sync_txt = " ".join(f"{ch} {r['sync'][c]['fold']:.1f}×"
                            for c, ch in enumerate(an.CH_NAMES)
                            if an.is_block_sync(r["sync"][c])) or "-"
        table.append(f"| {name} | {r['hz']:.0f} | {t['ref']['amp_mv']:.1f} | {t['err']['amp_mv']:.1f} "
                     f"| {db(t['ref']['amp_mv'], t['err']['amp_mv']):+.1f} | {t['ref']['h3_db']:.1f} "
                     f"| {r['lf'][1]:.1f} | {r['hf'][1]:.1f} | {r['clip'][0]} | {r['clip'][1]} | {sync_txt} |")
    check.write(d / "report.md", "P0 硬件对照",
                f"帧数 {args.frames}，喇叭 {args.spk_hz:.0f} Hz amp {amp_used}，"
                f"声源 {args.tone_hz:.0f} Hz，ref 目标 "
                f"{args.ref_target[0]:.0f}–{args.ref_target[1]:.0f} mV\n\n" + "\n".join(table),
                images)
    return check.ok()


def do_selftest(dev: Device, args):
    d = new_record_dir("selftest")
    check = Checks()

    dev.cmd("STREAM 0")
    dev.cmd("STOP")

    # 1 蓝牙链路
    rtts = []
    for i in range(args.pings):
        try:
            rtts.append(dev.ping(f"s{i}"))
        except DeviceError:
            dev.drain(0.2)
    check("蓝牙 PING", len(rtts) == args.pings,
          f"{len(rtts)}/{args.pings} ok, avg RTT {np.mean(rtts) if rtts else float('nan'):.0f} ms")

    info = dev.info()
    check("时钟 168 MHz", info.get("clk_ok") == "1", f"sysclk={info.get('sysclk')}")
    check("ADC/DAC 无溢出", info.get("ovr") == "0" and info.get("udr") == "0",
          f"ovr={info.get('ovr')} udr={info.get('udr')} late={info.get('late')}")

    # 2 麦克风静态（外部声源必须关）
    prompt(args, "请关闭外部声源、保持安静")
    quiet_lv, quiet = check_quiet(dev, args, check, d, "quiet")

    # 3 扬声器 → 两只麦
    dev.cmd(f"TONE {args.freq} {args.amp}")
    time.sleep(0.8)
    tone_lv = dev.level()
    print("采集单音段 ...")
    tone = save_capture(*dev.capture(args.frames), d, f"tone{args.freq:.0f}")
    dev.cmd("STOP")
    for c, ch in enumerate(an.CH_NAMES):
        q = an.tone_level_db(quiet["f"], quiet["p"][:, c], args.freq)
        t = an.tone_level_db(tone["f"], tone["p"][:, c], args.freq)
        clip = tone_lv[ch]["clip"]
        check(f"扬声器→{ch}", t - q >= args.min_rise and clip == 0,
              f"{args.freq:.0f} Hz 处 +{t - q:.1f} dB (要求 ≥{args.min_rise} dB), clip={clip}")

    # 4 可选：手动确认通道没有接反
    if args.check_swap:
        input("对着【参考麦】持续吹气或轻敲管壁靠近参考麦处，然后按回车 ...")
        lv = dev.level()
        rise = {ch: lv[ch]["dbfs"] - quiet_lv[ch]["dbfs"] for ch in lv}
        check("通道顺序", rise["ref"] > rise["err"],
              f"ref +{rise['ref']:.1f} dB, err +{rise['err']:.1f} dB")

    check.write(d / "report.md", "硬件自检",
                f"固件：{info.get('fw')}，波特率 {dev.baud}，单音 {args.freq} Hz / amp {args.amp}",
                ["quiet", f"tone{args.freq:.0f}"])
    return check.ok()


def main(argv=None):
    for stream in (sys.stdout, sys.stderr):  # 中文 Windows 控制台是 GBK，避免个别符号导致崩溃
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(description="Duct ANC 硬件测试上位机")
    ap.add_argument("-p", "--port", help="蓝牙 SPP 的 COM 口，如 COM7")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="与固件 BT_BAUD 一致，用于估算传输时间")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ports", help="列出串口")
    sub.add_parser("shell", help="交互式命令行")
    sub.add_parser("info", help="固件状态")
    s = sub.add_parser("ping", help="链路往返测试")
    s.add_argument("-n", type=int, default=20)
    s = sub.add_parser("tone", help="扬声器单音")
    s.add_argument("freq", type=float)
    s.add_argument("--amp", type=float, default=0.1)
    s = sub.add_parser("noise", help="扬声器白噪声")
    s.add_argument("--amp", type=float, default=0.1)
    s = sub.add_parser("sweep", help="扬声器对数扫频")
    s.add_argument("f1", type=float)
    s.add_argument("f2", type=float)
    s.add_argument("sec", type=float)
    s.add_argument("--amp", type=float, default=0.1)
    sub.add_parser("stop", help="扬声器静音")
    s = sub.add_parser("level", help="麦克风电平")
    s.add_argument("--watch", action="store_true")
    s = sub.add_parser("capture", help="采集两路麦克风原始数据并分析")
    s.add_argument("--frames", type=int, default=4000,
                   help=f"帧数，8 kHz 下 4000 = 0.5 s，上限 {CAP_MAX}（带 --dac 时 {CAP_MAX_DAC}）")
    s.add_argument("--tag", default="capture")
    s.add_argument("--dac", action="store_true", help="同时采 DAC 输出（与 ADC 逐帧对齐），用于辨识次级路径")
    s = sub.add_parser("selftest", help="一键自检：链路、偏置、扬声器→麦克风")
    s.add_argument("--pings", type=int, default=10)
    s.add_argument("--frames", type=int, default=2000)
    s.add_argument("--freq", type=float, default=200.0)
    s.add_argument("--amp", type=float, default=0.1)
    s.add_argument("--min-rise", type=float, default=10.0)
    s.add_argument("--max-quiet-mv", type=float, default=30.0, help="本底 100–400 Hz rms 上限")
    s.add_argument("--check-swap", action="store_true", help="交互确认参考/误差麦没有接反")
    s.add_argument("--no-prompt", action="store_true", help="不提示关外部声源")
    s = sub.add_parser("mictest", help="麦克风专项：本底、敲击定位、扬声器→麦、外部声源、相干性、声程差")
    s.add_argument("--frames", type=int, default=4000, help="每段帧数，8 kHz 下 4000 = 0.5 s")
    s.add_argument("--amp", type=float, default=0.02, help="扬声器白噪声幅度（0.1 已会打满麦克风）")
    s.add_argument("--min-rise", type=float, default=10.0)
    s.add_argument("--max-quiet-mv", type=float, default=30.0, help="本底 100–400 Hz rms 上限")
    s.add_argument("--d-ref-err", type=float, default=428, help="参考麦到误差麦的管长 mm")
    s.add_argument("--d-spk-ref", type=float, default=366, help="喇叭纸盆到参考麦的声程 mm（支管+主管）")
    s.add_argument("--d-spk-err", type=float, default=238, help="喇叭纸盆到误差麦的声程 mm")
    s.add_argument("--lag-tol", type=float, default=3, help="声程差允许偏差，样本")
    s.add_argument("--no-prompt", action="store_true", help="不交互（跳过敲击、不提示开关外部声源）")
    s = sub.add_parser("p0test", help="P0 硬件对照：参考麦电串扰（拔喇叭线/关功放）、外部声源音量与低频来源")
    s.add_argument("--part", choices=("couple", "source", "all"), default="all")
    s.add_argument("--frames", type=int, default=4000, help="每段帧数，8 kHz 下 4000 = 0.5 s，上限 16000")
    s.add_argument("--spk-hz", type=float, default=300.0)
    s.add_argument("--amps", default="0.02,0.01,0.005,0.002")
    s.add_argument("--tone-hz", type=float, default=200.0)
    s.add_argument("--ref-target", type=float, nargs=2, default=[200, 300])
    s.add_argument("--d-spk-ref", type=float, default=366, help="喇叭纸盆到参考麦的声程 mm（支管+主管）")
    s.add_argument("--d-spk-err", type=float, default=238, help="喇叭纸盆到误差麦的声程 mm")
    s.add_argument("--no-prompt", action="store_true", help="不交互（不提示操作、不调音量）")
    args = ap.parse_args(argv)

    if args.cmd == "ports":
        do_ports(args)
        return 0
    if not args.port:
        ap.error("需要 -p/--port，先运行 `python hwtest.py ports` 查看")

    dev = Device(args.port, args.baud)
    try:
        dev.drain()
        if args.cmd == "shell":
            do_shell(dev, args)
        elif args.cmd == "info":
            print(dev.cmd("INFO", prefixes=("INFO",)))
        elif args.cmd == "ping":
            return 0 if do_ping(dev, args) else 1
        elif args.cmd == "tone":
            print(dev.cmd(f"TONE {args.freq} {args.amp}"))
        elif args.cmd == "noise":
            print(dev.cmd(f"NOISE {args.amp}"))
        elif args.cmd == "sweep":
            print(dev.cmd(f"SWEEP {args.f1} {args.f2} {args.sec} {args.amp}"))
        elif args.cmd == "stop":
            print(dev.cmd("STOP"))
        elif args.cmd == "level":
            do_level(dev, args)
        elif args.cmd == "capture":
            do_capture(dev, args)
        elif args.cmd == "selftest":
            return 0 if do_selftest(dev, args) else 1
        elif args.cmd == "mictest":
            return 0 if do_mictest(dev, args) else 1
        elif args.cmd == "p0test":
            return 0 if do_p0test(dev, args) else 1
    except DeviceError as e:
        print(f"错误：{e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        pass
    finally:
        dev.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
