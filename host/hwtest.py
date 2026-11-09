"""管道 ANC 基础硬件测试上位机（蓝牙 SPP 虚拟串口）。

用法示例：
    python hwtest.py ports
    python hwtest.py -p COM7 ping -n 20
    python hwtest.py -p COM7 tone 200 --amp 0.1
    python hwtest.py -p COM7 level --watch
    python hwtest.py -p COM7 capture --frames 4000 --tag fan_on
    python hwtest.py -p COM7 selftest
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

ROOT = Path(__file__).resolve().parent.parent
RECORD_DIR = ROOT / "实验记录"
LVL_RE = re.compile(r"\|\s*(\w+) dc=([-\d.]+) rms=([-\d.]+) mv=([-\d.]+) dbfs=([-\d.]+) "
                    r"min=(\d+) max=(\d+) clip=(\d+)")


class DeviceError(RuntimeError):
    pass


class Device:
    """行协议：命令一行，回复 OK/ERR/数据行；'#' 开头是注释；CAP 回复 #CAP 头 + 二进制 + #END。"""

    def __init__(self, port: str, baud: int = 9600, ser=None):
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

    def capture(self, frames: int, progress=True) -> tuple[np.ndarray, int]:
        self.send(f"CAP {frames}")
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
    return {"f": f, "p": p, "stats": stats}


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
    codes, fs = dev.capture(args.frames)
    d = new_record_dir(args.tag)
    res = save_capture(codes, fs, d, args.tag)
    print_stats(res)
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
    """麦克风专项：本底 → 敲击定位 → 扬声器白噪声 → 风扇噪声（信噪比、削波、相干性、声程差）。"""
    d = new_record_dir("mictest")
    check = Checks()
    c_mm = 343000.0
    dev.cmd("STREAM 0")
    dev.cmd("STOP")

    print("\n== 1/4 本底噪声 ==")
    prompt(args, "请关闭风扇、保持安静")
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

    print("\n== 4/4 风扇噪声 ==")
    prompt(args, "请打开风扇，等转速稳定")
    time.sleep(1.5)
    lv = dev.level()
    fan, mv, fs = capture_mv(dev, args, d, "3_fan")
    for c, ch in enumerate(an.CH_NAMES):
        st = fan["stats"][c]
        use = (st["max"] - st["min"]) / 4095
        clip = st["clip"] + lv[ch]["clip"]
        check(f"{ch} 风扇不削波", "FAIL" if clip else "WARN" if use > 0.9 or use < 0.05 else "PASS",
              f"占用量程 {use * 100:.0f}%，clip={clip}（5%–90% 合适；过小说明增益偏低）")
        snr = st["band_100_400_db"] - quiet["stats"][c]["band_100_400_db"]
        check(f"{ch} 风扇信噪比", "PASS" if snr >= 20 else "WARN" if snr >= 12 else "FAIL",
              f"100–400 Hz 比本底高 {snr:.1f} dB（≥20 理想；要测出 10 dB 降噪至少 ≥12）"
              + ("  → 开风扇前后几乎没变化：确认第 1 步时风扇已断电；若确实断电，"
                 "说明这只风扇在 100–400 Hz 几乎不出声" if snr < 3 else ""))
    coh = fan["stats"][0]["coh_100_400"]
    check("相干性 100–400 Hz", "PASS" if coh >= 0.9 else "WARN" if coh >= 0.7 else "FAIL",
          f"平均 γ^2={coh:.2f} → 前馈降噪理论上限约 {-10 * np.log10(1 - min(coh, 0.999)):.1f} dB")
    lag, _ = an.gcc_phat_lag(mv[:, 0], mv[:, 1], fs)
    exp = args.d_ref_err / c_mm * fs
    check("声程差·风扇", lag_status(lag, exp, args.lag_tol),
          f"err 比 ref 晚 {lag:+d} 样本（≈{lag * c_mm / fs:.0f} mm），按几何应 {exp:+.1f}")
    m = (fan["f"] >= 50) & (fan["f"] <= 1000)
    peaks = fan["f"][m][np.argsort(fan["p"][m, 1])[-5:][::-1]]
    print(f"风扇噪声误差麦主要频率：{', '.join(f'{p:.0f}' for p in peaks)} Hz")
    prompt(args, "测试结束，可以关闭风扇")

    check.write(d / "report.md", "麦克风测试",
                f"帧数 {args.frames}（{args.frames / fs:.2f} s），扬声器白噪声 amp {args.amp}；"
                f"几何：ref→err {args.d_ref_err:.0f} mm，喇叭→ref {args.d_spk_ref:.0f} mm，"
                f"喇叭→err {args.d_spk_err:.0f} mm。风扇噪声误差麦主要频率："
                f"{', '.join(f'{p:.0f}' for p in peaks)} Hz",
                ["1_quiet", "2_speaker_noise", "3_fan"])
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

    # 2 麦克风静态（风扇必须关）
    prompt(args, "请关闭风扇、保持安静")
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
    ap.add_argument("-b", "--baud", type=int, default=9600, help="与固件 BT_BAUD 一致，用于估算传输时间")
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
    s.add_argument("--frames", type=int, default=4000, help="帧数，8 kHz 下 4000 = 0.5 s，上限 16000")
    s.add_argument("--tag", default="capture")
    s = sub.add_parser("selftest", help="一键自检：链路、偏置、扬声器→麦克风")
    s.add_argument("--pings", type=int, default=10)
    s.add_argument("--frames", type=int, default=2000)
    s.add_argument("--freq", type=float, default=200.0)
    s.add_argument("--amp", type=float, default=0.1)
    s.add_argument("--min-rise", type=float, default=10.0)
    s.add_argument("--max-quiet-mv", type=float, default=30.0, help="本底 100–400 Hz rms 上限")
    s.add_argument("--check-swap", action="store_true", help="交互确认参考/误差麦没有接反")
    s.add_argument("--no-prompt", action="store_true", help="不提示关风扇")
    s = sub.add_parser("mictest", help="麦克风专项：本底、敲击定位、扬声器→麦、风扇噪声、相干性、声程差")
    s.add_argument("--frames", type=int, default=4000, help="每段帧数，8 kHz 下 4000 = 0.5 s")
    s.add_argument("--amp", type=float, default=0.02, help="扬声器白噪声幅度（0.1 已会打满麦克风）")
    s.add_argument("--min-rise", type=float, default=10.0)
    s.add_argument("--max-quiet-mv", type=float, default=30.0, help="本底 100–400 Hz rms 上限")
    s.add_argument("--d-ref-err", type=float, default=428, help="参考麦到误差麦的管长 mm")
    s.add_argument("--d-spk-ref", type=float, default=366, help="喇叭纸盆到参考麦的声程 mm（支管+主管）")
    s.add_argument("--d-spk-err", type=float, default=238, help="喇叭纸盆到误差麦的声程 mm")
    s.add_argument("--lag-tol", type=float, default=3, help="声程差允许偏差，样本")
    s.add_argument("--no-prompt", action="store_true", help="不交互（跳过敲击、不提示开关风扇）")
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
