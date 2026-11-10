"""上位机离线测试：用模拟固件（与 app.c 的输出格式一致）跑通协议解析和自检流程。

    cd host && python -m pytest -q
"""
import argparse

import numpy as np
import pytest

import hwtest


class FakeFirmware:
    """模拟 USART 另一端。write() 收命令，read() 吐回复字节。"""

    def __init__(self, fs=8000):
        self.fs = fs
        self.out = bytearray()
        self.inbuf = b""
        self.tone = None
        self.rng = np.random.default_rng(0)
        self.out += b"\r\n# duct-anc-hwtest-0.1  STM32F407VET6  sysclk=168000000Hz fs=8000 baud=9600\r\n"

    in_waiting = property(lambda self: len(self.out))

    def close(self):
        pass

    def read(self, n):
        d, self.out = bytes(self.out[:n]), self.out[n:]
        return d

    def write(self, data):
        self.inbuf += data
        while b"\n" in self.inbuf:
            line, self.inbuf = self.inbuf.split(b"\n", 1)
            self.handle(line.decode().strip().split())

    noise = False
    source = False
    phone = False      # 手机单音（外部声源）
    touch = False      # 手机贴管/手扶：给 err 加 30 Hz 低频
    wired = True       # 喇叭线接着
    amp_on = True      # 功放通电
    couple = 0.0       # 电串扰比例（ref 直接吃到喇叭信号）
    mcu_sync = False   # 与固件 128 样本块同步的跳变干扰

    def frames(self, n, dac=False):
        t = np.arange(n) / self.fs
        i = np.arange(n)
        x = self.rng.normal(0, 3, (n, 2))
        u = np.zeros(n)  # DAC 输出（相对中点的码值）
        if self.mcu_sync:  # 采集从块边界开始：每个块头一个指数衰减台阶
            x[:, 0] += 200 * np.exp(-(i % 128) / 40)
            x[:, 1] += 800 * np.exp(-(i % 128) / 40)
        if self.tone:
            u += 200 * np.sin(2 * np.pi * self.tone * t)
            g = [300, 120] if (self.wired and self.amp_on) else [0, 0]
            x += np.outer(np.sin(2 * np.pi * self.tone * t), g)
            if self.amp_on:  # 电串扰：功放通电就有，和喇叭线无关
                x[:, 0] += 300 * self.couple * np.sin(2 * np.pi * self.tone * t)
        if self.phone:  # 手机在管口上游：err 比 ref 晚 10 个样本
            v = np.sin(2 * np.pi * 200 * np.arange(n + 10) / self.fs)
            x[:, 0] += 440 * v[10:]
            x[:, 1] += 25 * v[:n]
        if self.touch:
            x[:, 1] += 150 * np.sin(2 * np.pi * 30 * t)
        if self.noise:  # 喇叭离误差麦更近：err 比 ref 早 3 个样本；DAC→err 20 样本，DAC→ref 23 样本
            s = self.rng.normal(0, 1, n + 23)
            x[:, 0] += 150 * s[:n]
            x[:, 1] += 150 * s[3:n + 3]
            u += 500 * s[23:]
        if self.source:  # 外部声源在上游：err 比 ref 晚 10 个样本
            v = self.rng.normal(0, 1, n + 10) + 2 * np.sin(2 * np.pi * 300 * np.arange(n + 10) / self.fs)
            x[:, 0] += 200 * v[10:]
            x[:, 1] += 160 * v[:n]
        if dac:
            x = np.column_stack([x, u])
        return np.clip(np.round(2048 + x), 0, 4095).astype("<u2")

    def emit(self, s):
        self.out += (s + "\r\n").encode()

    def handle(self, argv):
        if not argv:
            return
        c = argv[0].upper()
        if c == "PING":
            self.emit(f"PONG {argv[1] if len(argv) > 1 else '-'} 1234")
        elif c == "INFO":
            self.emit("INFO fw=duct-anc-hwtest-0.1 sysclk=168000000 clk_ok=1 fs=8000 baud=9600 up=1.0"
                      " out=OFF f1=0.0 f2=0.0 amp=0.000 active=0 blocks=10 late=0 ovr=0 udr=0 rxovf=0 heap=1000")
        elif c == "TONE":
            self.tone = float(argv[1])
            self.emit(f"OK out=TONE f1={argv[1]} f2={argv[1]} amp=0.100")
        elif c == "NOISE":
            self.noise = True
            self.emit("OK out=NOISE f1=0.0 f2=0.0 amp=0.100")
        elif c in ("STOP", "STREAM"):
            if c == "STOP":
                self.tone, self.noise = None, False
            self.emit("OK")
        elif c == "LEVEL":
            x = self.frames(3968)
            parts = []
            for ch, name in enumerate(("ref", "err")):
                v = x[:, ch].astype(float)
                rms = v.std()
                parts.append(f" | {name} dc={v.mean():.1f} rms={rms:.2f} mv={rms * 3300 / 4095:.2f} "
                             f"dbfs={20 * np.log10(rms / 1447.5):.1f} min={int(v.min())} max={int(v.max())} clip=0")
            self.emit("LVL t=1000 n=3968" + "".join(parts))
        elif c == "CAP":
            n, dac = int(argv[1]), len(argv) > 2 and argv[2].upper() == "D"
            if n > (10666 if dac else 16000):
                self.emit("ERR usage: CAP <frames 1..16000> | CAP <frames 1..10666> D")
                return
            x = self.frames(n, dac)
            self.emit(f"#CAP {n} {self.fs} {x.shape[1]}")
            self.out += x.tobytes()
            self.emit(f"#END {int(x.astype(np.uint64).sum()) & 0xFFFFFFFF:08X}")
        else:
            self.emit(f"ERR unknown command '{c}', type HELP")


@pytest.fixture
def dev():
    d = hwtest.Device("FAKE", 1_000_000, ser=FakeFirmware())
    d.drain(0.05)
    return d


def test_ping_and_errors(dev):
    assert dev.ping("abc") >= 0
    with pytest.raises(hwtest.DeviceError):
        dev.cmd("FOO")


def test_level_parse(dev):
    lv = dev.level()
    assert set(lv) == {"ref", "err"}
    assert abs(lv["ref"]["dc"] - 2048) < 5


def test_capture_checksum(dev):
    codes, fs = dev.capture(1000, progress=False)
    assert codes.shape == (1000, 2) and fs == 8000


def test_capture_with_dac(dev):
    dev.cmd("NOISE 0.02")
    codes, fs = dev.capture(8000, progress=False, dac=True)
    assert codes.shape == (8000, 3)
    k = hwtest.an.dac_check(codes[:, 2], hwtest.an.codes_to_mv(codes[:, :2]), fs)
    assert k["err"]["peak_lag"] == 20 and k["ref"]["peak_lag"] == 23
    assert k["err"]["onset"] == 20 and k["ref"]["onset"] == 23 and k["err"]["coh_200"] > 0.9
    assert k["err"]["coh_100_400"] > 0.9
    with pytest.raises(hwtest.DeviceError):
        dev.capture(10667, progress=False, dac=True)


def test_do_capture_dac_files(dev, tmp_path, monkeypatch):
    monkeypatch.setattr(hwtest, "RECORD_DIR", tmp_path)
    dev.cmd("NOISE 0.02")
    hwtest.do_capture(dev, argparse.Namespace(frames=4000, tag="sid", dac=True))
    d = next(tmp_path.glob("*_sid"))
    z = np.load(d / "sid.npz")
    assert z["codes"].shape == (4000, 2) and z["dac"].shape == (4000,)
    assert (d / "sid_dac.png").exists() and (d / "sid.png").exists()
    with pytest.raises(hwtest.DeviceError):
        hwtest.do_capture(dev, argparse.Namespace(frames=12000, tag="x", dac=True))


def test_crosstalk_detects_floating_channel():
    rng = np.random.default_rng(1)
    ref = 2048 + 800 * np.sin(np.arange(4000) * 0.1) + rng.normal(0, 5, 4000)
    ghost = np.stack([ref, 0.7 * ref + 350 + rng.normal(0, 20, 4000)], axis=1)
    acoustic = np.stack([ref, np.roll(ref, 10)], axis=1)
    xt = hwtest.an.crosstalk(ghost)
    assert xt["corr"] > 0.95 and xt["lag"] == 0
    assert hwtest.an.crosstalk(acoustic)["lag"] == 10


def test_gcc_phat_lag():
    rng = np.random.default_rng(2)
    s = rng.normal(0, 1, 4010)
    assert hwtest.an.gcc_phat_lag(s[10:], s[:4000], 8000)[0] == 10
    assert hwtest.an.gcc_phat_lag(s[:4000], s[3:4003], 8000)[0] == -3


def test_mictest_end_to_end(dev, tmp_path, monkeypatch):
    monkeypatch.setattr(hwtest, "RECORD_DIR", tmp_path)
    monkeypatch.setattr(hwtest.time, "sleep", lambda s: None)
    fw = dev.ser

    def fake_prompt(args, msg):  # 代替人工开关外部声源
        fw.source = "播放 200 Hz" in msg

    monkeypatch.setattr(hwtest, "prompt", fake_prompt)
    args = argparse.Namespace(frames=4000, amp=0.1, min_rise=10.0, max_quiet_mv=50.0,
                              d_ref_err=428, d_spk_ref=366, d_spk_err=238, lag_tol=3,
                              no_prompt=True)
    assert hwtest.do_mictest(dev, args)
    report = next(tmp_path.glob("*_mictest/report.md")).read_text(encoding="utf-8")
    assert "FAIL" not in report and "声程差·声源 | PASS" in report and "声程差·扬声器 | PASS" in report


def test_selftest_end_to_end(dev, tmp_path, monkeypatch):
    monkeypatch.setattr(hwtest, "RECORD_DIR", tmp_path)
    monkeypatch.setattr(hwtest.time, "sleep", lambda s: None)
    args = argparse.Namespace(
        pings=3, frames=2000, freq=200.0, amp=0.1, min_rise=10.0, check_swap=False,
        no_prompt=True, max_quiet_mv=50.0)
    assert hwtest.do_selftest(dev, args)
    report = next(tmp_path.glob("*_selftest/report.md")).read_text(encoding="utf-8")
    assert "FAIL" not in report


def test_fir_ls_recovers_known_impulse():
    rng = np.random.default_rng(0)
    h = np.array([0.0, 0.0, 0.5, -0.2, 0.1])
    u = rng.normal(size=4000)
    y = np.convolve(u, h)[:len(u)]
    hat = hwtest.an.fir_ls(u, y, 8)
    assert np.max(np.abs(hat[:5] - h)) < 1e-6
    assert np.max(np.abs(hat[5:])) < 1e-6


def test_sine_fit_and_tone_metrics():
    fs, n = 8000, 4000
    t = np.arange(n) / fs
    rng = np.random.default_rng(3)
    ref = 300 * np.sqrt(2) * np.sin(2 * np.pi * 200 * t + 0.3) + rng.normal(0, 3, n)
    err = np.roll(30 * np.sqrt(2) * np.sin(2 * np.pi * 200 * t + 0.3), 10)
    err += 30 * np.sqrt(2) * 10 ** (-20 / 20) * np.sin(2 * np.pi * 600 * t)
    tm = hwtest.an.tone_metrics(np.stack([ref, err], axis=1), fs, 200)
    assert abs(tm["f"] - 200) < 0.2
    assert abs(tm["ref"]["amp_mv"] - 300) / 300 < 0.02
    assert abs(tm["err"]["amp_mv"] - 30) / 30 < 0.02
    assert abs(tm["delay_samples"] - 10) < 0.5
    assert abs(tm["err"]["h3_db"] - (-20)) < 1
    amp, phi = hwtest.an.sine_fit(np.sqrt(2) * 100 * np.sin(2 * np.pi * 200 * t + 0.5), fs, 200)
    assert abs(amp - 100) < 1 and abs(phi - 0.5) < 0.01


def test_block_sync_steps():
    fs = 8000
    i = np.arange(4000)
    rng = np.random.default_rng(4)
    x = rng.normal(0, 3, 4000) + 600 * np.exp(-(i % 128) / 40)
    r = hwtest.an.block_sync_steps(x)
    assert r["n"] >= 20 and r["sync_frac"] >= 0.9 and hwtest.an.is_block_sync(r)
    x = rng.normal(0, 3, 4000) + 600 * np.exp(-(i % 150) / 40)
    r = hwtest.an.block_sync_steps(x)
    assert r["sync_frac"] < 0.5 and not hwtest.an.is_block_sync(r)
    for f0 in (200, 250):  # 250 Hz 每块正好 4 个周期，折叠后不能被误判
        x = 1400 * np.sin(2 * np.pi * f0 * i / fs) + rng.normal(0, 3, 4000)
        assert not hwtest.an.is_block_sync(hwtest.an.block_sync_steps(x))
    # 同步跳变叠在削波的大单音上（真实 4_stand 的情况）也要能检出
    x = np.clip(1400 * np.sin(2 * np.pi * 200 * i / fs), -1000, 1000) + 300 * np.exp(-(i % 128) / 40)
    assert hwtest.an.is_block_sync(hwtest.an.block_sync_steps(x + rng.normal(0, 3, 4000)))


def test_band_rms_mv():
    fs = 8000
    t = np.arange(4000) / fs
    x = 100 * np.sin(2 * np.pi * 30 * t) + 100 * np.sin(2 * np.pi * 300 * t)
    assert abs(hwtest.an.band_rms_mv(x, fs, hi=80) - 100 / np.sqrt(2)) / (100 / np.sqrt(2)) < 0.05
    assert abs(hwtest.an.band_rms_mv(x, fs, lo=80) - 100 / np.sqrt(2)) / (100 / np.sqrt(2)) < 0.05


def _p0_args():
    return argparse.Namespace(frames=4000, part="all", spk_hz=300.0, amps="0.02,0.01",
                              tone_hz=200.0, ref_target=[200, 300],
                              d_spk_ref=366, d_spk_err=238, no_prompt=True)


def _p0_prompt(fw):
    def fake_prompt(args, msg):  # 代替人工操作喇叭线/功放/手机
        if "拔掉喇叭线" in msg:
            fw.wired = False
        elif "关闭 PAM8403" in msg:
            fw.amp_on = False
        elif "恢复" in msg:
            fw.wired = fw.amp_on = True
        elif "贴在管壁" in msg:
            fw.phone = fw.touch = True
        elif "播放 200 Hz" in msg:
            fw.phone, fw.touch = True, False
        elif "静音文件" in msg or "关闭外部声源" in msg:
            fw.phone = fw.touch = False
    return fake_prompt


def test_p0test_clean(dev, tmp_path, monkeypatch):
    monkeypatch.setattr(hwtest, "RECORD_DIR", tmp_path)
    monkeypatch.setattr(hwtest.time, "sleep", lambda s: None)
    monkeypatch.setattr(hwtest, "prompt", _p0_prompt(dev.ser))
    assert hwtest.do_p0test(dev, _p0_args())
    report = next(tmp_path.glob("*_p0test/report.md")).read_text(encoding="utf-8")
    assert "ref 电串扰·功放通电 | PASS" in report
    assert "ref 电串扰·功放断电 | PASS" in report
    assert "ref 声源电平 | PASS" in report
    assert "MCU 同步干扰 | PASS" in report
    row = next(l for l in report.splitlines() if "低频来源" in l)
    assert "碰管" in row


def test_p0test_coupled(dev, tmp_path, monkeypatch):
    monkeypatch.setattr(hwtest, "RECORD_DIR", tmp_path)
    monkeypatch.setattr(hwtest.time, "sleep", lambda s: None)
    monkeypatch.setattr(hwtest, "prompt", _p0_prompt(dev.ser))
    dev.ser.couple = 0.3
    assert not hwtest.do_p0test(dev, _p0_args())
    report = next(tmp_path.glob("*_p0test/report.md")).read_text(encoding="utf-8")
    assert "ref 电串扰·功放通电 | FAIL" in report
    assert "经功放电源/地线" in report


def test_p0test_mcu_sync(dev, tmp_path, monkeypatch):
    monkeypatch.setattr(hwtest, "RECORD_DIR", tmp_path)
    monkeypatch.setattr(hwtest.time, "sleep", lambda s: None)

    def fake_prompt(args, msg):
        _p0_prompt(dev.ser)(args, msg)
        if "恢复" in msg:
            dev.ser.mcu_sync = True

    monkeypatch.setattr(hwtest, "prompt", fake_prompt)
    assert not hwtest.do_p0test(dev, _p0_args())
    report = next(tmp_path.glob("*_p0test/report.md")).read_text(encoding="utf-8")
    assert "MCU 同步干扰 | FAIL" in report
    row = next(l for l in report.splitlines() if "低频来源" in l)
    assert "无法判断" in row


def test_shat_eff_matches_firmware_bits_and_200hz():
    import anc_coeff as ac
    src = (ac.SHAT_NPZ.parents[2] / "firmware" / "hwtest" / "User" / "anc.c").read_text(encoding="utf-8")
    for u in ac.HP_B_BITS + ac.HP_A_BITS + (ac.MV_PER_CODE_BITS, ac.GOERTZEL_COS2_BITS, ac.GOERTZEL_SIN_BITS):
        assert f"0x{u:08X}" in src
    coeff, info = ac.shat_eff()
    assert coeff.shape == (384,) and coeff.dtype == np.float32
    assert abs(info["db_200"]) < 0.05 and abs(info["deg_200"]) < 0.1
    z = np.load(ac.PRIMARY)
    xf = ac.replay_xf(z["codes"][:64, 0], coeff)
    assert np.all(np.isfinite(xf))
    # 零状态下第一拍只有 h[0]*x
    x0 = np.float32((int(z["codes"][0, 0]) - 2048) * ac.MV_PER_CODE)
    zstate = [np.float32(0), np.float32(0)]
    y0 = ac._biquad(x0, zstate)
    assert abs(float(xf[0] - np.float32(coeff[0] * y0))) < 1e-5


def test_goertzel_rms_on_a_known_tone():
    import anc_coeff as ac
    n = 2000
    t = np.arange(n) / ac.FS
    peak = 100.0
    x = (peak * np.sin(2 * np.pi * 200 * t)).astype(np.float32)
    rms = ac.goertzel_rms(x)
    assert abs(rms - peak / np.sqrt(2)) / (peak / np.sqrt(2)) < 0.01


def test_anc_status_parse(dev):
    dev.ser.emit("ANC on=0 prot=0 shat=1 mu=0.0020 baseline=152.00 cpu=12.5 "
                 "late=0 ovr=0 udr=0 slip=0 rxovf=0")
    st = dev.anc_status()
    assert st["on"] == "0" and st["shat"] == "1" and float(st["baseline"]) == 152.0
