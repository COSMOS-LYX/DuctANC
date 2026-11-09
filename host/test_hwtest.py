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
    fan = False

    def frames(self, n):
        t = np.arange(n) / self.fs
        x = self.rng.normal(0, 3, (n, 2))
        if self.tone:
            x += np.outer(np.sin(2 * np.pi * self.tone * t), [300, 120])
        if self.noise:  # 喇叭离误差麦更近：err 比 ref 早 3 个样本
            s = self.rng.normal(0, 1, n + 3)
            x[:, 0] += 150 * s[:n]
            x[:, 1] += 150 * s[3:]
        if self.fan:  # 风扇在上游：err 比 ref 晚 10 个样本
            v = self.rng.normal(0, 1, n + 10) + 2 * np.sin(2 * np.pi * 300 * np.arange(n + 10) / self.fs)
            x[:, 0] += 200 * v[10:]
            x[:, 1] += 160 * v[:n]
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
            n = int(argv[1])
            x = self.frames(n)
            self.emit(f"#CAP {n} {self.fs} 2")
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

    def fake_prompt(args, msg):  # 代替人工开关风扇
        fw.fan = "打开风扇" in msg

    monkeypatch.setattr(hwtest, "prompt", fake_prompt)
    args = argparse.Namespace(frames=4000, amp=0.1, min_rise=10.0, max_quiet_mv=50.0,
                              d_ref_err=428, d_spk_ref=366, d_spk_err=238, lag_tol=3,
                              no_prompt=True)
    assert hwtest.do_mictest(dev, args)
    report = next(tmp_path.glob("*_mictest/report.md")).read_text(encoding="utf-8")
    assert "FAIL" not in report and "声程差·风扇 | PASS" in report and "声程差·扬声器 | PASS" in report


def test_selftest_end_to_end(dev, tmp_path, monkeypatch):
    monkeypatch.setattr(hwtest, "RECORD_DIR", tmp_path)
    monkeypatch.setattr(hwtest.time, "sleep", lambda s: None)
    args = argparse.Namespace(
        pings=3, frames=2000, freq=200.0, amp=0.1, min_rise=10.0, check_swap=False,
        no_prompt=True, max_quiet_mv=50.0)
    assert hwtest.do_selftest(dev, args)
    report = next(tmp_path.glob("*_selftest/report.md")).read_text(encoding="utf-8")
    assert "FAIL" not in report
