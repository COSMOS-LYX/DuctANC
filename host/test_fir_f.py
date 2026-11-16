"""宽带 FIR 回流：辨识 h_ref，以及仿真植物卷积。

    cd host && python -m pytest -q test_fir_f.py
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

import analysis as an
import anc_coeff as ac
import anc_p4
import hwtest
import identify_s as ids
import sim_p3 as sim
from test_hwtest import FakeFirmware


def _wrap_deg(deg: float) -> float:
    return float((deg + 180.0) % 360.0 - 180.0)


def test_fit_secondary_href_matches_known_sine():
    """三段白噪声上的 pooled LS，F(200) 对准一条已知延迟正弦。"""
    fs = ids.FS
    n_taps = ids.N_TAPS
    h_ref_true = np.zeros(n_taps)
    h_err_true = np.zeros(n_taps)
    h_ref_true[3] = 4.67
    h_err_true[5] = 10.61
    rng = np.random.default_rng(0)
    recs = []
    for _ in range(3):
        u = rng.normal(size=8000)
        ref = np.convolve(u, h_ref_true)[:len(u)]
        err = np.convolve(u, h_err_true)[:len(u)]
        recs.append((u, np.column_stack([ref, err])))
    h_err, h_ref = ids.fit_secondary(recs, n_taps)
    assert h_err.shape == (n_taps,) and h_ref.shape == (n_taps,)
    assert abs(h_ref[3] - 4.67) / 4.67 < 0.05
    assert abs(h_err[5] - 10.61) / 10.61 < 0.05

    t = np.arange(fs) / fs
    u = np.sin(2 * np.pi * 200 * t)
    y = np.convolve(u, h_ref_true)[:len(u)]
    sl = slice(400, None)
    au, pu = an.sine_fit(u[sl], fs, 200)
    ay, py = an.sine_fit(y[sl], fs, 200)
    gain = ay / au
    dphi = np.degrees((py - pu + np.pi) % (2 * np.pi) - np.pi)
    _, mag, deg = ids.tone_response(h_ref, 200.0, fs)
    assert abs(mag - gain) / gain < 0.02
    assert abs(_wrap_deg(deg - dphi)) < 2.0
    assert abs(gain - 4.67) / 4.67 < 0.02
    expect = -3 * 360.0 * 200.0 / fs
    assert abs(_wrap_deg(dphi - expect)) < 2.0


def test_load_fit_without_href(tmp_path):
    path = tmp_path / "shat.npz"
    np.savez(path, h_err=np.arange(384, dtype=np.float64))
    h_err, h_ref = ids.load_fit(path)
    assert h_ref is None and h_err.shape == (384,)
    assert ac.load_h_ref(path) is None
    kept, href = ids.load_fit(ac.SHAT_NPZ)
    assert kept.shape == (384,) and href is not None and href.shape == (384,)


def test_nofeedback_float_baseline_cancels():
    """无回流、浮点、模型和植物一致时，200 Hz 应被压下去，并且不炸。"""
    fs = sim.FS
    n = 3 * fs
    t = np.arange(n) / fs
    x0 = 100.0 * np.sqrt(2.0) * np.sin(2 * np.pi * 200 * t)
    d = np.zeros(n)
    d[8:] = 0.8 * x0[:-8]
    S = np.zeros(16)
    S[2] = 1.0
    b, a = sim.hp_coeff(0)
    e, y, mute, blew, _, _ = sim.simulate(
        np.ascontiguousarray(x0), np.ascontiguousarray(d),
        S, S.copy(), np.zeros(1), 16, 0.0, 0.0, b, a,
        0.01, 0.0, 1.0, 0.0,
        sim.FB_OFF, 0, 0, 0,
        1.0, 0.0, 4, np.zeros(16),
    )
    assert blew == 0 and mute < 0
    assert np.all(np.isfinite(e)) and np.all(np.isfinite(y))
    tail = slice(n // 2, None)
    assert np.std(e[tail]) < 0.25 * np.std(d[tail])


def test_sim_fir_plant_matches_known_sine():
    """w=1、μ=0 时，y = x0 + F*y。h[2] 再加仿真里那 1 拍，相对 y 晚 3 个样本。"""
    fs = sim.FS
    n = 2000
    t = np.arange(n) / fs
    x0 = np.sin(2 * np.pi * 200 * t)
    d = np.zeros(n)
    # h[2] 是同一拍辨识的第 2 拍，仿真再晚 1 拍，相对 y 共晚 3 个样本。
    F = np.zeros(8)
    F[2] = 0.25
    delay = 2 + 1
    b, a = sim.hp_coeff(0)
    _e, y, _mute, blew, _, _ = sim.simulate(
        np.ascontiguousarray(x0), d, np.zeros(4), np.zeros(1), F,
        1, 0.0, 0.0, b, a,
        0.0, 0.0, 1.0, 0.0,
        sim.FB_FIR, 0, 0, 0,
        1.0, 0.0, 4, np.array([1.0]),
    )
    assert blew == 0
    y_exp = np.zeros(n)
    for i in range(n):
        fb = 0.25 * y_exp[i - delay] if i >= delay else 0.0
        y_exp[i] = x0[i] + fb
    assert np.max(np.abs(y - y_exp)) < 1e-8
    planted = sim.fir_plant(y, F, plant_lag=1)
    assert np.max(np.abs((y - x0) - planted)) < 1e-8
    sl = slice(400, None)
    ay, py = an.sine_fit(y[sl], fs, 200)
    af, pf = an.sine_fit((y - x0)[sl], fs, 200)
    assert abs(af / ay - 0.25) < 0.01
    dphi = (pf - py + np.pi) % (2 * np.pi) - np.pi
    expect = -delay * 2 * np.pi * 200 / fs
    assert abs(dphi - expect) < np.radians(2.0)


def test_official_href_wires_into_quantized_plant():
    S, F = sim.load_plant()
    assert F is not None and F.shape == S.shape == (384,)
    mag, _lag, deg = sim.fir_response(F)
    assert np.isfinite(mag) and mag > 0 and np.isfinite(deg)
    n = sim.FS
    rng = np.random.default_rng(1)
    x0 = rng.normal(size=n)
    d = np.zeros(n)
    d[10:] = 0.5 * x0[:-10]
    r = sim.run_case(
        x0, d, S, S, 0.0, 0.0, N=sim.N0, mu=sim.MU0, gamma=sim.G0,
        hp=1, fb="fir", align=True, quant=True, F=F,
    )
    assert np.isfinite(r["y"]).all()
    assert np.max(np.abs(r["y"])) <= sim.DAC_LIM + 1.0


def test_anc_step_does_not_read_fref():
    root = Path(__file__).resolve().parents[1] / "firmware" / "hwtest" / "User"
    src = (root / "anc.c").read_text(encoding="utf-8")
    body = src.split("void anc_step", 1)[1].split("int anc_replay_xf", 1)[0]
    assert "fref" not in body
    app = (root / "app.c").read_text(encoding="utf-8")
    assert "FREF" in app and "cmd_fref" in app


class _Banks(FakeFirmware):
    def __init__(self, know_fref=True):
        super().__init__()
        self.know_fref = know_fref
        self.banks = {
            "SHAT": np.zeros(ac.N_SHAT, np.float32),
            "FREF": np.zeros(ac.N_SHAT, np.float32),
        }
        self.mask = {"SHAT": 0, "FREF": 0}

    def handle(self, argv):
        if not argv:
            return
        c = argv[0].upper()
        if c not in ("SHAT", "FREF"):
            return super().handle(argv)
        if c == "FREF" and not self.know_fref:
            self.emit("ERR unknown command 'FREF', type HELP")
            return
        if len(argv) == 2 and argv[1].upper() == "RST":
            self.banks[c][:] = 0
            self.mask[c] = 0
            self.emit(f"OK {c.lower()} rst")
            return
        if len(argv) == 2 and argv[1].upper() == "CRC":
            crc = ac.crc32_f32(self.banks[c])
            ready = int(self.mask[c] == (1 << (ac.N_SHAT // 8)) - 1)
            self.emit(f"OK {c.lower()} crc={crc:08X} ready={ready}")
            return
        index = int(float(argv[1]))
        raw = np.frombuffer(bytes.fromhex(argv[2]), dtype="<f4")
        self.banks[c][index:index + 8] = raw
        self.mask[c] |= 1 << (index // 8)
        self.emit(f"OK {c.lower()} index={index} ready=0 crc=00000000")


def test_upload_fref_roundtrip_and_skips_old_firmware():
    se = np.zeros(ac.N_SHAT, np.float32)
    se[0] = np.float32(1.25)
    href = ac.fref_f32(np.linspace(-0.2, 0.4, ac.N_SHAT))
    fw = _Banks(True)
    dev = hwtest.Device("FAKE", 1_000_000, ser=fw)
    dev.drain(0.05)
    got = anc_p4.upload(dev, se, href, require_fref=True)
    assert got["fref_skipped"] is False
    assert got["fref"]
    assert np.array_equal(fw.banks["SHAT"], se)
    assert np.array_equal(fw.banks["FREF"], href)

    old = _Banks(False)
    dev_old = hwtest.Device("FAKE", 1_000_000, ser=old)
    dev_old.drain(0.05)
    skipped = anc_p4.upload(dev_old, se, href, require_fref=False)
    assert skipped["fref_skipped"] is True and skipped["fref"] is None
    assert np.array_equal(old.banks["SHAT"], se)
    with pytest.raises(hwtest.DeviceError):
        anc_p4.upload(dev_old, se, href, require_fref=True)
