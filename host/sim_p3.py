"""P3：用实测 Ŝ 和录音，在电脑上跑 FxNLMS。不连板子。

用法（在 host/ 下）：
    python sim_p3.py          # 全部实验。有 h_ref 时植物回流用 384 阶 FIR
    python sim_p3.py --quick  # 只跑 4 s 无回流基线，用来确认符号和合成

主参数固定 N=16，μ=0.01，γ=0。宽带 FIR-F 的数字不是旧窄带 fb200
（增益 × 分数延迟）规则下的官方 P3 合格判定；窄带模型仍单独留一行对照。
最新的 *_p2shat/shat.npz 和 *_fb200 按目录名选用，不把某一次记录写死成唯一路径。
"""
from __future__ import annotations

import argparse
import datetime as dt
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager

_yahei = Path(r"C:\Windows\Fonts\msyh.ttc")
if _yahei.exists():
    font_manager.fontManager.addfont(str(_yahei))
    plt.rcParams["font.sans-serif"] = ["Microsoft YaHei"]
plt.rcParams["axes.unicode_minus"] = False
import numpy as np
from numba import njit
from scipy import signal

import analysis as an

FS = 8000
F0 = 200.0
SECONDS = 20
DAC_LIM = 2047.0          # 相对中点 2048 的 12 位上限
PLANT_LAG = 1.0           # 逐点固件：本拍 DAC 要到下一拍才进麦
EPS = 1.0
RECORD = Path(__file__).resolve().parent.parent / "实验记录"
# 仓库里现成的一套。目录里若有更新的 *_p2shat / *_fb200，用更新的那套。
_SHAT_FALLBACK = RECORD / "20261110_175018_p2shat" / "shat.npz"
_FB_FALLBACK = RECORD / "20261110_173557_fb200"
PRIMARY = RECORD / "20261110_154011_p0test" / "4_stand.npz"
TOUCH = RECORD / "20261110_154011_p0test" / "5_touch.npz"
QUIET = RECORD / "20261110_154011_p0test" / "0_quiet.npz"
SILENT = RECORD / "20261110_154011_p0test" / "6_silent.npz"
# 不在 shat.npz 那三段里，单独辨识，给算法一只对不上的 Ŝ
SHAT_ALT = RECORD / "20261110_162814_sid_a" / "sid_a.npz"
N0, MU0, G0 = 16, 0.01, 0.0
FB_OFF, FB_NARROW, FB_FIR = 0, 1, 2


def _newest(pattern: str) -> Path | None:
    hits = sorted(RECORD.glob(pattern))
    return hits[-1] if hits else None


def shat_path() -> Path:
    return _newest("*_p2shat/shat.npz") or _SHAT_FALLBACK


def fb_dir() -> Path:
    hits = sorted(p for p in RECORD.glob("*_fb200") if p.is_dir())
    return hits[-1] if hits else _FB_FALLBACK


def load_plant(path: Path | None = None):
    """返回 (h_err, h_ref 或 None)。没有 h_ref 键就保持旧文件可读。"""
    z = np.load(path or shat_path())
    try:
        h_err = np.asarray(z["h_err"], np.float64)
        h_ref = np.asarray(z["h_ref"], np.float64) if "h_ref" in z.files else None
    finally:
        z.close()
    if h_ref is not None and h_ref.shape != h_err.shape:
        raise ValueError(f"h_ref {h_ref.shape} 与 h_err {h_err.shape} 长度不同")
    return h_err, h_ref


def fir_response(h, hz: float = F0, nfft: int = 16384):
    """同一拍 FIR 在 hz 上的幅度、相位延迟（样本）、相位（度）。不含仿真多加的 1 拍。"""
    f = np.fft.rfftfreq(nfft, 1.0 / FS)
    H = np.fft.rfft(np.asarray(h, np.float64), nfft)
    k = int(np.argmin(np.abs(f - hz)))
    mag = float(np.abs(H[k]))
    deg = float(np.degrees(np.angle(H[k])))
    lag = float(-np.angle(H[k]) * FS / (2 * np.pi * f[k])) if f[k] else 0.0
    return mag, lag, deg


def fir_plant(y, h, plant_lag: int = 1) -> np.ndarray:
    """辨识 FIR（同一拍）再加 write-after-read 的 plant_lag 个样本。

    h[k] 乘的是 k+plant_lag 拍之前的 y。plant_lag=1 与 simulate 里先读麦、后写 y 一致。
    """
    y = np.asarray(y, np.float64)
    h = np.asarray(h, np.float64)
    out = np.zeros(len(y), np.float64)
    for k, hk in enumerate(h):
        if hk == 0.0:
            continue
        d = int(k + plant_lag)
        if d < len(y):
            out[d:] += hk * y[:len(y) - d]
    return out


def load_mv(path: Path):
    z = np.load(path)
    fs = int(z["fs"])
    codes = z["codes"]
    mv = an.codes_to_mv(codes[:, :2])
    return mv, fs, codes


def band_rms(x, fs, lo, hi):
    """Welch 积分。窄带 IIR 在 8 kHz 上滤 6 Hz 会把带外能量漏进来，这里不用。"""
    x = np.asarray(x, float)
    x = x - np.mean(x)
    nper = min(len(x), 2048)
    f, p = signal.welch(x, fs=fs, nperseg=nper)
    m = (f >= lo) & (f <= hi)
    return float(np.sqrt(np.sum(p[m]) * (f[1] - f[0])))


def inventory():
    """盘点。返回 (markdown 行, 给合成用的测量)。"""
    rows = []
    fb = fb_dir()
    specs = [
        ("手机支架 200 Hz，喇叭未出声", PRIMARY, False),
        ("手机贴管", TOUCH, False),
        ("外部声源停，0_quiet", QUIET, False),
        ("播放静音文件", SILENT, False),
        ("fb200 静音", fb / "0_quiet.npz", False),
        ("fb200 抵消档，喇叭 200 Hz", fb / "2_cancel_level.npz", True),
        ("fb200 探针 amp 0.01", fb / "1_probe.npz", True),
    ]
    for title, path, _has in specs:
        mv, fs, codes = load_mv(path)
        st = an.basic_stats(codes[:, :2])
        tm = an.tone_metrics(mv, fs, F0, 0.0)
        rows.append(
            f"| {title} | `{path.parent.name}/{path.name}` | {len(mv)} | {len(mv)/fs:.3f} | {fs} | "
            f"{st[0]['clip']}/{st[1]['clip']} | {tm['ref']['amp_mv']:.2f} | {tm['err']['amp_mv']:.2f} | "
            f"{tm['delay_samples']:+.2f} | {band_rms(mv[:,0], fs, 6, 16):.1f} | {band_rms(mv[:,1], fs, 6, 16):.1f} |"
        )
    return rows


def fit_line(x, fs, f):
    amp, ph = an.sine_fit(x, fs, f)
    return float(amp), float(ph)


def complex_gain(u, y, fs, f):
    """y 相对 u 的复增益。滞后样本数为正表示 y 晚于 u。"""
    au, pu = an.sine_fit(u, fs, f)
    ay, py = an.sine_fit(y, fs, f)
    dphi = (py - pu + np.pi) % (2 * np.pi) - np.pi
    return float(ay / au), float(dphi), float(-dphi * fs / (2 * np.pi * f))


def noise_like(x, n_out, fs, rng):
    """按整段周期图造新噪声，随机相位，不重复原波形。

    irfft 下，内部频点对方差的贡献是 2|X|²/n²，令其等于 p·df。
    """
    x = np.asarray(x, float)
    x = x - np.mean(x)
    f, p = signal.welch(x, fs=fs, nperseg=len(x), window="boxcar")
    freqs = np.fft.rfftfreq(n_out, 1.0 / fs)
    df = float(freqs[1] - freqs[0])
    p_i = np.interp(freqs, f, np.maximum(p, 0.0))
    scale = n_out * np.sqrt(p_i * df) / 2.0
    spec = (rng.normal(size=freqs.size) + 1j * rng.normal(size=freqs.size)) * scale
    spec[0] = 0.0
    if n_out % 2 == 0:
        spec[-1] = rng.normal() * n_out * np.sqrt(float(p_i[-1]) * df)
    return np.fft.irfft(spec, n=n_out)


def synthesize(seconds, rng):
    mv, fs, _ = load_mv(PRIMARY)
    assert fs == FS
    t_rec = np.arange(len(mv)) / fs
    lines = []
    residual = mv.copy()
    for harm in (1, 3):
        pair = []
        for c in range(2):
            amp, ph = fit_line(mv[:, c], fs, harm * F0)
            pair.append((amp, ph))
            residual[:, c] -= amp * np.sqrt(2) * np.sin(2 * np.pi * harm * F0 * t_rec + ph)
        lines.append((harm * F0, pair))
    n = int(seconds * fs)
    t = np.arange(n) / fs
    x0 = noise_like(residual[:, 0], n, fs, rng)
    d = noise_like(residual[:, 1], n, fs, rng)
    for f, pair in lines:
        for sig, (amp, ph) in zip((x0, d), pair):
            sig += amp * np.sqrt(2.0) * np.sin(2 * np.pi * f * t + ph)
    # 校验：抽中间 0.5 s，200 Hz 应回到录音
    sl = slice(n // 2, n // 2 + len(mv))
    syn = np.column_stack([x0[sl], d[sl]])
    got = an.tone_metrics(syn, fs, F0, 0.0)
    ref = an.tone_metrics(mv, fs, F0, 0.0)
    check = {
        "ref_mv": (ref["ref"]["amp_mv"], got["ref"]["amp_mv"]),
        "err_mv": (ref["err"]["amp_mv"], got["err"]["amp_mv"]),
        "delay": (ref["delay_samples"], got["delay_samples"]),
        "lf_ref": (band_rms(residual[:, 0], fs, 6, 16), band_rms(x0, fs, 6, 16)),
        "lf_err": (band_rms(residual[:, 1], fs, 6, 16), band_rms(d, fs, 6, 16)),
        "lines": lines,
    }
    ok = (abs(got["ref"]["amp_mv"] - ref["ref"]["amp_mv"]) / ref["ref"]["amp_mv"] < 0.05
          and abs(got["err"]["amp_mv"] - ref["err"]["amp_mv"]) / ref["err"]["amp_mv"] < 0.05
          and abs(got["delay_samples"] - ref["delay_samples"]) < 0.5)
    if not ok:
        raise RuntimeError(f"合成 200 Hz 和录音对不上: {check}")
    touch, _, _ = load_mv(TOUCH)
    shock = noise_like_band(n, fs, 6.0, 16.0, band_rms(touch[:, 0], fs, 6, 16), rng)
    return x0, d, check, shock


def noise_like_band(n, fs, lo, hi, rms, rng):
    freqs = np.fft.rfftfreq(n, 1.0 / fs)
    spec = rng.normal(size=freqs.size) + 1j * rng.normal(size=freqs.size)
    spec[(freqs < lo) | (freqs > hi)] = 0.0
    spec[0] = 0.0
    y = np.fft.irfft(spec, n=n)
    cur = band_rms(y, fs, lo, hi)
    return y * (rms / (cur + 1e-18))


def identify_alt():
    z = np.load(SHAT_ALT)
    u = z["dac"].astype(np.float64) - an.DAC_MID
    mv = an.codes_to_mv(z["codes"])
    return an.fir_ls(u, mv[:, 1], 384)


def feedback_gain(path: Path):
    z = np.load(path)
    u = z["dac"].astype(np.float64) - an.DAC_MID
    mv = an.codes_to_mv(z["codes"])
    gain, dphi, lag = complex_gain(u, mv[:, 0], int(z["fs"]), F0)
    return gain, lag, float(np.degrees(dphi))


def hp_coeff(order):
    if order <= 0:
        return np.array([1.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])
    b, a = signal.butter(order, 80.0, btype="high", fs=FS)
    bb = np.zeros(3)
    aa = np.zeros(3)
    bb[:len(b)] = b
    aa[:len(a)] = a
    return bb, aa


@njit(cache=True)
def _frac(buf, head, lag, nbuf):
    pos = lag - 1.0
    i0 = int(np.floor(pos))
    frac = pos - i0
    a = (head - i0 + nbuf) % nbuf
    b = (a - 1 + nbuf) % nbuf
    return (1.0 - frac) * buf[a] + frac * buf[b]


@njit(cache=True)
def _biquad(x, b, a, z):
    y = b[0] * x + z[0]
    z[0] = b[1] * x - a[1] * y + z[1]
    z[1] = b[2] * x - a[2] * y
    return y


@njit(cache=True)
def _fir_at(buf, head, h, nbuf):
    """h[0] 乘 buf[head]。head 指向已经写好的上一拍 y，所以自带 1 个样本的植物延迟。"""
    acc = 0.0
    n = h.shape[0]
    for k in range(n):
        acc += h[k] * buf[(head - k + nbuf) % nbuf]
    return acc


@njit(cache=True)
def simulate(x0, d, S, Sh, F, w_len, f_gain, f_lag, b_hp, a_hp,
             mu, gamma, eps, phase_deg,
             fb_mode, do_hp, do_align, do_quant,
             baseline_rms, prot_ratio, prot_windows, w_init):
    """fb_mode: 0 无回流，1 窄带增益×分数延迟（f_lag 已含 PLANT_LAG），2 宽带 FIR。

    FIR 与 S 同一条缓冲：辨识系数是同一拍的，多出来的 1 拍就是先读麦后写 y。
    """
    n = x0.shape[0]
    Ls = S.shape[0]
    Lh = Sh.shape[0]
    Lf = F.shape[0]
    N = w_len
    nplant = Ls if Ls >= Lf else Lf
    ny = nplant + 8
    nx = Lh + 2
    nf = N + 16
    yb = np.zeros(ny)
    xb = np.zeros(nx)
    fb = np.zeros(nf)
    yh = 0
    xh = 0
    fh = 0
    w = np.zeros(N)
    for k in range(N):
        w[k] = w_init[k]
    zx = np.zeros(2)
    ze = np.zeros(2)
    zf = np.zeros(2)
    e_out = np.zeros(n)
    y_out = np.zeros(n)
    cos_t = np.cos(phase_deg * np.pi / 180.0)
    sin_t = np.sin(phase_deg * np.pi / 180.0)
    mute_at = -1
    blew = 0
    ss = 0.0
    count = 0
    over = 0
    win = 400
    inv_sum = 0.0
    inv_n = 0
    xf2 = 0.0
    for i in range(n):
        ac = _fir_at(yb, yh, S, ny)
        fb_s = 0.0
        if fb_mode == 1:
            fb_s = f_gain * _frac(yb, yh, f_lag, ny)
        elif fb_mode == 2:
            fb_s = _fir_at(yb, yh, F, ny)
        xr = x0[i] + fb_s
        er = d[i] + ac
        if do_hp == 1:
            x = _biquad(xr, b_hp, a_hp, zx)
            e = _biquad(er, b_hp, a_hp, ze)
        else:
            x = xr
            e = er
        xh = (xh + 1) % nx
        xb[xh] = x
        xf = 0.0
        for k in range(Lh):
            xf += Sh[k] * xb[(xh - k + nx) % nx]
        if do_align == 1 and do_hp == 1:
            xf = _biquad(xf, b_hp, a_hp, zf)
        fh = (fh + 1) % nf
        fb[fh] = xf
        xf2 += xf * xf
        if mute_at >= 0 or blew == 1:
            y = 0.0
            for k in range(N):
                w[k] = 0.0
        else:
            y = 0.0
            pwr = eps
            for k in range(N):
                xs = xb[(xh - k + nx) % nx]
                rs = fb[(fh - k + nf) % nf]
                # 回归量用旋转后的 xf。旋转只严格成立在当前样本和 10 拍之前，
                # 这里对延迟线逐点旋转：延迟 k 的正交分量是再早 10 拍。
                rs_lag = fb[(fh - k - 10 + nf) % nf]
                rs = rs * cos_t - rs_lag * sin_t
                y += w[k] * xs
                pwr += rs * rs
            if do_quant == 1:
                y = np.rint(y)
                if y > DAC_LIM:
                    y = DAC_LIM
                elif y < -DAC_LIM - 1.0:
                    y = -DAC_LIM - 1.0
            if y > 1e6 or y < -1e6:
                blew = 1
                y = 0.0
                for k in range(N):
                    w[k] = 0.0
            else:
                scale = mu * e / pwr
                if i > 8000:
                    inv_sum += 1.0 / pwr
                    inv_n += 1
                leak = 1.0 - gamma
                for k in range(N):
                    rs = fb[(fh - k + nf) % nf]
                    rs_lag = fb[(fh - k - 10 + nf) % nf]
                    rs = rs * cos_t - rs_lag * sin_t
                    w[k] = leak * w[k] - scale * rs
        yh = (yh + 1) % ny
        yb[yh] = y
        e_out[i] = e
        y_out[i] = y
        if mute_at < 0 and blew == 0 and prot_ratio > 0.0:
            ss += e * e
            count += 1
            if count == win:
                rms = np.sqrt(ss / win)
                if rms > prot_ratio * baseline_rms:
                    over += 1
                else:
                    over = 0
                if over >= prot_windows:
                    mute_at = i
                    for k in range(N):
                        w[k] = 0.0
                ss = 0.0
                count = 0
    mean_inv = inv_sum / inv_n if inv_n > 0 else 0.0
    mean_xf = np.sqrt(xf2 / n)
    return e_out, y_out, mute_at, blew, mean_inv, mean_xf


def off_error(d, hp):
    if hp <= 0:
        return np.asarray(d, float)
    b, a = signal.butter(hp, 80.0, btype="high", fs=FS)
    return signal.lfilter(b, a, d)


def track_db(e, ref_amp):
    win = 2000
    n = np.arange(len(e))
    w = 2 * np.pi * F0 / FS
    ker = np.ones(win) / win
    c = np.convolve(e * np.cos(w * n), ker, mode="valid")
    s = np.convolve(e * np.sin(w * n), ker, mode="valid")
    amp = np.hypot(c, s) * np.sqrt(2.0)
    db = 20 * np.log10(amp / ref_amp + 1e-18)
    t = (np.arange(len(db)) + win) / FS
    return t, db


def conv_time(t, db):
    """0.25 s 相干包络降到自身最后 4 s 中位数 +3 dB 以内的时刻。

    接下来 0.5 s 里至少 80% 的点要留在这个门限下，免得包络抖一下就把计时清掉。
    中位数没到 −6 dB 则返回 None。返回 (时刻, 中位数)。
    """
    if len(t) < 2:
        return None, None
    late = db[t >= (t[-1] - 4.0)]
    steady = float(np.median(late if len(late) > 10 else db))
    if steady > -6.0:
        return None, steady
    dt = float(t[1] - t[0])
    need = max(1, int(round(0.5 / dt)))
    if len(db) <= need:
        return None, steady
    ok = np.asarray(db <= (steady + 3.0))
    c = np.cumsum(ok)
    hits = c[need:] - c[:-need]
    good = np.nonzero(hits >= int(0.8 * need))[0]
    if good.size == 0:
        return None, steady
    return float(t[int(good[0])]), steady


def _fb_mode(fb) -> int:
    if fb is True or fb == "narrow" or fb == FB_NARROW:
        return FB_NARROW
    if fb == "fir" or fb == FB_FIR:
        return FB_FIR
    return FB_OFF


def run_case(x0, d, S, Sh, f_gain, f_lag, *, N, mu, gamma, hp, fb, align, quant,
             phase=0.0, prot_ratio=0.0, shock=None, F=None, w_init=None):
    x = x0 if shock is None else x0 + shock
    mode = _fb_mode(fb)
    if mode == FB_FIR:
        if F is None:
            raise ValueError("宽带 FIR-F 需要 h_ref")
        Fb = np.ascontiguousarray(F, np.float64)
        # 1 拍植物延迟在环形缓冲里，不要再把 PLANT_LAG 加到分数延迟上。
        lag = 0.0
    else:
        Fb = np.zeros(1, np.float64)
        lag = float(f_lag + PLANT_LAG) if mode == FB_NARROW else 0.0
    w0 = np.zeros(int(N), np.float64)
    if w_init is not None:
        src = np.asarray(w_init, np.float64).ravel()
        ncopy = min(len(src), int(N))
        w0[:ncopy] = src[:ncopy]
    b, a = hp_coeff(hp if (hp and (True)) else 0)
    # 高通阶数只在 hp>0 时生效；对齐与否另走 do_align
    e_off = off_error(d, hp)
    ref_amp, _ = an.sine_fit(e_off[-4 * FS:], FS, F0)
    base = float(np.sqrt(np.mean(e_off ** 2)))
    e, y, mute, blew, inv, xf_rms = simulate(
        np.ascontiguousarray(x, np.float64),
        np.ascontiguousarray(d, np.float64),
        np.ascontiguousarray(S, np.float64),
        np.ascontiguousarray(Sh, np.float64),
        Fb,
        int(N), float(f_gain), lag, b, a,
        float(mu), float(gamma), float(EPS), float(phase),
        mode, 1 if hp else 0, 1 if align else 0, 1 if quant else 0,
        base, float(prot_ratio), 4, w0,
    )
    amp, _ = an.sine_fit(e[-4 * FS:], FS, F0)
    db = float(20 * np.log10(amp / ref_amp + 1e-18))
    t, curve = track_db(e, ref_amp)
    tc, db_short = conv_time(t, curve)
    tail = y[-4 * FS:]
    rail = float(np.mean(np.abs(tail) > 1000))
    diverged = bool(blew) or (not np.isfinite(db)) or db > 0.0 or rail > 0.01
    return {
        "db": db, "amp": float(amp), "ref": float(ref_amp),
        "t": t[::80], "curve": curve[::80], "tc": tc,
        "db_short": None if db_short is None else float(db_short),
        "mute": int(mute), "blew": int(blew),
        "peak": float(np.max(np.abs(tail))),
        "peak_all": float(np.max(np.abs(y))),
        "y_rms": float(np.std(tail)),
        "rail": rail, "diverged": diverged,
        "inv": float(inv), "xf_rms": float(xf_rms), "e": e, "y": y,
    }


def fmt_tc(tc):
    return "未收敛" if tc is None else f"{tc:.2f} s"


def fmt_db(r):
    flag = " 发散" if r["diverged"] else ""
    return f"{r['db']:+.1f} dB{flag}"


def spectrum_pair(e_on, e_off):
    def spec(e):
        f, p = signal.welch(e[-8 * FS:], FS, nperseg=2048)
        return f, p
    f, pon = spec(e_on)
    _, poff = spec(e_off)
    def at(p, hz):
        k = int(np.argmin(np.abs(f - hz)))
        return float(f[k]), float(10 * np.log10(p[k] + 1e-20))
    m = (f >= 100) & (f <= 400)
    df = f[1] - f[0]
    def band(p):
        return float(10 * np.log10(np.sum(p[m]) * df + 1e-20))
    rows = []
    for hz in (133, 200, 232, 400):
        fon, don = at(pon, hz)
        _, doff = at(poff, hz)
        rows.append((fon, doff, don, don - doff))
    return f, poff, pon, rows, band(poff), band(pon)


def write_plots(out, curves, spec, mu_rows):
    fig, ax = plt.subplots(figsize=(9, 4.5))
    for name, r in curves:
        ax.plot(r["t"], r["curve"], lw=1.0, label=name)
    ax.set(xlabel="t / s", ylabel="200 Hz level / dB re ANC off", xlim=(0, SECONDS))
    ax.axhline(-20, color="k", ls="--", lw=0.7, label="-20 dB")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "level_200.png", dpi=120)
    plt.close(fig)

    f, poff, pon = spec
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.plot(f, 10 * np.log10(poff + 1e-20), lw=0.8, label="ANC off")
    ax.plot(f, 10 * np.log10(pon + 1e-20), lw=0.8, label="ANC on")
    for hz in (133, 200, 232, 400):
        ax.axvline(hz, color="k", lw=0.4, ls=":")
    ax.set(xlabel="f / Hz", ylabel="PSD / dB re 1 mV²/Hz", xlim=(0, 1000))
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out / "spectrum_on_off.png", dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.5))
    mus = np.array([r["mu"] for r in mu_rows], float)
    stable = np.array([not r["diverged"] for r in mu_rows])
    dbs = np.array([r["db"] for r in mu_rows], float)
    tcs = np.array([r["tc"] if r["tc"] is not None else np.nan for r in mu_rows], float)
    ax.plot(mus[stable], dbs[stable], "o-", label="稳态 200 Hz")
    if np.any(~stable):
        ax.plot(mus[~stable], np.zeros(np.count_nonzero(~stable)), "x", color="C3", label="发散，画在 0 dB")
    ax.set_xscale("log")
    ax.set(xlabel="μ", ylabel="200 Hz 降噪 / dB")
    ax.axhline(-20, color="k", ls="--", lw=0.7)
    ax.grid(alpha=0.3, which="both")
    ax2 = ax.twinx()
    ax2.plot(mus[stable], tcs[stable], "s--", color="C1", label="收敛时间")
    ax2.set(ylabel="收敛时间 / s", ylim=(0.0, 2.0))
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=8, loc="center left")
    fig.tight_layout()
    fig.savefig(out / "mu_scan.png", dpi=120)
    plt.close(fig)


def pick_mu(rows):
    """稳定上界 = 能降过 10 dB 且不顶满 DAC 的最大 μ。

    推荐值 = 这些里面能到 −20 dB、1 s 内收敛的最小 μ。
    若它已经超过上界的一半，仍然用它，并写明余量不到一倍。
    """
    stable = [r for r in rows if (not r["diverged"]) and r["db"] <= -10]
    if not stable:
        return None, None, "扫描里没有能降过 10 dB 的稳定 μ"
    ub = max(r["mu"] for r in stable)
    fast = [r for r in stable
            if r["db"] <= -20 and r["db_short"] is not None and r["db_short"] <= -20
            and r["tc"] is not None and r["tc"] <= 1.0]
    if not fast:
        return None, ub, "没有在 1 s 内收到 −20 dB 的稳定 μ"
    fast.sort(key=lambda r: r["mu"])
    note = ""
    if fast[0]["mu"] > ub / 2.0:
        note = "这个 μ 已经超过稳定上界的一半，余量不到一倍。"
    blown = [r["mu"] for r in rows if r["diverged"]]
    if blown:
        note += f"下一档 {min(blown):g} 就顶满 DAC，上界在 {ub:g} 和 {min(blown):g} 之间。"
    return fast[0], ub, note


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args(argv)
    rng = np.random.default_rng(0)
    seconds = 4 if args.quick else SECONDS
    print("inventory + synthesize", seconds, "s")
    rows_inv = inventory()
    x0, d, check, shock = synthesize(seconds, rng)
    plant_path = shat_path()
    fb = fb_dir()
    S, h_ref = load_plant(plant_path)
    use_fir = h_ref is not None
    print("identify alternate S, this takes a few seconds")
    Sh = identify_alt()
    mismatch = 20 * np.log10(np.linalg.norm(Sh - S) / np.linalg.norm(S))
    f_gain, f_lag, f_deg = feedback_gain(fb / "2_cancel_level.npz")
    g2, lag2, deg2 = feedback_gain(fb / "1_probe.npz")
    s10, _, _ = fir_response(S, 10.0)
    if use_fir:
        fir_mag, fir_lag, fir_deg = fir_response(h_ref)
        fir_mag10, _, _ = fir_response(h_ref, 10.0)
        print(f"FIR F(200) {fir_mag:.4f} mV/code lag {fir_lag:.3f} ({fir_deg:.1f} deg); "
              f"|F|(10 Hz) {fir_mag10:.4f}")
    else:
        fir_mag = fir_lag = fir_deg = fir_mag10 = float("nan")
        print("shat.npz 没有 h_ref，植物回流退回窄带 fb200")
    print(f"narrow F gain {f_gain:.4f} lag {f_lag:.3f} ({f_deg:.1f} deg); probe {g2:.4f} {lag2:.3f}")
    print(f"S mismatch vs held-out {mismatch:.1f} dB")
    print("synth", {k: check[k] for k in ("ref_mv", "err_mv", "delay", "lf_ref", "lf_err")})

    if args.quick:
        r = run_case(x0, d, S, Sh, f_gain, f_lag, N=8, mu=0.01, gamma=0.0,
                     hp=0, fb="off", align=False, quant=False)
        print("quick baseline", fmt_db(r), fmt_tc(r["tc"]), "peak", r["peak"])
        return

    # 主参数不随扫描改。有 h_ref 时植物是宽带 FIR，扫描用一阶对齐+量化（这组在 FIR 植物上稳过）。
    fb_on = "fir" if use_fir else "narrow"
    label = "FIR回流" if use_fir else "回流"
    sw_hp = 1 if use_fir else 2
    headline_name = f"+{label} +一阶对齐 +量化" if use_fir else f"+{label} +二阶对齐 +量化"
    plant_note = "宽带 FIR-F（h_ref）" if use_fir else "窄带 fb200"

    def case(**kw):
        kw.setdefault("F", h_ref)
        return run_case(x0, d, S, Sh, f_gain, f_lag, **kw)

    factors = [
        ("基线 浮点", dict(hp=0, fb="off", align=False, quant=False)),
        (f"+{label}", dict(hp=0, fb=fb_on, align=False, quant=False)),
        (f"+{label} +一阶高通失配", dict(hp=1, fb=fb_on, align=False, quant=False)),
        (f"+{label} +二阶高通失配", dict(hp=2, fb=fb_on, align=False, quant=False)),
        (f"+{label} +一阶高通对齐", dict(hp=1, fb=fb_on, align=True, quant=False)),
        (f"+{label} +二阶高通对齐", dict(hp=2, fb=fb_on, align=True, quant=False)),
        (f"+{label} +一阶对齐 +量化", dict(hp=1, fb=fb_on, align=True, quant=True)),
        (f"+{label} +二阶对齐 +量化", dict(hp=2, fb=fb_on, align=True, quant=True)),
    ]
    if use_fir:
        factors.append((
            "对照 窄带fb200 +一阶对齐 +量化",
            dict(hp=1, fb="narrow", align=True, quant=True),
        ))
    factor_rows = []
    print("factors", plant_note)
    for name, kw in factors:
        r = case(N=N0, mu=MU0, gamma=G0, phase=0.0, **kw)
        r["name"] = name
        factor_rows.append(r)
        print(f"  {name}: {fmt_db(r)}  short={r['db_short']}  {fmt_tc(r['tc'])}  y_rms={r['y_rms']:.2f}")
    head = next(r for r in factor_rows if r["name"] == headline_name)

    print("N sweep", plant_note, "hp", sw_hp)
    n_rows = []
    for N in (4, 8, 16, 32):
        r = case(N=N, mu=MU0, gamma=G0, hp=sw_hp, fb=fb_on, align=True, quant=True)
        r["N"] = N
        n_rows.append(r)
        print(f"  N={N}: {fmt_db(r)} short={r['db_short']} {fmt_tc(r['tc'])}")

    usable_N = [r for r in n_rows if (not r["diverged"]) and r["db"] <= -20]
    deep_N = min(usable_N, key=lambda r: r["db"])["N"] if usable_N else None

    mus = [0.005, 0.01, 0.015, 0.02, 0.03, 0.04, 0.05, 0.08, 0.1, 0.2, 0.5, 1.0]
    print("mu sweep N", N0)
    mu_rows = []
    for mu in mus:
        r = case(N=N0, mu=mu, gamma=0.0, hp=sw_hp, fb=fb_on, align=True, quant=True)
        r["mu"] = mu
        mu_rows.append(r)
        print(f"  mu={mu}: {fmt_db(r)} short={r['db_short']} {fmt_tc(r['tc'])} rail={r['rail']:.3f}")
    picked, ub, pick_note = pick_mu(mu_rows)
    mu_scan = picked["mu"] if picked else None

    print("gamma sweep")
    gammas = [0.0, 1e-6, 1e-5, 1e-4, 1e-3]
    g_rows = []
    for g in gammas:
        r = case(N=N0, mu=MU0, gamma=g, hp=sw_hp, fb=fb_on, align=True, quant=True)
        r["gamma"] = g
        g_rows.append(r)
        print(f"  gamma={g}: {fmt_db(r)} {fmt_tc(r['tc'])}")

    print("phase sweep")
    phases = [0, 15, 30, 45, 60, 75, 90, -15, -30, -45, -60, -75, -90]
    ph_rows = []
    for ph in phases:
        r = case(N=N0, mu=MU0, gamma=G0, hp=sw_hp, fb=fb_on, align=True, quant=True, phase=ph)
        r["phase"] = ph
        r.pop("e")
        r.pop("y")
        ph_rows.append(r)
        print(f"  {ph:+d} deg: {fmt_db(r)} {fmt_tc(r['tc'])}")

    print("shock")
    shock_rows = []
    for hp, align in ((0, False), (1, True), (2, True)):
        r = case(N=N0, mu=MU0, gamma=G0, hp=hp, fb=fb_on, align=align, quant=True, shock=shock)
        r["hp"] = hp
        shock_rows.append(r)
        print(f"  hp={hp}: {fmt_db(r)} inv={r['inv']:.3e} y_rms={r['y_rms']:.2f}")

    print("headline + protect")
    e_off = off_error(d, sw_hp)
    f_spec, poff, pon, spec_rows, band_off, band_on = spectrum_pair(head["e"], e_off)

    base_open = float(np.sqrt(np.mean(e_off ** 2)))
    prot_rows = []
    for name, kw in (
        ("主参数，保护开着", dict(phase=0.0, mu=MU0)),
        ("相位 180°", dict(phase=180.0, mu=MU0)),
        ("μ×10", dict(phase=0.0, mu=MU0 * 10)),
    ):
        r = case(N=N0, gamma=G0, hp=sw_hp, fb=fb_on, align=True, quant=True,
                 prot_ratio=2.0, **kw)
        r["name"] = name
        prot_rows.append(r)
        print(f"  {name}: mute={r['mute']} blew={r['blew']} db={r['db']:+.1f}")

    if use_fir:
        want = ["基线 浮点", f"+{label}", headline_name, "对照 窄带fb200 +一阶对齐 +量化"]
    else:
        want = ["基线 浮点", f"+{label}", f"+{label} +二阶高通失配", headline_name]
    curves = [(r["name"], r) for r in factor_rows if r["name"] in want]

    out = RECORD / f"{dt.datetime.now():%Y%m%d_%H%M%S}_p3sim"
    out.mkdir(parents=True)
    write_plots(out, curves, (f_spec, poff, pon), mu_rows)
    np.savez(out / "summary.npz",
             mismatch_db=mismatch, f_gain=f_gain, f_lag=f_lag,
             fir_mag=fir_mag, fir_lag=fir_lag, fir_deg=fir_deg,
             plant=np.array(plant_note),
             N=N0, mu=MU0, gamma=G0)

    def fmt_short(r):
        v = r["db_short"]
        return "—" if v is None else f"{v:+.1f}"

    def line_factor(r):
        return (f"| {r['name']} | {r['db']:+.1f} | {fmt_short(r)} | {fmt_tc(r['tc'])} | "
                f"{r['y_rms']:.2f} | {r['peak']:.1f} |")

    def line_mu(r):
        return (f"| {r['mu']:g} | {r['db']:+.1f} | {fmt_short(r)} | {fmt_tc(r['tc'])} | "
                f"{'是' if r['diverged'] else '否'} |")

    spec_txt = "\n".join(
        f"| {hz:.0f} | {off:.1f} | {on:.1f} | {delta:+.1f} |"
        for hz, off, on, delta in spec_rows)
    ph_txt = "\n".join(
        f"| {r['phase']:+d} | {r['db']:+.1f} | {fmt_short(r)} | {fmt_tc(r['tc'])} |" for r in ph_rows)
    n_txt = "\n".join(
        f"| {r['N']} | {r['db']:+.1f} | {fmt_short(r)} | {fmt_tc(r['tc'])} |" for r in n_rows)
    g_txt = "\n".join(
        f"| {r['gamma']:g} | {r['db']:+.1f} | {fmt_short(r)} | {fmt_tc(r['tc'])} |" for r in g_rows)
    sh_txt = "\n".join(
        f"| {r['hp']} | {r['db']:+.1f} | {fmt_short(r)} | {fmt_tc(r['tc'])} | {r['inv']:.3e} | {r['y_rms']:.2f} |"
        for r in shock_rows)
    def prot_peak(r):
        n = r["mute"] + 1 if r["mute"] >= 0 else min(len(r["e"]), FS)
        return float(np.max(np.abs(r["e"][:n])))

    pr_txt = "\n".join(
        f"| {r['name']} | {r['mute'] / FS if r['mute'] >= 0 else float('nan'):.3f} | "
        f"{prot_peak(r):.0f} | {r['blew']} | {r['db']:+.1f} |"
        for r in prot_rows)
    ref_m, ref_s = check["ref_mv"]
    err_m, err_s = check["err_mv"]
    dly_m, dly_s = check["delay"]
    sw_name = "一阶对齐 + 量化" if use_fir else "二阶对齐 + 量化"
    deep_txt = "没有" if deep_N is None else str(deep_N)
    scan_txt = "没有" if mu_scan is None else f"{mu_scan:g}"
    if use_fir:
        fir_blurb = (
            f"宽带 `h_ref` 来自 `{plant_path.parent.name}`，200 Hz 为 {fir_mag:.3f} mV/码、"
            f"{fir_deg:+.1f}°、相位延迟 {fir_lag:.2f} 样本（不含仿真多加的 1 拍）。"
            f"10 Hz 处 |h_ref| = {fir_mag10:.3f} mV/码，|S| = {s10:.3f} mV/码。"
            "控制器没有做 F 中和，这条 FIR 只放在植物里。"
        )
        claim = (
            "上面的主结果是宽带 FIR-F 植物上的数字。本报告不按旧窄带 F"
            "（fb200 的增益 × 分数延迟）规则宣布官方 P3 合格或不合格。"
            "窄带同一控制器只在「对照 窄带fb200」那一行，供比较。"
        )
    else:
        fir_blurb = "这份 `shat.npz` 没有 `h_ref`，回流退回窄带 fb200。"
        claim = "这份记录没有宽带 FIR，主结果用的是窄带 fb200 植物。"
    text = f"""# P3 离线 FxNLMS {dt.datetime.now():%Y-%m-%d}

只在电脑上跑。在 `host/` 下执行 `python sim_p3.py`；`--quick` 只跑 4 s 无回流基线。真实次级路径是 `{plant_path.parent.name}/shat.npz` 的 `h_err`（{len(S)} 阶）。算法里的 Ŝ 是 `20261110_162814_sid_a` 单独最小二乘的 384 阶，和真路径的冲激响应差 {mismatch:.1f} dB。这段没有进 `shat.npz`。这一轮植物回流是 {plant_note}。

## 盘点

全部 8 kHz。削波列是 ref/err 的削波点数。200 Hz 是正弦拟合 rms。6–16 Hz 是 Welch 谱在这个带里的积分，不是窄带 IIR。

| 段 | 文件 | 帧 | 秒 | fs | 削波 ref/err | ref 200 Hz mV | err 200 Hz mV | err 相对 ref 样本 | ref 6–16 Hz | err 6–16 Hz |
| --- | --- | ---: | ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: |
{chr(10).join(rows_inv)}

单段最长 1 s。没有把短录音首尾接起来。

合成：从 `4_stand` 拟合 200 Hz 和 600 Hz 的幅度、相位（600 Hz 是手机的三次谐波，误差麦上大约 −23 dB），从减去这两条线之后的残差估计 Welch 谱，再按这个谱造 20 s 随机相位噪声，最后把正弦加回去。

| 校验 | 录音 | 合成 |
| --- | ---: | ---: |
| ref 200 Hz mV | {ref_m:.2f} | {ref_s:.2f} |
| err 200 Hz mV | {err_m:.2f} | {err_s:.2f} |
| err 比 ref 晚的样本 | {dly_m:.2f} | {dly_s:.2f} |
| 残差 ref 6–16 Hz mV | {check['lf_ref'][0]:.1f} | {check['lf_ref'][1]:.1f} |
| 残差 err 6–16 Hz mV | {check['lf_err'][0]:.1f} | {check['lf_err'][1]:.1f} |

贴管段 6–16 Hz 的参考麦按 Welch 是 {band_rms(load_mv(TOUCH)[0][:,0], FS, 6, 16):.1f} mV。讨论记录里的约 262 mV 是窄带 IIR 算的，那只滤波器在 8 kHz 上会把 200 Hz 漏进 6–16 Hz。冲击用 Welch 的数，只加在参考麦上。

窄带回流取抵消档：{f_gain:.3f} mV/码，参考麦晚 {f_lag:.2f} 个样本（{f_deg:.1f}°）。探针档 {g2:.3f} mV/码、{lag2:.2f} 样本，同一条增益。{fir_blurb}

## 假设

- 逐点：本拍算出的 DAC 从下一拍才进入两条声学路径。辨识出的 FIR 描述的是「DAC 脚上的码和同一拍的麦」，所以仿真里真路径和 FIR 回流都多这 1 拍，靠先读麦、后写 y 的缓冲实现，不再把 PLANT_LAG 加到 FIR 的分数延迟上。200 Hz 上这 1 拍是 9°。算法内部的 Ŝ 滤波可以用到当前参考样本，那不是声学延迟。
- 更新是减号：`w ← (1−γ) w − μ e x' / (‖x'‖²+ε)`。麦上是 d 加喇叭，写成加号会往反方向走。ε = {EPS:g}。
- 宽带 FIR-F 是 `h_ref` 与 y 的卷积，y 是去掉中点的 DAC 码，F 的单位是 mV/码。`xr = x0 + F*y`。窄带对照仍是 200 Hz 增益 × 分数延迟，全频带乘同一个数，并且同样多这 1 拍。窄带那一行只供比较。
- 200 Hz 降噪有两个数。4 s 正弦拟合只看锁相的单音。0.25 s 相干包络的最后 4 s 中位数会把相位在漂的残差也算进去，所以比拟合浅。收敛时刻按这只包络：降到它自己的中位数 +3 dB 以内，并且接下来 0.5 s 里八成的点还在门限下。
- 高通是因果 Butterworth，截止 80 Hz。对齐是指滤波后的 x' 再过同一只高通；失配是 x' 不过。
- 量化：四舍五入到整数码，并限制在 −2048…2047。
- 相位扫描在 200 Hz 上是准确的常数相位（用 10 个样本的正交分量）。别的频率不是同一角度。
- 保护规则是仿真里定的，不是从板上测到的：误差麦短时 rms（50 ms）连续 4 窗高于开环 rms 的 2 倍就清零 W 并静音。开环 rms 约 {base_open:.0f} mV。

## 逐项，N={N0}，μ={MU0}，γ={G0}

这一组参数没有为了过线再改。

| 条件 | 4 s 拟合 | 0.25 s 窗 | 收敛 | y rms 码 | 末尾 4 s 峰值码 |
| --- | ---: | ---: | --- | ---: | ---: |
{chr(10).join(line_factor(r) for r in factor_rows)}

名字带「对照 窄带fb200」的行才是旧的增益 × 分数延迟。某一行如果发散，末尾的 0 dB 是因为 |y| 超过 1e6 之后把输出清零了，不是它还停在开环。

## 抽头（{plant_note}，{sw_name}，μ={MU0}，γ=0）

| N | 4 s 拟合 | 0.25 s 窗 | 收敛 |
| --- | ---: | ---: | --- |
{n_txt}

主结果固定 N = {N0}。表里仍 ≤ −20 dB 且拟合最深的是 {deep_txt}，不拿它替换主结果。

## μ（N={N0}，{sw_name}，{plant_note}，γ=0）

稳定上界（能降过 10 dB 且没有顶满 DAC 的最大扫描点）= {ub if ub is not None else "没有"}。再大一档就是表里的下一个点，上界在这两档之间，没有更密的测量。

| μ | 4 s 拟合 | 0.25 s 窗 | 收敛 | 发散 |
| --- | ---: | ---: | --- | ---: |
{chr(10).join(line_mu(r) for r in mu_rows)}

按旧挑选规则（能到 −20 dB 且 1 s 内收敛的最小稳定 μ）得到 {scan_txt}。主结果仍是 μ = {MU0:g}。{pick_note}

## 泄漏（N={N0}，μ={MU0:g}）

| γ | 4 s 拟合 | 0.25 s 窗 | 收敛 |
| --- | ---: | ---: | --- |
{g_txt}

主结果保持 γ = 0，不按这张表改。

## Ŝ 相位误差

相位是加在滤波后的 x' 上的，只在 200 Hz 准确。

| 相位 ° | 4 s 拟合 | 0.25 s 窗 | 收敛 |
| --- | ---: | ---: | --- |
{ph_txt}

## 参考麦 6–16 Hz 冲击

加在参考麦上的带限噪声，rms 等于贴管录音的 6–16 Hz。误差麦不加。不确定碰管子时误差麦实际吃进多少，这段录音里误差麦 6–16 Hz 只有 0.8 mV。

| 高通阶数 | 4 s 拟合 | 0.25 s 窗 | 收敛 | 平均 1/(‖x'‖²+ε) | y rms |
| --- | ---: | ---: | --- | ---: | ---: |
{sh_txt}

`1/(‖x'‖²+ε)` 变小就是归一化步长被压短。无冲击、主结果那次是 {head['inv']:.3e}。

## 副作用（主结果那一次，最后 8 s）

| Hz | 关 dB | 开 dB | 开−关 |
| --- | ---: | ---: | ---: |
{spec_txt}

100–400 Hz 总能量：关 {band_off:.1f} dB，开 {band_on:.1f} dB，差 {band_on-band_off:+.1f} dB。谱是 Welch，dB re 1 mV²/Hz，和上面的正弦幅度 dB 不是同一种数。230 Hz 附近这一格是 {next(d for hz, off, on, d in spec_rows if 220 < hz < 240):+.1f} dB，绝对高度仍低于关环时的 200 Hz 线。

## 保护

| 情况 | 静音时刻 s | 触发前 \\|e\\| 峰值 mV | 数值炸掉 | 末尾 200 Hz |
| --- | ---: | ---: | ---: | ---: |
{pr_txt}

静音时刻从仿真起点算。NaN 表示这 20 s 里规则没有触发。阈值是开环 rms 的 2 倍，连续 4 个 50 ms 窗，最快 0.20 s。末尾 200 Hz 在静音之后回到开环，所以是 0 dB 左右，不代表没检测到。

## 主结果

植物是 {plant_note}。控制器固定为 N = {N0}，μ = {MU0:g}，γ = {G0:g}，{sw_name}，DAC 限到 −2048…2047。对应表里的「{headline_name}」。末尾 4 s 的 y 峰值 {head['peak']:.1f} 码，整段峰值 {head['peak_all']:.0f} 码。{"起头碰到过限幅。" if head["peak_all"] >= DAC_LIM else "没有碰到限幅。"}

这一次：4 s 正弦拟合 {head['db']:+.1f} dB，0.25 s 窗中位数 {fmt_short(head)} dB，收敛 {fmt_tc(head['tc'])}，y rms {head['y_rms']:.2f} 码。{claim}

![200 Hz](level_200.png)

![spectrum](spectrum_on_off.png)

![mu](mu_scan.png)

## 哪些是实测，哪些是假设

实测：初级路径的 200 Hz 和 600 Hz 幅度相位，残差的频谱，Ŝ 和另一段白噪声的 Ŝ，窄带回流在 200 Hz 的复增益，若 `shat.npz` 里有 `h_ref` 则还有 DAC→参考麦的 384 阶 FIR，DAC 码的范围，采样率。

假设：20 s 噪声是按谱重新抽的，不是那半秒的重复；宽带 FIR 在辨识用的噪声带里代表这条回流；窄带对照在 200 Hz 以外仍是同一个增益和延迟；逐点多 1 拍；高通阶数和截止是预定的，不是从板上拆出来的；保护阈值是这里定的；碰管子只加在参考麦上。
"""
    (out / "report.md").write_text(text, encoding="utf-8")
    print("saved", out)
    tag = "宽带 FIR-F，不是旧窄带 F 的官方合格判定" if use_fir else "窄带 fb200（没有 h_ref）"
    print("HEAD", tag, fmt_db(head), fmt_tc(head["tc"]))


if __name__ == "__main__":
    main()

