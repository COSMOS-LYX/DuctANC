"""麦克风数据分析：电压换算、频谱、频段电平、相干函数。后续 ANC 实测也复用这里。"""
from __future__ import annotations

import numpy as np
from scipy import signal

VREF_MV = 3300.0
ADC_FS = 4095.0
DAC_MID = 2048.0
CH_NAMES = ("ref", "err")


def codes_to_mv(codes: np.ndarray) -> np.ndarray:
    """12 位 ADC 码 -> 去直流后的 mV。"""
    mv = codes.astype(np.float64) * VREF_MV / ADC_FS
    return mv - mv.mean(axis=0)


def basic_stats(codes: np.ndarray) -> list[dict]:
    out = []
    for c in range(codes.shape[1]):
        x = codes[:, c].astype(np.float64)
        ac = x - x.mean()
        out.append({
            "dc_code": x.mean(),
            "dc_mv": x.mean() * VREF_MV / ADC_FS,
            "rms_mv": np.sqrt(np.mean(ac ** 2)) * VREF_MV / ADC_FS,
            "min": int(x.min()),
            "max": int(x.max()),
            "clip": int(np.sum((x <= 8) | (x >= 4087))),
        })
    return out


def psd(x_mv: np.ndarray, fs: float, nperseg: int = 1024):
    """Welch 功率谱密度，单位 mV²/Hz。"""
    nperseg = min(nperseg, len(x_mv))
    return signal.welch(x_mv, fs=fs, nperseg=nperseg, axis=0)


def band_level_db(f: np.ndarray, p: np.ndarray, lo: float, hi: float) -> float:
    """频段内能量，dB re 1 mV²。"""
    m = (f >= lo) & (f <= hi)
    df = f[1] - f[0]
    return 10 * np.log10(np.sum(p[m]) * df + 1e-20)


def tone_level_db(f: np.ndarray, p: np.ndarray, f0: float, half_bw: float = 10.0) -> float:
    return band_level_db(f, p, f0 - half_bw, f0 + half_bw)


def coherence(x_mv: np.ndarray, y_mv: np.ndarray, fs: float, nperseg: int = 1024):
    nperseg = min(nperseg, len(x_mv))
    return signal.coherence(x_mv, y_mv, fs=fs, nperseg=nperseg)


def sine_fit(x, fs, f0) -> tuple[float, float]:
    """最小二乘拟合 x ≈ a·cos(ωt) + b·sin(ωt) + c，返回 (rms 幅度, 相位 rad)，相位按 A·sin(ωt+φ) 定义。"""
    t = np.arange(len(x)) / fs
    w = 2 * np.pi * f0 * t
    mat = np.column_stack([np.cos(w), np.sin(w), np.ones(len(x))])
    (a, b, _c), *_ = np.linalg.lstsq(mat, x, rcond=None)
    return float(np.hypot(a, b) / np.sqrt(2)), float(np.arctan2(a, b))


def band_rms_mv(x, fs, lo=None, hi=None, order=4) -> float:
    """Butterworth（sosfiltfilt）带通/低通/高通后的 rms。lo=None 表示低通 hi，hi=None 表示高通 lo。"""
    if lo is not None and hi is not None:
        sos = signal.butter(order, [lo, hi], "band", fs=fs, output="sos")
    elif hi is not None:
        sos = signal.butter(order, hi, "low", fs=fs, output="sos")
    else:
        sos = signal.butter(order, lo, "high", fs=fs, output="sos")
    y = signal.sosfiltfilt(sos, x)
    return float(np.sqrt(np.mean(y ** 2)))


def tone_metrics(mv, fs, f0, search_hz=20.0) -> dict:
    """单音分析（mv: N×2，ch0=ref, ch1=err）。

    delay_samples 是 err 相对 ref 的相位差折算的样本数（正值 = err 晚到），
    相位法只能确定到模一个周期，周期长度见 period_samples。
    search_hz <= 0 时直接在 f0 上拟合（频率已知，如 DAC 输出或没有单音的对照段）。
    """
    f_est = float(f0)
    if search_hz > 0:
        ref = mv[:, 0] - mv[:, 0].mean()
        n = 1 << 18
        X = np.abs(np.fft.rfft(ref * np.hanning(len(ref)), n))
        fr = np.fft.rfftfreq(n, 1 / fs)
        m = (fr >= f0 - search_hz) & (fr <= f0 + search_hz)
        f_est = float(fr[m][np.argmax(X[m])])
    out = {"f": f_est}
    for c, ch in enumerate(CH_NAMES[: mv.shape[1]]):
        amp, phi = sine_fit(mv[:, c], fs, f_est)
        d = {"amp_mv": amp, "phase": phi}
        for k in (2, 3):
            if k * f_est < fs / 2:
                h, _ = sine_fit(mv[:, c], fs, k * f_est)
                d[f"h{k}_db"] = float(20 * np.log10(h / amp + 1e-12))
            else:
                d[f"h{k}_db"] = float("nan")
        out[ch] = d
    dphi = np.pi - ((np.pi - (out["ref"]["phase"] - out["err"]["phase"])) % (2 * np.pi))
    out["delay_samples"] = float(dphi * fs / (2 * np.pi * f_est))
    out["period_samples"] = float(fs / f_est)
    fc, cxy = coherence(mv[:, 0], mv[:, 1], fs)
    out["coh"] = float(cxy[np.argmin(np.abs(fc - f_est))])
    return out


def block_sync_steps(x_mv, block=128, thresh_mv=120.0, tol=2) -> dict:
    """检测与固件处理块同步的跳变（采集第 0 帧是块起点）。

    fold：把 |diff| 按块内位置折叠平均，块边界 ±tol 处的最大值 / 其余位置的中位数。
    干净信号约 1，块同步干扰明显 >2；对单音、削波等不同步的大跳变不敏感，判定只看它。
    n / sync_frac / period：|diff| 峰值（height=thresh_mv）个数、落在块边界的比例、间隔中位数，仅供参考。
    """
    d = np.abs(np.diff(x_mv))
    pos_all = (np.arange(len(d)) + 1) % block
    fold = np.array([d[pos_all == k].mean() for k in range(block)])
    edge = np.r_[0:tol + 1, block - tol:block]
    rest = np.delete(fold, edge)
    pk, _ = signal.find_peaks(d, height=thresh_mv, distance=block // 2)
    pos = (pk + 1) % block
    sync = (pos <= tol) | (pos >= block - tol)
    gaps = np.diff(pk)
    return {"n": int(len(pk)),
            "sync_frac": float(np.mean(sync)) if len(pk) else 0.0,
            "period": float(np.median(gaps)) if len(gaps) else 0,
            "fold": float(fold[edge].max() / (np.median(rest) + 1e-12))}


def is_block_sync(r) -> bool:
    return r["fold"] >= 2.0


def fir_ls(u: np.ndarray, y: np.ndarray, n_taps: int) -> np.ndarray:
    """因果 FIR 的最小二乘。h[k] 使 y[n] ≈ Σ h[k] u[n−k]，k = 0 是同一拍。

    拟合前各自去直流。只使用窗口完整落在记录内的样本，记录开头的 n_taps−1 点不进方程。
    """
    u = np.ascontiguousarray(u, dtype=np.float64)
    y = np.ascontiguousarray(y, dtype=np.float64)
    if u.ndim != 1 or u.shape != y.shape:
        raise ValueError("u and y must be 1-D and the same length")
    if not 1 <= n_taps <= len(u):
        raise ValueError("n_taps out of range")
    u = u - u.mean()
    y = y - y.mean()
    X = np.lib.stride_tricks.sliding_window_view(u, n_taps)[:, ::-1]
    h, *_ = np.linalg.lstsq(X, y[n_taps - 1:], rcond=None)
    return h


def xcorr_ir(x: np.ndarray, y: np.ndarray, nlag: int = 200) -> np.ndarray:
    """白噪声激励下的冲激响应粗估：h[k] = Σ x[n]·y[n+k] / Σ x²，k = 0..nlag-1（y 相对 x 晚 k 个样本）。"""
    x = x - x.mean()
    y = y - y.mean()
    n = 1 << int(np.ceil(np.log2(2 * len(x))))
    r = np.fft.irfft(np.conj(np.fft.rfft(x, n)) * np.fft.rfft(y, n), n)[:nlag]
    return r / (np.dot(x, x) + 1e-12)


def dac_check(dac: np.ndarray, mv: np.ndarray, fs: float, nlag: int = 200) -> dict:
    """DAC 通道（码值）→ 两只麦（mV）：冲激响应粗估、到达时刻、相干。

    h 的单位是 mV / DAC 码；只有激励接近白噪声时 h 才接近真实冲激响应，单音只能看相干。
    onset：|h| 第一次超过最大值 30% 的样本，即声音到达时刻（管道共振会让 |h| 最大值落在后面的振荡里，
    peak_lag 不代表延迟）。coh_200：200 Hz 处相干；100–400 Hz 平均会被频响凹陷拉低。
    """
    u = dac.astype(np.float64) - DAC_MID
    out = {"dac_rms": float(u.std()), "dac_pp": int(dac.max()) - int(dac.min())}
    for c, ch in enumerate(CH_NAMES[: mv.shape[1]]):
        h = xcorr_ir(u, mv[:, c], nlag)
        f, cxy = coherence(u, mv[:, c], fs)
        m = (f >= 100) & (f <= 400)
        a = np.abs(h)
        out[ch] = {"ir": h, "peak_lag": int(np.argmax(a)), "onset": int(np.argmax(a > 0.3 * a.max())),
                   "f": f, "coh": cxy, "coh_100_400": float(np.mean(cxy[m])),
                   "coh_200": float(cxy[np.argmin(np.abs(f - 200))])}
    return out


def plot_dac_check(chk: dict, fs: float, path, title: str = "") -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 1, figsize=(9, 6.5))
    for ch in CH_NAMES:
        if ch in chk:
            r = chk[ch]
            ax[0].plot(r["ir"], lw=0.8, label=f"{ch} (onset {r['onset']}, max at {r['peak_lag']})")
            ax[0].axvline(r["onset"], lw=0.6, ls="--", color=ax[0].lines[-1].get_color())
            ax[1].plot(r["f"], r["coh"], lw=0.8,
                       label=f"{ch} (200 Hz {r['coh_200']:.2f}, 100-400 Hz mean {r['coh_100_400']:.2f})")
    ax[0].set(xlabel="lag / samples", ylabel="h / mV per DAC code", title=title)
    ax[1].set(xlabel="f / Hz", ylabel="γ² DAC-mic", xlim=(0, fs / 2), ylim=(0, 1.05))
    ax[1].axvspan(100, 400, color="orange", alpha=0.15)
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def crosstalk(codes: np.ndarray) -> dict:
    """两路的相关系数、互相关峰值滞后（样本）和线性拟合斜率。

    两只麦在管道里相距几百 mm，真实声信号到误差麦要晚若干个采样。
    若相关系数接近 1 且滞后为 0，说明第二路是 ADC 扫描串扰（输入悬空），不是声音。
    """
    x = codes[:, 0].astype(np.float64)
    y = codes[:, 1].astype(np.float64)
    x -= x.mean()
    y -= y.mean()
    c = np.correlate(y, x, "full")
    corr = float(np.dot(x, y) / (np.linalg.norm(x) * np.linalg.norm(y) + 1e-12))
    return {"corr": corr, "lag": int(np.argmax(np.abs(c)) - (len(x) - 1)),
            "slope": float(np.dot(x, y) / (np.dot(x, x) + 1e-12))}


def gcc_phat_lag(x: np.ndarray, y: np.ndarray, fs: float, fmin: float = 100.0,
                 fmax: float = 2000.0, max_lag: int = 40) -> tuple[int, float]:
    """GCC-PHAT 估计 y 相对 x 的时延（样本，正值表示 y 晚到），返回 (lag, 峰值强度 0..1)。

    PHAT 加权只保留相位，对管道共振和喇叭频响不敏感，适合在管道里测两麦声程差。
    """
    n = 1 << int(np.ceil(np.log2(2 * len(x))))
    X = np.fft.rfft(x - x.mean(), n)
    Y = np.fft.rfft(y - y.mean(), n)
    R = Y * np.conj(X)
    R /= np.abs(R) + 1e-12
    f = np.fft.rfftfreq(n, 1 / fs)
    R[(f < fmin) | (f > fmax)] = 0
    r = np.fft.irfft(R, n)
    r = np.concatenate((r[-max_lag:], r[: max_lag + 1]))
    k = int(np.argmax(r))
    return k - max_lag, float(r[k] / (np.sum((f >= fmin) & (f <= fmax)) / (n / 2 + 1) + 1e-12))


def plot_capture(codes: np.ndarray, fs: float, path, title: str = "") -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    mv = codes_to_mv(codes)
    t = np.arange(len(mv)) / fs
    f, p = psd(mv, fs)
    fig, ax = plt.subplots(3, 1, figsize=(9, 9))
    for c, name in enumerate(CH_NAMES[: mv.shape[1]]):
        ax[0].plot(t * 1e3, mv[:, c], lw=0.6, label=name)
        ax[1].plot(f, 10 * np.log10(p[:, c] + 1e-20), lw=0.8, label=name)
    ax[0].set(xlabel="t / ms", ylabel="mV (AC)", title=title)
    ax[1].set(xlabel="f / Hz", ylabel="PSD / dB re 1 mV²/Hz", xlim=(0, fs / 2))
    ax[1].axvspan(100, 400, color="orange", alpha=0.15, label="100-400 Hz")
    if mv.shape[1] >= 2:
        fc, cxy = coherence(mv[:, 0], mv[:, 1], fs)
        ax[2].plot(fc, cxy, lw=0.8)
        ax[2].axvspan(100, 400, color="orange", alpha=0.15)
        ax[2].axhline(0.9, color="r", ls="--", lw=0.8, label="γ²=0.9 (10 dB limit)")
    ax[2].set(xlabel="f / Hz", ylabel="γ² ref-err", xlim=(0, fs / 2), ylim=(0, 1.05))
    for a in ax:
        a.grid(alpha=0.3)
        a.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def save_wav(codes: np.ndarray, fs: int, path) -> None:
    """去直流后放大 16 倍存成 16 位 WAV（12 位满幅对应 int16 满幅）。"""
    from scipy.io import wavfile

    x = codes.astype(np.float64)
    x = (x - x.mean(axis=0)) * 16
    wavfile.write(path, fs, np.clip(x, -32768, 32767).astype(np.int16))
