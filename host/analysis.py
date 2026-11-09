"""麦克风数据分析：电压换算、频谱、频段电平、相干函数。后续 ANC 实测也复用这里。"""
from __future__ import annotations

import numpy as np
from scipy import signal

VREF_MV = 3300.0
ADC_FS = 4095.0
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
