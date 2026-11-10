"""用已有的 DAC 同步白噪声记录辨识次级路径 Ŝ（DAC 码 → 误差麦 mV）。

在 host/ 下运行：python identify_s.py
读三段 NOISE 0.03（误差麦相干最高的那一组），写出 实验记录/<时间>_p2shat/。
"""
from __future__ import annotations

import numpy as np
from pathlib import Path
from scipy import signal

import analysis as an
import hwtest

FS = 8000
N_TAPS = 384
ROOT = Path(__file__).resolve().parent.parent / "实验记录"
# 17:00 这一组：dac_rms≈35 码（NOISE 0.03），200 Hz 处 DAC→err 相干 0.98。
# 16:50 那组相干掉到 0.5，不用。
SOURCES = (
    "20261110_165959_sid_a/sid_a.npz",
    "20261110_170008_sid_b/sid_b.npz",
    "20261110_170017_sid_c/sid_c.npz",
)
EARLY = (
    "20261110_162814_sid_a/sid_a.npz",
    "20261110_162826_sid_b/sid_b.npz",
    "20261110_162837_sid_c/sid_c.npz",
)
TONE = "20261110_173557_fb200/2_cancel_level.npz"


def load(name: str):
    z = np.load(ROOT / name)
    u = z["dac"].astype(np.float64) - an.DAC_MID
    return u, an.codes_to_mv(z["codes"])


def pooled(recs, ch: int, n_taps: int) -> np.ndarray:
    """三段各自去掉直流后叠在一起做最小二乘，段与段的接缝不进方程。"""
    xs, ys = [], []
    for u, mv in recs:
        uu = np.ascontiguousarray(u - u.mean())
        yy = mv[:, ch] - mv[:, ch].mean()
        xs.append(np.lib.stride_tricks.sliding_window_view(uu, n_taps)[:, ::-1])
        ys.append(yy[n_taps - 1:])
    h, *_ = np.linalg.lstsq(np.vstack(xs), np.concatenate(ys), rcond=None)
    return h


def predict(u, h):
    return np.convolve(u - u.mean(), h, mode="full")[:len(u)]


def nmse_db(y, yh, start: int) -> float:
    y = y - y.mean()
    e = y[start:] - yh[start:]
    r = y[start:]
    return float(10 * np.log10(np.dot(e, e) / np.dot(r, r)))


def band_nmse_db(y, yh, start: int, lo=100.0, hi=400.0) -> float:
    sos = signal.butter(4, [lo, hi], "band", fs=FS, output="sos")
    return nmse_db(signal.sosfiltfilt(sos, y), signal.sosfiltfilt(sos, yh), start)


def resp(h, nfft=16384):
    f = np.fft.rfftfreq(nfft, 1 / FS)
    return f, np.fft.rfft(h, nfft)


def at(f, H, hz):
    k = int(np.argmin(np.abs(f - hz)))
    return float(f[k]), float(np.abs(H[k])), float(np.degrees(np.angle(H[k])))


def main():
    recs = [load(n) for n in SOURCES]
    early = [load(n) for n in EARLY]
    h = pooled(recs, 1, N_TAPS)
    hf = pooled(recs, 0, N_TAPS)
    f, H = resp(h)
    _, F = resp(hf)

    # 留一段做验证：用另外两段拟合，在留出的那段上算 NMSE
    loo = []
    for i in range(3):
        ho = pooled([recs[j] for j in range(3) if j != i], 1, N_TAPS)
        y = recs[i][1][:, 1]
        yh = predict(recs[i][0], ho)
        loo.append((nmse_db(y, yh, N_TAPS - 1), band_nmse_db(y, yh, N_TAPS - 1)))

    singles = [an.fir_ls(u, mv[:, 1], N_TAPS) for u, mv in recs]
    gaps = []
    for i in range(3):
        for j in range(i + 1, 3):
            d = singles[i] - singles[j]
            gaps.append(20 * np.log10(np.linalg.norm(d) / np.linalg.norm(singles[i])))

    # 阶数：只在第一段上拟合，看后两段的带内 NMSE
    order = {}
    for L in (128, 256, 384, 512):
        hs = an.fir_ls(recs[0][0], recs[0][1][:, 1], L)
        order[L] = np.mean([
            band_nmse_db(recs[k][1][:, 1], predict(recs[k][0], hs), L - 1) for k in (1, 2)
        ])

    # 单音：Ŝ 是用白噪声拟合的，这里看它能不能预言后来那次 200 Hz
    tz = np.load(ROOT / TONE)
    tu = tz["dac"].astype(np.float64) - an.DAC_MID
    tmv = an.codes_to_mv(tz["codes"])
    th = predict(tu, h)
    meas_amp, meas_phi = an.sine_fit(tmv[:, 1], FS, 200)
    pred_amp, pred_phi = an.sine_fit(th, FS, 200)
    dphi = float(np.degrees((pred_phi - meas_phi + np.pi) % (2 * np.pi) - np.pi))

    early_nmse = [
        nmse_db(mv[:, 1], predict(u, h), N_TAPS - 1) for u, mv in early
    ]

    # 频响上 100–400 Hz 里 |Ŝ| 的峰，以及 350 Hz 附近的谷
    m = (f >= 100) & (f <= 400)
    idx = np.where(m)[0]
    peaks, _ = signal.find_peaks(np.abs(H[idx]), distance=8, height=20)
    peak_rows = [(at(f, H, f[idx[i]]) ) for i in peaks]
    s200 = at(f, H, 200)
    s350 = at(f, H, 350)
    f200 = at(f, F, 200)

    d = hwtest.new_record_dir("p2shat")
    np.savez(
        d / "shat.npz",
        h_err=h, h_ref=hf, fs=FS, n_taps=N_TAPS,
        sources=np.array(SOURCES),
    )
    plot(d / "shat.png", h, f, H)
    write_report(d / "report.md", loo, gaps, order, s200, peak_rows, s350, f200,
                 meas_amp, pred_amp, dphi, early_nmse)
    print(f"saved {d}")
    print(f"S(200) = {s200[1]:.2f} mV/code  {s200[2]:+.1f} deg")
    print(f"tone {meas_amp:.2f} mV pred {pred_amp:.2f} mV  dphi {dphi:+.2f} deg")
    print(f"LOO band NMSE {[round(b, 2) for _, b in loo]}")
    print(f"IR gaps {[round(g, 2) for g in gaps]}")


def plot(path, h, f, H):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, 1, figsize=(9, 6.5))
    ax[0].plot(h, lw=0.8)
    ax[0].set(xlabel="lag / samples", ylabel="h / mV per DAC code",
              title=f"S hat, {N_TAPS} taps")
    ax[1].plot(f, 20 * np.log10(np.abs(H) + 1e-6), lw=0.8)
    ax[1].axvspan(100, 400, color="orange", alpha=0.15)
    ax[1].axvline(200, color="g", ls="--", lw=0.8, label="200 Hz")
    ax[1].set(xlabel="f / Hz", ylabel="|S| / dB re 1 mV/code", xlim=(0, 1000))
    for a in ax:
        a.grid(alpha=0.3)
    ax[1].legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def write_report(path, loo, gaps, order, s200, peaks, s350, f200,
                 meas_amp, pred_amp, dphi, early_nmse):
    loo_b = ", ".join(f"{b:.1f}" for _, b in loo)
    gap_s = ", ".join(f"{g:.1f}" for g in gaps)
    order_s = ", ".join(f"{L} 阶 {v:.1f} dB" for L, v in order.items())
    peak_s = "，".join(
        f"{hz:.0f} Hz {mag:.1f} mV/码（比 200 Hz 高 {-20 * np.log10(s200[1] / mag):.1f} dB）"
        for hz, mag, _ph in peaks)
    drop = -20 * np.log10(s350[1] / s200[1])
    text = f"""# 次级路径辨识 2026-11-10

三段白噪声，DAC 与误差麦逐帧对齐：`165959_sid_a`、`170008_sid_b`、`170017_sid_c`（NOISE 0.03，各 10666 帧）。回归量是 DAC 码减 2048，不是参考麦。三段叠在一起做最小二乘，段与段的接缝不进方程。

Ŝ 是 **{N_TAPS}** 阶，单位 mV / DAC 码，`shat.npz` 的 `h_err`。`h[k]` 是误差麦相对 DAC 晚 k 个样本的系数。

| | |
| --- | --- |
| 留一段验证，100–400 Hz NMSE | {loo_b} dB |
| 三段各自拟合的冲激响应互差 | {gap_s} dB |
| Ŝ(200 Hz) | {s200[1]:.2f} mV/码，{s200[2]:+.1f}° |
| 随后的 200 Hz 单音（amp 0.00433） | 实测 {meas_amp:.1f} mV，Ŝ 预测 {pred_amp:.1f} mV，相位差 {dphi:+.2f}° |
| 16:28 那三段用这只 Ŝ 去预测 | NMSE {early_nmse[0]:.1f} / {early_nmse[1]:.1f} / {early_nmse[2]:.1f} dB |

阶数：只用第一段拟合，后两段的 100–400 Hz NMSE 平均为 {order_s}。384 再加到 512 只多约 0.3 dB，取 384。

100–400 Hz 里 |Ŝ| 高于 20 mV/码的峰：{peak_s}。350 Hz 处 {s350[1]:.2f} mV/码，比 200 Hz 低 {drop:.0f} dB。232 Hz 是次级路径自己的峰，不是回流放大倍数的测量；参考麦那条回路的相干只有 0.5 左右，不能据此说圈在那儿更大。

同一拟合给出的参考麦通路 `h_ref` 在 200 Hz 是 {f200[1]:.2f} mV/码、{f200[2]:+.1f}°，和回流测量的 1.74 mV/码同一量级。参考麦里还有一半以上能量和 DAC 不相干，`h_ref` 只描述其中对得上的那部分。

![shat](shat.png)
"""
    path.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
