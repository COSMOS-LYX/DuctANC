"""P4 的 Ŝ_eff 和与固件逐位对齐的 float32 滤波参考。

固件里的高通系数、mV 换算、Goertzel 系数是这里这些 uint32 位型，不要改一边。
"""
from __future__ import annotations

import zlib
from pathlib import Path

import numpy as np
from scipy import signal

FS = 8000
N_SHAT = 384
N_W = 16
F0 = 200.0
# 高通冲激响应截到剩余能量低于这个比例。128 点时约 1e-6。
HP_TAIL = 1e-6

# 与 firmware/hwtest/User/anc.c 的 HP_*_BITS / MV_PER_CODE_BITS / GOERTZEL 一致
HP_B_BITS = (0x3F74E004, 0xBFF4E004, 0x3F74E004)
HP_A_BITS = (0xBFF4A21B, 0x3F6A3BDB)  # a1, a2
MV_PER_CODE_BITS = 0x3F4E4CE5
GOERTZEL_COS2_BITS = 0x3FFCD925
GOERTZEL_SIN_BITS = 0x3E20305B

SHAT_NPZ = Path(__file__).resolve().parent.parent / "实验记录" / "20261110_175018_p2shat" / "shat.npz"
PRIMARY = Path(__file__).resolve().parent.parent / "实验记录" / "20261110_154011_p0test" / "4_stand.npz"


def f32_bits(u: int) -> np.float32:
    return np.array([u], np.uint32).view(np.float32)[0]


HP_B = np.array([f32_bits(u) for u in HP_B_BITS], np.float32)
HP_A1 = f32_bits(HP_A_BITS[0])
HP_A2 = f32_bits(HP_A_BITS[1])
MV_PER_CODE = f32_bits(MV_PER_CODE_BITS)


def load_h_err(path: Path = SHAT_NPZ) -> np.ndarray:
    h = np.load(path)["h_err"].astype(np.float64)
    if h.shape != (N_SHAT,):
        raise ValueError(f"h_err 长度是 {h.shape}，要 {N_SHAT}")
    return h


def highpass_ir(n: int = 4096) -> np.ndarray:
    """同一组 float32 系数的冲激响应，float64 只用于决定截多长。"""
    b = np.array([HP_B[0], HP_B[1], HP_B[2], 0, 0], np.float64)[:3]
    a = np.array([1.0, HP_A1, HP_A2], np.float64)
    imp = np.zeros(n)
    imp[0] = 1.0
    return signal.lfilter(b, a, imp)


def shat_eff(h_err: np.ndarray | None = None) -> tuple[np.ndarray, dict]:
    """384 阶 Ŝ 与二阶高通冲激响应卷积，保留前 384 点。返回 float32。"""
    h = load_h_err() if h_err is None else np.asarray(h_err, np.float64)
    hp = highpass_ir()
    tail = np.cumsum(hp[::-1] ** 2)[::-1]
    cut = int(np.argmax(tail < HP_TAIL * tail[0])) + 1
    full = np.convolve(h, hp[:cut])
    coeff = full[:N_SHAT].astype(np.float32)
    # 在线 biquad 打在 Ŝ 上，用来看截断在 200 Hz 差多少。这不是对账本身。
    b = np.array([float(HP_B[0]), float(HP_B[1]), float(HP_B[2])], np.float64)
    a = np.array([1.0, float(HP_A1), float(HP_A2)], np.float64)
    online = signal.lfilter(b, a, h)[:N_SHAT]
    w = np.exp(-2j * np.pi * F0 * np.arange(N_SHAT) / FS)
    h1 = np.dot(coeff.astype(np.float64), w)
    h2 = np.dot(online, w)
    info = {
        "hp_taps": cut,
        "tail_energy": float(tail[cut - 1] / tail[0]),
        "max_abs_vs_biquad": float(np.max(np.abs(coeff.astype(np.float64) - online))),
        "db_200": float(20 * np.log10(abs(h1) / abs(h2))),
        "deg_200": float(np.degrees(np.angle(h1 / h2))),
    }
    return coeff, info


def crc32_f32(coeff: np.ndarray) -> int:
    return zlib.crc32(np.ascontiguousarray(coeff, dtype="<f4").tobytes()) & 0xFFFFFFFF


def _biquad(x: np.float32, z: list[np.float32]) -> np.float32:
    y = np.float32(HP_B[0] * x + z[0])
    z[0] = np.float32(np.float32(HP_B[1] * x) - np.float32(HP_A1 * y) + z[1])
    z[1] = np.float32(np.float32(HP_B[2] * x) - np.float32(HP_A2 * y))
    return y


def replay_xf(ref_codes: np.ndarray, coeff: np.ndarray) -> np.ndarray:
    """复刻 anc_replay_xf：参考码去 2048、换 mV、二阶高通、384 点 FIR。误差麦不参与。"""
    coeff = np.ascontiguousarray(coeff, np.float32)
    n_h = len(coeff)
    hist = np.zeros(n_h, np.float32)
    pos = n_h - 1
    z = [np.float32(0), np.float32(0)]
    out = np.empty(len(ref_codes), np.float32)
    for i, code in enumerate(np.asarray(ref_codes)):
        x = np.float32((int(code) - 2048) * MV_PER_CODE)
        x = _biquad(np.float32(x), z)
        pos += 1
        if pos == n_h:
            pos = 0
        hist[pos] = x
        acc = np.float32(0)
        for k in range(n_h):
            j = pos - k
            if j < 0:
                j += n_h
            acc = np.float32(acc + np.float32(coeff[k] * hist[j]))
        out[i] = acc
    return out


def goertzel_rms(x: np.ndarray, n: int = FS // 4) -> float:
    """复刻 anc_goertzel_step 满 0.25 s 时给出的 rms。"""
    coeff = f32_bits(GOERTZEL_COS2_BITS)
    sw = f32_bits(GOERTZEL_SIN_BITS)
    q1 = q2 = np.float32(0)
    x = np.asarray(x, np.float32)
    if len(x) < n:
        raise ValueError("不够一窗")
    for v in x[:n]:
        q0 = np.float32(v + np.float32(coeff * q1) - q2)
        q2, q1 = q1, q0
    cw = np.float32(coeff * np.float32(0.5))
    real = np.float32(q1 - np.float32(q2 * cw))
    imag = np.float32(q2 * sw)
    mag = np.float32(np.sqrt(np.float32(real * real + imag * imag)))
    return float(np.float32(mag * np.float32(1.41421356) / np.float32(n)))
