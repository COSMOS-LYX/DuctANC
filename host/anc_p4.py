"""P4 第 2、3 步：对账 x'，以及 ANC 开着但 DAC 保持中点时的 CPU / 时序。

不发 TONE / NOISE / SWEEP。CLAMP 0 让输出码保持 2048，喇叭不应出声。
声学上的「DAC 到误差麦几个样本」要等喇叭可以出声之后再测，这个脚本不做。

    python anc_p4.py -p COM16 xchk
    python anc_p4.py -p COM16 bench
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

import anc_coeff as ac
import hwtest


def upload(dev: hwtest.Device, coeff: np.ndarray) -> str:
    dev.cmd("SHAT RST", timeout=3)
    raw = np.ascontiguousarray(coeff, dtype="<f4")
    for index in range(0, ac.N_SHAT, 8):
        blob = raw[index:index + 8].tobytes().hex()
        dev.cmd(f"SHAT {index} {blob}", timeout=3)
    crc = dev.cmd("SHAT CRC", timeout=3)
    expect = f"{ac.crc32_f32(raw):08X}"
    if f"crc={expect}" not in crc or "ready=1" not in crc:
        raise hwtest.DeviceError(f"shat crc {crc}, expect {expect}")
    return expect


def do_xchk(dev: hwtest.Device, d: Path) -> dict:
    coeff, info = ac.shat_eff()
    crc = upload(dev, coeff)
    z = np.load(ac.PRIMARY)
    codes = z["codes"][:400, 0]
    host = ac.replay_xf(codes, coeff)
    fw = dev.xchk(codes)
    err = np.abs(host.astype(np.float64) - fw.astype(np.float64))
    rel = err / (np.abs(host.astype(np.float64)) + 1e-6)
    np.savez(d / "xchk.npz", host=host, fw=fw, codes=codes, coeff=coeff, **{k: np.array(v) for k, v in info.items()})
    return {
        "crc": crc,
        "n": len(codes),
        "max_abs": float(err.max()),
        "max_rel": float(rel.max()),
        "rms": float(np.sqrt(np.mean(err ** 2))),
        **info,
    }


def do_bench(dev: hwtest.Device, d: Path) -> dict:
    coeff, info = ac.shat_eff()
    upload(dev, coeff)
    dev.cmd("CLAMP 0")
    dev.cmd("MU 0.002")
    dev.cmd("GAMMA 0")
    on = None
    for _ in range(5):
        try:
            on = dev.cmd("ANC ON", timeout=3)
            break
        except hwtest.DeviceError as e:
            if "settling" not in str(e):
                raise
            time.sleep(0.3)
    if on is None:
        raise hwtest.DeviceError("ANC ON 一直等滤波器稳定")
    time.sleep(0.5)
    rows = [dev.anc_status()]
    caps = []
    for i in range(3):
        codes, fs = dev.capture(2000, progress=True, dac=True)
        st = dev.anc_status()
        rows.append(st)
        dac = codes[:, 2]
        caps.append({"i": i, "fs": fs, "dac_min": int(dac.min()), "dac_max": int(dac.max()),
                     "dac_not_mid": int(np.sum(dac != 2048))})
        np.savez(d / f"cap{i}.npz", codes=codes, fs=fs)
    off = dev.cmd("ANC OFF", timeout=3)
    return {"on": on, "off": off, "status": rows, "caps": caps, "trunc": info}


def write_report(d: Path, xchk: dict | None, bench: dict | None, note: str):
    lines = ["# P4 第 1–3 步", "", note, ""]
    if xchk:
        lines += [
            "## x' 对账",
            "",
            f"Ŝ_eff 校验 {xchk['crc']}。高通冲激响应留了 {xchk['hp_taps']} 点"
            f"（尾部能量 {xchk['tail_energy']:.2e}）。"
            f"和在线 biquad 比，200 Hz 差 {xchk['db_200']:+.4f} dB、{xchk['deg_200']:+.4f}°。",
            "",
            f"400 点参考麦码：固件减上位机，最大绝对误差 {xchk['max_abs']:.3e}，"
            f"最大相对误差 {xchk['max_rel']:.3e}，rms {xchk['rms']:.3e}。",
            "",
        ]
    if bench:
        lines += ["## CPU 和时序（CLAMP 0，DAC 应保持 2048）", ""]
        lines.append("| 次序 | cpu % | lag_us | slip | late | ovr | udr | prot | baseline |")
        lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |")
        for i, st in enumerate(bench["status"]):
            lines.append(
                f"| {i} | {st.get('cpu')} | {st.get('lag_us')} | {st.get('slip')} | {st.get('late')} "
                f"| {st.get('ovr')} | {st.get('udr')} | {st.get('prot')} | {st.get('baseline')} |"
            )
        lines += ["", "| CAP | dac 不是 2048 的帧 | min | max |", "| --- | ---: | ---: | ---: |"]
        for c in bench["caps"]:
            lines.append(f"| {c['i']} | {c['dac_not_mid']} | {c['dac_min']} | {c['dac_max']} |")
        lines += [
            "",
            "cpu 是两次 STATUS 之间最慢一拍占采样周期的百分比。lag_us 是 DAC 中断到任务写槽的最大时间，超过 250 µs 会计 slip。",
            "这三次 CAP 串口在传数据。DAC 列如果出现不是 2048 的码，说明限幅没有把输出按在中点。",
            "DAC 到误差麦的到达样本数这次没测：那要喇叭出声。",
            "",
        ]
    (d / "report.md").write_text("\n".join(lines), encoding="utf-8")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", required=True)
    ap.add_argument("-b", "--baud", type=int, default=115200)
    ap.add_argument("step", choices=("xchk", "bench", "both"))
    args = ap.parse_args(argv)
    dev = hwtest.Device(args.port, args.baud)
    dev.drain(0.4)
    info = dev.info()
    try:
        dev.cmd("ANC OFF", timeout=2)
    except hwtest.DeviceError:
        pass
    d = hwtest.new_record_dir("p4prep")
    xchk = bench = None
    try:
        if info.get("fw") != "duct-anc-hwtest-0.3":
            raise hwtest.DeviceError(f"固件是 {info.get('fw')}，要 0.3。先烧录。")
        if args.step in ("xchk", "both"):
            xchk = do_xchk(dev, d)
            print("xchk", xchk)
        if args.step in ("bench", "both"):
            bench = do_bench(dev, d)
            print("bench caps", bench["caps"])
            print("status", bench["status"])
    finally:
        try:
            dev.cmd("ANC OFF", timeout=2)
        except hwtest.DeviceError:
            pass
        write_report(d, xchk, bench, f"固件 {info.get('fw')}，端口 {args.port}。喇叭指令没有发。")
        dev.close()
    print("saved", d)


if __name__ == "__main__":
    main()
