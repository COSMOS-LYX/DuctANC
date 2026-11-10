"""测 DAC→误差麦的到达样本数。ANC 保持关，NOISE 0.03，测完 STOP。

到达样本用和 P1 相同的定义：白噪声互相关 |h| 第一次超过峰值 30% 的下标。
Ŝ 的 h_err 用这个定义是 5。三次不一致就只写记录，不开 ANC。
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np

import analysis as an
import hwtest


FRAMES = 10666
AMP = 0.03
SHAT = Path(__file__).resolve().parents[1] / "实验记录" / "20261110_175018_p2shat" / "shat.npz"


def onset_of(h: np.ndarray) -> int:
    a = np.abs(h)
    return int(np.argmax(a > 0.3 * a.max()))


def main():
    dev = hwtest.Device("COM16", 115200)
    dev.drain(0.3)
    d = hwtest.new_record_dir("p4delay")
    rows = []
    st0 = {}
    st_end = {}
    try:
        st0 = dev.anc_status()
        if st0.get("on") != "0":
            dev.cmd("ANC OFF")
            st0 = dev.anc_status()
        if st0.get("on") != "0":
            raise hwtest.DeviceError(f"ANC 关不掉: {st0}")
        if abs(float(st0.get("clamp", "0")) - 40) > 0.1:
            dev.cmd("CLAMP 40")
            st0 = dev.anc_status()
        dev.cmd("NOISE 0.03")
        time.sleep(0.4)
        for i in range(3):
            try:
                codes, fs = dev.capture(FRAMES, progress=True, dac=True)
            except hwtest.DeviceError as e:
                print(f"cap{i} 回传不完整，重采一次：{e}")
                dev.drain(0.3)
                codes, fs = dev.capture(FRAMES, progress=True, dac=True)
            st = dev.anc_status()
            if st.get("on") != "0":
                raise hwtest.DeviceError(f"第 {i} 次采集时 ANC 被打开")
            np.savez(d / f"cap{i}.npz", codes=codes, fs=fs)
            chk = an.dac_check(codes[:, 2], an.codes_to_mv(codes[:, :2]), fs)
            an.plot_dac_check(chk, fs, d / f"cap{i}_dac.png", title=f"cap{i} DAC -> mic")
            h = chk["err"]["ir"]
            rows.append({
                "i": i,
                "onset": chk["err"]["onset"],
                "peak": chk["err"]["peak_lag"],
                "coh200": chk["err"]["coh_200"],
                "dac_rms": chk["dac_rms"],
                "h": h[:16].copy(),
                "st": st,
            })
            print(f"cap{i} onset={rows[-1]['onset']} peak={rows[-1]['peak']} "
                  f"dac_rms={rows[-1]['dac_rms']:.1f} coh200={rows[-1]['coh200']:.3f}")
        same = len({r["onset"] for r in rows}) == 1
        if not same:
            print("三次到达样本数不一致，停在这里，不开 ANC")
    finally:
        try:
            print(dev.cmd("STOP"))
        except hwtest.DeviceError as e:
            print("STOP", e)
        time.sleep(3.0)
        try:
            st_end = dev.anc_status()
        except hwtest.DeviceError as e:
            st_end = {"error": str(e)}
        dev.close()
        if rows:
            h_err = np.load(SHAT)["h_err"]
            shat_on = onset_of(h_err)
            write(d, st0, rows, st_end, shat_on, h_err[:16])
            print("saved", d, "onsets", [r["onset"] for r in rows], "shat", shat_on)
            print("baseline_end", st_end.get("baseline"), "on", st_end.get("on"))


def write(d, st0, rows, st_end, shat_on, h0):
    lines = [
        "# DAC→误差麦到达样本",
        "",
        "ANC 全程 OFF。激励 `NOISE 0.03`，和 P2 相同。每段 10666 帧，带 DAC。测完已 `STOP`。",
        "",
        f"Ŝ 的 `h_err` 用同一条规则（|h| 第一次超过峰值 30%）落在第 {shat_on} 个样本。"
        "P2 记录里写的 5–6，是这个上升沿：第 4 点还只有峰值的 1.5%，第 5 点 32%，第 6 点 54%。",
        "",
        "| 次 | 到达 | |h| 最大 | DAC rms | 200 Hz 相干 | cpu % | slip | late | ovr | udr | baseline |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for r in rows:
        s = r["st"]
        lines.append(
            f"| {r['i']} | {r['onset']} | {r['peak']} | {r['dac_rms']:.1f} | {r['coh200']:.3f} | "
            f"{s.get('cpu')} | {s.get('slip')} | {s.get('late')} | {s.get('ovr')} | {s.get('udr')} | {s.get('baseline')} |"
        )
    onsets = [r["onset"] for r in rows]
    same = len(set(onsets)) == 1 and len(onsets) == 3
    shown = "、".join(str(v) for v in onsets) if onsets else "没有"
    lines += [
        "",
        f"到达样本数：{shown}。"
        + ("三次相同。" if same else "没有得到三次相同的结果，按约定停在这里，没有开 ANC。"),
        "",
        "前 12 个系数（mV/码）：",
        "",
    ]
    for r in rows:
        txt = " ".join(f"{v:+.3f}" for v in r["h"][:12])
        lines.append(f"- 第 {r['i']} 次：{txt}")
    lines.append("- Ŝ：" + " ".join(f"{v:+.3f}" for v in h0[:12]))
    lines += [
        "",
        f"开噪声前：on={st0.get('on')} clamp={st0.get('clamp')} wnorm={st0.get('wnorm')} "
        f"baseline={st0.get('baseline')} e200={st0.get('e200')} "
        f"cpu={st0.get('cpu')} slip={st0.get('slip')} late={st0.get('late')} "
        f"ovr={st0.get('ovr')} udr={st0.get('udr')}",
        "",
        "计数器是累积值。cpu 是上一次 STATUS 之后最慢一拍占采样周期的百分比，所以每一行盖住这一次采集和回传。",
        "",
        f"STOP 后约 3 秒：on={st_end.get('on')} wnorm={st_end.get('wnorm')} clamp={st_end.get('clamp')} "
        f"baseline={st_end.get('baseline')} prot={st_end.get('prot')} "
        f"slip={st_end.get('slip')} late={st_end.get('late')} ovr={st_end.get('ovr')} udr={st_end.get('udr')} "
        f"cpu={st_end.get('cpu')}",
        "",
        "没有发 `ANC ON`。",
        "",
    ]
    (d / "report.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
