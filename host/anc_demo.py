"""一次演示：不设预检和限幅停止。μ=0.005，开 8 s，然后关掉。"""
from __future__ import annotations

import time

import numpy as np

import analysis as an
import anc_run3
import hwtest


def fnum(st, key):
    return float(st[key])


def main():
    dev = hwtest.Device("COM16", 115200)
    dev.drain(0.3)
    d = hwtest.new_record_dir("p4demo")
    started = False
    rows = []
    codes = fs = None
    try:
        st = dev.anc_status()
        if st.get("on") != "0":
            dev.cmd("ANC OFF")
            st = dev.anc_status()
        e0, r0, b0 = fnum(st, "e200"), fnum(st, "r200"), fnum(st, "baseline")
        print(f"pre e200={e0:.2f} r200={r0:.2f} baseline={b0:.2f} shat={st.get('shat')}")
        if st.get("shat") != "1":
            raise SystemExit("Ŝ 不在芯片上，没有开")
        dev.cmd("MU 0.005")
        dev.cmd("GAMMA 0")
        dev.cmd("CLAMP 40")
        dev.cmd("CARM 10666 D")
        print(dev.cmd("ANC ON", timeout=3))
        started = True
        t0 = time.perf_counter()
        next_t = 0.0
        while True:
            now = time.perf_counter() - t0
            if now < next_t:
                time.sleep(next_t - now)
            st = dev.anc_status()
            t = time.perf_counter() - t0
            e = fnum(st, "e200")
            c = int(float(st["clamp_n"]))
            rows.append((t, st, e, c))
            print(f"t={t:5.2f} e200={e:7.2f} r200={fnum(st,'r200'):6.2f} yrms={fnum(st,'yrms'):6.2f} "
                  f"clamp_n={c} prot={st.get('prot')} cpu={st.get('cpu')}")
            if t >= 8.0:
                break
            next_t += 0.25
    finally:
        if started:
            try:
                print(dev.cmd("ANC OFF", timeout=3))
            except hwtest.DeviceError as ex:
                print("ANC OFF", ex)
            try:
                codes, fs = anc_run3.cget(dev)
                print("cap", None if codes is None else codes.shape)
            except hwtest.DeviceError as ex:
                print("CGET", ex)
        end = None
        try:
            end = dev.anc_status()
        except hwtest.DeviceError as ex:
            print("status", ex)
        dev.close()
        write(d, e0, r0, b0, rows, codes, fs, end)
        print("saved", d)


def write(d, e0, r0, b0, rows, codes, fs, end):
    lines = [
        "# 一次演示，没有预检、没有因限幅提前停",
        "",
        f"开环：误差麦 200 Hz {e0:.2f} mV，参考麦 {r0:.2f} mV，基线 {b0:.2f} mV。",
        "μ=0.005，γ=0，限幅 ±40。计划开 8 s。",
        "",
    ]
    if not rows:
        lines.append("没有打开 ANC。")
    else:
        lines += [
            "| t / s | e200 | r200 | yrms | clamp_n | prot | cpu | slip | late | ovr | udr |",
            "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
        es, ts = [], []
        for t, st, e, c in rows:
            es.append(e)
            ts.append(t)
            lines.append(
                f"| {t:.2f} | {e:.2f} | {float(st['r200']):.2f} | {float(st['yrms']):.2f} | {c} | "
                f"{st.get('prot')} | {st.get('cpu')} | {st.get('slip')} | {st.get('late')} | "
                f"{st.get('ovr')} | {st.get('udr')} |"
            )
        es = np.asarray(es, float)
        ts = np.asarray(ts, float)
        steady = float(np.median(es[ts >= ts[-1] - 2.0]))
        db = 20 * np.log10(e0 / steady) if steady > 0 else float("nan")
        hold = None
        for i, t in enumerate(ts):
            if e0 <= 0 or es[i] > e0 * 10 ** (-3 / 20):
                continue
            m = (ts >= t) & (ts <= t + 0.5)
            if int(m.sum()) >= 2 and np.all(es[m] <= e0 * 10 ** (-3 / 20)):
                hold = float(t)
                break
        lines += [
            "",
            f"末 2 s 中位数 {steady:.2f} mV，相对开前 {e0:.2f} mV "
            f"{'低' if db > 0 else '高'} {abs(db):.1f} dB。",
            f"比开前低 3 dB 并保持 0.5 s：{('约 %.2f s' % hold) if hold is not None else '没有'}。",
            f"末次 prot={rows[-1][1].get('prot')} clamp_n={rows[-1][3]} "
            f"slip={rows[-1][1].get('slip')} late={rows[-1][1].get('late')} "
            f"ovr={rows[-1][1].get('ovr')} udr={rows[-1][1].get('udr')} cpu={rows[-1][1].get('cpu')}。",
        ]
    if codes is not None:
        y = codes[:, 2].astype(np.float64) - 2048.0
        np.savez(d / "cap.npz", codes=codes, fs=fs)
        hit = np.where(np.abs(y) >= 39.5)[0]
        lines += ["", f"开头 {len(y) / fs:.2f} s 的 DAC：碰到 ±40 的样本 {len(hit)} 个。"]
        if len(hit):
            lines.append(f"第一个 {hit[0] / fs * 1000:.0f} ms，最后一个 {hit[-1] / fs * 1000:.0f} ms。")
        lines += ["", "| 频率 | 这段 y 的 rms / 码 |", "| ---: | ---: |"]
        for f0 in (200.0, 133.0, 230.0, 400.0):
            lines.append(f"| {f0:.0f} Hz | {an.sine_fit(y, fs, f0)[0]:.2f} |")
    if end:
        lines += ["", f"关掉之后 on={end.get('on')} prot={end.get('prot')} e200={end.get('e200')}。"]
    lines.append("")
    (d / "report.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
