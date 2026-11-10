"""一次 10 s 闭环。电平偏离就不开 ANC。触发保护、限幅持续增加、或误差麦 200 Hz 升高超过 2 s，立刻关掉。"""
from __future__ import annotations

import time

import numpy as np

import hwtest


ERR_LO, ERR_HI = 100.0, 260.0
REF_LO, REF_HI = 25.0, 60.0
RATIO_LO, RATIO_HI = 3.8, 5.8


def fnum(st, key):
    return float(st[key])


def main():
    dev = hwtest.Device("COM16", 115200)
    dev.drain(0.3)
    d = hwtest.new_record_dir("p4try")
    started = False
    rows = []
    abort = None
    try:
        info = dev.info()
        if info.get("out") not in (None, "OFF"):
            dev.cmd("STOP")
        st = dev.anc_status()
        if st.get("on") != "0":
            raise SystemExit(f"ANC 已经开着，没有做这次试听: {st}")
        e0 = fnum(st, "e200")
        r0 = fnum(st, "r200")
        b0 = fnum(st, "baseline")
        ratio = e0 / r0 if r0 > 0 else float("inf")
        print(f"pre e200={e0:.2f} r200={r0:.2f} ratio={ratio:.2f} baseline={b0:.2f} gate={2 * b0:.2f}")
        if not (ERR_LO <= e0 <= ERR_HI):
            raise SystemExit(f"误差麦 200 Hz {e0:.1f} mV，不在 100–260 mV，不开 ANC")
        if not (REF_LO <= r0 <= REF_HI):
            raise SystemExit(f"参考麦 200 Hz {r0:.1f} mV，不在 25–60 mV，不开 ANC")
        if not (RATIO_LO <= ratio <= RATIO_HI):
            raise SystemExit(f"两路比值 {ratio:.2f}，不在 3.8–5.8，不开 ANC")
        dev.cmd("MU 0.005")
        dev.cmd("GAMMA 0")
        dev.cmd("CLAMP 40")
        chk = dev.anc_status()
        if abs(fnum(chk, "mu") - 0.005) > 1e-4 or abs(fnum(chk, "gamma")) > 1e-6 or abs(fnum(chk, "clamp") - 40) > 0.1:
            raise SystemExit(f"参数没有停在指定值: {chk}")
        if chk.get("shat") != "1":
            raise SystemExit("Ŝ 不在芯片上，不开 ANC")
        print(dev.cmd("ANC ON", timeout=3))
        started = True
        t0 = time.perf_counter()
        prev_c = 0
        rise = 0
        up_since = None
        next_t = 0.0
        while next_t <= 10.0:
            now = time.perf_counter() - t0
            if now < next_t:
                time.sleep(next_t - now)
            st = dev.anc_status()
            t = time.perf_counter() - t0
            e = fnum(st, "e200")
            c = int(float(st["clamp_n"]))
            rows.append((t, st, e, c))
            print(f"t={t:5.2f} e200={e:7.2f} r200={fnum(st,'r200'):6.2f} yrms={fnum(st,'yrms'):6.2f} "
                  f"wnorm={fnum(st,'wnorm'):7.4f} clamp_n={c} prot={st.get('prot')} "
                  f"cpu={st.get('cpu')} slip={st.get('slip')} late={st.get('late')}")
            if st.get("prot") == "1":
                abort = "保护触发"
                break
            if c > prev_c:
                rise += 1
                if rise >= 2 or (c - prev_c) >= 20:
                    abort = "限幅次数持续增加"
                    break
            else:
                rise = 0
            prev_c = c
            if e > e0 * 1.02:
                if up_since is None:
                    up_since = t
                elif t - up_since > 2.0:
                    abort = "误差麦 200 Hz 升高并持续超过 2 s"
                    break
            else:
                up_since = None
            next_t += 0.5
    finally:
        off = None
        if started:
            try:
                off = dev.cmd("ANC OFF", timeout=3)
                print(off)
            except hwtest.DeviceError as e:
                print("ANC OFF", e)
        end = None
        try:
            end = dev.anc_status()
            print("end", {k: end.get(k) for k in ("on", "prot", "e200", "r200", "baseline", "clamp_n", "slip", "late", "ovr", "udr")})
        except hwtest.DeviceError as e:
            print("status", e)
        dev.close()
        write(d, e0 if "e0" in dir() else None, r0 if "r0" in dir() else None,
              b0 if "b0" in dir() else None, rows, abort, off, end)
        print("saved", d, "abort", abort)


def write(d, e0, r0, b0, rows, abort, off, end):
    lines = ["# 10 s 闭环试一次", ""]
    if e0 is None:
        lines.append("开环读数没有完成。")
    else:
        ratio = e0 / r0 if r0 else float("inf")
        lines.append(
            f"开环：误差麦 200 Hz {e0:.2f} mV，参考麦 200 Hz {r0:.2f} mV，比值 {ratio:.2f}，"
            f"基线 {b0:.2f} mV，保护门槛 {2 * b0:.2f} mV。"
        )
        lines.append("通过条件：误差麦 100–260 mV，参考麦 25–60 mV，比值 3.8–5.8。")
    lines.append("")
    if not rows:
        lines.append("没有打开 ANC。" if abort is None else f"没有进入闭环。{abort or ''}")
    else:
        lines += [
            "μ=0.005，γ=0，限幅 ±40。",
            "",
            "| t / s | e200 | r200 | yrms | wnorm | clamp_n | prot | cpu | slip | late | ovr | udr |",
            "| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
        es, ts = [], []
        for t, st, e, c in rows:
            es.append(e)
            ts.append(t)
            lines.append(
                f"| {t:.2f} | {e:.2f} | {float(st['r200']):.2f} | {float(st['yrms']):.2f} | "
                f"{float(st['wnorm']):.4f} | {c} | {st.get('prot')} | {st.get('cpu')} | "
                f"{st.get('slip')} | {st.get('late')} | {st.get('ovr')} | {st.get('udr')} |"
            )
        es = np.array(es)
        ts = np.array(ts)
        if abort:
            lines += ["", f"提前关掉：{abort}。没有重试，参数没有改。"]
        tail = es[ts >= max(0.0, ts[-1] - 2.0)]
        steady = float(np.median(tail))
        db = 20 * np.log10((e0 if e0 else 1) / steady) if steady > 0 and e0 else float("nan")
        down = np.where(es <= e0 * 10 ** (-3 / 20))[0] if e0 else []
        t_drop = None
        for i in down:
            if ts[i] + 1.0 <= ts[-1] and np.all(es[(ts >= ts[i]) & (ts <= ts[i] + 1.0)] <= e0 * 10 ** (-3 / 20)):
                t_drop = float(ts[i])
                break
        lines += [
            "",
            f"开前误差麦 200 Hz {e0:.2f} mV。末段 2 s 中位数 {steady:.2f} mV，"
            f"{'降了' if db > 0 else '升高了'} {abs(db):.1f} dB。",
            f"降到比开前低 3 dB 并保持 1 s：{('约 %.1f s' % t_drop) if t_drop is not None else '这段记录里没有'}。",
            f"末次 prot={rows[-1][1].get('prot')} clamp_n={rows[-1][3]} "
            f"slip={rows[-1][1].get('slip')} late={rows[-1][1].get('late')} "
            f"ovr={rows[-1][1].get('ovr')} udr={rows[-1][1].get('udr')}。",
        ]
    if end:
        lines += ["", f"关掉之后：on={end.get('on')} prot={end.get('prot')} e200={end.get('e200')} baseline={end.get('baseline')}。"]
    lines.append("")
    (d / "report.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
