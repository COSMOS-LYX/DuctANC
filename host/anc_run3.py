"""μ=0.002，ANC 开 3 s。CARM 先于 ANC ON，这段 CAP 盖住启动。前 1.5 s 的限幅不停。"""
from __future__ import annotations

import time

import numpy as np

import analysis as an
import anc_coeff as ac
import anc_p4
import hwtest


ERR_LO, ERR_HI = 100.0, 260.0
REF_LO, REF_HI = 25.0, 60.0
RATIO_LO, RATIO_HI = 3.8, 5.8
FRAMES = 10666
LINES = (200.0, 133.0, 230.0, 400.0)


def fnum(st, key):
    return float(st[key])


def cget(dev: hwtest.Device):
    dev.send("CGET")
    deadline = time.monotonic() + 3
    head = None
    while time.monotonic() < deadline:
        r = dev.readline(max(0.0, deadline - time.monotonic()))
        if r is None:
            break
        if r.startswith("ERR"):
            raise hwtest.DeviceError(r)
        if r.startswith("#CAP"):
            head = r
            break
    if head is None:
        raise hwtest.DeviceError("no #CAP header")
    _, n, fs, nch = head.split()
    n, fs, nch = int(n), int(fs), int(nch)
    nbytes = n * nch * 2
    raw = dev.read_exact(nbytes, nbytes * 10 / dev.baud * 1.5 + 5)
    end = dev.readline(3)
    if not end or not end.startswith("#END"):
        raise hwtest.DeviceError(f"bad trailer: {end!r}")
    codes = np.frombuffer(raw, dtype="<u2").reshape(n, nch).copy()
    if (int(codes.astype(np.uint64).sum()) & 0xFFFFFFFF) != int(end.split()[1], 16):
        raise hwtest.DeviceError("checksum mismatch")
    return codes, fs


def main():
    dev = hwtest.Device("COM16", 115200)
    dev.drain(0.4)
    d = hwtest.new_record_dir("p4run3")
    started = False
    armed = False
    rows = []
    abort = None
    codes = None
    fs = None
    e0 = r0 = b0 = None
    try:
        st = dev.anc_status()
        if st.get("on") != "0":
            dev.cmd("ANC OFF")
            st = dev.anc_status()
        if st.get("shat") != "1":
            coeff, _ = ac.shat_eff()
            anc_p4.upload(dev, coeff)
            st = dev.anc_status()
        if st.get("on") != "0":
            raise SystemExit(f"ANC 已经开着: {st}")
        e0, r0, b0 = fnum(st, "e200"), fnum(st, "r200"), fnum(st, "baseline")
        ratio = e0 / r0 if r0 > 0 else float("inf")
        print(f"pre e200={e0:.2f} r200={r0:.2f} ratio={ratio:.2f} baseline={b0:.2f} gate={2 * b0:.2f}")
        if not (ERR_LO <= e0 <= ERR_HI and REF_LO <= r0 <= REF_HI and RATIO_LO <= ratio <= RATIO_HI):
            raise SystemExit("预检没过，不开 ANC")
        dev.cmd("MU 0.002")
        dev.cmd("GAMMA 0")
        dev.cmd("CLAMP 40")
        chk = dev.anc_status()
        if abs(fnum(chk, "mu") - 0.002) > 1e-4 or abs(fnum(chk, "gamma")) > 1e-6 or abs(fnum(chk, "clamp") - 40) > 0.1:
            raise SystemExit(f"参数没有停在指定值: {chk}")
        print(dev.cmd(f"CARM {FRAMES} D"))
        armed = True
        print(dev.cmd("ANC ON", timeout=3))
        started = True
        t0 = time.perf_counter()
        mark = None
        up_since = None
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
                  f"wnorm={fnum(st,'wnorm'):7.4f} clamp_n={c} prot={st.get('prot')} cpu={st.get('cpu')} "
                  f"slip={st.get('slip')} late={st.get('late')}")
            if st.get("prot") == "1":
                abort = "保护触发"
                break
            if t >= 1.5:
                if mark is None:
                    mark = c
                elif c > mark:
                    abort = "1.5 s 之后限幅仍在增加"
                    break
            if e > e0 * 1.02:
                if up_since is None:
                    up_since = t
                elif t - up_since > 2.0:
                    abort = "误差麦 200 Hz 升高并持续超过 2 s"
                    break
            else:
                up_since = None
            if t >= 3.0:
                break
            next_t += 0.25
    finally:
        if started:
            try:
                print(dev.cmd("ANC OFF", timeout=3))
            except hwtest.DeviceError as ex:
                print("ANC OFF", ex)
        if armed:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                try:
                    codes, fs = cget(dev)
                    print("cap", codes.shape, fs)
                    break
                except hwtest.DeviceError as ex:
                    if "busy" not in str(ex):
                        print("CGET", ex)
                        break
                    time.sleep(0.1)
        end = None
        try:
            end = dev.anc_status()
            print("end", {k: end.get(k) for k in ("on", "prot", "e200", "baseline", "clamp_n", "slip", "late", "ovr", "udr")})
        except hwtest.DeviceError as ex:
            print("status", ex)
        dev.close()
        write(d, e0, r0, b0, rows, abort, codes, fs, end)
        print("saved", d, "abort", abort)


def write(d, e0, r0, b0, rows, abort, codes, fs, end):
    lines = ["# μ=0.002 的 3 s", ""]
    if e0 is None:
        lines.append("预检没有完成。")
    else:
        ratio = e0 / r0 if r0 else float("inf")
        lines.append(
            f"开环：误差麦 200 Hz {e0:.2f} mV，参考麦 200 Hz {r0:.2f} mV，比值 {ratio:.2f}，"
            f"基线 {b0:.2f} mV，保护门槛 {2 * b0:.2f} mV。"
        )
    lines.append("")
    if abort:
        lines.append(f"提前关掉：{abort}。没有重试，参数没有改。")
    elif rows:
        lines.append("跑满约 3 s 后 `ANC OFF`。前 1.5 s 的限幅不作为停止条件。")
    else:
        lines.append("没有打开 ANC。")
    if rows:
        lines += [
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
        es = np.asarray(es, float)
        ts = np.asarray(ts, float)
        if e0 and len(es):
            tail = es[ts >= max(0.0, ts[-1] - 0.75)]
            steady = float(np.median(tail))
            db = 20 * np.log10(e0 / steady) if steady > 0 else float("nan")
            hold = None
            for i, t in enumerate(ts):
                if es[i] > e0 * 10 ** (-3 / 20):
                    continue
                m = (ts >= t) & (ts <= t + 0.5)
                if m.sum() >= 2 and np.all(es[m] <= e0 * 10 ** (-3 / 20)):
                    hold = float(t)
                    break
            lines += [
                "",
                f"末段中位数 {steady:.2f} mV，相对开前 {e0:.2f} mV "
                f"{'低' if db > 0 else '高'} {abs(db):.1f} dB。",
                f"比开前低 3 dB 并在随后 0.5 s 里保持："
                f"{('约 %.2f s' % hold) if hold is not None else '这段记录里没有'}。",
                f"末次 prot={rows[-1][1].get('prot')} clamp_n={rows[-1][3]} "
                f"cpu={rows[-1][1].get('cpu')} slip={rows[-1][1].get('slip')} "
                f"late={rows[-1][1].get('late')} ovr={rows[-1][1].get('ovr')} udr={rows[-1][1].get('udr')}。",
            ]
    if codes is not None:
        y = codes[:, 2].astype(np.float64) - 2048.0
        np.savez(d / "cap.npz", codes=codes, fs=fs)
        hit = np.where(np.abs(y) >= 39.5)[0]
        lines += ["", f"CAP {len(y)} 帧，{len(y) / fs:.3f} s，含 DAC。碰到 ±40 的样本 {len(hit)} 个。"]
        if len(hit):
            edges = [0, 0.2, 0.5, 1.0, len(y) / fs]
            for a, b in zip(edges[:-1], edges[1:]):
                n = int(np.sum((hit >= a * fs) & (hit < b * fs)))
                lines.append(f"- {a:.1f}–{b:.1f} s：{n} 个")
            lines.append(f"第一个在 {hit[0] / fs * 1000:.0f} ms，最后一个在 {hit[-1] / fs * 1000:.0f} ms。")
        lines.append("")
        lines.append("| 频率 | 整段 y rms / 码 | 0.5 s 之后 |")
        lines.append("| ---: | ---: | ---: |")
        for f0 in LINES:
            whole = an.sine_fit(y, fs, f0)[0]
            late = an.sine_fit(y[int(0.5 * fs):], fs, f0)[0] if len(y) > int(0.5 * fs) else float("nan")
            lines.append(f"| {f0:.0f} Hz | {whole:.2f} | {late:.2f} |")
    if end:
        lines += ["", f"关掉之后：on={end.get('on')} prot={end.get('prot')} e200={end.get('e200')} baseline={end.get('baseline')}。"]
    lines.append("")
    (d / "report.md").write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
