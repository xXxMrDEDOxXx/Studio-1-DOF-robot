#!/usr/bin/env python3
"""
Lab 4 Control Integration - analysis toolkit (1-DOF, STM32G474RE).

CSV schema (SI units, see docs/lab4_plan.md):
    t,ref_q,q,ref_qd,qd,ref_qdd,V,i_est
    t[s]  ref_q,q[rad]  ref_qd,qd[rad/s]  ref_qdd[rad/s^2]  V[V]  i_est[A]

Usage:
    python lab4_analysis.py step    E11_step90.csv
    python lab4_analysis.py track   E2_ff_on.csv
    python lab4_analysis.py effort  E8_scurve.csv
    python lab4_analysis.py compare E8_trapz.csv E8_scurve.csv
    python lab4_analysis.py paramid E1_t1.csv E1_t2.csv ...   # coast-down -> damping tau

Add --plot to save a PNG next to the first input file.
Requires: numpy, scipy, matplotlib
"""
import sys, argparse, os
import numpy as np

RAD2DEG = 180.0 / np.pi
SPEC_TOL_DEG = 0.10          # studio accuracy spec
SPEC_PO = 1.0                # % overshoot
SPEC_TS = 0.5                # s settling


def load(path):
    """Load CSV with a header row matching the schema. Missing columns -> NaN."""
    import csv
    cols = {}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        names = r.fieldnames
        for n in names:
            cols[n.strip()] = []
        for row in r:
            for n in names:
                try:
                    cols[n.strip()].append(float(row[n]))
                except (ValueError, TypeError):
                    cols[n.strip()].append(np.nan)
    return {k: np.asarray(v, float) for k, v in cols.items()}


# ----------------------------------------------------------------------------- metrics
def step_metrics(d, tol_deg=SPEC_TOL_DEG):
    t, q = d["t"], d["q"]
    ref = d.get("ref_q", q)
    q0 = ref[0]
    q_ss = ref[-1]                      # commanded final position
    step = q_ss - q0
    if abs(step) < 1e-9:
        return {"note": "no step (ref start==end)"}
    # overshoot beyond final, in the direction of motion
    if step > 0:
        peak = np.nanmax(q); over = (peak - q_ss) / step * 100
    else:
        peak = np.nanmin(q); over = (q_ss - peak) / (-step) * 100
    over = max(over, 0.0)
    # settling: last time outside band, then time to re-enter for good
    band2 = 0.02 * abs(step)            # 2% band
    bandtol = tol_deg / RAD2DEG         # absolute spec band
    def settle(band):
        outside = np.where(np.abs(q - q_ss) > band)[0]
        if len(outside) == 0:
            return 0.0
        return t[min(outside[-1] + 1, len(t) - 1)] - t[0]
    # rise time 10->90% of step
    lo, hi = q0 + 0.1 * step, q0 + 0.9 * step
    def cross(level):
        idx = np.where((q[:-1] - level) * (q[1:] - level) <= 0)[0]
        return t[idx[0]] if len(idx) else np.nan
    tr = cross(hi) - cross(lo)
    ss_err = q[-1] - q_ss
    return {
        "step_deg": step * RAD2DEG,
        "overshoot_%": over,
        "settling_2%_s": settle(band2),
        f"settling_{tol_deg}deg_s": settle(bandtol),
        "rise_10_90_s": tr,
        "ss_error_deg": ss_err * RAD2DEG,
        "PASS_PO": over < SPEC_PO,
        "PASS_settle": settle(bandtol) < SPEC_TS,
        "PASS_acc": abs(ss_err) * RAD2DEG <= tol_deg,
    }


def track_metrics(d):
    e = d["ref_q"] - d["q"]
    e = e[~np.isnan(e)]
    rms = float(np.sqrt(np.mean(e ** 2)))
    mae = float(np.mean(np.abs(e)))
    return {"RMS_err_deg": rms * RAD2DEG, "MAE_deg": mae * RAD2DEG,
            "RMS_err_rad": rms, "MAE_rad": mae}


def effort_metrics(d):
    t, V = d["t"], d["V"]
    m = ~np.isnan(V)
    t, V = t[m], V[m]
    out = {"int_absV_Vs": float(np.trapz(np.abs(V), t)),
           "int_V2_V2s": float(np.trapz(V ** 2, t)),
           "peak_V": float(np.nanmax(np.abs(V)))}
    if "ref_qdd" in d and not np.all(np.isnan(d["ref_qdd"])):
        a = d["ref_qdd"]; tt = d["t"]
        jerk = np.gradient(a, tt)
        out["max_jerk_rad_s3"] = float(np.nanmax(np.abs(jerk)))
        out["rms_jerk_rad_s3"] = float(np.sqrt(np.nanmean(jerk ** 2)))
    return out


def paramid(files):
    """Coast-down damping: fit qd(t)=qd0*exp(-t/tau) on the decay -> tau=J/b per trial."""
    from scipy.optimize import curve_fit
    from scipy import stats
    f = lambda t, a, tau: a * np.exp(-t / tau)
    taus, resids = [], []
    for p in files:
        d = load(p)
        t, qd = d["t"], np.abs(d["qd"])
        i0 = int(np.nanargmax(qd))            # start at peak speed (drive cut)
        t, qd = t[i0:] - t[i0], qd[i0:]
        m = qd > 0.02 * qd[0]                  # ignore near-zero tail (noise)
        t, qd = t[m], qd[m]
        try:
            (a, tau), _ = curve_fit(f, t, qd, p0=[qd[0], 0.2], maxfev=10000)
        except Exception as e:
            print(f"  fit failed {p}: {e}"); continue
        taus.append(tau)
        resids.append(qd - f(t, a, tau))
        print(f"  {os.path.basename(p)}: tau={tau:.4f}s  (b/J={1/tau:.3f} 1/s)")
    taus = np.array(taus)
    n = len(taus)
    if n < 2:
        return {"n": n, "tau_mean_s": float(taus[0]) if n else None}
    mean, std = taus.mean(), taus.std(ddof=1)
    ci = stats.t.ppf(0.975, n - 1) * std / np.sqrt(n)
    allr = np.concatenate(resids)
    sh = stats.shapiro(allr) if 3 <= len(allr) <= 5000 else (np.nan, np.nan)
    loo = np.sqrt(np.mean([(taus[i] - np.delete(taus, i).mean()) ** 2 for i in range(n)]))
    return {"n": n, "tau_mean_s": mean, "tau_std_s": std, "CI95_s": (mean - ci, mean + ci),
            "residual_normality_p": float(sh[1]), "crossval_LOO_RMSE_s": float(loo)}


# ----------------------------------------------------------------------------- plotting
def _plot(datasets, labels, outpng):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for d, lab in zip(datasets, labels):
        ax[0].plot(d["t"], d["q"] * RAD2DEG, label=f"{lab} q")
        if "ref_q" in d:
            ax[0].plot(d["t"], d["ref_q"] * RAD2DEG, "--", lw=1, label=f"{lab} ref")
        if "qd" in d:  ax[1].plot(d["t"], d["qd"], label=lab)
        if "V" in d:   ax[2].plot(d["t"], d["V"], label=lab)
    ax[0].set_ylabel("position [deg]"); ax[1].set_ylabel("velocity [rad/s]")
    ax[2].set_ylabel("voltage [V]");    ax[2].set_xlabel("time [s]")
    for a in ax: a.grid(True, alpha=0.3); a.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(outpng, dpi=130)
    print(f"  saved plot -> {outpng}")


def show(title, m):
    print(f"\n== {title} ==")
    for k, v in m.items():
        if isinstance(v, float):
            print(f"  {k:24s}: {v:.4f}")
        else:
            print(f"  {k:24s}: {v}")


# ----------------------------------------------------------------------------- cli
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["step", "track", "effort", "compare", "paramid"])
    ap.add_argument("files", nargs="+")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()

    if a.cmd == "paramid":
        show("Param ID (coast-down damping)", paramid(a.files))
        return

    if a.cmd == "compare":
        ds = [load(f) for f in a.files]
        for f, d in zip(a.files, ds):
            show(f"{os.path.basename(f)} step", step_metrics(d))
            show(f"{os.path.basename(f)} track", track_metrics(d))
            show(f"{os.path.basename(f)} effort", effort_metrics(d))
        if a.plot:
            _plot(ds, [os.path.basename(f) for f in a.files],
                  os.path.splitext(a.files[0])[0] + "_compare.png")
        return

    d = load(a.files[0])
    if a.cmd == "step":   show("Step response", step_metrics(d))
    if a.cmd == "track":  show("Tracking", track_metrics(d))
    if a.cmd == "effort": show("Control effort", effort_metrics(d))
    if a.plot:
        _plot([d], [os.path.basename(a.files[0])],
              os.path.splitext(a.files[0])[0] + ".png")


if __name__ == "__main__":
    main()
