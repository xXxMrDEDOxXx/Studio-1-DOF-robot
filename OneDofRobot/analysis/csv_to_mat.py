#!/usr/bin/env python3
"""
CSV (lab4 schema) -> .mat  สำหรับ Simulink / MATLAB parameter estimation (Lab 1).

  schema: t,ref_q,q,ref_qd,qd,ref_qdd,V,i_est   (SI; ref_qd ว่างได้)

Usage:
  python csv_to_mat.py logs/E1_t1.csv [logs/E1_t2.csv ...]
  python csv_to_mat.py logs/*.csv

สร้าง .mat ข้างๆ ไฟล์ CSV แต่ละไฟล์ — ตัวแปรใน .mat (column vectors):
  t, ref_q, q, ref_qd, qd, ref_qdd, V, i_est   + Ts, fs
  u_V  (=V, input)   y_qd (=qd, output)   y_q (=q, output)

Requires: numpy, scipy
"""
import sys, os, csv
import numpy as np
from scipy.io import savemat

COLS = ["t", "ref_q", "q", "ref_qd", "qd", "ref_qdd", "V", "i_est"]


def load(path):
    d = {c: [] for c in COLS}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        hdr = [n.strip() for n in (r.fieldnames or [])]
        for row in r:
            rr = {k.strip(): v for k, v in row.items()}
            for c in COLS:
                if c in hdr:
                    try:
                        d[c].append(float(rr[c]))
                    except (ValueError, TypeError):
                        d[c].append(np.nan)
    return {c: np.asarray(v, float).reshape(-1, 1) for c, v in d.items() if v}


def convert(path):
    d = load(path)
    if "t" not in d or len(d["t"]) < 2:
        print(f"  skip {path}: ไม่มีคอลัมน์ t / ข้อมูลน้อยเกิน"); return None
    t  = d["t"][:, 0]
    Ts = float(np.median(np.diff(t)))
    m = dict(d)
    m["Ts"] = Ts
    m["fs"] = 1.0 / Ts if Ts > 0 else 0.0
    if "V"  in d: m["u_V"]  = d["V"]
    if "qd" in d: m["y_qd"] = d["qd"]
    if "q"  in d: m["y_q"]  = d["q"]
    out = os.path.splitext(path)[0] + ".mat"
    savemat(out, m, do_compression=True)
    print(f"  {os.path.basename(path)} -> {os.path.basename(out)}  "
          f"(N={len(t)}, Ts={Ts*1e3:.2f} ms, fs={1/Ts:.0f} Hz)")
    return out


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    for p in sys.argv[1:]:
        convert(p)
    print("\n--- MATLAB / Simulink ---")
    print("  S = load('E1_xxx.mat');")
    print("  %% System Identification Toolbox:")
    print("  z = iddata(S.qd, S.V, S.Ts);          % output=velocity, input=voltage")
    print("  %% Simulink Design Optimization (Parameter Estimation):")
    print("  V_ts  = timeseries(S.V,  S.t);")
    print("  qd_ts = timeseries(S.qd, S.t);")
    print("  q_ts  = timeseries(S.q,  S.t);")
