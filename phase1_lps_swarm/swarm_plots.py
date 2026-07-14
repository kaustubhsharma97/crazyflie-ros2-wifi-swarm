#!/usr/bin/env python3
"""
swarm_plots.py — detailed two-drone swarm visualisation
=======================================================
Kaustubh Sharma | IIIT-Delhi | Crazyflie 2.1+ x2 | TDoA2 LPS

Reads a swarm CSV (columns: time, <drone>_x/y/z, <drone>_ex/ey/ez for each
drone, optional separation_xy) and produces a detailed multi-panel PNG showing
BOTH drones together:

  Panel 1: XY top-down — both drones' paths (different colors) + safe zone
  Panel 2: altitude (Z) vs time for both drones
  Panel 3: inter-drone separation vs time (with safety floor line)
  Panel 4: per-drone 3D tracking error vs time

Usage:
  python3 swarm_plots.py ~/swarm_formation_log.csv
  python3 swarm_plots.py ~/swarm_leader_follower_log.csv --drones cf231 cf2 --title "Leader-Follower"
"""

import sys, csv, math, os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SAFE_X = (0.0, 3.0); SAFE_Y = (1.0, 4.0)
COLORS = ["tab:blue", "tab:red", "tab:green", "tab:purple"]


def load(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            try:
                rows.append({k: float(v) for k, v in r.items() if v not in ("", None)})
            except ValueError:
                pass
    return rows


def detect_drones(rows):
    if not rows: return []
    drones = []
    for k in rows[0].keys():
        if k.endswith("_x") and not k.endswith("ex"):
            d = k[:-2]
            if d not in drones and d != "separation":
                drones.append(d)
    return drones


def plot_swarm(path, drones=None, title=None, min_sep=0.5):
    rows = load(path)
    if not rows:
        print("no data"); return
    if drones is None:
        drones = detect_drones(rows)
    if title is None:
        title = os.path.basename(path).replace("_log.csv", "")

    t = np.array([r["time"] for r in rows])

    fig, axs = plt.subplots(2, 2, figsize=(14, 11))
    fig.suptitle(f"{title} — Two-Drone Swarm Analysis", fontsize=15, fontweight="bold")

    # ── Panel 1: XY top-down, both paths ──
    p = axs[0, 0]
    bx = [SAFE_X[0], SAFE_X[1], SAFE_X[1], SAFE_X[0], SAFE_X[0]]
    by = [SAFE_Y[0], SAFE_Y[0], SAFE_Y[1], SAFE_Y[1], SAFE_Y[0]]
    p.plot(bx, by, "r--", alpha=0.5, linewidth=1.0, label="Safe zone")
    for i, d in enumerate(drones):
        xs = [r.get(f"{d}_x") for r in rows if r.get(f"{d}_x") is not None]
        ys = [r.get(f"{d}_y") for r in rows if r.get(f"{d}_y") is not None]
        c = COLORS[i % len(COLORS)]
        p.plot(xs, ys, "-", color=c, alpha=0.4, linewidth=0.8)
        p.scatter(xs, ys, s=8, color=c, alpha=0.7, label=d)
        if xs: p.plot(xs[0], ys[0], "o", color=c, markersize=10, markeredgecolor="k")
    p.set_title("XY top-down — both drone paths")
    p.set_xlabel("X (m)"); p.set_ylabel("Y (m)")
    p.legend(fontsize=8); p.grid(True, alpha=0.3); p.axis("equal")

    # ── Panel 2: altitude vs time ──
    p = axs[0, 1]
    for i, d in enumerate(drones):
        zs = [r.get(f"{d}_z") for r in rows]
        ez = [r.get(f"{d}_ez") for r in rows]
        c = COLORS[i % len(COLORS)]
        p.plot(t, zs, "-", color=c, linewidth=1.2, label=f"{d} actual")
        if any(e is not None for e in ez):
            p.plot(t, ez, "--", color=c, alpha=0.5, linewidth=1.0, label=f"{d} target")
    p.axhline(0, color="k", linewidth=0.6, alpha=0.5)
    p.set_title("Altitude (Z) vs time")
    p.set_xlabel("Time (s)"); p.set_ylabel("Z (m)")
    p.legend(fontsize=8); p.grid(True, alpha=0.3)

    # ── Panel 3: inter-drone separation ──
    p = axs[1, 0]
    if len(drones) >= 2:
        d1, d2 = drones[0], drones[1]
        seps = []
        for r in rows:
            if all(r.get(f"{d}_{a}") is not None for d in (d1, d2) for a in ("x", "y")):
                seps.append(math.hypot(r[f"{d1}_x"]-r[f"{d2}_x"],
                                       r[f"{d1}_y"]-r[f"{d2}_y"]))
            else:
                seps.append(np.nan)
        p.plot(t, seps, color="purple", linewidth=1.3, label="separation (XY)")
        p.axhline(min_sep, color="red", linestyle=":", linewidth=1.5,
                  label=f"min safe {min_sep}m")
        valid = [s for s in seps if not math.isnan(s)]
        if valid:
            p.axhline(np.mean(valid), color="orange", linestyle="--", linewidth=1.0,
                      label=f"mean {np.mean(valid):.2f}m")
        p.set_title("Inter-drone separation vs time")
        p.set_xlabel("Time (s)"); p.set_ylabel("Separation (m)")
        p.legend(fontsize=8); p.grid(True, alpha=0.3)
    else:
        p.text(0.5, 0.5, "need 2 drones for separation", ha="center")

    # ── Panel 4: per-drone 3D tracking error ──
    p = axs[1, 1]
    for i, d in enumerate(drones):
        errs = []
        for r in rows:
            if all(r.get(f"{d}_{a}") is not None for a in ("x","y","z","ex","ey","ez")):
                e = math.sqrt((r[f"{d}_x"]-r[f"{d}_ex"])**2 +
                              (r[f"{d}_y"]-r[f"{d}_ey"])**2 +
                              (r[f"{d}_z"]-r[f"{d}_ez"])**2)
                errs.append(e*100)
            else:
                errs.append(np.nan)
        c = COLORS[i % len(COLORS)]
        p.plot(t, errs, color=c, linewidth=1.1, label=f"{d} 3D err")
    p.set_title("Per-drone 3D tracking error vs time")
    p.set_xlabel("Time (s)"); p.set_ylabel("Error (cm)")
    p.legend(fontsize=8); p.grid(True, alpha=0.3)

    out = path.replace("_log.csv", "_swarm_analysis.png")
    if out == path: out = path + "_swarm_analysis.png"
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(out, dpi=120); plt.close()
    print(f"saved -> {out}")
    return out


if __name__ == "__main__":
    args = sys.argv[1:]
    path = None; drones = None; title = None
    i = 0
    while i < len(args):
        if args[i] == "--drones":
            drones = []; i += 1
            while i < len(args) and not args[i].startswith("--"):
                drones.append(args[i]); i += 1
        elif args[i] == "--title":
            title = args[i+1]; i += 2
        else:
            path = os.path.expanduser(args[i]); i += 1
    if not path:
        print("usage: python3 swarm_plots.py <swarm_log.csv> [--drones cf231 cf2] [--title T]")
    else:
        plot_swarm(path, drones=drones, title=title)


def auto_plot(csv_path, title=None, min_sep=0.5):
    """Convenience entry point for trajectory scripts to call after saving CSV.
    Safe: never raises into the caller (prints a message on failure)."""
    try:
        return plot_swarm(csv_path, drones=None, title=title, min_sep=min_sep)
    except Exception as e:
        print(f"[swarm_plots] PNG generation skipped: {e}")
        return None


def plot_follow_me(path, title="Follow-Me"):
    """Dedicated plot for follow_me.py logs (follower + tag + distance)."""
    rows = load(path)
    if not rows:
        print("no data"); return None
    t = np.array([r["time"] for r in rows])

    fig, axs = plt.subplots(2, 2, figsize=(14, 11))
    fig.suptitle(f"{title} — Drone Follows Person (held tag)", fontsize=15, fontweight="bold")

    # Panel 1: XY paths — follower drone vs tag (you)
    p = axs[0, 0]
    fx = [r.get("follower_x") for r in rows if r.get("follower_x") is not None]
    fy = [r.get("follower_y") for r in rows if r.get("follower_y") is not None]
    tx = [r.get("tag_x") for r in rows if r.get("tag_x") is not None]
    ty = [r.get("tag_y") for r in rows if r.get("tag_y") is not None]
    bx = [SAFE_X[0], SAFE_X[1], SAFE_X[1], SAFE_X[0], SAFE_X[0]]
    by = [SAFE_Y[0], SAFE_Y[0], SAFE_Y[1], SAFE_Y[1], SAFE_Y[0]]
    p.plot(bx, by, "r--", alpha=0.5, linewidth=1.0, label="Safe zone")
    p.plot(tx, ty, "-", color="tab:green", alpha=0.5, linewidth=1.0)
    p.scatter(tx, ty, s=10, color="tab:green", alpha=0.7, label="tag (you)")
    p.plot(fx, fy, "-", color="tab:blue", alpha=0.5, linewidth=1.0)
    p.scatter(fx, fy, s=10, color="tab:blue", alpha=0.7, label="follower drone")
    p.set_title("XY top-down — drone follows you"); p.set_xlabel("X (m)"); p.set_ylabel("Y (m)")
    p.legend(fontsize=8); p.grid(True, alpha=0.3); p.axis("equal")

    # Panel 2: altitude
    p = axs[0, 1]
    fz = [r.get("follower_z") for r in rows]
    tz = [r.get("tag_z") for r in rows]
    p.plot(t, fz, color="tab:blue", linewidth=1.2, label="follower drone Z")
    p.plot(t, tz, color="tab:green", linewidth=1.0, alpha=0.7, label="tag Z (your hand)")
    p.axhline(0, color="k", linewidth=0.6, alpha=0.5)
    p.set_title("Altitude vs time"); p.set_xlabel("Time (s)"); p.set_ylabel("Z (m)")
    p.legend(fontsize=8); p.grid(True, alpha=0.3)

    # Panel 3: follow distance (the key safety metric)
    p = axs[1, 0]
    dist = [r.get("distance") for r in rows]
    p.plot(t, dist, color="purple", linewidth=1.3, label="follow distance")
    valid = [d for d in dist if d is not None]
    if valid:
        p.axhline(np.mean(valid), color="orange", linestyle="--", linewidth=1.0,
                  label=f"mean {np.mean(valid):.2f}m")
    p.set_title("Follow distance vs time"); p.set_xlabel("Time (s)"); p.set_ylabel("Distance (m)")
    p.legend(fontsize=8); p.grid(True, alpha=0.3)

    # Panel 4: distance distribution
    p = axs[1, 1]
    if valid:
        p.hist(valid, bins=25, color="mediumpurple", edgecolor="white")
        p.axvline(np.mean(valid), color="orange", linestyle="--", label=f"mean {np.mean(valid):.2f}m")
        p.legend(fontsize=8)
    p.set_title("Follow distance distribution"); p.set_xlabel("Distance (m)"); p.set_ylabel("Samples")
    p.grid(True, alpha=0.3)

    out = path.replace("_log.csv", "_analysis.png")
    if out == path: out = path + "_analysis.png"
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.savefig(out, dpi=120); plt.close()
    print(f"saved -> {out}")
    return out


def auto_plot_follow(csv_path, title="Follow-Me"):
    try:
        return plot_follow_me(csv_path, title=title)
    except Exception as e:
        print(f"[swarm_plots] follow-me PNG skipped: {e}")
        return None
