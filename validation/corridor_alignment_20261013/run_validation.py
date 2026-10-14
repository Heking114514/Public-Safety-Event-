#!/usr/bin/env python3
"""Reproduce standalone corridor alignment validation; no ROS nodes or controllers are changed."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def compile_and_run(results):
    command = ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra",
               "-I" + str(ROOT / "src/vehicle_loopback_sim/include"),
               "-I" + str(ROOT / "src/navigation_safety/include"),
               "-I" + str(ROOT / "src/contrast_navigation_core/include"),
               str(HERE / "corridor_alignment.cpp"),
               str(ROOT / "src/vehicle_loopback_sim/src/motion.cpp"),
               str(ROOT / "src/navigation_safety/src/checker.cpp"),
               "-lyaml-cpp", "-o", str(HERE / "corridor_alignment")]
    subprocess.run(command, check=True, cwd=ROOT)
    subprocess.run([str(HERE / "corridor_alignment"), str(ROOT), str(results)], check=True, cwd=ROOT)
    physical_command = list(command)
    physical_command[physical_command.index(str(HERE / "corridor_alignment.cpp"))] = str(HERE / "physical_accel_validation.cpp")
    physical_command[-1] = str(HERE / "physical_accel_validation")
    subprocess.run(physical_command, check=True, cwd=ROOT)
    subprocess.run([str(HERE / "physical_accel_validation"), str(ROOT), str(results)], check=True, cwd=ROOT)


def derivative(state, v, w, d):
    theta = state[2]
    return np.array([v * math.cos(theta) - d * w * math.sin(theta),
                     v * math.sin(theta) + d * w * math.cos(theta), w])


def independent_verification(results, summary):
    """Use independent RK4 ODE integration, not the C++ exact axle integration."""
    max_error, min_clear, samples = 0.0, 1.0, 0
    traces = sorted(results.glob("trace_*.csv"))
    for path in traces:
        trace = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        state = np.array([trace[0]["x_m"], trace[0]["y_m"], trace[0]["yaw_rad"]])
        for previous, row in zip(trace[:-1], trace[1:]):
            dt = row["time_s"] - previous["time_s"]
            h = dt / 100
            v, w = row["linear_mps"], row["angular_rps"]
            for _ in range(100):
                k1 = derivative(state, v, w, summary["base_offset_x_m"])
                k2 = derivative(state + .5*h*k1, v, w, summary["base_offset_x_m"])
                k3 = derivative(state + .5*h*k2, v, w, summary["base_offset_x_m"])
                k4 = derivative(state + h*k3, v, w, summary["base_offset_x_m"])
                state += h*(k1 + 2*k2 + 2*k3 + k4)/6
                extent = (.125/2 + .002) * (abs(math.sin(state[2])) + abs(math.cos(state[2])))
                min_clear = min(min_clear, .1-abs(state[1])-extent)
                samples += 1
            target = np.array([row["x_m"], row["y_m"], row["yaw_rad"]])
            max_error = max(max_error, float(np.max(np.abs(state-target))))
        assert abs(state[2]) <= math.radians(3) + 1e-9, path
        assert all(abs(row["linear_mps"]) < 1e-12 and abs(row["angular_rps"]) < 1e-12 for row in trace[-7:]), path
    assert max_error < 1e-7, max_error
    assert min_clear >= -1e-8, min_clear
    result = dict(passed=True, traces=len(traces), independent_rk4_substeps=samples,
                  max_endpoint_state_error=max_error, min_sampled_clearance_after_margin_m=min_clear,
                  substeps_per_control_interval=100, method="RK4 of base_link differential equations")
    (results / "independent_verification.json").write_text(json.dumps(result, indent=2)+"\n")
    return result


def plots(results, rows):
    grid = [r for r in rows if r["suite"] == "grid"]
    angles = sorted(set(float(r["angle_deg"]) for r in grid))
    fractions = sorted(set(float(r["lateral_fraction"]) for r in grid))
    ai, fi = {v:i for i,v in enumerate(angles)}, {v:i for i,v in enumerate(fractions)}
    baseline = np.zeros((len(fractions), len(angles)))
    grouped = {a:[] for a in angles}
    for row in grid:
        angle = float(row["angle_deg"])
        baseline[fi[float(row["lateral_fraction"])]][ai[angle]] = int(row["baseline_pass"])
        grouped[angle].append(row)
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.3), constrained_layout=True)
    theta = np.radians(angles)
    legal = 100 - 64.5*(np.abs(np.sin(theta)) + np.abs(np.cos(theta)))
    ax = axes[0, 0]
    ax.fill_between(angles, -legal, legal, color="#dbeafe", label="Initially legal centre y")
    moving = [a for a in angles if abs(a)>3]
    lows = [float(grouped[a][0]["turn_start_lower_m"])*1000 for a in moving]
    highs = [float(grouped[a][0]["turn_start_upper_m"])*1000 for a in moving]
    # Plot each side separately so the stationary goal band is not interpolated.
    for sign in (-1, 1):
        idx = [i for i,a in enumerate(moving) if a*sign>0]
        ax.fill_between([moving[i] for i in idx], [lows[i] for i in idx], [highs[i] for i in idx], color="#60a5fa", alpha=.75, label="Safe arc start (1 mm reserve)" if sign==1 else None)
    ax.set(xlabel="Initial heading error (deg)", ylabel="Lateral centre offset (mm)", title="Legal positions and safe arc starting positions")
    ax.legend(fontsize=8);ax.grid(alpha=.2)
    ax=axes[0,1]
    from matplotlib.colors import ListedColormap
    ax.imshow(baseline, origin="lower", aspect="auto", extent=[-40,40,-1,1], cmap=ListedColormap(["#ef4444","#d1fae5"]), vmin=0,vmax=1, interpolation="nearest")
    ax.set(xlabel="Initial heading error (deg)", ylabel="Fraction of legal lateral offset", title="Arc alone: red fails; with pre-adjustment: all pass")
    ax=axes[1,0]
    for name, color in [("trace_a40_f1.csv", "#2563eb"), ("trace_a-40_f-1.csv", "#d97706")]:
        trace=np.genfromtxt(results/name, delimiter=",", names=True, dtype=None, encoding="utf-8")
        ax.plot(trace["x_m"]*1000, trace["y_m"]*1000, color=color, lw=2, label=name.replace("trace_", "").replace(".csv", ""))
        for index in sorted({0,len(trace)//2,len(trace)-1}):
            row=trace[index];th=row["yaw_rad"];co,si=math.cos(th),math.sin(th)
            corners=np.array([[row["x_m"]+sx*.0645*co-sy*.0645*si,row["y_m"]+sx*.0645*si+sy*.0645*co] for sx,sy in [(-1,-1),(1,-1),(1,1),(-1,1)]])*1000
            ax.add_patch(Polygon(corners, fill=False, edgecolor=color, alpha=.45, lw=1))
    ax.axhline(100,color="black");ax.axhline(-100,color="black")
    ax.set(xlim=(-100,150),ylim=(-112,112),xlabel="Longitudinal x (mm)",ylabel="Lateral y (mm)",title="Near-wall examples; outlines include safety margin")
    ax.set_aspect("equal",adjustable="box");ax.legend(fontsize=8);ax.grid(alpha=.2)
    ax=axes[1,1]
    ax.plot(angles,[max(float(r["max_base_displacement_x_m"]) for r in grouped[a])*100 for a in angles],label="Max centre displacement")
    ax.plot(angles,[max(max(-float(r["footprint_xmin_m"]),float(r["footprint_xmax_m"])) for r in grouped[a])*100 for a in angles],label="Max inflated corner extent")
    ax.axhline(20, color="#dc2626",ls="--",label="Available each direction: 20 cm")
    ax.set(xlabel="Initial heading error (deg)",ylabel="Absolute longitudinal extent (cm)",ylim=(0,22),title="Longitudinal space stays within the 40 cm window")
    ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.savefig(results / "validation_overview.png", dpi=170)
    plt.close(fig)


def manifest(results):
    files = ["src/vehicle_config/config/vehicle_physical.yaml", "src/visual_navigation/config/waypoint_navigation.yaml",
             "src/contrast_controller/config/contrast_controller.yaml", "src/vehicle_loopback_sim/config/vehicle_loopback.yaml",
             "src/vehicle_loopback_sim/src/motion.cpp", "src/navigation_safety/src/checker.cpp",
             "src/contrast_navigation_core/include/visual_navigation/rate_limiter.hpp", "src/contrast_navigation_core/src/output.cpp"]
    data = {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in files}
    (results/"input_manifest.json").write_text(json.dumps(data,indent=2)+"\n")


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--verify-only",action="store_true")
    parser.add_argument("--results",type=Path,default=HERE/"results")
    args=parser.parse_args();results=args.results.resolve();results.mkdir(parents=True,exist_ok=True)
    if not args.verify_only:compile_and_run(results)
    summary=json.loads((results/"summary.json").read_text())
    cert=json.loads((results/"continuous_certificate.json").read_text())
    physical=json.loads((results/"physical_accel_summary.json").read_text())
    assert summary["failed"]==0 and cert["certified"] and physical["failed"]==0
    with (results/"cases.csv").open() as stream:rows=list(csv.DictReader(stream))
    assert len(rows)==summary["cases"]
    verified=independent_verification(results,summary)
    plots(results,rows);manifest(results)
    print(json.dumps({"simulation":summary,"physical_accel_model":physical,"continuous_certificate":cert,"independent_verification":verified},indent=2))

if __name__=="__main__":main()
