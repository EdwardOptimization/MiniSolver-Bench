#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
CACHE_ROOT = ROOT / ".cache" / "sources"
ASSET_ROOT = ROOT / "assets" / "reference"
CATALOG_PATH = ROOT / "assets" / "catalog.json"


def run_command(cmd: list[str], cwd: Path | None = None) -> None:
    print("+", " ".join(str(part) for part in cmd))
    subprocess.run(cmd, cwd=cwd, check=True)


def ensure_repo(name: str, url: str) -> Path:
    CACHE_ROOT.mkdir(parents=True, exist_ok=True)
    repo_dir = CACHE_ROOT / name
    if (repo_dir / ".git").exists():
        run_command(["git", "-C", str(repo_dir), "fetch", "--depth", "1", "origin", "HEAD"])
        run_command(["git", "-C", str(repo_dir), "reset", "--hard", "FETCH_HEAD"])
        return repo_dir
    if repo_dir.exists():
        shutil.rmtree(repo_dir)
    run_command(["git", "clone", "--depth", "1", url, str(repo_dir)])
    return repo_dir


def git_commit(repo_dir: Path) -> str:
    return subprocess.check_output(
        ["git", "-C", str(repo_dir), "rev-parse", "HEAD"],
        text=True,
    ).strip()


def cumulative_arclength(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    dx = np.diff(x)
    dy = np.diff(y)
    ds = np.sqrt(dx * dx + dy * dy)
    return np.concatenate(([0.0], np.cumsum(ds)))


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, record: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def import_mpcc_track(catalog: list[dict[str, object]]) -> None:
    repo_dir = ensure_repo("mpcc", "https://github.com/alexliniger/MPCC")
    commit = git_commit(repo_dir)
    track_data = json.loads((repo_dir / "C++" / "Params" / "track.json").read_text(encoding="utf-8"))
    x_center = np.asarray(track_data["X"], dtype=float)
    y_center = np.asarray(track_data["Y"], dtype=float)
    x_inner = np.asarray(track_data["X_i"], dtype=float)
    y_inner = np.asarray(track_data["Y_i"], dtype=float)
    x_outer = np.asarray(track_data["X_o"], dtype=float)
    y_outer = np.asarray(track_data["Y_o"], dtype=float)
    s = cumulative_arclength(x_center, y_center)
    w_inner = np.sqrt((x_center - x_inner) ** 2 + (y_center - y_inner) ** 2)
    w_outer = np.sqrt((x_center - x_outer) ** 2 + (y_center - y_outer) ** 2)

    rows = []
    for i in range(len(x_center)):
        rows.append(
            {
                "index": i,
                "s": f"{s[i]:.17g}",
                "x_center": f"{x_center[i]:.17g}",
                "y_center": f"{y_center[i]:.17g}",
                "x_inner": f"{x_inner[i]:.17g}",
                "y_inner": f"{y_inner[i]:.17g}",
                "x_outer": f"{x_outer[i]:.17g}",
                "y_outer": f"{y_outer[i]:.17g}",
                "width_inner": f"{w_inner[i]:.17g}",
                "width_outer": f"{w_outer[i]:.17g}",
            }
        )

    csv_path = ASSET_ROOT / "driving" / "mpcc_track.csv"
    meta_path = csv_path.with_suffix(".json")
    write_csv(
        csv_path,
        ["index", "s", "x_center", "y_center", "x_inner", "y_inner", "x_outer", "y_outer", "width_inner", "width_outer"],
        rows,
    )
    meta = {
        "name": "mpcc_track",
        "domain": "autonomous_driving",
        "kind": "closed_track",
        "schema": "driving_track_center_bounds_v1",
        "file": str(csv_path.relative_to(ROOT)),
        "points": len(rows),
        "source_repo": "https://github.com/alexliniger/MPCC",
        "source_commit": commit,
        "source_path": "C++/Params/track.json",
        "description": "ETH MPCC closed track with centerline and inner/outer boundaries.",
    }
    write_json(meta_path, meta)
    catalog.append(meta)


def import_nonlinear_mpcc(track_name: str, subdir: str, catalog: list[dict[str, object]]) -> None:
    repo_dir = ensure_repo(
        "nonlinear_mpcc_for_autonomous_racing",
        "https://github.com/nirajbasnet/Nonlinear_MPCC_for_autonomous_racing",
    )
    commit = git_commit(repo_dir)
    base = repo_dir / "nonlinear_mpc_casadi" / "scripts" / subdir
    centerline = np.loadtxt(base / "centerline_waypoints.csv", delimiter=",")
    widths = np.loadtxt(base / "track_widths.csv", delimiter=",")
    x = centerline[:, 0]
    y = centerline[:, 1]
    s = cumulative_arclength(x, y)

    rows = []
    for i in range(centerline.shape[0]):
        rows.append(
            {
                "index": i,
                "s": f"{s[i]:.17g}",
                "x_center": f"{x[i]:.17g}",
                "y_center": f"{y[i]:.17g}",
                "width_left": f"{widths[i, 0]:.17g}",
                "width_right": f"{widths[i, 1]:.17g}",
            }
        )

    csv_path = ASSET_ROOT / "driving" / f"{track_name}.csv"
    meta_path = csv_path.with_suffix(".json")
    write_csv(csv_path, ["index", "s", "x_center", "y_center", "width_left", "width_right"], rows)
    meta = {
        "name": track_name,
        "domain": "autonomous_driving",
        "kind": "closed_track",
        "schema": "driving_track_center_widths_v1",
        "file": str(csv_path.relative_to(ROOT)),
        "points": len(rows),
        "source_repo": "https://github.com/nirajbasnet/Nonlinear_MPCC_for_autonomous_racing",
        "source_commit": commit,
        "source_path": f"nonlinear_mpc_casadi/scripts/{subdir}",
        "description": f"Nonlinear MPCC {subdir} track with centerline and left/right widths.",
    }
    write_json(meta_path, meta)
    catalog.append(meta)


def import_argoverse_sample(catalog: list[dict[str, object]]) -> None:
    repo_dir = ensure_repo("argoverse_api", "https://github.com/argoverse/argoverse-api")
    commit = git_commit(repo_dir)
    sample_path = repo_dir / "tests" / "test_data" / "forecasting" / "0.csv"
    with sample_path.open("r", encoding="utf-8", newline="") as fh:
        rows = list(csv.DictReader(fh))

    for object_type, asset_name in (("AV", "argoverse_sample_av"), ("AGENT", "argoverse_sample_agent")):
        filtered = [row for row in rows if row["OBJECT_TYPE"] == object_type]
        filtered.sort(key=lambda row: int(row["TIMESTAMP"]))
        out_rows = []
        for row in filtered:
            out_rows.append(
                {
                    "timestamp": row["TIMESTAMP"],
                    "track_id": row["TRACK_ID"],
                    "object_type": row["OBJECT_TYPE"],
                    "x": row["X"],
                    "y": row["Y"],
                    "city_name": row["CITY_NAME"],
                }
            )
        csv_path = ASSET_ROOT / "driving" / f"{asset_name}.csv"
        meta_path = csv_path.with_suffix(".json")
        write_csv(csv_path, ["timestamp", "track_id", "object_type", "x", "y", "city_name"], out_rows)
        meta = {
            "name": asset_name,
            "domain": "autonomous_driving",
            "kind": "trajectory",
            "schema": "xy_trajectory_v1",
            "file": str(csv_path.relative_to(ROOT)),
            "points": len(out_rows),
            "source_repo": "https://github.com/argoverse/argoverse-api",
            "source_commit": commit,
            "source_path": "tests/test_data/forecasting/0.csv",
            "description": f"Argoverse forecasting sample trajectory for {object_type}.",
        }
        write_json(meta_path, meta)
        catalog.append(meta)


def build_time_profile(dt: float, radius: float, lin_acc: float, v_max: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    ramp_up_t = 2.0
    t_total = 2.0 * v_max / lin_acc + 2.0 * ramp_up_t
    alpha_acc = lin_acc / radius

    ramp_t = np.arange(0.0, ramp_up_t, dt)
    ramp_up_alpha = alpha_acc * np.sin(np.pi / (2.0 * ramp_up_t) * ramp_t) ** 2

    coast_duration = (t_total - 4.0 * ramp_up_t) / 2.0
    coast_t = ramp_up_t + np.arange(0.0, coast_duration, dt)
    coast_alpha = np.ones_like(coast_t) * alpha_acc

    transition_t = np.arange(0.0, 2.0 * ramp_up_t, dt)
    transition_alpha = alpha_acc * np.cos(np.pi / (2.0 * ramp_up_t) * transition_t)
    transition_time = transition_t + coast_t[-1] + dt

    down_coast_t = transition_time[-1] + np.arange(0.0, coast_duration, dt) + dt
    down_coast_alpha = -np.ones_like(down_coast_t) * alpha_acc

    final_t = down_coast_t[-1] + np.arange(0.0, ramp_up_t, dt) + dt
    final_alpha = ramp_up_alpha - alpha_acc

    t_ref = np.concatenate((ramp_t, coast_t, transition_time, down_coast_t, final_t))
    alpha_vec = np.concatenate((ramp_up_alpha, coast_alpha, transition_alpha, down_coast_alpha, final_alpha))
    w_vec = np.cumsum(alpha_vec) * dt
    angle_vec = np.cumsum(w_vec) * dt
    return t_ref, w_vec, angle_vec


def write_robot_reference(
    asset_name: str,
    source_path: str,
    description: str,
    t_ref: np.ndarray,
    xyz: np.ndarray,
    vel: np.ndarray,
    acc: np.ndarray,
    commit: str,
    catalog: list[dict[str, object]],
) -> None:
    rows = []
    for i in range(len(t_ref)):
        rows.append(
            {
                "index": i,
                "t": f"{t_ref[i]:.17g}",
                "x": f"{xyz[i, 0]:.17g}",
                "y": f"{xyz[i, 1]:.17g}",
                "z": f"{xyz[i, 2]:.17g}",
                "vx": f"{vel[i, 0]:.17g}",
                "vy": f"{vel[i, 1]:.17g}",
                "vz": f"{vel[i, 2]:.17g}",
                "ax": f"{acc[i, 0]:.17g}",
                "ay": f"{acc[i, 1]:.17g}",
                "az": f"{acc[i, 2]:.17g}",
            }
        )
    csv_path = ASSET_ROOT / "robotics" / f"{asset_name}.csv"
    meta_path = csv_path.with_suffix(".json")
    write_csv(csv_path, ["index", "t", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az"], rows)
    meta = {
        "name": asset_name,
        "domain": "robotics",
        "kind": "reference_trajectory",
        "schema": "xyz_reference_kinematics_v1",
        "file": str(csv_path.relative_to(ROOT)),
        "points": len(rows),
        "source_repo": "https://github.com/uzh-rpg/data_driven_mpc",
        "source_commit": commit,
        "source_path": source_path,
        "description": description,
    }
    write_json(meta_path, meta)
    catalog.append(meta)


def import_data_driven_mpc(catalog: list[dict[str, object]]) -> None:
    repo_dir = ensure_repo("data_driven_mpc", "https://github.com/uzh-rpg/data_driven_mpc")
    commit = git_commit(repo_dir)

    dt = 0.04
    radius = 5.0
    z = 1.0
    lin_acc = 1.0
    v_max = 8.0

    t_ref, w_vec, angle_vec = build_time_profile(dt, radius, lin_acc, v_max)
    alpha_vec = np.gradient(w_vec, dt)

    loop_xyz = np.stack(
        (
            radius * np.sin(angle_vec),
            radius * np.cos(angle_vec),
            np.ones_like(angle_vec) * z,
        ),
        axis=1,
    )
    loop_vel = np.stack(
        (
            radius * w_vec * np.cos(angle_vec),
            -radius * w_vec * np.sin(angle_vec),
            np.zeros_like(angle_vec),
        ),
        axis=1,
    )
    loop_acc = np.stack(
        (
            radius * (alpha_vec * np.cos(angle_vec) - w_vec**2 * np.sin(angle_vec)),
            -radius * (alpha_vec * np.sin(angle_vec) + w_vec**2 * np.cos(angle_vec)),
            np.zeros_like(angle_vec),
        ),
        axis=1,
    )
    write_robot_reference(
        "data_driven_mpc_loop",
        "ros_gp_mpc/src/experiments/trajectory_test.py + ros_gp_mpc/src/utils/trajectories.py",
        "Loop reference reconstructed from the public trajectory_test defaults in data_driven_mpc.",
        t_ref,
        loop_xyz,
        loop_vel,
        loop_acc,
        commit,
        catalog,
    )

    lem_xyz = np.stack(
        (
            radius * np.cos(angle_vec),
            radius * np.sin(angle_vec) * np.cos(angle_vec),
            np.ones_like(angle_vec) * z,
        ),
        axis=1,
    )
    lem_vel = np.stack(
        (
            -radius * w_vec * np.sin(angle_vec),
            radius * w_vec * (np.cos(angle_vec) ** 2 - np.sin(angle_vec) ** 2),
            np.zeros_like(angle_vec),
        ),
        axis=1,
    )
    lem_acc = np.stack(
        (
            -radius * (alpha_vec * np.sin(angle_vec) + w_vec**2 * np.cos(angle_vec)),
            radius
            * (
                alpha_vec * (np.cos(angle_vec) ** 2 - np.sin(angle_vec) ** 2)
                - 4.0 * w_vec**2 * np.sin(angle_vec) * np.cos(angle_vec)
            ),
            np.zeros_like(angle_vec),
        ),
        axis=1,
    )
    write_robot_reference(
        "data_driven_mpc_lemniscate",
        "ros_gp_mpc/src/experiments/trajectory_test.py + ros_gp_mpc/src/utils/trajectories.py",
        "Lemniscate reference reconstructed from the public trajectory_test defaults in data_driven_mpc.",
        t_ref,
        lem_xyz,
        lem_vel,
        lem_acc,
        commit,
        catalog,
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Import public benchmark reference assets into a canonical CSV form.")
    parser.add_argument(
        "--group",
        choices=["all", "driving", "robotics"],
        default="all",
        help="Which asset family to import.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    catalog: list[dict[str, object]] = []

    if args.group in {"all", "driving"}:
        import_mpcc_track(catalog)
        import_nonlinear_mpcc("nonlinear_mpcc_porto", "porto", catalog)
        import_nonlinear_mpcc("nonlinear_mpcc_fssim", "fssim", catalog)
        import_argoverse_sample(catalog)

    if args.group in {"all", "robotics"}:
        import_data_driven_mpc(catalog)

    CATALOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    CATALOG_PATH.write_text(json.dumps(catalog, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {CATALOG_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
