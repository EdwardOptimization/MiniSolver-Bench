from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parent
CANDIDATE_DIR = ROOT / "case_candidates"


@dataclass
class DrivingTrackAsset:
    s: list[float]
    x: list[float]
    y: list[float]
    psi: list[float]
    nx: list[float]
    ny: list[float]
    w_left: list[float]
    w_right: list[float]
    avg_ds: float


@dataclass
class RobotReferenceAsset:
    t: list[float]
    x: list[float]
    y: list[float]
    z: list[float]
    vx: list[float]
    vy: list[float]
    vz: list[float]
    dt: float


def list_candidate_names() -> list[str]:
    return sorted(path.stem for path in CANDIDATE_DIR.glob("*.json"))


def load_candidate(name_or_path: str | Path) -> dict:
    path = Path(name_or_path)
    if not path.is_absolute():
        if path.suffix == ".json":
            path = ROOT / path
        else:
            path = CANDIDATE_DIR / f"{path}.json"
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def benchmark_kind(candidate: dict) -> str:
    task = candidate["task"]
    if task == "closed_track_following":
        return "driving_track_following"
    if task == "reference_tracking":
        return "robotics_reference_tracking"
    raise ValueError(f"unsupported candidate task: {task}")


def split_csv_line(line: str) -> list[str]:
    parts = [part.strip() for part in line.rstrip("\n").rstrip("\r").split(",")]
    return parts


def load_driving_track(path: str | Path) -> DrivingTrackAsset:
    path = Path(path)
    if not path.is_absolute():
        path = ROOT / path
    lines = path.read_text(encoding="utf-8").splitlines()
    headers = split_csv_line(lines[0])
    index = {name: idx for idx, name in enumerate(headers)}

    s: list[float] = []
    x: list[float] = []
    y: list[float] = []
    w_left: list[float] = []
    w_right: list[float] = []
    for line in lines[1:]:
        if not line.strip():
            continue
        parts = split_csv_line(line)
        s.append(float(parts[index["s"]]))
        x.append(float(parts[index["x_center"]]))
        y.append(float(parts[index["y_center"]]))
        if "width_left" in index:
            w_left.append(float(parts[index["width_left"]]))
            w_right.append(float(parts[index["width_right"]]))
        else:
            width = min(float(parts[index["width_inner"]]), float(parts[index["width_outer"]]))
            w_left.append(width)
            w_right.append(width)

    n = len(x)
    psi = [0.0] * n
    nx = [0.0] * n
    ny = [0.0] * n
    for i in range(n):
        prev_i = n - 1 if i == 0 else i - 1
        next_i = 0 if i + 1 == n else i + 1
        dx = x[next_i] - x[prev_i]
        dy = y[next_i] - y[prev_i]
        angle = math.atan2(dy, dx)
        psi[i] = angle
        nx[i] = -math.sin(angle)
        ny[i] = math.cos(angle)
    avg_ds = s[-1] / max(1, n - 1) if n > 1 else 0.1
    return DrivingTrackAsset(s=s, x=x, y=y, psi=psi, nx=nx, ny=ny, w_left=w_left, w_right=w_right, avg_ds=avg_ds)


def load_robot_reference(path: str | Path) -> RobotReferenceAsset:
    path = Path(path)
    if not path.is_absolute():
        path = ROOT / path
    lines = path.read_text(encoding="utf-8").splitlines()
    headers = split_csv_line(lines[0])
    index = {name: idx for idx, name in enumerate(headers)}

    t: list[float] = []
    x: list[float] = []
    y: list[float] = []
    z: list[float] = []
    vx: list[float] = []
    vy: list[float] = []
    vz: list[float] = []
    for line in lines[1:]:
        if not line.strip():
            continue
        parts = split_csv_line(line)
        t.append(float(parts[index["t"]]))
        x.append(float(parts[index["x"]]))
        y.append(float(parts[index["y"]]))
        z.append(float(parts[index["z"]]))
        vx.append(float(parts[index["vx"]]))
        vy.append(float(parts[index["vy"]]))
        vz.append(float(parts[index["vz"]]))

    dt = (t[1] - t[0]) if len(t) > 1 else 0.04
    return RobotReferenceAsset(t=t, x=x, y=y, z=z, vx=vx, vy=vy, vz=vz, dt=dt)
