#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CATALOG_PATH = ROOT / "assets" / "catalog.json"
SUMMARY_JSON = ROOT / "assets" / "summary.json"
SUMMARY_MD = ROOT / "assets" / "summary.md"


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


def floats(rows: list[dict[str, str]], key: str) -> list[float]:
    return [float(row[key]) for row in rows]


def path_length_xy(xs: list[float], ys: list[float]) -> float:
    total = 0.0
    for i in range(1, len(xs)):
        dx = xs[i] - xs[i - 1]
        dy = ys[i] - ys[i - 1]
        total += math.hypot(dx, dy)
    return total


def summarize_record(record: dict[str, object]) -> dict[str, object]:
    path = ROOT / str(record["file"])
    rows = read_rows(path)
    schema = str(record["schema"])

    summary: dict[str, object] = {
        "name": record["name"],
        "domain": record["domain"],
        "kind": record["kind"],
        "schema": schema,
        "file": record["file"],
        "points": len(rows),
        "source_repo": record["source_repo"],
        "source_commit": record["source_commit"],
    }

    if schema == "driving_track_center_bounds_v1":
        s = floats(rows, "s")
        x = floats(rows, "x_center")
        y = floats(rows, "y_center")
        w_inner = floats(rows, "width_inner")
        w_outer = floats(rows, "width_outer")
        total_width = [a + b for a, b in zip(w_inner, w_outer)]
        summary.update(
            {
                "track_length": s[-1] if s else 0.0,
                "bbox_x": [min(x), max(x)],
                "bbox_y": [min(y), max(y)],
                "min_total_width": min(total_width),
                "max_total_width": max(total_width),
                "mean_total_width": sum(total_width) / len(total_width),
            }
        )
        return summary

    if schema == "driving_track_center_widths_v1":
        s = floats(rows, "s")
        x = floats(rows, "x_center")
        y = floats(rows, "y_center")
        w_left = floats(rows, "width_left")
        w_right = floats(rows, "width_right")
        total_width = [a + b for a, b in zip(w_left, w_right)]
        summary.update(
            {
                "track_length": s[-1] if s else 0.0,
                "bbox_x": [min(x), max(x)],
                "bbox_y": [min(y), max(y)],
                "min_total_width": min(total_width),
                "max_total_width": max(total_width),
                "mean_total_width": sum(total_width) / len(total_width),
            }
        )
        return summary

    if schema == "xy_trajectory_v1":
        timestamps = floats(rows, "timestamp")
        x = floats(rows, "x")
        y = floats(rows, "y")
        summary.update(
            {
                "duration": (timestamps[-1] - timestamps[0]) if len(timestamps) >= 2 else 0.0,
                "path_length": path_length_xy(x, y),
                "bbox_x": [min(x), max(x)],
                "bbox_y": [min(y), max(y)],
            }
        )
        return summary

    if schema == "xyz_reference_kinematics_v1":
        t = floats(rows, "t")
        x = floats(rows, "x")
        y = floats(rows, "y")
        z = floats(rows, "z")
        vx = floats(rows, "vx")
        vy = floats(rows, "vy")
        vz = floats(rows, "vz")
        ax = floats(rows, "ax")
        ay = floats(rows, "ay")
        az = floats(rows, "az")
        speeds = [math.sqrt(a * a + b * b + c * c) for a, b, c in zip(vx, vy, vz)]
        accels = [math.sqrt(a * a + b * b + c * c) for a, b, c in zip(ax, ay, az)]
        summary.update(
            {
                "duration": (t[-1] - t[0]) if len(t) >= 2 else 0.0,
                "path_length": path_length_xy(x, y) + sum(
                    abs(z[i] - z[i - 1]) for i in range(1, len(z))
                ),
                "bbox_x": [min(x), max(x)],
                "bbox_y": [min(y), max(y)],
                "bbox_z": [min(z), max(z)],
                "max_speed": max(speeds),
                "mean_speed": sum(speeds) / len(speeds),
                "max_accel": max(accels),
            }
        )
        return summary

    return summary


def render_md(records: list[dict[str, object]]) -> str:
    lines = [
        "# Reference Asset Summary",
        "",
        "| Name | Domain | Kind | Points | Main Stats |",
        "|---|---|---|---:|---|",
    ]
    for record in records:
        parts: list[str] = []
        if "track_length" in record:
            parts.append(f"track_length={float(record['track_length']):.3f}")
            parts.append(f"mean_width={float(record['mean_total_width']):.3f}")
        if "duration" in record:
            parts.append(f"duration={float(record['duration']):.3f}")
        if "path_length" in record:
            parts.append(f"path_length={float(record['path_length']):.3f}")
        if "max_speed" in record:
            parts.append(f"max_speed={float(record['max_speed']):.3f}")
        if "max_accel" in record:
            parts.append(f"max_accel={float(record['max_accel']):.3f}")
        lines.append(
            "| `{name}` | `{domain}` | `{kind}` | {points} | {stats} |".format(
                name=record["name"],
                domain=record["domain"],
                kind=record["kind"],
                points=record["points"],
                stats=", ".join(parts) if parts else "-",
            )
        )
    lines.extend(
        [
            "",
            "## Notes",
            "",
            "- Driving assets with explicit track geometry are the best candidates for near-term solver benchmarks.",
            "- The Argoverse sample is only a tiny import sanity check, not yet a serious benchmark case.",
            "- The robotics loop and lemniscate assets are reconstructed from public trajectory formulas and defaults in `data_driven_mpc`.",
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    catalog = json.loads(CATALOG_PATH.read_text(encoding="utf-8"))
    records = [summarize_record(record) for record in catalog]
    SUMMARY_JSON.write_text(json.dumps(records, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    SUMMARY_MD.write_text(render_md(records), encoding="utf-8")
    print(f"wrote {SUMMARY_JSON}")
    print(f"wrote {SUMMARY_MD}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
