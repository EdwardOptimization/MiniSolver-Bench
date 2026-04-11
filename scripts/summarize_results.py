#!/usr/bin/env python3
from __future__ import annotations

import csv
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RAW_ROOT = ROOT / "results" / "raw"
SUMMARY_CSV = ROOT / "results" / "summary.csv"
SUMMARY_JSON = ROOT / "results" / "summary.json"


def percentile(values: list[float], frac: float) -> float:
    if not values:
        return 0.0
    values = sorted(values)
    idx = frac * (len(values) - 1)
    lo = math.floor(idx)
    hi = math.ceil(idx)
    if lo == hi:
        return values[lo]
    weight = idx - lo
    return values[lo] * (1.0 - weight) + values[hi] * weight


def parse_float(row: dict[str, str], key: str) -> float | None:
    value = row.get(key)
    if value is None or value == "":
        return None
    return float(value)


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


def summarize_file(path: Path) -> dict[str, object] | None:
    if path.name.startswith("."):
        return None
    if path.suffix == ".json" and path.name.endswith(".failure.json"):
        with path.open("r", encoding="utf-8") as fh:
            return json.load(fh)
    if path.suffix != ".csv" or path.name.endswith("_summary.csv"):
        return None
    if path.stem.endswith("_smoke") or path.stem.endswith("_codegen_smoke"):
        return None

    source_backend = path.parent.name
    case_name = path.stem

    rows = read_csv_rows(path)
    if not rows:
        return None

    times = [float(row["time_ms"]) for row in rows]
    record: dict[str, object] = {
        "backend": source_backend,
        "backend_family": source_backend,
        "case_name": case_name,
        "file": str(path.relative_to(ROOT)),
        "steps": len(rows),
        "median_ms": percentile(times, 0.5),
        "p95_ms": percentile(times, 0.95),
        "max_ms": max(times),
    }

    if "status" in rows[0]:
        if source_backend == "minisolver":
            success_statuses = {"SOLVED", "OPTIMAL", "FEASIBLE"}
            success = sum(1 for row in rows if row["status"] in success_statuses)
        else:
            success = sum(1 for row in rows if row["status"] == "0")
        record["success_rate"] = success / len(rows)

    if "iterations" in rows[0]:
        record["avg_iterations"] = sum(float(row["iterations"]) for row in rows) / len(rows)
    if "sqp_iter" in rows[0]:
        record["avg_iterations"] = sum(float(row["sqp_iter"]) for row in rows) / len(rows)

    if "max_constraint_violation" in rows[0]:
        record["max_constraint_violation"] = max(float(row["max_constraint_violation"]) for row in rows)
    if "pole_theta" in rows[0]:
        record["final_theta_abs"] = abs(float(rows[-1]["pole_theta"]))
    if "theta" in rows[0]:
        record["final_theta_abs"] = abs(float(rows[-1]["theta"]))
    if "v" in rows[0]:
        record["avg_speed"] = sum(float(row["v"]) for row in rows) / len(rows)
    if "abs_n" in rows[0]:
        record["avg_abs_n"] = sum(float(row["abs_n"]) for row in rows) / len(rows)
    if "abs_b" in rows[0]:
        record["avg_abs_b"] = sum(float(row["abs_b"]) for row in rows) / len(rows)
    if "wall_dist" in rows[0]:
        record["min_wall_dist"] = min(float(row["wall_dist"]) for row in rows)
    if "tracking_error" in rows[0]:
        tracking = [float(row["tracking_error"]) for row in rows]
        record["avg_tracking_error"] = sum(tracking) / len(tracking)
        record["final_tracking_error"] = tracking[-1]

    return record


def main() -> int:
    records = []
    for path in sorted(RAW_ROOT.glob("*/*")):
        record = summarize_file(path)
        if record is not None:
            records.append(record)

    SUMMARY_CSV.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = sorted({key for record in records for key in record.keys()})
    with SUMMARY_CSV.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)

    with SUMMARY_JSON.open("w", encoding="utf-8") as fh:
        json.dump(records, fh, indent=2, sort_keys=True)
        fh.write("\n")

    print(f"wrote {SUMMARY_CSV}")
    print(f"wrote {SUMMARY_JSON}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
