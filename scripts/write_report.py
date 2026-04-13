#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import subprocess
from datetime import date
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SUMMARY_JSON = ROOT / "results" / "summary.json"
REPORT_MD = ROOT / "results" / "latest_report.md"


def try_run_git(args: list[str], cwd: Path) -> str | None:
    try:
        return subprocess.check_output(
            ["git", *args],
            cwd=cwd,
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except subprocess.CalledProcessError:
        return None


def load_records() -> list[dict]:
    return json.loads(SUMMARY_JSON.read_text(encoding="utf-8"))


def format_float(value: float | None, digits: int = 6) -> str:
    if value is None:
        return "-"
    return f"{value:.{digits}g}"


def format_extra(record: dict) -> str:
    keys = (
        "avg_iterations",
        "max_constraint_violation",
        "final_theta_abs",
        "avg_speed",
        "avg_abs_n",
        "avg_abs_b",
        "min_wall_dist",
        "avg_tracking_error",
        "final_tracking_error",
    )
    parts = []
    if "failure_stage" in record:
        parts.append(f"`failure_stage={record['failure_stage']}`")
    if "status_note" in record:
        parts.append(f"`status_note={record['status_note']}`")
    for key in keys:
        if key in record:
            parts.append(f"`{key}={format_float(float(record[key]))}`")
    return ", ".join(parts) if parts else "-"


def result_lookup(records: list[dict]) -> dict[tuple[str, str], dict]:
    lookup: dict[tuple[str, str], dict] = {}
    for record in records:
        lookup[(record["backend"], record["case_name"])] = record
    return lookup


def render_summary_table(records: list[dict]) -> list[str]:
    case_order = {
        "pendulum_on_cart": 0,
        "race_cars": 1,
        "quadrotor_nav": 2,
        "chain_mass": 3,
    }
    backend_order = {
        "minisolver": 0,
        "acados": 1,
        "casadi": 2,
    }
    visible_backends = {"minisolver", "acados", "casadi"}
    ordered_records = sorted(
        [
            record
            for record in records
            if record["backend"] in visible_backends and record["case_name"] in case_order
        ],
        key=lambda record: (
            case_order.get(record["case_name"], 99),
            backend_order.get(record["backend"], 99),
            record["backend"],
        ),
    )

    lines = [
        "| Backend | Case | Steps | Success | Median ms | P95 ms | Max ms | Extra |",
        "|---|---:|---:|---:|---:|---:|---:|---|",
    ]
    for record in ordered_records:
        lines.append(
            "| `{backend}` | `{case}` | {steps} | {success} | {median} | {p95} | {maxv} | {extra} |".format(
                backend=record["backend"],
                case=record["case_name"],
                steps=record["steps"],
                success=format_float(float(record.get("success_rate", 0.0)), 3),
                median=format_float(float(record["median_ms"])) if "median_ms" in record else "-",
                p95=format_float(float(record["p95_ms"])) if "p95_ms" in record else "-",
                maxv=format_float(float(record["max_ms"])) if "max_ms" in record else "-",
                extra=format_extra(record),
            )
        )
    return lines


def render_case_notes() -> list[str]:
    return [
        "## Fairness Notes",
        "",
        "- `acados` is benchmarked through official Python export plus compiled C runtime. Python codegen is not included in runtime timings.",
        "- `MiniSolver` is benchmarked as a native in-process C++ target, not through an adapter layer.",
        "- `CasADi` is benchmarked through a pre-built `nlpsol('sqpmethod')` graph with `qrqp` as the QP backend. Model construction is excluded from the per-step timings; the timed region is the repeated SQP solve call in the closed loop.",
        "- `pendulum_on_cart`, `race_cars`, and `quadrotor_nav` use RTI-style single-iteration closed-loop runs on both sides.",
        "- `chain_mass` uses full SQP-style solves on both sides, matching the official acados setup.",
        "- `race_cars` and `quadrotor_nav` are official closed-loop cases with solver-dependent early termination. `steps` therefore measure executed closed-loop steps, not a fixed common horizon count.",
        "",
    ]


def render_comparisons(records: list[dict]) -> list[str]:
    lookup = result_lookup(records)
    lines = ["## Pairwise Comparison", ""]
    for case_name in ("pendulum_on_cart", "race_cars", "quadrotor_nav", "chain_mass"):
        mini = lookup.get(("minisolver", case_name))
        aca = lookup.get(("acados", case_name))
        if mini is None or aca is None:
            continue
        if not all(key in mini for key in ("median_ms", "p95_ms")):
            continue
        if not all(key in aca for key in ("median_ms", "p95_ms")):
            continue
        median_ratio = float(aca["median_ms"]) / float(mini["median_ms"])
        p95_ratio = float(aca["p95_ms"]) / float(mini["p95_ms"])
        lines.append(
            f"- `{case_name}`: `acados/MiniSolver` latency ratio is `median {median_ratio:.2f}x`, `p95 {p95_ratio:.2f}x`."
        )
        if "success_rate" in mini and "success_rate" in aca:
            lines.append(
                f"- `{case_name}`: success is `MiniSolver {float(mini['success_rate']):.2%}` vs `acados {float(aca['success_rate']):.2%}`."
            )
        casadi = lookup.get(("casadi", case_name))
        if casadi is not None and "median_ms" in casadi and "p95_ms" in casadi:
            lines.append(
                f"- `{case_name}`: `CasADi/MiniSolver` latency ratio is `median {float(casadi['median_ms']) / float(mini['median_ms']):.2f}x`, `p95 {float(casadi['p95_ms']) / float(mini['p95_ms']):.2f}x`."
            )
            if "success_rate" in casadi:
                lines.append(
                    f"- `{case_name}`: success is `MiniSolver {float(mini['success_rate']):.2%}` vs `CasADi {float(casadi['success_rate']):.2%}`."
                )
    lines.append("")
    return lines


def main() -> int:
    records = load_records()
    nmpc_bench_commit = try_run_git(["rev-parse", "HEAD"], ROOT) or "uncommitted-workspace"
    acados_dir = ROOT / "third_party" / "acados"
    acados_commit = try_run_git(["rev-parse", "HEAD"], acados_dir) or "unknown"

    minisolver_dir = None
    for candidate in (ROOT / "third_party" / "MiniSolver", ROOT.parent / "MiniSolver"):
        if (candidate / "CMakeLists.txt").exists() or (candidate / ".git").exists():
            minisolver_dir = candidate
            break

    minisolver_label = "unknown"
    minisolver_branch = "unknown"
    minisolver_commit = "unknown"
    if minisolver_dir is not None:
        minisolver_label = os.path.relpath(minisolver_dir, ROOT)
        minisolver_branch = try_run_git(["branch", "--show-current"], minisolver_dir) or "detached"
        if minisolver_branch == "":
            minisolver_branch = "detached"
        minisolver_commit = try_run_git(["rev-parse", "HEAD"], minisolver_dir) or "unknown"

    lines = [
        "# Latest Benchmark Report",
        "",
        f"Date: {date.today().isoformat()}",
        "",
        "## Provenance",
        "",
        f"- `nmpc-bench`: `{nmpc_bench_commit}`",
        f"- `acados`: `third_party/acados` at `{acados_commit}`",
        f"- `MiniSolver`: `{minisolver_label}` at `{minisolver_commit}`",
        f"  - branch: `{minisolver_branch}`",
        "",
        "## Summary",
        "",
        *render_summary_table(records),
        "",
        *render_case_notes(),
        *render_comparisons(records),
        "## Files",
        "",
        "- Raw data: `results/raw/`",
        "- Aggregate CSV: `results/summary.csv`",
        "- Aggregate JSON: `results/summary.json`",
    ]

    REPORT_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {REPORT_MD}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
