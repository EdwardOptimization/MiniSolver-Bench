#!/usr/bin/env python3
from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SUMMARY_JSON = ROOT / "results" / "summary.json"
CASE_CANDIDATE_DIR = ROOT / "case_candidates"
REPORT_MD = ROOT / "results" / "asset_benchmark_report.md"
BACKEND_ORDER = ("minisolver", "acados", "casadi", "clarabel", "altro")


def load_records() -> list[dict]:
    return json.loads(SUMMARY_JSON.read_text(encoding="utf-8"))


def load_candidate_names() -> list[str]:
    names = []
    for path in sorted(CASE_CANDIDATE_DIR.glob("*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        names.append(data["name"])
    return names


def load_candidate_specs() -> dict[str, dict]:
    specs: dict[str, dict] = {}
    for path in sorted(CASE_CANDIDATE_DIR.glob("*.json")):
        data = json.loads(path.read_text(encoding="utf-8"))
        specs[data["name"]] = data
    return specs


def fmt(value: object | None, digits: int = 6) -> str:
    if value is None:
        return "-"
    return f"{float(value):.{digits}g}"


def supported_backends(spec: dict) -> tuple[str, ...]:
    model_family = spec["model_family"]
    if model_family == "double_integrator_3d_tracking":
        return BACKEND_ORDER
    if model_family == "kinematic_bicycle_track_following":
        return ("minisolver", "acados", "casadi")
    raise ValueError(f"unsupported model_family in candidate spec: {model_family}")


def lookup(records: list[dict]) -> dict[tuple[str, str], dict]:
    out: dict[tuple[str, str], dict] = {}
    for record in records:
        out[(record["backend"], record["case_name"])] = record
    return out


def comparison_status(spec: dict, backend: str, record: dict | None) -> str:
    if backend not in supported_backends(spec):
        return "unsupported"
    if record is None:
        return "missing"

    expected_steps = int(spec["closed_loop_steps"])
    observed_steps = int(record["steps"])
    if observed_steps != expected_steps:
        return f"step-mismatch {observed_steps}/{expected_steps}"
    return "comparable"


def contract_summary(case_names: list[str], specs: dict[str, dict], by_key: dict[tuple[str, str], dict]) -> list[str]:
    rows: list[str] = []
    for case in case_names:
        spec = specs[case]
        comparable = supported_backends(spec)
        problems: list[str] = []
        for backend in comparable:
            status = comparison_status(spec, backend, by_key.get((backend, case)))
            if status != "comparable":
                problems.append(f"{backend}:{status}")
        contract = "complete" if not problems else "; ".join(problems)
        rows.append(
            "| `{case}` | `{domain}` | `{model}` | {dt} | {horizon} | {steps} | {backends} | {contract} |".format(
                case=case,
                domain=spec["domain"],
                model=spec["model_family"],
                dt=fmt(spec["dt"]),
                horizon=spec["horizon"],
                steps=spec["closed_loop_steps"],
                backends=", ".join(f"`{backend}`" for backend in comparable),
                contract=contract,
            )
        )
    return rows


def render_rows(case_names: list[str], specs: dict[str, dict], by_key: dict[tuple[str, str], dict]) -> list[str]:
    rows: list[str] = []
    for case in case_names:
        spec = specs[case]
        for backend in BACKEND_ORDER:
            record = by_key.get((backend, case))
            status = comparison_status(spec, backend, record)
            if record is None:
                rows.append(
                    "| `{case}` | `{backend}` | {status} | {requested} | - | - | - | - | - | - | - |".format(
                        case=case,
                        backend=backend,
                        status=status,
                        requested=spec["closed_loop_steps"],
                    )
                )
                continue
            rows.append(
                "| `{case}` | `{backend}` | {status} | {requested} | {steps} | {success} | {median} | {p95} | {maxv} | {avg_track} | {max_viol} |".format(
                    case=case,
                    backend=backend,
                    status=status,
                    requested=spec["closed_loop_steps"],
                    steps=record["steps"],
                    success=fmt(record.get("success_rate"), 3),
                    median=fmt(record.get("median_ms")),
                    p95=fmt(record.get("p95_ms")),
                    maxv=fmt(record.get("max_ms")),
                    avg_track=fmt(record.get("avg_tracking_error")),
                    max_viol=fmt(record.get("max_constraint_violation")),
                )
            )
    return rows


def main() -> int:
    records = load_records()
    candidate_specs = load_candidate_specs()
    candidate_names = set(candidate_specs)
    filtered = [r for r in records if r["case_name"] in candidate_names]
    by_key = lookup(filtered)

    stable_cases = [
        name for name, spec in sorted(candidate_specs.items())
        if name != "mpcc_track_following"
    ]
    stress_cases = [name for name in sorted(candidate_specs) if name == "mpcc_track_following"]

    lines = [
        "# End-to-End Asset Solver Comparison",
        "",
        "Same-case, closed-loop, end-to-end solver comparison for public-asset-derived candidates.",
        "",
        "This is the repository's current headline report for solver ranking. "
        "It excludes official `race_cars` and `quadrotor_nav` because those cases require piecewise track functions; "
        "MiniSolver currently runs them from checked-in generated C++ models when Python MiniModel lacks `ppoly`.",
        "",
        "Each solver row uses the same asset, model family, `dt`, horizon, and requested closed-loop step count for that candidate. "
        "The timed region is the per-step solver call inside the closed-loop runner; one-time code generation, CMake configure/build, Python import time, and report aggregation are excluded.",
        "",
        "The asset candidates do not rely on Python MiniModel piecewise modeling. "
        "Robotics candidates use native reference-tracking data, and driving candidates use runner-side track sampling plus stage parameters. "
        "For any callback-backed track model, the benchmark owner must provide smooth references and first-derivative behavior before treating the case as a headline comparison.",
        "",
        "## Fairness Contract",
        "",
        "| Case | Domain | Model Family | dt | Horizon | Requested Steps | Comparable Backends | Contract Status |",
        "|---|---|---|---:|---:|---:|---|---|",
        *contract_summary(stable_cases + stress_cases, candidate_specs, by_key),
        "",
        "## Current Baseline Cases",
        "",
        "These cases currently provide usable same-condition cross-solver comparisons under the present harness.",
        "",
        "| Case | Backend | Contract | Requested Steps | Observed Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        *render_rows(stable_cases, candidate_specs, by_key),
        "",
        "## Stress Case",
        "",
        "`mpcc_track_following` is kept separate because it is still a stress case, not a stable baseline.",
        "",
        "| Case | Backend | Contract | Requested Steps | Observed Steps | Success | Median ms | P95 ms | Max ms | Avg Tracking Error | Max Constraint Violation |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        *render_rows(stress_cases, candidate_specs, by_key),
        "",
        "## Notes",
        "",
        "- `time_ms` is end-to-end wall-clock time around each solver call inside the closed-loop runner.",
        "- One-time export, graph construction, CMake configure/build, and report aggregation are excluded from timing.",
        "- `comparable` means that backend supports the candidate and produced the candidate's requested closed-loop step count.",
        "- `missing` means the backend should be comparable for that candidate but no raw result is present in `results/raw/`.",
        "- `unsupported` means the backend does not implement that model family in this harness and is excluded from ranking for that case.",
        "- `MiniSolver` uses generated models plus native C++ closed-loop runners.",
        "- `acados` uses Python export/codegen plus compiled C closed-loop runners; export time is excluded.",
        "- `CasADi` uses native C++ runners with a pre-built `nlpsol('sqpmethod')` graph.",
        "- `ALTRO` uses ALTRO's C++ API and is currently limited to the convex robotics reference-tracking cases.",
        "- `max_constraint_violation` is the current closed-loop step violation, not the maximum predicted violation over the full horizon.",
        "- `Clarabel` is included only for convex robotics QP cases. It is not run on the nonconvex driving NMPC cases.",
        "- `ALTRO` is included only for the robotics reference-tracking cases. It is not run on the nonconvex driving NMPC cases.",
        "- `data_driven_mpc_loop_tracking` is the cleanest current all-backend robotics baseline, including ALTRO.",
        "- `data_driven_mpc_lemniscate_tracking` remains a useful robotics benchmark, but ALTRO currently hits `MaxInnerIterations` on the later closed-loop steps. Treat that row as robustness/tuning evidence, not as a pure ALTRO latency comparison.",
        "- `nonlinear_mpcc_porto_following` and `nonlinear_mpcc_fssim_following` are the current driving baselines. Interpret success, tracking, and latency together; none of the three solvers dominates every metric.",
        "- `mpcc_track_following` should not be used for headline conclusions yet. It is still exposing model/setup mismatch rather than a stable solver ranking.",
        "",
        "## Files",
        "",
        "- Raw CSVs: `results/raw/minisolver/`, `results/raw/acados/`, `results/raw/casadi/`, `results/raw/clarabel/`, `results/raw/altro/`",
        "- Aggregate CSV: `results/summary.csv`",
        "- Aggregate JSON: `results/summary.json`",
    ]

    REPORT_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {REPORT_MD}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
