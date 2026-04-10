#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from pathlib import Path

import clarabel
import numpy as np
import scipy.sparse as sp


if __package__ in (None, ""):
    ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(ROOT))
    from candidate_specs import load_candidate, load_robot_reference
else:
    ROOT = Path(__file__).resolve().parents[2]
    from ...candidate_specs import load_candidate, load_robot_reference


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one Clarabel convex benchmark candidate.")
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def xidx(k: int, nx: int) -> int:
    return k * nx


def uidx(k: int, n_x: int, nu: int) -> int:
    return n_x + k * nu


def stage_params(asset, idx: int) -> list[float]:
    return [asset.x[idx], asset.y[idx], asset.z[idx], asset.vx[idx], asset.vy[idx], asset.vz[idx]]


def build_static_problem(horizon: int, dt: float) -> tuple[sp.csc_matrix, sp.csc_matrix, list[object], dict[str, object]]:
    nx = 6
    nu = 3
    n_x = (horizon + 1) * nx
    n_u = (horizon + 1) * nu
    n = n_x + n_u

    q_state = np.diag([30.0, 30.0, 30.0, 5.0, 5.0, 5.0])
    r_control = np.diag([0.1, 0.1, 0.1])

    p_mat = sp.lil_matrix((n, n))
    for k in range(horizon + 1):
        xi = xidx(k, nx)
        p_mat[xi : xi + nx, xi : xi + nx] += q_state
        ui = uidx(k, n_x, nu)
        p_mat[ui : ui + nu, ui : ui + nu] += r_control
    p_mat = sp.triu(p_mat.tocsc()).tocsc()

    a_dyn = np.array(
        [
            [1.0, 0.0, 0.0, dt, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, dt, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, dt],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    b_dyn = np.array(
        [
            [0.5 * dt * dt, 0.0, 0.0],
            [0.0, 0.5 * dt * dt, 0.0],
            [0.0, 0.0, 0.5 * dt * dt],
            [dt, 0.0, 0.0],
            [0.0, dt, 0.0],
            [0.0, 0.0, dt],
        ],
        dtype=float,
    )

    aeq_rows: list[sp.lil_matrix] = []
    for i in range(nx):
        row = sp.lil_matrix((1, n))
        row[0, xidx(0, nx) + i] = 1.0
        aeq_rows.append(row)

    for k in range(horizon):
        for i in range(nx):
            row = sp.lil_matrix((1, n))
            row[0, xidx(k + 1, nx) + i] = 1.0
            for j in range(nx):
                row[0, xidx(k, nx) + j] -= a_dyn[i, j]
            for j in range(nu):
                row[0, uidx(k, n_x, nu) + j] -= b_dyn[i, j]
            aeq_rows.append(row)
    aeq = sp.vstack(aeq_rows).tocsc()

    g_rows: list[sp.lil_matrix] = []
    h_template: list[float] = []
    for k in range(horizon):
        for j in range(nu):
            row_pos = sp.lil_matrix((1, n))
            row_pos[0, uidx(k, n_x, nu) + j] = 1.0
            g_rows.append(row_pos)
            h_template.append(12.0)

            row_neg = sp.lil_matrix((1, n))
            row_neg[0, uidx(k, n_x, nu) + j] = -1.0
            g_rows.append(row_neg)
            h_template.append(12.0)

    for j in range(nu):
        row_pos = sp.lil_matrix((1, n))
        row_pos[0, uidx(horizon, n_x, nu) + j] = 1.0
        g_rows.append(row_pos)
        h_template.append(0.0)

        row_neg = sp.lil_matrix((1, n))
        row_neg[0, uidx(horizon, n_x, nu) + j] = -1.0
        g_rows.append(row_neg)
        h_template.append(0.0)

    g_mat = sp.vstack(g_rows).tocsc()
    a_mat = sp.vstack([aeq, g_mat]).tocsc()
    cones = [clarabel.ZeroConeT(aeq.shape[0]), clarabel.NonnegativeConeT(g_mat.shape[0])]

    meta = {
        "nx": nx,
        "nu": nu,
        "n_x": n_x,
        "n_u": n_u,
        "n": n,
        "aeq_rows": aeq.shape[0],
        "h_template": np.asarray(h_template, dtype=float),
        "q_state": q_state,
    }
    return p_mat, a_mat, cones, meta


def build_dynamic_vectors(current: np.ndarray, refs: list[list[float]], meta: dict[str, object]) -> tuple[np.ndarray, np.ndarray]:
    nx = int(meta["nx"])
    n = int(meta["n"])
    q_state = np.asarray(meta["q_state"])
    q_vec = np.zeros(n, dtype=float)
    for k, ref in enumerate(refs):
        xi = xidx(k, nx)
        q_vec[xi : xi + nx] = -(q_state @ np.asarray(ref))

    b_eq = np.zeros(int(meta["aeq_rows"]), dtype=float)
    b_eq[:nx] = current
    b_all = np.concatenate([b_eq, np.asarray(meta["h_template"])])
    return q_vec, b_all


def tracking_error(current: np.ndarray, ref: list[float]) -> float:
    dx = current[0] - ref[0]
    dy = current[1] - ref[1]
    dz = current[2] - ref[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def success_status(status: object) -> bool:
    return str(status) == "Solved"


def main() -> int:
    args = parse_args()
    candidate = load_candidate(args.candidate)
    candidate_name = candidate["name"]
    if candidate["model_family"] != "double_integrator_3d_tracking":
        raise SystemExit(
            f"Clarabel backend only supports double_integrator_3d_tracking, got {candidate['model_family']}"
        )

    asset = load_robot_reference(ROOT / candidate["asset"])
    steps = min(args.steps if args.steps is not None else candidate["closed_loop_steps"], len(asset.x))
    output = args.output or (ROOT / "results" / "raw" / "clarabel" / f"{candidate_name}.csv")
    output.parent.mkdir(parents=True, exist_ok=True)

    dt = float(candidate["dt"])
    horizon = int(candidate["horizon"])
    p_mat, a_mat, cones, meta = build_static_problem(horizon, dt)

    current = np.asarray(
        [asset.x[0], asset.y[0], asset.z[0], asset.vx[0], asset.vy[0], asset.vz[0]],
        dtype=float,
    )
    refs0 = [stage_params(asset, min(k, len(asset.x) - 1)) for k in range(horizon + 1)]
    q_vec, b_vec = build_dynamic_vectors(current, refs0, meta)

    settings = clarabel.DefaultSettings()
    settings.verbose = False
    settings.max_iter = 200
    settings.tol_feas = 1e-9
    settings.tol_gap_abs = 1e-9
    settings.tol_gap_rel = 1e-9
    solver = clarabel.DefaultSolver(p_mat, q_vec, a_mat, b_vec, cones, settings)

    times_ms: list[float] = []
    success = 0
    nu = int(meta["nu"])
    n_x = int(meta["n_x"])
    with output.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(
            [
                "step",
                "status",
                "time_ms",
                "iterations",
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "tracking_error",
                "max_constraint_violation",
            ]
        )

        for step in range(steps):
            refs = [stage_params(asset, min(step + k, len(asset.x) - 1)) for k in range(horizon + 1)]
            q_vec, b_vec = build_dynamic_vectors(current, refs, meta)

            t0 = time.perf_counter()
            solver.update(q=q_vec, b=b_vec)
            solution = solver.solve()
            t1 = time.perf_counter()

            time_ms = (t1 - t0) * 1000.0
            times_ms.append(time_ms)
            status_ok = success_status(solution.status)
            if status_ok:
                success += 1

            sol_x = np.asarray(solution.x, dtype=float)
            u0 = sol_x[uidx(0, n_x, nu) : uidx(0, n_x, nu) + nu]
            max_viol = max(
                0.0,
                float(u0[0] - 12.0),
                float(-12.0 - u0[0]),
                float(u0[1] - 12.0),
                float(-12.0 - u0[1]),
                float(u0[2] - 12.0),
                float(-12.0 - u0[2]),
            )
            writer.writerow(
                [
                    step,
                    0 if status_ok else 1,
                    f"{time_ms:.17g}",
                    getattr(solution, "iterations", 0),
                    f"{current[0]:.17g}",
                    f"{current[1]:.17g}",
                    f"{current[2]:.17g}",
                    f"{current[3]:.17g}",
                    f"{current[4]:.17g}",
                    f"{current[5]:.17g}",
                    f"{tracking_error(current, refs[0]):.17g}",
                    f"{max_viol:.17g}",
                ]
            )

            current[0] += current[3] * dt + 0.5 * float(u0[0]) * dt * dt
            current[1] += current[4] * dt + 0.5 * float(u0[1]) * dt * dt
            current[2] += current[5] * dt + 0.5 * float(u0[2]) * dt * dt
            current[3] += float(u0[0]) * dt
            current[4] += float(u0[1]) * dt
            current[5] += float(u0[2]) * dt

    sorted_times = sorted(times_ms)
    mid = sorted_times[len(sorted_times) // 2] if sorted_times else 0.0
    p95_idx = int(round(0.95 * max(0, len(sorted_times) - 1)))
    p95 = sorted_times[p95_idx] if sorted_times else 0.0
    worst = max(sorted_times) if sorted_times else 0.0
    print(
        f"steps={steps} success={success} median_ms={mid:.6g} p95_ms={p95:.6g} max_ms={worst:.6g} output={output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
