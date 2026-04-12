#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Tuple

import casadi as ca
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate scenario ground-truth json by CasADi+IPOPT open-loop solve.")
    parser.add_argument(
        "--scenario",
        type=Path,
        action="append",
        required=False,
        help="Scenario json path(s). If omitted, generates for all scenario directories.",
    )
    parser.add_argument("--scenario-root", type=Path, default=ROOT / "scenarios")
    return parser.parse_args()


def load_csv_rows(path: Path) -> Tuple[np.ndarray, list[str]]:
    raw = np.loadtxt(path, delimiter=",", dtype=str, comments="#", ndmin=2)
    header = raw[0].tolist()
    rows = raw[1:].astype(float)
    return rows.astype(float), header


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def parse_ref_csv(path: Path, model: str) -> tuple[np.ndarray, float]:
    rows, header = load_csv_rows(path)
    idx = {name: header.index(name) for name in header}
    if model == "kinematic_bicycle_v1":
        keys = ["t", "x_ref", "y_ref", "yaw_ref", "v_ref", "w_left", "w_right"]
    elif model == "double_integrator_3d_v1":
        keys = ["t", "x_ref", "y_ref", "z_ref", "vx_ref", "vy_ref", "vz_ref"]
    else:
        raise ValueError(f"unsupported model {model}")
    missing = [k for k in keys if k not in idx]
    if missing:
        raise ValueError(f"missing columns in {path}: {missing}")
    return rows, header


def solve_kinematic_bicycle(N: int, dt: float, x0: np.ndarray, refs: dict[str, np.ndarray]) -> dict:
    X = [ca.MX.sym(f"X_{k}", 5) for k in range(N + 1)]
    U = [ca.MX.sym(f"U_{k}", 2) for k in range(N)]
    all_vars = ca.vertcat(*X, *U)

    wheelbase = 0.33
    obj = 0.0
    g = []
    lbg = []
    ubg = []

    # Initial state equality.
    g.append(X[0] - ca.DM(x0))
    lbg.extend([0.0] * 5)
    ubg.extend([0.0] * 5)

    for k in range(N):
        xk = X[k]
        uk = U[k]
        x_ref = refs["x_ref"][k]
        y_ref = refs["y_ref"][k]
        yaw_ref = refs["yaw_ref"][k]
        v_ref = refs["v_ref"][k]
        w_left = refs["w_left"][k]
        w_right = refs["w_right"][k]
        nx = -ca.sin(yaw_ref)
        ny = ca.cos(yaw_ref)
        lateral = nx * (xk[0] - x_ref) + ny * (xk[1] - y_ref)

        obj += 10.0 * (xk[0] - x_ref) ** 2
        obj += 10.0 * (xk[1] - y_ref) ** 2
        psi_error = ca.atan2(ca.sin(xk[2] - yaw_ref), ca.cos(xk[2] - yaw_ref))
        obj += 2.5 * (psi_error ** 2)
        obj += 1.0 * (xk[3] - v_ref) ** 2
        obj += 0.25 * xk[4] ** 2
        obj += 0.025 * uk[0] ** 2
        obj += 0.025 * uk[1] ** 2

        # Inequality constraints are defined in "g(x) <= 0" form.
        g.extend([
            lateral - w_left,
            -lateral - w_right,
            xk[3] - 8.0,
            0.1 - xk[3],
            xk[4] - 0.5,
            -0.5 - xk[4],
            uk[0] - 4.0,
            -4.0 - uk[0],
            uk[1] - 2.5,
            -2.5 - uk[1],
        ])
        lbg.extend([-ca.inf] * 10)
        ubg.extend([0.0] * 10)

        def f(x: ca.MX, u: ca.MX) -> ca.MX:
            return ca.vertcat(
                x[3] * ca.cos(x[2]),
                x[3] * ca.sin(x[2]),
                x[3] * ca.tan(x[4]) / wheelbase,
                u[0],
                u[1],
            )

        # RK4 discretization to match MiniSolver default integrator (RK4_EXPLICIT).
        k1 = f(xk, uk)
        k2 = f(xk + 0.5 * dt * k1, uk)
        k3 = f(xk + 0.5 * dt * k2, uk)
        k4 = f(xk + dt * k3, uk)
        x_next = xk + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        g.append(X[k + 1] - x_next)
        lbg.extend([0.0] * 5)
        ubg.extend([0.0] * 5)

    obj += (
        10.0 * (X[N][0] - refs["x_ref"][N]) ** 2
        + 10.0 * (X[N][1] - refs["y_ref"][N]) ** 2
        + 2.5 * (ca.atan2(ca.sin(X[N][2] - refs["yaw_ref"][N]), ca.cos(X[N][2] - refs["yaw_ref"][N])) ** 2)
        + 1.0 * (X[N][3] - refs["v_ref"][N]) ** 2
        + 0.25 * X[N][4] ** 2
    )

    # Terminal stage state constraints (no control constraints).
    xN = X[N]
    x_refN = refs["x_ref"][N]
    y_refN = refs["y_ref"][N]
    yaw_refN = refs["yaw_ref"][N]
    w_leftN = refs["w_left"][N]
    w_rightN = refs["w_right"][N]
    nxN = -ca.sin(yaw_refN)
    nyN = ca.cos(yaw_refN)
    lateralN = nxN * (xN[0] - x_refN) + nyN * (xN[1] - y_refN)
    g.extend([
        lateralN - w_leftN,
        -lateralN - w_rightN,
        xN[3] - 8.0,
        0.1 - xN[3],
        xN[4] - 0.5,
        -0.5 - xN[4],
    ])
    lbg.extend([-ca.inf] * 6)
    ubg.extend([0.0] * 6)

    nlp = {"x": all_vars, "f": obj, "g": ca.vertcat(*g)}
    options = {
        "ipopt.print_level": 0,
        "print_time": 0,
        "ipopt.sb": "yes",
        "ipopt.tol": 1e-10,
        "ipopt.acceptable_tol": 1e-10,
        "ipopt.max_iter": 3000,
    }
    solver = ca.nlpsol("solve", "ipopt", nlp, options)

    x_guess = []
    for k in range(N + 1):
        x_guess.extend([
            refs["x_ref"][k],
            refs["y_ref"][k],
            refs["yaw_ref"][k],
            refs["v_ref"][k],
            0.0,
        ])
    x_guess.extend([0.0, 0.0] * N)

    sol = solver(x0=ca.DM(x_guess), lbg=ca.DM(lbg), ubg=ca.DM(ubg))
    w = sol["x"].full().ravel().astype(float)

    x = w[: 5 * (N + 1)].reshape((N + 1, 5))
    u = w[5 * (N + 1):].reshape((N, 2))
    return {
        "objective": float(sol["f"]),
        "terminal_state": x[-1].tolist(),
        "first_control": u[0].tolist(),
        "status": str(solver.stats().get("return_status", "unknown")),
        "solver": "ipopt",
    }


def solve_double_integrator_3d(N: int, dt: float, x0: np.ndarray, refs: dict[str, np.ndarray]) -> dict:
    X = [ca.MX.sym(f"X_{k}", 6) for k in range(N + 1)]
    U = [ca.MX.sym(f"U_{k}", 3) for k in range(N)]
    all_vars = ca.vertcat(*X, *U)

    obj = 0.0
    g = []
    lbg = []
    ubg = []

    g.append(X[0] - ca.DM(x0))
    lbg.extend([0.0] * 6)
    ubg.extend([0.0] * 6)
    for k in range(N):
        xk = X[k]
        uk = U[k]
        obj += 15.0 * (xk[0] - refs["x_ref"][k]) ** 2
        obj += 15.0 * (xk[1] - refs["y_ref"][k]) ** 2
        obj += 15.0 * (xk[2] - refs["z_ref"][k]) ** 2
        obj += 2.5 * (xk[3] - refs["vx_ref"][k]) ** 2
        obj += 2.5 * (xk[4] - refs["vy_ref"][k]) ** 2
        obj += 2.5 * (xk[5] - refs["vz_ref"][k]) ** 2
        obj += 0.05 * uk[0] ** 2
        obj += 0.05 * uk[1] ** 2
        obj += 0.05 * uk[2] ** 2

        # Inequality constraints are defined in "g(x) <= 0" form.
        g.extend([
            uk[0] - 12.0,
            -12.0 - uk[0],
            uk[1] - 12.0,
            -12.0 - uk[1],
            uk[2] - 12.0,
            -12.0 - uk[2],
        ])
        lbg.extend([-ca.inf] * 6)
        ubg.extend([0.0] * 6)

        def f(x: ca.MX, u: ca.MX) -> ca.MX:
            return ca.vertcat(x[3], x[4], x[5], u[0], u[1], u[2])

        # RK4 discretization to match MiniSolver default integrator (RK4_EXPLICIT).
        k1 = f(xk, uk)
        k2 = f(xk + 0.5 * dt * k1, uk)
        k3 = f(xk + 0.5 * dt * k2, uk)
        k4 = f(xk + dt * k3, uk)
        x_next = xk + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        g.append(X[k + 1] - x_next)
        lbg.extend([0.0] * 6)
        ubg.extend([0.0] * 6)

    obj += (
        15.0 * (X[N][0] - refs["x_ref"][N]) ** 2
        + 15.0 * (X[N][1] - refs["y_ref"][N]) ** 2
        + 15.0 * (X[N][2] - refs["z_ref"][N]) ** 2
        + 2.5 * (X[N][3] - refs["vx_ref"][N]) ** 2
        + 2.5 * (X[N][4] - refs["vy_ref"][N]) ** 2
        + 2.5 * (X[N][5] - refs["vz_ref"][N]) ** 2
    )

    nlp = {"x": all_vars, "f": obj, "g": ca.vertcat(*g)}
    options = {
        "ipopt.print_level": 0,
        "print_time": 0,
        "ipopt.sb": "yes",
        "ipopt.tol": 1e-10,
        "ipopt.acceptable_tol": 1e-10,
        "ipopt.max_iter": 3000,
    }
    solver = ca.nlpsol("solve", "ipopt", nlp, options)

    x_guess = []
    for k in range(N + 1):
        x_guess.extend([
            refs["x_ref"][k],
            refs["y_ref"][k],
            refs["z_ref"][k],
            refs["vx_ref"][k],
            refs["vy_ref"][k],
            refs["vz_ref"][k],
        ])
    x_guess.extend([0.0, 0.0, 0.0] * N)

    sol = solver(x0=ca.DM(x_guess), lbg=ca.DM(lbg), ubg=ca.DM(ubg))
    w = sol["x"].full().ravel().astype(float)

    x = w[: 6 * (N + 1)].reshape((N + 1, 6))
    u = w[6 * (N + 1):].reshape((N, 3))
    return {
        "objective": float(sol["f"]),
        "terminal_state": x[-1].tolist(),
        "first_control": u[0].tolist(),
        "status": str(solver.stats().get("return_status", "unknown")),
        "solver": "ipopt",
    }


def build_refs(row_data: np.ndarray, model: str) -> dict[str, np.ndarray]:
    if model == "kinematic_bicycle_v1":
        cols = {
            "t": 0,
            "x_ref": 1,
            "y_ref": 2,
            "yaw_ref": 3,
            "v_ref": 4,
            "w_left": 5,
            "w_right": 6,
        }
    else:
        cols = {
            "t": 0,
            "x_ref": 1,
            "y_ref": 2,
            "z_ref": 3,
            "vx_ref": 4,
            "vy_ref": 5,
            "vz_ref": 6,
        }
    return {k: row_data[:, v] for k, v in cols.items()}


def solve_scenario(scenario_path: Path) -> Path:
    scenario = load_json(scenario_path)
    model = scenario["model"]
    N = int(scenario["N"])
    dt = float(scenario["dt"])
    x0 = np.array(scenario["x0"], dtype=float)
    ref_path = scenario_path.parent / scenario["ref"]

    raw_rows = np.loadtxt(ref_path, delimiter=",", dtype=float, skiprows=1)
    if raw_rows.ndim == 1:
        raw_rows = raw_rows.reshape(1, -1)
    header = Path(ref_path).read_text(encoding="utf-8").splitlines()[0].split(",")
    row_data = raw_rows[: N + 1]
    refs = build_refs(row_data, model)
    if model == "kinematic_bicycle_v1":
        result = solve_kinematic_bicycle(N, dt, x0, refs)
    elif model == "double_integrator_3d_v1":
        result = solve_double_integrator_3d(N, dt, x0, refs)
    else:
        raise ValueError(f"unsupported model: {model}")

    result.update({
        "status": result.get("status", "UNKNOWN"),
        "solver": "ipopt",
        "tolerance": 1e-10,
    })
    gt_path = scenario_path.parent / "gt.json"
    gt_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    return gt_path


def discover_scenarios(root: Path) -> list[Path]:
    scenarios = []
    if not root.exists():
        return scenarios
    for candidate in root.iterdir():
        if not candidate.is_dir():
            continue
        sj = candidate / "scenario.json"
        if sj.exists():
            scenarios.append(sj)
    return sorted(scenarios)


def main() -> None:
    args = parse_args()
    args.scenario_root.mkdir(parents=True, exist_ok=True)
    if args.scenario:
        scenario_paths = args.scenario
    else:
        scenario_paths = discover_scenarios(args.scenario_root)

    outputs = {}
    for path in scenario_paths:
        gt_path = solve_scenario(path)
        outputs[str(path)] = str(gt_path)
        print(f"wrote {gt_path}")

    if not outputs:
        raise RuntimeError("no scenario found")


if __name__ == "__main__":
    main()
