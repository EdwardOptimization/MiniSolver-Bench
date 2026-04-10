#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import casadi as ca
import numpy as np


if __package__ in (None, ""):
    ROOT = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(ROOT))
    from cases import backend_case_names
else:
    ROOT = Path(__file__).resolve().parents[2]
    from ...cases import backend_case_names


INF = 1e20


def default_acados_repo() -> Path:
    submodule_dir = ROOT / "third_party" / "acados"
    if (submodule_dir / "CMakeLists.txt").exists():
        return submodule_dir
    return Path("/tmp/acados-official")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one official CasADi SQP benchmark case.")
    parser.add_argument("--case", choices=backend_case_names("casadi"), required=True)
    parser.add_argument("--acados-repo", type=Path, default=default_acados_repo())
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--output", type=Path, default=None)
    return parser.parse_args()


def run_with_env(cmd: list[str]) -> None:
    print("+", " ".join(str(part) for part in cmd))
    import subprocess

    subprocess.run(cmd, check=True)


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


def configure_acados_python(acados_repo: Path) -> None:
    acados_repo = acados_repo.resolve()
    template_root = acados_repo / "interfaces" / "acados_template"
    if str(template_root) not in sys.path:
        sys.path.insert(0, str(template_root))


def rk4_discrete(ode_fun: ca.Function, nx: int, nu: int, dt: float, np_dim: int = 0) -> ca.Function:
    x = ca.MX.sym("x", nx)
    u = ca.MX.sym("u", nu)
    args = [x, u]
    if np_dim > 0:
        p = ca.MX.sym("p", np_dim)
        args.append(p)
    else:
        p = None

    def call(xv: ca.MX) -> ca.MX:
        if p is None:
            return ode_fun(xv, u)
        return ode_fun(xv, u, p)

    k1 = call(x)
    k2 = call(x + 0.5 * dt * k1)
    k3 = call(x + 0.5 * dt * k2)
    k4 = call(x + dt * k3)
    x_next = x + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    if p is None:
        return ca.Function("F_rk4", [x, u], [x_next])
    return ca.Function("F_rk4", [x, u, p], [x_next])


@dataclass
class StepRecord:
    step: int
    status: int
    time_ms: float
    iterations: int
    metrics: dict[str, float]


CASADI_ACCEPTED_RETURNS = {
    "Solve_Succeeded",
    "Solved_To_Acceptable_Level",
    "Search_Direction_Becomes_Too_Small",
}


def casadi_status_code(stats: dict) -> int:
    if stats.get("success", False):
        return 0
    if stats.get("return_status") in CASADI_ACCEPTED_RETURNS:
        return 0
    if stats.get("unified_return_status") in CASADI_ACCEPTED_RETURNS:
        return 0
    return 1


def build_sqpmethod_solver(name: str, nlp: dict, *, max_iter: int, tol: float = 1e-4) -> ca.Function:
    opts = {
        "print_header": False,
        "print_iteration": False,
        "print_status": False,
        "print_time": False,
        "record_time": False,
        "max_iter": max_iter,
        "tol_pr": tol,
        "tol_du": tol,
        "qpsol": "qrqp",
        "qpsol_options": {
            "error_on_fail": False,
            "print_iter": False,
            "print_header": False,
        },
    }
    return ca.nlpsol(name, "sqpmethod", nlp, opts)


class BaseCasadiRunner:
    case_name: str

    def __init__(self, max_steps: int) -> None:
        self.max_steps = max_steps

    def initial_state(self) -> np.ndarray:
        raise NotImplementedError

    def warmup_runs(self) -> int:
        return 5

    def reset_guess(self, state: np.ndarray) -> None:
        raise NotImplementedError

    def warmup(self) -> None:
        state = self.initial_state().copy()
        self.reset_guess(state)
        for _ in range(self.warmup_runs()):
            _, control = self.solve_step(state)
            state = self.plant_step(state, control)
        self.reset_guess(self.initial_state().copy())

    def solve_step(self, state: np.ndarray) -> tuple[StepRecord, np.ndarray]:
        raise NotImplementedError

    def plant_step(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def should_stop(self, state: np.ndarray, step: int) -> bool:
        return step >= self.max_steps

    def csv_header(self) -> list[str]:
        raise NotImplementedError

    def csv_row(self, record: StepRecord) -> list[object]:
        raise NotImplementedError

    def print_summary(self, output: Path, records: list[StepRecord]) -> None:
        times = [record.time_ms for record in records]
        success = sum(1 for record in records if record.status == 0)
        print(
            f"case={self.case_name} steps={len(records)} success={success} "
            f"median_ms={percentile(times, 0.5):.6f} "
            f"p95_ms={percentile(times, 0.95):.6f} "
            f"max_ms={max(times) if times else 0.0:.6f} "
            f"output={output}"
        )

    def run(self, output: Path) -> None:
        self.warmup()
        state = self.initial_state().copy()
        records: list[StepRecord] = []
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(self.csv_header())
            fh.flush()
            for step in range(self.max_steps):
                if self.should_stop(state, step):
                    break
                record, control = self.solve_step(state)
                record.step = step
                records.append(record)
                writer.writerow(self.csv_row(record))
                fh.flush()
                state = self.plant_step(state, control)
        self.print_summary(output, records)


class MultipleShootingRunner(BaseCasadiRunner):
    def __init__(self, case_name: str, nx: int, nu: int, horizon: int, max_steps: int) -> None:
        super().__init__(max_steps)
        self.case_name = case_name
        self.nx = nx
        self.nu = nu
        self.horizon = horizon
        self.num_x = (horizon + 1) * nx
        self.num_u = horizon * nu
        self.num_s = 0
        self.soft_dim = 0
        self.last_w = np.zeros(self.num_x + self.num_u)
        self.last_control = np.zeros(self.nu)
        self.solver: ca.Function | None = None
        self.lbx: list[float] = []
        self.ubx: list[float] = []
        self.lbg: list[float] = []
        self.ubg: list[float] = []

    def make_initial_guess(self, state: np.ndarray) -> np.ndarray:
        guess = np.zeros(self.num_x + self.num_u + self.num_s)
        x_guess = np.tile(state.reshape(1, -1), (self.horizon + 1, 1))
        guess[: self.num_x] = x_guess.reshape(-1)
        return guess

    def reset_guess(self, state: np.ndarray) -> None:
        self.last_w = self.make_initial_guess(state)
        self.last_control = np.zeros(self.nu)

    def shift_guess(self, state: np.ndarray, solution: np.ndarray) -> None:
        x_block = solution[: self.num_x].reshape(self.horizon + 1, self.nx)
        u_block = solution[self.num_x : self.num_x + self.num_u].reshape(self.horizon, self.nu)
        shifted = np.zeros_like(solution)
        shifted_x = np.vstack([x_block[1:], x_block[-1:]])
        shifted_x[0] = state
        shifted[: self.num_x] = shifted_x.reshape(-1)
        shifted_u = np.vstack([u_block[1:], u_block[-1:]])
        shifted[self.num_x : self.num_x + self.num_u] = shifted_u.reshape(-1)
        if self.soft_dim > 0:
            s_block = solution[self.num_x + self.num_u :].reshape(self.horizon, self.soft_dim)
            shifted_s = np.vstack([s_block[1:], s_block[-1:]])
            shifted[self.num_x + self.num_u :] = shifted_s.reshape(-1)
        self.last_w = shifted

    def solve_nlp(self, params: np.ndarray) -> tuple[dict | None, dict]:
        assert self.solver is not None
        t0 = time.perf_counter()
        sol = None
        try:
            sol = self.solver(
                x0=self.last_w,
                lbx=self.lbx,
                ubx=self.ubx,
                lbg=self.lbg,
                ubg=self.ubg,
                p=params,
            )
            stats = self.solver.stats()
        except RuntimeError:
            stats = self.solver.stats()
        stats = dict(stats)
        stats["time_ms"] = (time.perf_counter() - t0) * 1000.0
        return sol, stats


class PendulumRunner(MultipleShootingRunner):
    def __init__(self, acados_repo: Path, max_steps: int) -> None:
        sys.path.insert(0, str(acados_repo / "examples" / "acados_python" / "pendulum_on_cart" / "common"))
        from pendulum_model import export_pendulum_ode_model

        model = export_pendulum_ode_model()
        self.tf = 0.8
        self.dt = self.tf / 40.0
        self.x0 = np.array([0.0, np.pi, 0.0, 0.0], dtype=float)
        self.f_fun = ca.Function("pendulum_f", [model.x, model.u], [model.f_expl_expr])
        self.F_fun = rk4_discrete(self.f_fun, 4, 1, self.dt)
        super().__init__("pendulum_on_cart", 4, 1, 40, max_steps)
        self._build_solver()

    def _build_solver(self) -> None:
        X = [ca.MX.sym(f"X_{k}", self.nx) for k in range(self.horizon + 1)]
        U = [ca.MX.sym(f"U_{k}", self.nu) for k in range(self.horizon)]
        P = ca.MX.sym("P", self.nx)
        q = ca.DM(np.diag([2e3, 2e3, 2e-2, 2e-2]))
        r = ca.DM(np.diag([2e-2]))

        w = X + U
        self.lbx = [-INF] * self.num_x + [-80.0] * self.num_u
        self.ubx = [INF] * self.num_x + [80.0] * self.num_u
        g = [X[0] - P]
        self.lbg = [0.0] * self.nx
        self.ubg = [0.0] * self.nx

        obj = 0
        for k in range(self.horizon):
            obj += 0.5 * ca.mtimes([X[k].T, q, X[k]]) + 0.5 * ca.mtimes([U[k].T, r, U[k]])
            g.append(X[k + 1] - self.F_fun(X[k], U[k]))
            self.lbg += [0.0] * self.nx
            self.ubg += [0.0] * self.nx
        obj += 0.5 * ca.mtimes([X[-1].T, q, X[-1]])

        self.solver = build_sqpmethod_solver(
            "pendulum_casadi",
            {"x": ca.vertcat(*w), "f": obj, "g": ca.vertcat(*g), "p": P},
            max_iter=40,
        )

    def initial_state(self) -> np.ndarray:
        return self.x0

    def solve_step(self, state: np.ndarray) -> tuple[StepRecord, np.ndarray]:
        sol, stats = self.solve_nlp(state)
        if sol is not None:
            solution = np.array(sol["x"], dtype=float).reshape(-1)
            control = solution[self.num_x : self.num_x + self.nu]
            self.shift_guess(state, solution)
        else:
            solution = self.last_w
            control = self.last_control
        self.last_control = control.copy()
        record = StepRecord(
            step=0,
            status=casadi_status_code(stats),
            time_ms=float(stats["time_ms"]),
            iterations=int(stats.get("iter_count", 0)),
            metrics={"theta": float(state[1]), "force": float(control[0])},
        )
        return record, control

    def plant_step(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        return np.array(self.F_fun(state, control), dtype=float).reshape(-1)

    def csv_header(self) -> list[str]:
        return ["step", "status", "time_ms", "iterations", "theta", "force"]

    def csv_row(self, record: StepRecord) -> list[object]:
        return [
            record.step,
            record.status,
            f"{record.time_ms:.9f}",
            record.iterations,
            f"{record.metrics['theta']:.9f}",
            f"{record.metrics['force']:.9f}",
        ]


class RaceCarsRunner(MultipleShootingRunner):
    SOFT_DIM = 10

    def __init__(self, acados_repo: Path, max_steps: int) -> None:
        sys.path.insert(0, str(acados_repo / "examples" / "acados_python" / "race_cars"))
        from bicycle_model import bicycle_model

        model, constraint = bicycle_model("LMS_Track.txt")
        self.track_pathlength = float(constraint.pathlength)
        self.tf = 1.0
        self.dt = self.tf / 50.0
        self.sref_n = 3.0
        self.x0 = np.array(model.x0, dtype=float).reshape(-1)
        self.h_fun = ca.Function("race_h", [model.x, model.u], [constraint.expr])
        self.f_fun = ca.Function("race_f", [model.x, model.u], [model.f_expl_expr])
        self.F_fun = rk4_discrete(self.f_fun, 6, 2, self.dt)
        self.unscale = 50.0 / self.tf
        self.stage_q = ca.DM(np.diag([1e-1, 1e-8, 1e-8, 1e-8, 1e-3, 5e-3]))
        self.stage_r = ca.DM(np.diag([1e-3, 5e-3]))
        self.term_q = ca.DM(np.diag([5e0, 1e1, 1e-8, 1e-8, 5e-3, 2e-3])) / self.unscale
        super().__init__("race_cars", 6, 2, 50, max_steps)
        self.soft_dim = self.SOFT_DIM
        self.num_s = self.horizon * self.soft_dim
        self.last_w = np.zeros(self.num_x + self.num_u + self.num_s)
        self._build_solver()

    def _build_solver(self) -> None:
        X = [ca.MX.sym(f"X_{k}", self.nx) for k in range(self.horizon + 1)]
        U = [ca.MX.sym(f"U_{k}", self.nu) for k in range(self.horizon)]
        S = [ca.MX.sym(f"S_{k}", self.soft_dim) for k in range(self.horizon)]
        P = ca.MX.sym("P", self.nx + 1)
        x0_param = P[: self.nx]
        s0_param = P[self.nx]

        w = X + U + S
        self.lbx = [0.0] * (self.num_x + self.num_u + self.num_s)
        self.ubx = [0.0] * (self.num_x + self.num_u + self.num_s)
        for stage in range(self.horizon + 1):
            self.lbx[stage * self.nx : (stage + 1) * self.nx] = [-INF] * self.nx
            self.ubx[stage * self.nx : (stage + 1) * self.nx] = [INF] * self.nx
            self.lbx[stage * self.nx + 1] = -12.0
            self.ubx[stage * self.nx + 1] = 12.0
        u_offset = self.num_x
        for k in range(self.horizon):
            self.lbx[u_offset + k * self.nu : u_offset + k * self.nu + self.nu] = [-10.0, -2.0]
            self.ubx[u_offset + k * self.nu : u_offset + k * self.nu + self.nu] = [10.0, 2.0]
        s_offset = self.num_x + self.num_u
        for k in range(self.horizon):
            self.lbx[s_offset + k * self.soft_dim : s_offset + (k + 1) * self.soft_dim] = [0.0] * self.soft_dim
            self.ubx[s_offset + k * self.soft_dim : s_offset + (k + 1) * self.soft_dim] = [INF] * self.soft_dim

        g = [X[0] - x0_param]
        self.lbg = [0.0] * self.nx
        self.ubg = [0.0] * self.nx
        obj = 0

        for k in range(self.horizon):
            sref_k = s0_param + self.sref_n * (k / self.horizon)
            xref = ca.vertcat(sref_k, 0, 0, 0, 0, 0)
            obj += 0.5 * self.unscale * ca.mtimes([(X[k] - xref).T, self.stage_q, (X[k] - xref)])
            obj += 0.5 * self.unscale * ca.mtimes([U[k].T, self.stage_r, U[k]])
            obj += 100.0 * ca.sum1(S[k]) + 0.5 * ca.dot(S[k], S[k])
            g.append(X[k + 1] - self.F_fun(X[k], U[k]))
            self.lbg += [0.0] * self.nx
            self.ubg += [0.0] * self.nx

            h = self.h_fun(X[k], U[k])
            raw = ca.vertcat(
                h[0] - 4.0,
                -4.0 - h[0],
                h[1] - 4.0,
                -4.0 - h[1],
                h[2] - 0.12,
                -0.12 - h[2],
                h[3] - 1.0,
                -1.0 - h[3],
                h[4] - 0.40,
                -0.40 - h[4],
            )
            g.append(raw - S[k])
            self.lbg += [-INF] * self.soft_dim
            self.ubg += [0.0] * self.soft_dim

        sref_terminal = s0_param + self.sref_n
        xref_terminal = ca.vertcat(sref_terminal, 0, 0, 0, 0, 0)
        obj += 0.5 * ca.mtimes([(X[-1] - xref_terminal).T, self.term_q, (X[-1] - xref_terminal)])

        self.solver = build_sqpmethod_solver(
            "race_casadi",
            {"x": ca.vertcat(*w), "f": obj, "g": ca.vertcat(*g), "p": P},
            max_iter=20,
        )

    def initial_state(self) -> np.ndarray:
        return self.x0

    def solve_step(self, state: np.ndarray) -> tuple[StepRecord, np.ndarray]:
        params = np.concatenate([state, [state[0]]])
        sol, stats = self.solve_nlp(params)
        if sol is not None:
            solution = np.array(sol["x"], dtype=float).reshape(-1)
            control = solution[self.num_x : self.num_x + self.nu]
            self.shift_guess(state, solution)
        else:
            control = self.last_control
        self.last_control = control.copy()
        next_state = self.plant_step(state, control)
        record = StepRecord(
            step=0,
            status=casadi_status_code(stats),
            time_ms=float(stats["time_ms"]),
            iterations=int(stats.get("iter_count", 0)),
            metrics={"s": float(next_state[0]), "v": float(next_state[3])},
        )
        return record, control

    def plant_step(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        return np.array(self.F_fun(state, control), dtype=float).reshape(-1)

    def should_stop(self, state: np.ndarray, step: int) -> bool:
        return step >= self.max_steps or state[0] > self.track_pathlength + 0.1

    def csv_header(self) -> list[str]:
        return ["step", "status", "time_ms", "iterations", "s", "v"]

    def csv_row(self, record: StepRecord) -> list[object]:
        return [
            record.step,
            record.status,
            f"{record.time_ms:.9f}",
            record.iterations,
            f"{record.metrics['s']:.9f}",
            f"{record.metrics['v']:.9f}",
        ]


class QuadrotorRunner(MultipleShootingRunner):
    def __init__(self, acados_repo: Path, max_steps: int) -> None:
        sys.path.insert(0, str(acados_repo / "examples" / "acados_python" / "quadrotor_nav"))
        import common
        from sys_dynamics import SysDyn

        zeta_f, dyn_f, u, proj_constr, _dyn_fun = SysDyn().SetupOde()
        self.common = common
        self.tf = float(common.Tf)
        self.dt = self.tf / float(common.N)
        self.x0 = np.array(common.init_zeta, dtype=float).reshape(-1)
        self.f_fun = ca.Function("quad_f", [zeta_f, u], [dyn_f])
        self.F_fun = rk4_discrete(self.f_fun, 20, 4, self.dt)
        self.h_fun = ca.Function("quad_h", [zeta_f, u], [proj_constr])
        super().__init__("quadrotor_nav", 20, 4, int(common.N), max_steps)
        self._build_solver()

    def _build_solver(self) -> None:
        X = [ca.MX.sym(f"X_{k}", self.nx) for k in range(self.horizon + 1)]
        U = [ca.MX.sym(f"U_{k}", self.nu) for k in range(self.horizon)]
        P = ca.MX.sym("P", self.nx)
        x0_param = P
        s0_param = P[0]

        w = X + U
        self.lbx = [-INF] * (self.num_x + self.num_u)
        self.ubx = [INF] * (self.num_x + self.num_u)
        g = [X[0] - x0_param]
        self.lbg = [0.0] * self.nx
        self.ubg = [0.0] * self.nx
        obj = 0
        q = ca.DM(self.common.Q)
        qn = ca.DM(self.common.Qn)
        r = ca.DM(self.common.R)

        for k in range(self.horizon):
            sref_k = s0_param + self.common.S_REF * (k / self.horizon)
            xref = ca.vertcat(
                sref_k, 0, 0,
                1, 0, 0, 0,
                self.common.S_REF / self.common.Tf, 0, 0,
                0, 0, 0,
                0, 0, 0,
                self.common.U_HOV, self.common.U_HOV, self.common.U_HOV, self.common.U_HOV,
            )
            obj += 0.5 * ca.mtimes([(X[k] - xref).T, q, (X[k] - xref)]) + 0.5 * ca.mtimes([U[k].T, r, U[k]])
            g.append(X[k + 1] - self.F_fun(X[k], U[k]))
            self.lbg += [0.0] * self.nx
            self.ubg += [0.0] * self.nx
            g.append(ca.vertcat(self.h_fun(X[k], U[k]) - 1.0))
            self.lbg += [-INF]
            self.ubg += [0.0]

        xref_terminal = ca.vertcat(
            s0_param + self.common.S_REF, 0, 0,
            1, 0, 0, 0,
            self.common.S_REF / self.common.Tf, 0, 0,
            0, 0, 0,
            0, 0, 0,
            self.common.U_HOV, self.common.U_HOV, self.common.U_HOV, self.common.U_HOV,
        )
        obj += 0.5 * ca.mtimes([(X[-1] - xref_terminal).T, qn, (X[-1] - xref_terminal)])
        g.append(ca.vertcat(self.h_fun(X[-1], ca.DM.zeros(self.nu)) - 1.0))
        self.lbg += [-INF]
        self.ubg += [0.0]

        self.solver = build_sqpmethod_solver(
            "quad_casadi",
            {"x": ca.vertcat(*w), "f": obj, "g": ca.vertcat(*g), "p": P},
            max_iter=20,
        )

    def initial_state(self) -> np.ndarray:
        return self.x0

    def solve_step(self, state: np.ndarray) -> tuple[StepRecord, np.ndarray]:
        sol, stats = self.solve_nlp(state)
        if sol is not None:
            solution = np.array(sol["x"], dtype=float).reshape(-1)
            control = solution[self.num_x : self.num_x + self.nu]
            self.shift_guess(state, solution)
        else:
            control = self.last_control
        self.last_control = control.copy()
        next_state = self.plant_step(state, control)
        record = StepRecord(
            step=0,
            status=casadi_status_code(stats),
            time_ms=float(stats["time_ms"]),
            iterations=int(stats.get("iter_count", 0)),
            metrics={"abs_n": float(abs(next_state[1])), "abs_b": float(abs(next_state[2]))},
        )
        return record, control

    def plant_step(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        return np.array(self.F_fun(state, control), dtype=float).reshape(-1)

    def should_stop(self, state: np.ndarray, step: int) -> bool:
        return step >= self.max_steps or state[0] >= self.common.S_MAX

    def csv_header(self) -> list[str]:
        return ["step", "status", "time_ms", "iterations", "abs_n", "abs_b"]

    def csv_row(self, record: StepRecord) -> list[object]:
        return [
            record.step,
            record.status,
            f"{record.time_ms:.9f}",
            record.iterations,
            f"{record.metrics['abs_n']:.9f}",
            f"{record.metrics['abs_b']:.9f}",
        ]


class ChainMassRunner(MultipleShootingRunner):
    def __init__(self, acados_repo: Path, max_steps: int) -> None:
        sys.path.insert(0, str(acados_repo / "examples" / "acados_python" / "chain_mass"))
        from export_disturbed_chain_mass_model import export_disturbed_chain_mass_model
        from utils import compute_steady_state, get_chain_params

        params = get_chain_params()
        self.params = params
        n_mass = params["n_mass"]
        m = params["m"]
        d_spring = params["D"]
        rest_length = params["L"]
        m_intermediate = n_mass - 2
        x_pos_first_mass = np.zeros((3, 1))
        x_end_ref = np.zeros((3, 1))
        x_end_ref[0] = rest_length * (m_intermediate + 1) * 6
        self.xrest = np.array(
            compute_steady_state(n_mass, m, d_spring, rest_length, x_pos_first_mass, x_end_ref),
            dtype=float,
        ).reshape(-1)
        model = export_disturbed_chain_mass_model(n_mass, m, d_spring, rest_length)
        self.ts = float(params["Ts"])
        self.f_fun = ca.Function("chain_f", [model.x, model.u, model.p], [model.f_expl_expr])
        self.F_fun = rk4_discrete(self.f_fun, int(model.x.size1()), int(model.u.size1()), self.ts, int(model.p.size1()))
        self.n_mass = n_mass
        self.M = m_intermediate
        self.y_pos_wall = float(params["yPosWall"])
        self.u_init = np.array(params["u_init"], dtype=float)
        self.rng = np.random.default_rng(int(params["seed"]))
        self._pending_next_state: np.ndarray | None = None
        super().__init__("chain_mass", int(model.x.size1()), int(model.u.size1()), int(params["N"]), max_steps)
        self.soft_dim = self.M + 1
        self.num_s = self.horizon * self.soft_dim
        self.last_w = np.zeros(self.num_x + self.num_u + self.num_s)
        self._build_solver()

    def _build_solver(self) -> None:
        X = [ca.MX.sym(f"X_{k}", self.nx) for k in range(self.horizon + 1)]
        U = [ca.MX.sym(f"U_{k}", self.nu) for k in range(self.horizon)]
        S = [ca.MX.sym(f"S_{k}", self.soft_dim) for k in range(self.horizon)]
        P = ca.MX.sym("P", self.nx)
        x0_param = P

        w = X + U + S
        self.lbx = [-INF] * (self.num_x + self.num_u + self.num_s)
        self.ubx = [INF] * (self.num_x + self.num_u + self.num_s)
        u_offset = self.num_x
        for k in range(self.horizon):
            self.lbx[u_offset + k * self.nu : u_offset + k * self.nu + self.nu] = [-1.0] * self.nu
            self.ubx[u_offset + k * self.nu : u_offset + k * self.nu + self.nu] = [1.0] * self.nu
        s_offset = self.num_x + self.num_u
        for k in range(self.horizon):
            self.lbx[s_offset + k * self.soft_dim : s_offset + (k + 1) * self.soft_dim] = [0.0] * self.soft_dim
            self.ubx[s_offset + k * self.soft_dim : s_offset + (k + 1) * self.soft_dim] = [INF] * self.soft_dim

        g = [X[0] - x0_param]
        self.lbg = [0.0] * self.nx
        self.ubg = [0.0] * self.nx
        obj = 0

        stage_wx = np.ones(self.nx) * 2.0
        strong_penalty = float(self.M + 1)
        stage_wx[3 * self.M + 0] = 2.0 * strong_penalty
        stage_wx[3 * self.M + 1] = 2.0 * strong_penalty
        stage_wx[3 * self.M + 2] = 2.0 * strong_penalty
        q = ca.DM(np.diag(stage_wx))
        r = ca.DM(np.diag([0.02, 0.02, 0.02]))
        xref = ca.DM(self.xrest)
        zero_w = ca.DM.zeros(self.M * 3)

        for k in range(self.horizon):
            obj += 0.5 * ca.mtimes([(X[k] - xref).T, q, (X[k] - xref)]) + 0.5 * ca.mtimes([U[k].T, r, U[k]])
            obj += ca.sum1(S[k]) + 0.5 * 1e3 * ca.dot(S[k], S[k])
            g.append(X[k + 1] - self.F_fun(X[k], U[k], zero_w))
            self.lbg += [0.0] * self.nx
            self.ubg += [0.0] * self.nx
            wall_residual = []
            for i in range(self.soft_dim):
                y_idx = 3 * i + 1
                wall_residual.append(self.y_pos_wall - X[k][y_idx] - S[k][i])
            g.append(ca.vertcat(*wall_residual))
            self.lbg += [-INF] * self.soft_dim
            self.ubg += [0.0] * self.soft_dim

        obj += 0.5 * ca.mtimes([(X[-1] - xref).T, q, (X[-1] - xref)])

        self.solver = build_sqpmethod_solver(
            "chain_casadi",
            {"x": ca.vertcat(*w), "f": obj, "g": ca.vertcat(*g), "p": P},
            max_iter=int(self.params["nlp_iter"]),
            tol=float(self.params["nlp_tol"]),
        )

    def initial_state(self) -> np.ndarray:
        zero_w = np.zeros(self.M * 3)
        state = self.xrest.copy()
        for _ in range(5):
            state = np.array(self.F_fun(state, self.u_init, zero_w), dtype=float).reshape(-1)
        return state

    def sample_disturbance(self) -> np.ndarray:
        dim = self.M * 3
        direction = self.rng.normal(size=dim)
        direction /= max(np.linalg.norm(direction), 1e-12)
        radius = self.rng.random() ** (1.0 / dim)
        return direction * radius * math.sqrt(float(self.params["perturb_scale"]))

    def chain_wall_distance(self, state: np.ndarray) -> float:
        return min(float(state[3 * i + 1] - self.y_pos_wall) for i in range(self.soft_dim))

    def solve_step(self, state: np.ndarray) -> tuple[StepRecord, np.ndarray]:
        sol, stats = self.solve_nlp(state)
        if sol is not None:
            solution = np.array(sol["x"], dtype=float).reshape(-1)
            control = solution[self.num_x : self.num_x + self.nu]
            self.shift_guess(state, solution)
        else:
            control = self.last_control
        self.last_control = control.copy()
        disturbance = self.sample_disturbance()
        next_state = np.array(self.F_fun(state, control, disturbance), dtype=float).reshape(-1)
        self._pending_next_state = next_state
        record = StepRecord(
            step=0,
            status=casadi_status_code(stats),
            time_ms=float(stats["time_ms"]),
            iterations=int(stats.get("iter_count", 0)),
            metrics={"wall_dist": self.chain_wall_distance(next_state)},
        )
        return record, control

    def plant_step(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        if self._pending_next_state is not None:
            next_state = self._pending_next_state
            self._pending_next_state = None
            return next_state
        disturbance = self.sample_disturbance()
        return np.array(self.F_fun(state, control, disturbance), dtype=float).reshape(-1)

    def csv_header(self) -> list[str]:
        return ["step", "status", "time_ms", "iterations", "wall_dist"]

    def csv_row(self, record: StepRecord) -> list[object]:
        return [
            record.step,
            record.status,
            f"{record.time_ms:.9f}",
            record.iterations,
            f"{record.metrics['wall_dist']:.9f}",
        ]


RUNNERS = {
    "pendulum_on_cart": PendulumRunner,
    "race_cars": RaceCarsRunner,
    "quadrotor_nav": QuadrotorRunner,
    "chain_mass": ChainMassRunner,
}


def main() -> int:
    args = parse_args()
    configure_acados_python(args.acados_repo)

    default_steps = {
        "pendulum_on_cart": 100,
        "race_cars": 500,
        "quadrotor_nav": 2250,
        "chain_mass": 25,
    }
    steps = args.steps if args.steps is not None else default_steps[args.case]
    runner = RUNNERS[args.case](args.acados_repo.resolve(), steps)
    output = args.output or (ROOT / "results" / "raw" / "casadi" / f"{args.case}.csv")
    runner.run(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
