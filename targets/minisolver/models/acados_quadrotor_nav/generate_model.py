import os
import sys
from pathlib import Path

import numpy as np
import sympy as sp
from scipy.interpolate import PPoly, make_interp_spline
from sympy.printing.c import C99CodePrinter

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from targets.minisolver.models.common import (
    acados_repo_dir,
    ensure_minisolver_python_path,
)


ensure_minisolver_python_path()

from minisolver.MiniModel import OptimalControlModel  # noqa: E402


_NORM_EPS = 1e-12


class _Norm3(sp.Function):
    nargs = 3

    @classmethod
    def eval(cls, a, b, c):
        return None

    def fdiff(self, argindex):
        return self.args[argindex - 1] / self

    def _ccode(self, printer):
        a, b, c = (printer._print(arg) for arg in self.args)
        return f"sqrt({a}*{a} + {b}*{b} + {c}*{c} + {_NORM_EPS})"

    def _sympystr(self, printer):
        return f"_Norm3({', '.join(str(a) for a in self.args)})"


_orig_ccode = C99CodePrinter._print_Function


def _ccode_with_norm3(self, expr):
    if isinstance(expr, _Norm3):
        a, b, c = (self._print(arg) for arg in expr.args)
        return f"sqrt({a}*{a} + {b}*{b} + {c}*{c} + {_NORM_EPS})"
    return _orig_ccode(self, expr)


C99CodePrinter._print_Function = _ccode_with_norm3


def load_track() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    track_path = (
        acados_repo_dir()
        / "examples"
        / "acados_python"
        / "quadrotor_nav"
        / "tracks"
        / "trefoil_track.txt"
    )
    data = np.loadtxt(track_path, skiprows=1)
    data = data[1:, :]
    return data[:, 0], data[:, 1], data[:, 2], data[:, 3]


if __name__ == "__main__":
    print("Building symbolic quadrotor_nav model...", flush=True)
    model = OptimalControlModel("QuadrotorNavModel")

    s, n, b = model.state("s", "n", "b")
    q1, q2, q3, q4 = model.state("q1", "q2", "q3", "q4")
    s_dot, n_dot, b_dot = model.state("sDot", "nDot", "bDot")
    wr, wp, wy = model.state("wr", "wp", "wy")
    vx, vy, vz = model.state("vx", "vy", "vz")
    ohm1, ohm2, ohm3, ohm4 = model.state("ohm1", "ohm2", "ohm3", "ohm4")
    alpha1, alpha2, alpha3, alpha4 = model.control("alpha1", "alpha2", "alpha3", "alpha4")

    x_ref = [model.parameter(f"xref{i}") for i in range(20)]
    u_ref = [model.parameter(f"uref{i}") for i in range(4)]
    w_x = [model.parameter(f"wx{i}") for i in range(20)]
    w_u = [model.parameter(f"wu{i}") for i in range(4)]

    s_samples, x_samples, y_samples, z_samples = load_track()
    spline_x = PPoly.from_spline(make_interp_spline(s_samples, x_samples, k=5))
    spline_y = PPoly.from_spline(make_interp_spline(s_samples, y_samples, k=5))
    spline_z = PPoly.from_spline(make_interp_spline(s_samples, z_samples, k=5))

    spline_x_breaks = spline_x.x.tolist()
    spline_x_coeffs = [spline_x.c[:, i].tolist() for i in range(spline_x.c.shape[1])]
    spline_y_breaks = spline_y.x.tolist()
    spline_y_coeffs = [spline_y.c[:, i].tolist() for i in range(spline_y.c.shape[1])]
    spline_z_breaks = spline_z.x.tolist()
    spline_z_coeffs = [spline_z.c[:, i].tolist() for i in range(spline_z.c.shape[1])]

    track_x = model.ppoly("quad_track_x", s, spline_x_breaks, spline_x_coeffs)
    track_y = model.ppoly("quad_track_y", s, spline_y_breaks, spline_y_coeffs)
    track_z = model.ppoly("quad_track_z", s, spline_z_breaks, spline_z_coeffs)

    gamma1 = sp.Matrix([
        sp.diff(track_x, s),
        sp.diff(track_y, s),
        sp.diff(track_z, s),
    ])
    gamma2 = sp.Matrix([
        sp.diff(track_x, s, 2),
        sp.diff(track_y, s, 2),
        sp.diff(track_z, s, 2),
    ])
    gamma3 = sp.Matrix([
        sp.diff(track_x, s, 3),
        sp.diff(track_y, s, 3),
        sp.diff(track_z, s, 3),
    ])

    kap = _Norm3(gamma2[0], gamma2[1], gamma2[2])
    et = gamma1
    en = gamma2 / kap
    eb = et.cross(en)
    kap_bar = sp.diff(kap, s)
    et_bar = gamma2
    en_bar = (kap**2 * gamma3 - (gamma2.dot(gamma3)) * gamma2) / kap**3
    eb_bar = et.cross(gamma3) / kap
    tau = en.dot(eb_bar)
    tau_bar = sp.diff(tau, s)

    g0 = 9.80665
    mq = 31e-3
    ix = 1.395e-5
    iy = 1.395e-5
    iz = 2.173e-5
    cd = 7.9379e-06
    ct = 3.25e-4
    arm = 92e-3 / 2.0

    v_c = sp.Matrix([vx, vy, vz])
    omg = sp.Matrix([wr, wp, wy])

    drag = sp.Matrix([(cd / mq) * vx * 2.0, (cd / mq) * vy * 2.0, (cd / mq) * vz**2])
    thrust = sp.Matrix([0.0, 0.0, ct * (ohm1**2 + ohm2**2 + ohm3**2 + ohm4**2)])
    gravity = sp.Matrix([0.0, 0.0, g0])

    rotation = sp.Matrix([
        [2 * (q1**2 + q2**2) - 1, -2 * (q1 * q4 - q2 * q3), 2 * (q1 * q3 + q2 * q4)],
        [2 * (q1 * q4 + q2 * q3), 2 * (q1**2 + q3**2) - 1, -2 * (q1 * q2 - q3 * q4)],
        [-2 * (q1 * q3 - q2 * q4), 2 * (q1 * q2 + q3 * q4), 2 * (q1**2 + q4**2) - 1],
    ])

    moments = sp.Matrix([
        ct * arm * (ohm1**2 + ohm2**2 - ohm3**2 - ohm4**2),
        ct * arm * (ohm1**2 - ohm2**2 - ohm3**2 + ohm4**2),
        cd * (ohm1**2 - ohm2**2 + ohm3**2 - ohm4**2),
    ])

    q1_dot = (-(q2 * wr) - (q3 * wp) - (q4 * wy)) / 2
    q2_dot = ((q1 * wr) - (q4 * wp) + (q3 * wy)) / 2
    q3_dot = ((q4 * wr) + (q1 * wp) - (q2 * wy)) / 2
    q4_dot = (-(q3 * wr) + (q2 * wp) + (q1 * wy)) / 2

    v_dot_c = -gravity + rotation * thrust / mq - drag
    omg_dot = sp.Matrix([
        (moments[0] - (iy - iz) * wp * wy) / ix,
        (moments[1] - (iz - ix) * wr * wy) / iy,
        (moments[2] - (ix - iy) * wr * wp) / iz,
    ])

    denom = 1.0 - kap * n
    s_prog_dot = et.dot(v_c) / denom
    n_prog_dot = en.dot(v_c) + tau * s_prog_dot * b
    b_prog_dot = eb.dot(v_c) - tau * s_prog_dot * n

    kap_dot = kap_bar * s_prog_dot
    tau_dot = tau_bar * s_prog_dot
    et_dot = et_bar * s_prog_dot
    en_dot = en_bar * s_prog_dot
    eb_dot = eb_bar * s_prog_dot

    s_acc = (et.dot(v_dot_c) + et_dot.dot(v_c)) / denom + et.dot(v_c) * (
        (n * kap_dot + n_prog_dot * kap) / denom**2
    )
    n_acc = en.dot(v_dot_c) + en_dot.dot(v_c) + tau_dot * s_prog_dot * b + tau * s_acc * b + tau * s_prog_dot * b_prog_dot
    b_acc = eb.dot(v_dot_c) + eb_dot.dot(v_c) - tau_dot * s_prog_dot * n - tau * s_acc * n - tau * s_prog_dot * n_prog_dot

    model.set_dynamics(s, s_prog_dot)
    model.set_dynamics(n, n_prog_dot)
    model.set_dynamics(b, b_prog_dot)
    model.set_dynamics(q1, q1_dot)
    model.set_dynamics(q2, q2_dot)
    model.set_dynamics(q3, q3_dot)
    model.set_dynamics(q4, q4_dot)
    model.set_dynamics(s_dot, s_acc)
    model.set_dynamics(n_dot, n_acc)
    model.set_dynamics(b_dot, b_acc)
    model.set_dynamics(wr, omg_dot[0])
    model.set_dynamics(wp, omg_dot[1])
    model.set_dynamics(wy, omg_dot[2])
    model.set_dynamics(vx, v_dot_c[0])
    model.set_dynamics(vy, v_dot_c[1])
    model.set_dynamics(vz, v_dot_c[2])
    model.set_dynamics(ohm1, alpha1)
    model.set_dynamics(ohm2, alpha2)
    model.set_dynamics(ohm3, alpha3)
    model.set_dynamics(ohm4, alpha4)

    states = [s, n, b, q1, q2, q3, q4, s_dot, n_dot, b_dot, wr, wp, wy, vx, vy, vz, ohm1, ohm2, ohm3, ohm4]
    controls = [alpha1, alpha2, alpha3, alpha4]
    for i, state in enumerate(states):
        diff = state - x_ref[i]
        model.minimize(sp.Rational(1, 2) * w_x[i] * diff * diff)
    for i, control in enumerate(controls):
        diff = control - u_ref[i]
        model.minimize(sp.Rational(1, 2) * w_u[i] * diff * diff)

    model.subject_to(kap * n <= 1.0)

    output_dir = Path(__file__).resolve().parents[2] / "generated"
    print("Generating C++ model header from spline track geometry...", flush=True)
    # The official acados example uses Gauss-Newton (no constraint Hessian). Computing constraint Hessians
    # can be very expensive for this model, so skip them during codegen.
    model.generate(
        str(output_dir),
        use_fused_riccati=False,
        dynamics_mode="continuous_rk",
        include_constraint_hessian=False,
    )
    print(f"Generated QuadrotorNavModel in {output_dir}")
