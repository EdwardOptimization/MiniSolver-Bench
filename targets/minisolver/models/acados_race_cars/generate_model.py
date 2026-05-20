import os
import sys
from pathlib import Path

import numpy as np
import sympy as sp
from scipy.interpolate import PPoly, make_interp_spline

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from targets.minisolver.models.common import (
    acados_repo_dir,
    ensure_minisolver_python_path,
    set_dynamics,
)


ensure_minisolver_python_path()

from minisolver.MiniModel import OptimalControlModel  # noqa: E402


def load_track() -> tuple[np.ndarray, np.ndarray]:
    track_path = (
        acados_repo_dir()
        / "examples"
        / "acados_python"
        / "race_cars"
        / "tracks"
        / "LMS_Track.txt"
    )
    data = np.loadtxt(track_path)
    s_ref = data[:, 0]
    kappa_ref = data[:, 4]
    length = len(s_ref)

    s_ext = np.append(s_ref, [s_ref[length - 1] + s_ref[1:length]])
    kappa_ext = np.append(kappa_ref, kappa_ref[1:length])
    s_ext = np.append([-s_ext[length - 2] + s_ext[length - 81 : length - 2]], s_ext)
    kappa_ext = np.append(kappa_ext[length - 80 : length - 1], kappa_ext)
    return s_ext, kappa_ext


if __name__ == "__main__":
    print("Building symbolic race_cars model...", flush=True)
    model = OptimalControlModel("RaceCarsModel")

    s, n, alpha, v, throttle, delta = model.state("s", "n", "alpha", "v", "D", "delta")
    der_throttle, der_delta = model.control("derD", "derDelta")

    x_ref = [model.parameter(f"xref{i}") for i in range(6)]
    u_ref = [model.parameter(f"uref{i}") for i in range(2)]
    w_x = [model.parameter(f"wx{i}") for i in range(6)]
    w_u = [model.parameter(f"wu{i}") for i in range(2)]

    s_samples, kappa_samples = load_track()
    spline = make_interp_spline(s_samples, kappa_samples, k=3)
    kappa_ppoly = PPoly.from_spline(spline)
    kappa_breaks = kappa_ppoly.x.tolist()
    kappa_coeffs = [kappa_ppoly.c[:, i].tolist() for i in range(kappa_ppoly.c.shape[1])]
    kappa = model.ppoly("race_kappa", s, kappa_breaks, kappa_coeffs)

    m = 0.043
    c1 = 0.5
    c2 = 15.5
    cm1 = 0.28
    cm2 = 0.05
    cr0 = 0.011
    cr2 = 0.006

    fxd = (cm1 - cm2 * v) * throttle - cr2 * v**2 - cr0 * sp.tanh(5.0 * v)
    sdot = (v * sp.cos(alpha + c1 * delta)) / (1.0 - kappa * n)
    alat = c2 * v**2 * delta + fxd * sp.sin(c1 * delta) / m
    along = fxd / m

    set_dynamics(model, s, sdot)
    set_dynamics(model, n, v * sp.sin(alpha + c1 * delta))
    set_dynamics(model, alpha, v * c2 * delta - kappa * sdot)
    set_dynamics(model, v, along * sp.cos(c1 * delta))
    set_dynamics(model, throttle, der_throttle)
    set_dynamics(model, delta, der_delta)

    states = [s, n, alpha, v, throttle, delta]
    controls = [der_throttle, der_delta]
    for i, state in enumerate(states):
        diff = state - x_ref[i]
        model.minimize(sp.Rational(1, 2) * w_x[i] * diff * diff)
    for i, control in enumerate(controls):
        diff = control - u_ref[i]
        model.minimize(sp.Rational(1, 2) * w_u[i] * diff * diff)

    # Match official benchmark's soft-constraint configuration:
    # first 10 bounds are L1-soft with weight=100; input-rate bounds are hard.
    model.subject_to(along <= 4.0, weight=100.0, loss="L1")
    model.subject_to(along >= -4.0, weight=100.0, loss="L1")
    model.subject_to(alat <= 4.0, weight=100.0, loss="L1")
    model.subject_to(alat >= -4.0, weight=100.0, loss="L1")
    model.subject_to(n <= 0.12, weight=100.0, loss="L1")
    model.subject_to(n >= -0.12, weight=100.0, loss="L1")
    model.subject_to(throttle <= 1.0, weight=100.0, loss="L1")
    model.subject_to(throttle >= -1.0, weight=100.0, loss="L1")
    model.subject_to(delta <= 0.40, weight=100.0, loss="L1")
    model.subject_to(delta >= -0.40, weight=100.0, loss="L1")
    model.subject_to(der_throttle <= 10.0)
    model.subject_to(der_throttle >= -10.0)
    model.subject_to(der_delta <= 2.0)
    model.subject_to(der_delta >= -2.0)

    output_dir = Path(__file__).resolve().parents[2] / "generated"
    print("Generating C++ model header from bspline-expanded curvature...", flush=True)
    model.generate(str(output_dir), use_fused_riccati=False, dynamics_mode="continuous_rk")
    print(f"Generated RaceCarsModel in {output_dir}")
