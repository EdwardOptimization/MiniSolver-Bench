from __future__ import annotations

import sys
from pathlib import Path

import sympy as sp

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT))

from targets.minisolver.models.common import ensure_minisolver_python_path


ensure_minisolver_python_path()

from minisolver.MiniModel import OptimalControlModel  # noqa: E402


WHEELBASE = 0.33
V_MIN = 0.1
V_MAX = 12.0
DELTA_MIN = -0.50
DELTA_MAX = 0.50
A_MAX = 15.0
DELTA_RATE_MAX = 6.0


if __name__ == "__main__":
    model = OptimalControlModel("KinematicBicycleTrackModel")

    x, y, psi, v, delta = model.state("x", "y", "psi", "v", "delta")
    accel, delta_rate = model.control("a", "delta_rate")

    x_ref = model.parameter("x_ref")
    y_ref = model.parameter("y_ref")
    psi_ref = model.parameter("psi_ref")
    v_ref = model.parameter("v_ref")
    n_x = model.parameter("n_x")
    n_y = model.parameter("n_y")
    w_left = model.parameter("w_left")
    w_right = model.parameter("w_right")

    psi_error = sp.atan2(sp.sin(psi - psi_ref), sp.cos(psi - psi_ref))
    lateral = n_x * (x - x_ref) + n_y * (y - y_ref)

    model.set_dynamics(x, v * sp.cos(psi))
    model.set_dynamics(y, v * sp.sin(psi))
    model.set_dynamics(psi, v * sp.tan(delta) / WHEELBASE)
    model.set_dynamics(v, accel)
    model.set_dynamics(delta, delta_rate)

    model.minimize(10.0 * (x - x_ref) ** 2)
    model.minimize(10.0 * (y - y_ref) ** 2)
    model.minimize(2.5 * psi_error**2)
    model.minimize((v - v_ref) ** 2)
    model.minimize(0.25 * delta**2)
    model.minimize(0.025 * accel**2)
    model.minimize(0.025 * delta_rate**2)

    model.subject_to(lateral - w_left <= 0)
    model.subject_to(-lateral - w_right <= 0)
    model.subject_to(v - V_MAX <= 0)
    model.subject_to(V_MIN - v <= 0)
    model.subject_to(delta - DELTA_MAX <= 0)
    model.subject_to(DELTA_MIN - delta <= 0)
    model.subject_to(accel - A_MAX <= 0)
    model.subject_to(-A_MAX - accel <= 0)
    model.subject_to(delta_rate - DELTA_RATE_MAX <= 0)
    model.subject_to(-DELTA_RATE_MAX - delta_rate <= 0)

    output_dir = ROOT / "targets" / "minisolver" / "generated"
    model.generate(str(output_dir))
    print(f"Generated KinematicBicycleTrackModel in {output_dir}")
