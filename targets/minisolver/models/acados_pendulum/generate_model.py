import os
import sys

import sympy as sp

MINISOLVER_SOURCE_DIR = os.environ.get(
    "MINISOLVER_SOURCE_DIR",
    os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../MiniSolver")),
)
sys.path.append(os.path.join(MINISOLVER_SOURCE_DIR, "python"))

from minisolver.MiniModel import OptimalControlModel


if __name__ == "__main__":
    model = OptimalControlModel("PendulumModel")

    # States follow the official acados pendulum-on-cart ordering:
    # x = [cart_position, pole_angle, cart_velocity, pole_angular_velocity]
    x, theta, v, omega = model.state("x", "theta", "v", "omega")
    force = model.control("force")

    # Official acados example constants.
    M = 1.0
    m = 0.1
    g = 9.81
    l = 0.8
    f_max = 80.0

    denom = M + m - m * sp.cos(theta) ** 2

    model.set_dynamics(x, v)
    model.set_dynamics(theta, omega)
    model.set_dynamics(
        v,
        (
            -m * l * sp.sin(theta) * omega**2
            + m * g * sp.cos(theta) * sp.sin(theta)
            + force
        )
        / denom,
    )
    model.set_dynamics(
        omega,
        (
            -m * l * sp.cos(theta) * sp.sin(theta) * omega**2
            + force * sp.cos(theta)
            + (M + m) * g * sp.sin(theta)
        )
        / (l * denom),
    )

    # Match the official acados least-squares weights.
    model.minimize(2.0 * 1e3 * x**2)
    model.minimize(2.0 * 1e3 * theta**2)
    model.minimize(2.0 * 1e-2 * v**2)
    model.minimize(2.0 * 1e-2 * omega**2)
    model.minimize(2.0 * 1e-2 * force**2)

    model.subject_to(force <= f_max)
    model.subject_to(force >= -f_max)

    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../generated"))
    model.generate(output_dir)
    print(f"Generated PendulumModel in {output_dir}")
