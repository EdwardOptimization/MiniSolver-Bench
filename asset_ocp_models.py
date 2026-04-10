from __future__ import annotations

import casadi as ca
import numpy as np


DRIVING_WHEELBASE = 0.33
DRIVING_VMIN = 0.1
DRIVING_VMAX = 12.0
DRIVING_DELTA_MIN = -0.50
DRIVING_DELTA_MAX = 0.50
DRIVING_QPOS = 20.0
DRIVING_QYAW = 5.0
DRIVING_QVEL = 2.0
DRIVING_QDELTA = 0.5
DRIVING_RACC = 0.05
DRIVING_RDELTARATE = 0.05
DRIVING_SOFT_WEIGHTS = np.array([200.0, 200.0, 20.0, 20.0, 20.0, 20.0], dtype=float)

ROBOT_ACCEL_MAX = 15.0
ROBOT_QPOS = 30.0
ROBOT_QVEL = 5.0
ROBOT_RACC = 0.1
ROBOT_SOFT_WEIGHTS = np.array([20.0, 20.0, 20.0, 20.0, 20.0, 20.0], dtype=float)


def wrap_angle_expr(angle):
    return ca.atan2(ca.sin(angle), ca.cos(angle))


def driving_asset_model():
    x = ca.MX.sym("x", 5)
    u = ca.MX.sym("u", 2)
    p = ca.MX.sym("p", 8)

    psi = x[2]
    v = x[3]
    delta = x[4]
    f_expl = ca.vertcat(
        v * ca.cos(psi),
        v * ca.sin(psi),
        v * ca.tan(delta) / DRIVING_WHEELBASE,
        u[0],
        u[1],
    )

    lateral = p[4] * (x[0] - p[0]) + p[5] * (x[1] - p[1])
    h = ca.vertcat(
        lateral - p[6],
        -lateral - p[7],
        x[3] - DRIVING_VMAX,
        DRIVING_VMIN - x[3],
        x[4] - DRIVING_DELTA_MAX,
        DRIVING_DELTA_MIN - x[4],
    )

    y = ca.vertcat(
        x[0] - p[0],
        x[1] - p[1],
        wrap_angle_expr(x[2] - p[2]),
        x[3] - p[3],
        x[4],
        u[0],
        u[1],
    )
    w = np.diag([
        DRIVING_QPOS,
        DRIVING_QPOS,
        DRIVING_QYAW,
        DRIVING_QVEL,
        DRIVING_QDELTA,
        DRIVING_RACC,
        DRIVING_RDELTARATE,
    ])

    return {
        "x": x,
        "u": u,
        "p": p,
        "f_expl": f_expl,
        "h": h,
        "y": y,
        "W": w,
        "soft_weights": DRIVING_SOFT_WEIGHTS,
    }


def robot_asset_model():
    x = ca.MX.sym("x", 6)
    u = ca.MX.sym("u", 3)
    p = ca.MX.sym("p", 6)

    f_expl = ca.vertcat(
        x[3],
        x[4],
        x[5],
        u[0],
        u[1],
        u[2],
    )

    h = ca.vertcat(
        u[0] - ROBOT_ACCEL_MAX,
        -ROBOT_ACCEL_MAX - u[0],
        u[1] - ROBOT_ACCEL_MAX,
        -ROBOT_ACCEL_MAX - u[1],
        u[2] - ROBOT_ACCEL_MAX,
        -ROBOT_ACCEL_MAX - u[2],
    )

    y = ca.vertcat(
        x[0] - p[0],
        x[1] - p[1],
        x[2] - p[2],
        x[3] - p[3],
        x[4] - p[4],
        x[5] - p[5],
        u[0],
        u[1],
        u[2],
    )
    w = np.diag([
        ROBOT_QPOS,
        ROBOT_QPOS,
        ROBOT_QPOS,
        ROBOT_QVEL,
        ROBOT_QVEL,
        ROBOT_QVEL,
        ROBOT_RACC,
        ROBOT_RACC,
        ROBOT_RACC,
    ])

    return {
        "x": x,
        "u": u,
        "p": p,
        "f_expl": f_expl,
        "h": h,
        "y": y,
        "W": w,
        "soft_weights": ROBOT_SOFT_WEIGHTS,
    }
