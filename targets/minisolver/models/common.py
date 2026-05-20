from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import sympy as sp
from scipy.interpolate import PPoly


def minisolver_source_dir() -> Path:
    value = os.environ.get(
        "MINISOLVER_SOURCE_DIR",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../../MiniSolver")),
    )
    return Path(value).resolve()


def acados_repo_dir() -> Path:
    value = os.environ.get(
        "ACADOS_REPO",
        os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../third_party/acados")),
    )
    return Path(value).resolve()


def ensure_minisolver_python_path() -> None:
    root = minisolver_source_dir()
    sys.path.insert(0, str(root / "python"))


def set_dynamics(model, state, rhs) -> None:
    legacy = getattr(model, "set_dynamics", None)
    if legacy is not None:
        legacy(state, rhs)
        return

    from minisolver.MiniModel import Dot

    model.subject_to(Dot(state) == rhs)


def fit_fourier_series(s_samples: np.ndarray, y_samples: np.ndarray, harmonics: int) -> tuple[float, np.ndarray]:
    pathlength = float(s_samples[-1])
    omega = 2.0 * np.pi / pathlength
    cols = [np.ones_like(s_samples)]
    for k in range(1, harmonics + 1):
        cols.append(np.cos(k * omega * s_samples))
        cols.append(np.sin(k * omega * s_samples))
    design = np.column_stack(cols)
    coeffs, *_ = np.linalg.lstsq(design, y_samples, rcond=None)
    return pathlength, coeffs


def fourier_expr(s_sym: sp.Symbol, pathlength: float, coeffs: np.ndarray) -> sp.Expr:
    omega = 2.0 * np.pi / pathlength
    expr = sp.Float(coeffs[0])
    harmonics = (len(coeffs) - 1) // 2
    for k in range(1, harmonics + 1):
        expr += sp.Float(coeffs[2 * k - 1]) * sp.cos(sp.Float(k * omega) * s_sym)
        expr += sp.Float(coeffs[2 * k]) * sp.sin(sp.Float(k * omega) * s_sym)
    return expr


def fourier_derivative_expr(
    s_sym: sp.Symbol,
    pathlength: float,
    coeffs: np.ndarray,
    order: int,
) -> sp.Expr:
    omega = 2.0 * np.pi / pathlength
    expr = sp.Float(0.0 if order > 0 else coeffs[0])
    harmonics = (len(coeffs) - 1) // 2
    for k in range(1, harmonics + 1):
        freq = sp.Float(k * omega)
        amp_cos = sp.Float(coeffs[2 * k - 1])
        amp_sin = sp.Float(coeffs[2 * k])
        angle = freq * s_sym
        scale = freq**order
        mode = order % 4
        if mode == 0:
            expr += scale * (amp_cos * sp.cos(angle) + amp_sin * sp.sin(angle))
        elif mode == 1:
            expr += scale * (-amp_cos * sp.sin(angle) + amp_sin * sp.cos(angle))
        elif mode == 2:
            expr += scale * (-amp_cos * sp.cos(angle) - amp_sin * sp.sin(angle))
        else:
            expr += scale * (amp_cos * sp.sin(angle) - amp_sin * sp.cos(angle))
    return expr


def piecewise_linear_expr(s_sym: sp.Symbol, s_samples: np.ndarray, y_samples: np.ndarray) -> sp.Expr:
    pieces: list[tuple[sp.Expr, sp.Expr | bool]] = []
    for i in range(len(s_samples) - 1):
        s0 = float(s_samples[i])
        s1 = float(s_samples[i + 1])
        y0 = float(y_samples[i])
        y1 = float(y_samples[i + 1])
        slope = (y1 - y0) / (s1 - s0)
        expr = sp.Float(y0) + sp.Float(slope) * (s_sym - sp.Float(s0))
        cond = True if i == len(s_samples) - 2 else sp.And(s_sym >= sp.Float(s0), s_sym < sp.Float(s1))
        pieces.append((expr, cond))
    return sp.Piecewise(*pieces)


def ppoly_expr(s_sym: sp.Symbol, ppoly: PPoly) -> sp.Expr:
    pieces: list[tuple[sp.Expr, sp.Expr | bool]] = []
    degree = ppoly.c.shape[0] - 1
    for i in range(len(ppoly.x) - 1):
        x0 = float(ppoly.x[i])
        x1 = float(ppoly.x[i + 1])
        delta = s_sym - sp.Float(x0)
        expr = sp.Float(0.0)
        for j in range(degree + 1):
            expr += sp.Float(ppoly.c[j, i]) * delta ** (degree - j)
        cond = True if i == len(ppoly.x) - 2 else sp.And(s_sym >= sp.Float(x0), s_sym < sp.Float(x1))
        pieces.append((expr, cond))
    return sp.Piecewise(*pieces)


def ppoly_derivative_expr(s_sym: sp.Symbol, ppoly: PPoly, order: int) -> sp.Expr:
    return ppoly_expr(s_sym, ppoly.derivative(order))
