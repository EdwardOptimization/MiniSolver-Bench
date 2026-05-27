from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from scipy.interpolate import PPoly, make_interp_spline

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from targets.minisolver.models.common import acados_repo_dir  # noqa: E402


def load_race_track() -> tuple[np.ndarray, np.ndarray]:
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


def load_quad_track() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
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


def flatten_ppoly(ppoly: PPoly) -> list[float]:
    coeffs: list[float] = []
    for segment in range(ppoly.c.shape[1]):
        coeffs.extend(float(value) for value in ppoly.c[:, segment])
    return coeffs


def format_array(name: str, values: list[float] | np.ndarray) -> list[str]:
    lines = [f"inline constexpr std::array<double, {len(values)}> {name} = {{"]
    row: list[str] = []
    for value in values:
        row.append(f"{float(value):.17g}")
        if len(row) == 4:
            lines.append("    " + ", ".join(row) + ",")
            row = []
    if row:
        lines.append("    " + ", ".join(row) + ",")
    lines.append("};")
    return lines


def spline_ppoly(s_samples: np.ndarray, y_samples: np.ndarray, degree: int) -> PPoly:
    return PPoly.from_spline(make_interp_spline(s_samples, y_samples, k=degree))


def main() -> int:
    race_s, race_kappa = load_race_track()
    race_spline = spline_ppoly(race_s, race_kappa, 3)

    quad_s, quad_x, quad_y, quad_z = load_quad_track()
    quad_splines = {
        "x": spline_ppoly(quad_s, quad_x, 5),
        "y": spline_ppoly(quad_s, quad_y, 5),
        "z": spline_ppoly(quad_s, quad_z, 5),
    }

    output_path = Path(__file__).resolve().parents[1] / "generated" / "official_track_geometry.h"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        "#pragma once",
        "",
        "#include <array>",
        "#include <cstddef>",
        "",
        "namespace minisolver_bench::generated {",
        "",
        "template <std::size_t NBreaks, std::size_t NCoeffs>",
        "inline double ppoly_eval_derivative(",
        "    const std::array<double, NBreaks>& breaks,",
        "    const std::array<double, NCoeffs>& coeffs,",
        "    int degree,",
        "    int derivative_order,",
        "    double s)",
        "{",
        "    static_assert(NBreaks >= 2, \"ppoly requires at least one segment\");",
        "    const std::size_t segments = NBreaks - 1;",
        "    std::size_t segment = 0;",
        "    if (s >= breaks[NBreaks - 1]) {",
        "        segment = segments - 1;",
        "    } else {",
        "        while (segment + 1 < segments && s >= breaks[segment + 1]) {",
        "            ++segment;",
        "        }",
        "    }",
        "    const double dx = s - breaks[segment];",
        "    double result = 0.0;",
        "    const std::size_t offset = segment * static_cast<std::size_t>(degree + 1);",
        "    for (int j = 0; j <= degree; ++j) {",
        "        const int power = degree - j;",
        "        if (power < derivative_order) {",
        "            continue;",
        "        }",
        "        double factor = 1.0;",
        "        for (int d = 0; d < derivative_order; ++d) {",
        "            factor *= static_cast<double>(power - d);",
        "        }",
        "        double dx_power = 1.0;",
        "        for (int e = 0; e < power - derivative_order; ++e) {",
        "            dx_power *= dx;",
        "        }",
        "        result += coeffs[offset + static_cast<std::size_t>(j)] * factor * dx_power;",
        "    }",
        "    return result;",
        "}",
        "",
        *format_array("race_kappa_breaks", race_spline.x),
        "",
        *format_array("race_kappa_coeffs", flatten_ppoly(race_spline)),
        "",
    ]

    for axis, spline in quad_splines.items():
        lines.extend(format_array(f"quad_track_{axis}_breaks", spline.x))
        lines.append("")
        lines.extend(format_array(f"quad_track_{axis}_coeffs", flatten_ppoly(spline)))
        lines.append("")

    lines.extend(
        [
            "} // namespace minisolver_bench::generated",
            "",
        ]
    )

    output_path.write_text("\n".join(lines), encoding="utf-8")
    print(f"wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
