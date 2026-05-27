#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "minisolver/solver/solver.h"

using namespace minisolver;

namespace minisolver_asset_bench {

constexpr int kMaxHorizon = 128;
constexpr double kDrivingWheelbase = 0.33;

template <size_t N>
constexpr std::array<const char*, N> filled_names(const char* value) {
    std::array<const char*, N> result{};
    for (size_t i = 0; i < N; ++i) {
        result[i] = value;
    }
    return result;
}

template <int R, int C>
void set_zero_matrix(MSMat<double, R, C>& mat) {
    MatOps::setZero(mat);
}

template <int N>
void set_zero_vector(MSVec<double, N>& vec) {
    MatOps::setZero(vec);
}

template <int NX, int NU, int NP, typename EvalFn>
MSVec<double, NX> rk4_step(
    const MSVec<double, NX>& x,
    const MSVec<double, NU>& u,
    const MSVec<double, NP>& p,
    double dt,
    EvalFn&& eval,
    MSMat<double, NX, NX>* A,
    MSMat<double, NX, NU>* B) {
    MSVec<double, NX> k1;
    MSVec<double, NX> k2;
    MSVec<double, NX> k3;
    MSVec<double, NX> k4;
    MSMat<double, NX, NX> fx1;
    MSMat<double, NX, NX> fx2;
    MSMat<double, NX, NX> fx3;
    MSMat<double, NX, NX> fx4;
    MSMat<double, NX, NU> fu1;
    MSMat<double, NX, NU> fu2;
    MSMat<double, NX, NU> fu3;
    MSMat<double, NX, NU> fu4;
    MSMat<double, NX, NX> I;
    MatOps::setIdentity(I);

    eval(x, u, p, k1, fx1, fu1);
    const MSVec<double, NX> x2 = x + k1 * (0.5 * dt);
    const MSMat<double, NX, NX> dx2_dx = I + fx1 * (0.5 * dt);
    const MSMat<double, NX, NU> dx2_du = fu1 * (0.5 * dt);

    eval(x2, u, p, k2, fx2, fu2);
    const MSMat<double, NX, NX> dk2_dx = fx2 * dx2_dx;
    const MSMat<double, NX, NU> dk2_du = fx2 * dx2_du + fu2;

    const MSVec<double, NX> x3 = x + k2 * (0.5 * dt);
    const MSMat<double, NX, NX> dx3_dx = I + dk2_dx * (0.5 * dt);
    const MSMat<double, NX, NU> dx3_du = dk2_du * (0.5 * dt);

    eval(x3, u, p, k3, fx3, fu3);
    const MSMat<double, NX, NX> dk3_dx = fx3 * dx3_dx;
    const MSMat<double, NX, NU> dk3_du = fx3 * dx3_du + fu3;

    const MSVec<double, NX> x4 = x + k3 * dt;
    const MSMat<double, NX, NX> dx4_dx = I + dk3_dx * dt;
    const MSMat<double, NX, NU> dx4_du = dk3_du * dt;

    eval(x4, u, p, k4, fx4, fu4);
    const MSMat<double, NX, NX> dk4_dx = fx4 * dx4_dx;
    const MSMat<double, NX, NU> dk4_du = fx4 * dx4_du + fu4;

    if (A != nullptr) {
        *A = I + (fx1 + dk2_dx * 2.0 + dk3_dx * 2.0 + dk4_dx) * (dt / 6.0);
    }
    if (B != nullptr) {
        *B = (fu1 + dk2_du * 2.0 + dk3_du * 2.0 + dk4_du) * (dt / 6.0);
    }

    return x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);
}

double wrap_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> parts;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) {
        while (!item.empty() && (item.back() == '\r' || item.back() == '\n' || item.back() == ' ' || item.back() == '\t')) {
            item.pop_back();
        }
        size_t start = 0;
        while (start < item.size() && (item[start] == ' ' || item[start] == '\t')) {
            ++start;
        }
        if (start > 0) {
            item = item.substr(start);
        }
        parts.push_back(item);
    }
    return parts;
}

struct DrivingTrackAsset {
    std::vector<double> s;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> psi;
    std::vector<double> nx;
    std::vector<double> ny;
    std::vector<double> w_left;
    std::vector<double> w_right;
    double avg_ds = 0.1;
};

struct RobotReferenceAsset {
    std::vector<double> t;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> vx;
    std::vector<double> vy;
    std::vector<double> vz;
    double dt = 0.04;
};

DrivingTrackAsset load_driving_track(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("failed to open driving asset: " + path);
    }
    std::string header_line;
    std::getline(file, header_line);
    auto headers = split_csv_line(header_line);
    std::unordered_map<std::string, int> index;
    for (int i = 0; i < static_cast<int>(headers.size()); ++i) {
        index[headers[static_cast<size_t>(i)]] = i;
    }

    DrivingTrackAsset asset;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        auto parts = split_csv_line(line);
        auto value = [&](const std::string& key) -> double {
            return std::stod(parts.at(static_cast<size_t>(index.at(key))));
        };
        asset.s.push_back(value("s"));
        asset.x.push_back(value("x_center"));
        asset.y.push_back(value("y_center"));
        if (index.count("width_left") != 0) {
            asset.w_left.push_back(value("width_left"));
            asset.w_right.push_back(value("width_right"));
        } else {
            const double width = std::min(value("width_inner"), value("width_outer"));
            asset.w_left.push_back(width);
            asset.w_right.push_back(width);
        }
    }

    const int n = static_cast<int>(asset.x.size());
    asset.psi.resize(static_cast<size_t>(n));
    asset.nx.resize(static_cast<size_t>(n));
    asset.ny.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        const int prev = (i == 0) ? (n - 1) : (i - 1);
        const int next = (i + 1) % n;
        const double dx = asset.x[static_cast<size_t>(next)] - asset.x[static_cast<size_t>(prev)];
        const double dy = asset.y[static_cast<size_t>(next)] - asset.y[static_cast<size_t>(prev)];
        const double psi = std::atan2(dy, dx);
        asset.psi[static_cast<size_t>(i)] = psi;
        asset.nx[static_cast<size_t>(i)] = -std::sin(psi);
        asset.ny[static_cast<size_t>(i)] = std::cos(psi);
    }
    if (asset.s.size() > 1) {
        asset.avg_ds = asset.s.back() / static_cast<double>(asset.s.size() - 1);
    }
    return asset;
}

RobotReferenceAsset load_robot_reference(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::runtime_error("failed to open robot asset: " + path);
    }
    std::string header_line;
    std::getline(file, header_line);
    auto headers = split_csv_line(header_line);
    std::unordered_map<std::string, int> index;
    for (int i = 0; i < static_cast<int>(headers.size()); ++i) {
        index[headers[static_cast<size_t>(i)]] = i;
    }

    RobotReferenceAsset asset;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        auto parts = split_csv_line(line);
        auto value = [&](const std::string& key) -> double {
            return std::stod(parts.at(static_cast<size_t>(index.at(key))));
        };
        asset.t.push_back(value("t"));
        asset.x.push_back(value("x"));
        asset.y.push_back(value("y"));
        asset.z.push_back(value("z"));
        asset.vx.push_back(value("vx"));
        asset.vy.push_back(value("vy"));
        asset.vz.push_back(value("vz"));
    }
    if (asset.t.size() > 1) {
        asset.dt = asset.t[1] - asset.t[0];
    }
    return asset;
}

size_t nearest_cyclic_index(const DrivingTrackAsset& asset, double x, double y, size_t anchor, size_t window) {
    const size_t n = asset.x.size();
    size_t best = anchor;
    double best_dist = std::numeric_limits<double>::infinity();
    for (size_t offset = 0; offset <= window; ++offset) {
        const std::array<size_t, 2> candidates = {
            (anchor + offset) % n,
            (anchor + n - (offset % n)) % n,
        };
        for (size_t idx : candidates) {
            const double dx = x - asset.x[idx];
            const double dy = y - asset.y[idx];
            const double dist = dx * dx + dy * dy;
            if (dist < best_dist) {
                best_dist = dist;
                best = idx;
            }
        }
    }
    return best;
}

template <typename Model, int MAX_N>
double max_positive_constraint(const MiniSolver<Model, MAX_N>& solver) {
    double result = 0.0;
    const int N = solver.get_horizon();
    for (int k = 0; k <= N; ++k) {
        for (int i = 0; i < Model::NC; ++i) {
            result = std::max(result, std::max(0.0, solver.get_constraint_val(k, i)));
        }
    }
    return result;
}

bool success_status(SolverStatus status) {
    return status == SolverStatus::OPTIMAL || status == SolverStatus::FEASIBLE;
}

template <typename Model, int MAX_N>
void shift_primal_guess(MiniSolver<Model, MAX_N>& solver) {
    const int N = solver.get_horizon();
    if (N <= 0) {
        return;
    }

    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < Model::NX; ++i) {
            solver.set_state_guess(k, i, solver.get_state(k + 1, i));
        }
    }
    for (int k = 0; k < N - 1; ++k) {
        for (int i = 0; i < Model::NU; ++i) {
            solver.set_control_guess(k, i, solver.get_control(k + 1, i));
        }
    }
    for (int i = 0; i < Model::NU; ++i) {
        solver.set_control_guess(N - 1, i, solver.get_control(N - 1, i));
    }
}

struct DrivingTrackModel {
    static constexpr int NX = 5;
    static constexpr int NU = 2;
    static constexpr int NC = 6;
    static constexpr int NP = 8;

    static constexpr std::array<const char*, NX> state_names = {"x", "y", "psi", "v", "delta"};
    static constexpr std::array<const char*, NU> control_names = {"a", "delta_rate"};
    static constexpr std::array<const char*, NP> param_names = {"x_ref", "y_ref", "psi_ref", "v_ref", "n_x", "n_y", "w_left", "w_right"};
    static constexpr std::array<double, NC> constraint_weights = {200.0, 200.0, 20.0, 20.0, 20.0, 20.0};
    static constexpr std::array<int, NC> constraint_types = {1, 1, 1, 1, 1, 1};

    static constexpr double kVMin = 0.1;
    static constexpr double kVMax = 12.0;
    static constexpr double kDeltaMin = -0.50;
    static constexpr double kDeltaMax = 0.50;
    static constexpr double kQPos = 20.0;
    static constexpr double kQYaw = 5.0;
    static constexpr double kQVel = 2.0;
    static constexpr double kQDelta = 0.5;
    static constexpr double kRAcc = 0.05;
    static constexpr double kRDeltaRate = 0.05;

    template <typename T>
    static void continuous_eval(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>&,
        MSVec<T, NX>& f,
        MSMat<T, NX, NX>& fx,
        MSMat<T, NX, NU>& fu) {
        static_assert(std::is_same_v<T, double>, "DrivingTrackModel only supports double evaluation");
        set_zero_vector(f);
        set_zero_matrix(fx);
        set_zero_matrix(fu);

        const double psi = x(2);
        const double v = x(3);
        const double delta = x(4);

        f(0) = v * std::cos(psi);
        f(1) = v * std::sin(psi);
        f(2) = v * std::tan(delta) / kDrivingWheelbase;
        f(3) = u(0);
        f(4) = u(1);

        fx(0, 2) = -v * std::sin(psi);
        fx(0, 3) = std::cos(psi);
        fx(1, 2) = v * std::cos(psi);
        fx(1, 3) = std::sin(psi);
        fx(2, 3) = std::tan(delta) / kDrivingWheelbase;
        fx(2, 4) = v / (kDrivingWheelbase * std::cos(delta) * std::cos(delta));
        fu(3, 0) = 1.0;
        fu(4, 1) = 1.0;
    }

    template <typename T>
    static MSVec<T, NX> integrate(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>& p,
        double dt,
        IntegratorType) {
        auto eval = [](const auto& xv, const auto& uv, const auto& pv, auto& f, auto& fx, auto& fu) {
            continuous_eval(xv, uv, pv, f, fx, fu);
        };
        return rk4_step<NX, NU, NP>(x, u, p, dt, eval, nullptr, nullptr);
    }

    template <typename T>
    static void compute_dynamics(KnotPoint<T, NX, NU, NC, NP>& kp, IntegratorType, double dt) {
        auto eval = [](const auto& xv, const auto& uv, const auto& pv, auto& f, auto& fx, auto& fu) {
            continuous_eval(xv, uv, pv, f, fx, fu);
        };
        kp.f_resid = rk4_step<NX, NU, NP>(kp.x, kp.u, kp.p, dt, eval, &kp.A, &kp.B);
    }

    template <typename T>
    static void compute_constraints(KnotPoint<T, NX, NU, NC, NP>& kp) {
        const double lateral = kp.p(4) * (kp.x(0) - kp.p(0)) + kp.p(5) * (kp.x(1) - kp.p(1));
        kp.g_val(0) = lateral - kp.p(6);
        kp.g_val(1) = -lateral - kp.p(7);
        kp.g_val(2) = kp.x(3) - kVMax;
        kp.g_val(3) = kVMin - kp.x(3);
        kp.g_val(4) = kp.x(4) - kDeltaMax;
        kp.g_val(5) = kDeltaMin - kp.x(4);

        set_zero_matrix(kp.C);
        set_zero_matrix(kp.D);
        kp.C(0, 0) = kp.p(4);
        kp.C(0, 1) = kp.p(5);
        kp.C(1, 0) = -kp.p(4);
        kp.C(1, 1) = -kp.p(5);
        kp.C(2, 3) = 1.0;
        kp.C(3, 3) = -1.0;
        kp.C(4, 4) = 1.0;
        kp.C(5, 4) = -1.0;
    }

    template <typename T>
    static void compute_cost_exact(KnotPoint<T, NX, NU, NC, NP>& kp) {
        kp.cost = 0.0;
        set_zero_vector(kp.q);
        set_zero_vector(kp.r);
        set_zero_matrix(kp.Q);
        set_zero_matrix(kp.R);
        set_zero_matrix(kp.H);

        const double ex = kp.x(0) - kp.p(0);
        const double ey = kp.x(1) - kp.p(1);
        const double epsi = wrap_angle(kp.x(2) - kp.p(2));
        const double ev = kp.x(3) - kp.p(3);
        const double edelta = kp.x(4);

        kp.cost += 0.5 * kQPos * ex * ex;
        kp.cost += 0.5 * kQPos * ey * ey;
        kp.cost += 0.5 * kQYaw * epsi * epsi;
        kp.cost += 0.5 * kQVel * ev * ev;
        kp.cost += 0.5 * kQDelta * edelta * edelta;
        kp.cost += 0.5 * kRAcc * kp.u(0) * kp.u(0);
        kp.cost += 0.5 * kRDeltaRate * kp.u(1) * kp.u(1);

        kp.q(0) = kQPos * ex;
        kp.q(1) = kQPos * ey;
        kp.q(2) = kQYaw * epsi;
        kp.q(3) = kQVel * ev;
        kp.q(4) = kQDelta * edelta;
        kp.r(0) = kRAcc * kp.u(0);
        kp.r(1) = kRDeltaRate * kp.u(1);
        kp.Q(0, 0) = kQPos;
        kp.Q(1, 1) = kQPos;
        kp.Q(2, 2) = kQYaw;
        kp.Q(3, 3) = kQVel;
        kp.Q(4, 4) = kQDelta;
        kp.R(0, 0) = kRAcc;
        kp.R(1, 1) = kRDeltaRate;
    }

    template <typename T>
    static void compute_cost_gn(KnotPoint<T, NX, NU, NC, NP>& kp) { compute_cost_exact(kp); }
    template <typename T>
    static void compute_cost(KnotPoint<T, NX, NU, NC, NP>& kp) { compute_cost_exact(kp); }
    template <typename T>
    static void compute(KnotPoint<T, NX, NU, NC, NP>& kp, IntegratorType type, double dt) {
        compute_cost(kp);
        compute_dynamics(kp, type, dt);
        compute_constraints(kp);
    }
};

struct RobotReferenceModel {
    static constexpr int NX = 6;
    static constexpr int NU = 3;
    static constexpr int NC = 6;
    static constexpr int NP = 6;

    static constexpr std::array<const char*, NX> state_names = {"x", "y", "z", "vx", "vy", "vz"};
    static constexpr std::array<const char*, NU> control_names = {"ax", "ay", "az"};
    static constexpr std::array<const char*, NP> param_names = {"x_ref", "y_ref", "z_ref", "vx_ref", "vy_ref", "vz_ref"};
    static constexpr std::array<double, NC> constraint_weights = {20.0, 20.0, 20.0, 20.0, 20.0, 20.0};
    static constexpr std::array<int, NC> constraint_types = {1, 1, 1, 1, 1, 1};

    static constexpr double kAccelMax = 15.0;
    static constexpr double kQPos = 30.0;
    static constexpr double kQVel = 5.0;
    static constexpr double kRAcc = 0.1;

    template <typename T>
    static void continuous_eval(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>&,
        MSVec<T, NX>& f,
        MSMat<T, NX, NX>& fx,
        MSMat<T, NX, NU>& fu) {
        set_zero_vector(f);
        set_zero_matrix(fx);
        set_zero_matrix(fu);
        f(0) = x(3);
        f(1) = x(4);
        f(2) = x(5);
        f(3) = u(0);
        f(4) = u(1);
        f(5) = u(2);
        fx(0, 3) = 1.0;
        fx(1, 4) = 1.0;
        fx(2, 5) = 1.0;
        fu(3, 0) = 1.0;
        fu(4, 1) = 1.0;
        fu(5, 2) = 1.0;
    }

    template <typename T>
    static MSVec<T, NX> integrate(
        const MSVec<T, NX>& x,
        const MSVec<T, NU>& u,
        const MSVec<T, NP>& p,
        double dt,
        IntegratorType) {
        auto eval = [](const auto& xv, const auto& uv, const auto& pv, auto& f, auto& fx, auto& fu) {
            continuous_eval(xv, uv, pv, f, fx, fu);
        };
        return rk4_step<NX, NU, NP>(x, u, p, dt, eval, nullptr, nullptr);
    }

    template <typename T>
    static void compute_dynamics(KnotPoint<T, NX, NU, NC, NP>& kp, IntegratorType, double dt) {
        auto eval = [](const auto& xv, const auto& uv, const auto& pv, auto& f, auto& fx, auto& fu) {
            continuous_eval(xv, uv, pv, f, fx, fu);
        };
        kp.f_resid = rk4_step<NX, NU, NP>(kp.x, kp.u, kp.p, dt, eval, &kp.A, &kp.B);
    }

    template <typename T>
    static void compute_constraints(KnotPoint<T, NX, NU, NC, NP>& kp) {
        kp.g_val(0) = kp.u(0) - kAccelMax;
        kp.g_val(1) = -kAccelMax - kp.u(0);
        kp.g_val(2) = kp.u(1) - kAccelMax;
        kp.g_val(3) = -kAccelMax - kp.u(1);
        kp.g_val(4) = kp.u(2) - kAccelMax;
        kp.g_val(5) = -kAccelMax - kp.u(2);
        set_zero_matrix(kp.C);
        set_zero_matrix(kp.D);
        kp.D(0, 0) = 1.0;
        kp.D(1, 0) = -1.0;
        kp.D(2, 1) = 1.0;
        kp.D(3, 1) = -1.0;
        kp.D(4, 2) = 1.0;
        kp.D(5, 2) = -1.0;
    }

    template <typename T>
    static void compute_cost_exact(KnotPoint<T, NX, NU, NC, NP>& kp) {
        kp.cost = 0.0;
        set_zero_vector(kp.q);
        set_zero_vector(kp.r);
        set_zero_matrix(kp.Q);
        set_zero_matrix(kp.R);
        set_zero_matrix(kp.H);

        for (int i = 0; i < 3; ++i) {
            const double diff = kp.x(i) - kp.p(i);
            kp.cost += 0.5 * kQPos * diff * diff;
            kp.q(i) = kQPos * diff;
            kp.Q(i, i) = kQPos;
        }
        for (int i = 0; i < 3; ++i) {
            const double diff = kp.x(3 + i) - kp.p(3 + i);
            kp.cost += 0.5 * kQVel * diff * diff;
            kp.q(3 + i) = kQVel * diff;
            kp.Q(3 + i, 3 + i) = kQVel;
        }
        for (int i = 0; i < 3; ++i) {
            kp.cost += 0.5 * kRAcc * kp.u(i) * kp.u(i);
            kp.r(i) = kRAcc * kp.u(i);
            kp.R(i, i) = kRAcc;
        }
    }

    template <typename T>
    static void compute_cost_gn(KnotPoint<T, NX, NU, NC, NP>& kp) { compute_cost_exact(kp); }
    template <typename T>
    static void compute_cost(KnotPoint<T, NX, NU, NC, NP>& kp) { compute_cost_exact(kp); }
    template <typename T>
    static void compute(KnotPoint<T, NX, NU, NC, NP>& kp, IntegratorType type, double dt) {
        compute_cost(kp);
        compute_dynamics(kp, type, dt);
        compute_constraints(kp);
    }
};

struct Args {
    std::string benchmark;
    std::string asset_path;
    std::string output_path;
    int horizon = 40;
    int steps = 100;
    int warmup_runs = 0;
    double dt = 0.05;
    double target_speed = 2.5;
};

Args parse_args(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto require_value = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + name);
            return argv[++i];
        };
        if (key == "--benchmark") args.benchmark = require_value("--benchmark");
        else if (key == "--asset") args.asset_path = require_value("--asset");
        else if (key == "--output") args.output_path = require_value("--output");
        else if (key == "--horizon") args.horizon = std::stoi(require_value("--horizon"));
        else if (key == "--steps") args.steps = std::stoi(require_value("--steps"));
        else if (key == "--dt") args.dt = std::stod(require_value("--dt"));
        else if (key == "--target-speed") args.target_speed = std::stod(require_value("--target-speed"));
        else if (key == "--warmup") args.warmup_runs = std::stoi(require_value("--warmup"));
        else throw std::runtime_error("unknown argument: " + key);
    }
    if (args.benchmark.empty() || args.asset_path.empty() || args.output_path.empty()) {
        throw std::runtime_error("required args: --benchmark --asset --output");
    }
    if (args.horizon <= 0 || args.horizon > kMaxHorizon) {
        throw std::runtime_error("invalid horizon");
    }
    return args;
}

SolverConfig make_config() {
    SolverConfig config;
    config.print_level = PrintLevel::NONE;
    config.integrator = IntegratorType::RK4_EXPLICIT;
    config.barrier_strategy = BarrierStrategy::MEHROTRA;
    config.line_search_type = LineSearchType::FILTER;
    config.enable_soc = false;
    config.enable_feasibility_restoration = true;
    config.enable_slack_reset = true;
    config.max_iters = 60;
    config.tol_con = 1e-4;
    config.tol_dual = 1e-4;
    config.backend = Backend::CPU_SERIAL;
    return config;
}

struct DrivingState {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double v = 0.0;
    double delta = 0.0;
};

DrivingState simulate_driving(const DrivingState& state, double a, double delta_rate, double dt) {
    DrivingState next = state;
    next.x += state.v * std::cos(state.psi) * dt;
    next.y += state.v * std::sin(state.psi) * dt;
    next.psi = wrap_angle(state.psi + state.v * std::tan(state.delta) / kDrivingWheelbase * dt);
    next.v = std::clamp(state.v + a * dt, DrivingTrackModel::kVMin, DrivingTrackModel::kVMax);
    next.delta = std::clamp(state.delta + delta_rate * dt, DrivingTrackModel::kDeltaMin, DrivingTrackModel::kDeltaMax);
    return next;
}

struct RobotState {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
};

RobotState simulate_robot(const RobotState& state, double ax, double ay, double az, double dt) {
    RobotState next = state;
    next.x += state.vx * dt + 0.5 * ax * dt * dt;
    next.y += state.vy * dt + 0.5 * ay * dt * dt;
    next.z += state.vz * dt + 0.5 * az * dt * dt;
    next.vx += ax * dt;
    next.vy += ay * dt;
    next.vz += az * dt;
    return next;
}

void seed_driving_solver(MiniSolver<DrivingTrackModel, kMaxHorizon>& solver, const DrivingTrackAsset& asset, size_t ref_idx, size_t stride, double target_speed) {
    const int N = solver.get_horizon();
    for (int k = 0; k <= N; ++k) {
        const size_t idx = (ref_idx + static_cast<size_t>(k) * stride) % asset.x.size();
        solver.set_parameter(k, 0, asset.x[idx]);
        solver.set_parameter(k, 1, asset.y[idx]);
        solver.set_parameter(k, 2, asset.psi[idx]);
        solver.set_parameter(k, 3, target_speed);
        solver.set_parameter(k, 4, asset.nx[idx]);
        solver.set_parameter(k, 5, asset.ny[idx]);
        solver.set_parameter(k, 6, asset.w_left[idx]);
        solver.set_parameter(k, 7, asset.w_right[idx]);
        solver.set_state_guess(k, 0, asset.x[idx]);
        solver.set_state_guess(k, 1, asset.y[idx]);
        solver.set_state_guess(k, 2, asset.psi[idx]);
        solver.set_state_guess(k, 3, target_speed);
        solver.set_state_guess(k, 4, 0.0);
        if (k < N) {
            solver.set_control_guess(k, 0, 0.0);
            solver.set_control_guess(k, 1, 0.0);
        }
    }
}

void update_driving_parameters(MiniSolver<DrivingTrackModel, kMaxHorizon>& solver, const DrivingTrackAsset& asset, size_t ref_idx, size_t stride, double target_speed) {
    const int N = solver.get_horizon();
    for (int k = 0; k <= N; ++k) {
        const size_t idx = (ref_idx + static_cast<size_t>(k) * stride) % asset.x.size();
        solver.set_parameter(k, 0, asset.x[idx]);
        solver.set_parameter(k, 1, asset.y[idx]);
        solver.set_parameter(k, 2, asset.psi[idx]);
        solver.set_parameter(k, 3, target_speed);
        solver.set_parameter(k, 4, asset.nx[idx]);
        solver.set_parameter(k, 5, asset.ny[idx]);
        solver.set_parameter(k, 6, asset.w_left[idx]);
        solver.set_parameter(k, 7, asset.w_right[idx]);
    }
}

void seed_robot_solver(MiniSolver<RobotReferenceModel, kMaxHorizon>& solver, const RobotReferenceAsset& asset, size_t ref_idx) {
    const int N = solver.get_horizon();
    for (int k = 0; k <= N; ++k) {
        const size_t idx = std::min(ref_idx + static_cast<size_t>(k), asset.x.size() - 1);
        solver.set_parameter(k, 0, asset.x[idx]);
        solver.set_parameter(k, 1, asset.y[idx]);
        solver.set_parameter(k, 2, asset.z[idx]);
        solver.set_parameter(k, 3, asset.vx[idx]);
        solver.set_parameter(k, 4, asset.vy[idx]);
        solver.set_parameter(k, 5, asset.vz[idx]);
        solver.set_state_guess(k, 0, asset.x[idx]);
        solver.set_state_guess(k, 1, asset.y[idx]);
        solver.set_state_guess(k, 2, asset.z[idx]);
        solver.set_state_guess(k, 3, asset.vx[idx]);
        solver.set_state_guess(k, 4, asset.vy[idx]);
        solver.set_state_guess(k, 5, asset.vz[idx]);
        if (k < N) {
            solver.set_control_guess(k, 0, 0.0);
            solver.set_control_guess(k, 1, 0.0);
            solver.set_control_guess(k, 2, 0.0);
        }
    }
}

void update_robot_parameters(MiniSolver<RobotReferenceModel, kMaxHorizon>& solver, const RobotReferenceAsset& asset, size_t ref_idx) {
    const int N = solver.get_horizon();
    for (int k = 0; k <= N; ++k) {
        const size_t idx = std::min(ref_idx + static_cast<size_t>(k), asset.x.size() - 1);
        solver.set_parameter(k, 0, asset.x[idx]);
        solver.set_parameter(k, 1, asset.y[idx]);
        solver.set_parameter(k, 2, asset.z[idx]);
        solver.set_parameter(k, 3, asset.vx[idx]);
        solver.set_parameter(k, 4, asset.vy[idx]);
        solver.set_parameter(k, 5, asset.vz[idx]);
    }
}

int run_driving_benchmark(const Args& args) {
    const DrivingTrackAsset asset = load_driving_track(args.asset_path);
    const size_t stride = std::max<size_t>(1, static_cast<size_t>(std::llround(args.target_speed * args.dt / std::max(asset.avg_ds, 1e-6))));
    MiniSolver<DrivingTrackModel, kMaxHorizon> solver(args.horizon, Backend::CPU_SERIAL, make_config());
    solver.set_dt(args.dt);

    DrivingState current{asset.x.front(), asset.y.front(), asset.psi.front(), args.target_speed, 0.0};
    size_t ref_idx = 0;

    std::ofstream out(args.output_path);
    out << "step,status,iterations,time_ms,x,y,psi,v,delta,tracking_error,max_constraint_violation,ref_index\n";

    seed_driving_solver(solver, asset, ref_idx, stride, args.target_speed);
    solver.set_initial_state({current.x, current.y, current.psi, current.v, current.delta});
    solver.rollout_dynamics();

    for (int step = 0; step < args.steps; ++step) {
        if (step > 0) {
            shift_primal_guess(solver);
        }

        update_driving_parameters(solver, asset, ref_idx, stride, args.target_speed);
        solver.set_initial_state({current.x, current.y, current.psi, current.v, current.delta});

        const auto start = std::chrono::steady_clock::now();
        const SolverStatus status = solver.solve();
        const auto end = std::chrono::steady_clock::now();
        const double time_ms = std::chrono::duration<double, std::milli>(end - start).count();

        const double tracking_error = std::hypot(current.x - asset.x[ref_idx], current.y - asset.y[ref_idx]);
        const double max_viol = max_positive_constraint(solver);
        out << step << ','
            << status_to_string(status) << ','
            << solver.get_iteration_count() << ','
            << std::setprecision(17) << time_ms << ','
            << current.x << ','
            << current.y << ','
            << current.psi << ','
            << current.v << ','
            << current.delta << ','
            << tracking_error << ','
            << max_viol << ','
            << ref_idx << '\n';

        const auto u = solver.get_control(0);
        const double a = (u.size() >= 1) ? u[0] : 0.0;
        const double delta_rate = (u.size() >= 2) ? u[1] : 0.0;
        current = simulate_driving(current, a, delta_rate, args.dt);
        ref_idx = nearest_cyclic_index(asset, current.x, current.y, ref_idx + stride, 30 + 4 * stride);
    }

    std::cout << "wrote " << args.output_path << "\n";
    return 0;
}

int run_robotics_benchmark(const Args& args) {
    const RobotReferenceAsset asset = load_robot_reference(args.asset_path);
    const int steps = std::min<int>(args.steps, static_cast<int>(asset.x.size()));
    MiniSolver<RobotReferenceModel, kMaxHorizon> solver(args.horizon, Backend::CPU_SERIAL, make_config());
    solver.set_dt(args.dt);

    RobotState current{asset.x.front(), asset.y.front(), asset.z.front(), asset.vx.front(), asset.vy.front(), asset.vz.front()};

    std::ofstream out(args.output_path);
    out << "step,status,iterations,time_ms,x,y,z,vx,vy,vz,tracking_error,max_constraint_violation\n";

    seed_robot_solver(solver, asset, 0);
    solver.set_initial_state({current.x, current.y, current.z, current.vx, current.vy, current.vz});
    solver.rollout_dynamics();

    for (int step = 0; step < steps; ++step) {
        if (step > 0) {
            shift_primal_guess(solver);
        }

        const size_t ref_idx = static_cast<size_t>(step);
        update_robot_parameters(solver, asset, ref_idx);
        solver.set_initial_state({current.x, current.y, current.z, current.vx, current.vy, current.vz});

        const auto start = std::chrono::steady_clock::now();
        const SolverStatus status = solver.solve();
        const auto end = std::chrono::steady_clock::now();
        const double time_ms = std::chrono::duration<double, std::milli>(end - start).count();

        const double dx = current.x - asset.x[ref_idx];
        const double dy = current.y - asset.y[ref_idx];
        const double dz = current.z - asset.z[ref_idx];
        const double tracking_error = std::sqrt(dx * dx + dy * dy + dz * dz);
        const double max_viol = max_positive_constraint(solver);
        out << step << ','
            << status_to_string(status) << ','
            << solver.get_iteration_count() << ','
            << std::setprecision(17) << time_ms << ','
            << current.x << ','
            << current.y << ','
            << current.z << ','
            << current.vx << ','
            << current.vy << ','
            << current.vz << ','
            << tracking_error << ','
            << max_viol << '\n';

        const auto u = solver.get_control(0);
        const double ax = (u.size() >= 1) ? u[0] : 0.0;
        const double ay = (u.size() >= 2) ? u[1] : 0.0;
        const double az = (u.size() >= 3) ? u[2] : 0.0;
        current = simulate_robot(current, ax, ay, az, args.dt);
    }

    std::cout << "wrote " << args.output_path << "\n";
    return 0;
}

}  // namespace minisolver_asset_bench

int main(int argc, char** argv) {
    using namespace minisolver_asset_bench;
    try {
        const Args args = parse_args(argc, argv);
        if (args.benchmark == "driving_track_following") {
            return run_driving_benchmark(args);
        }
        if (args.benchmark == "robotics_reference_tracking") {
            return run_robotics_benchmark(args);
        }
        throw std::runtime_error("unsupported benchmark: " + args.benchmark);
    } catch (const std::exception& exc) {
        std::cerr << "asset benchmark error: " << exc.what() << "\n";
        return 1;
    }
}
