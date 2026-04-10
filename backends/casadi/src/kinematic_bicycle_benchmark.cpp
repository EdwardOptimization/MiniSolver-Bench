#include <casadi/casadi.hpp>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "asset_benchmark_common.h"

using namespace casadi;

namespace {

constexpr double kWheelbase = 0.33;
constexpr double kVMin = 0.1;
constexpr double kVMax = 12.0;
constexpr double kDeltaMin = -0.50;
constexpr double kDeltaMax = 0.50;
constexpr int kNx = 5;
constexpr int kNu = 2;
constexpr int kNp = 8;
constexpr int kNh = 6;

double percentile(std::vector<double> values, double frac) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double idx = frac * static_cast<double>(values.size() - 1);
    const auto lo = static_cast<size_t>(std::floor(idx));
    const auto hi = static_cast<size_t>(std::ceil(idx));
    if (lo == hi) return values[lo];
    const double w = idx - static_cast<double>(lo);
    return values[lo] * (1.0 - w) + values[hi] * w;
}

bool success_status(const Dict& stats) {
    if (static_cast<bool>(stats.at("success"))) return true;
    const std::string status = static_cast<std::string>(stats.at("return_status"));
    return status == "Search_Direction_Becomes_Too_Small" || status == "Solved_To_Acceptable_Level";
}

Function build_solver(int horizon, double dt) {
    SX x = SX::sym("x", kNx);
    SX u = SX::sym("u", kNu);
    SX p = SX::sym("p", kNp);

    SX f = SX::vertcat({
        x(3) * cos(x(2)),
        x(3) * sin(x(2)),
        x(3) * tan(x(4)) / kWheelbase,
        u(0),
        u(1),
    });

    Function ode("ode", {x, u, p}, {f});
    auto rk4 = [&](const SX& xk, const SX& uk, const SX& pk) {
        SX k1 = ode(std::vector<SX>{xk, uk, pk})[0];
        SX k2 = ode(std::vector<SX>{xk + 0.5 * dt * k1, uk, pk})[0];
        SX k3 = ode(std::vector<SX>{xk + 0.5 * dt * k2, uk, pk})[0];
        SX k4 = ode(std::vector<SX>{xk + dt * k3, uk, pk})[0];
        return xk + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
    };

    std::vector<SX> X;
    std::vector<SX> U;
    std::vector<SX> S;
    X.reserve(static_cast<size_t>(horizon + 1));
    U.reserve(static_cast<size_t>(horizon));
    S.reserve(static_cast<size_t>(horizon));
    for (int k = 0; k <= horizon; ++k) X.push_back(SX::sym("X_" + std::to_string(k), kNx));
    for (int k = 0; k < horizon; ++k) U.push_back(SX::sym("U_" + std::to_string(k), kNu));
    for (int k = 0; k < horizon; ++k) S.push_back(SX::sym("S_" + std::to_string(k), kNh));
    SX p0 = SX::sym("P0", kNx);
    SX P = SX::sym("P", (horizon + 1) * kNp);

    SX obj = SX(0);
    std::vector<SX> g;
    g.push_back(X[0] - p0);
    const SX W = SX(DM::diag(std::vector<double>{20.0, 20.0, 5.0, 2.0, 0.5, 0.05, 0.05}));
    const SX Wn = SX(DM::diag(std::vector<double>{20.0, 20.0, 5.0, 2.0, 0.5}));
    const SX slack_w = SX(DM(std::vector<double>{200.0, 200.0, 20.0, 20.0, 20.0, 20.0}));
    for (int k = 0; k < horizon; ++k) {
        SX pk = P(Slice(k * kNp, (k + 1) * kNp));
        SX lateral = pk(4) * (X[k](0) - pk(0)) + pk(5) * (X[k](1) - pk(1));
        SX yk = SX::vertcat({
            X[k](0) - pk(0),
            X[k](1) - pk(1),
            atan2(sin(X[k](2) - pk(2)), cos(X[k](2) - pk(2))),
            X[k](3) - pk(3),
            X[k](4),
            U[k](0),
            U[k](1),
        });
        SX hk = SX::vertcat({
            lateral - pk(6),
            -lateral - pk(7),
            X[k](3) - kVMax,
            kVMin - X[k](3),
            X[k](4) - kDeltaMax,
            kDeltaMin - X[k](4),
        });
        obj += 0.5 * SX::dot(yk, mtimes(W, yk)) + SX::dot(S[k], slack_w) + 0.5 * SX::dot(S[k], S[k]);
        g.push_back(X[k + 1] - rk4(X[k], U[k], pk));
        g.push_back(hk - S[k]);
    }
    SX pN = P(Slice(horizon * kNp, (horizon + 1) * kNp));
    SX yN = SX::vertcat({
        X[horizon](0) - pN(0),
        X[horizon](1) - pN(1),
        atan2(sin(X[horizon](2) - pN(2)), cos(X[horizon](2) - pN(2))),
        X[horizon](3) - pN(3),
        X[horizon](4),
    });
    obj += 0.5 * SX::dot(yN, mtimes(Wn, yN));

    std::vector<SX> w;
    for (const auto& v : X) w.push_back(v);
    for (const auto& v : U) w.push_back(v);
    for (const auto& v : S) w.push_back(v);

    std::map<std::string, SX> nlp{
        {"x", SX::vertcat(w)},
        {"f", obj},
        {"g", SX::vertcat(g)},
        {"p", SX::vertcat({p0, P})},
    };
    Dict opts;
    opts["print_header"] = false;
    opts["print_iteration"] = false;
    opts["print_status"] = false;
    opts["print_time"] = false;
    opts["qpsol"] = "qrqp";
    Dict qpsol_opts;
    qpsol_opts["error_on_fail"] = false;
    qpsol_opts["print_iter"] = false;
    qpsol_opts["print_header"] = false;
    opts["qpsol_options"] = qpsol_opts;
    opts["max_iter"] = 20;
    opts["tol_pr"] = 1e-4;
    opts["tol_du"] = 1e-4;
    return nlpsol("solver", "sqpmethod", nlp, opts);
}

struct Args {
    std::string asset_path;
    std::string output_path;
    int horizon = 40;
    int steps = 100;
    double dt = 0.05;
    double target_speed = 2.5;
};

Args parse_args(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error(std::string("missing value for ") + name);
            return argv[++i];
        };
        if (key == "--asset") args.asset_path = need("--asset");
        else if (key == "--output") args.output_path = need("--output");
        else if (key == "--horizon") args.horizon = std::stoi(need("--horizon"));
        else if (key == "--steps") args.steps = std::stoi(need("--steps"));
        else if (key == "--dt") args.dt = std::stod(need("--dt"));
        else if (key == "--target-speed") args.target_speed = std::stod(need("--target-speed"));
        else throw std::runtime_error("unknown argument: " + key);
    }
    if (args.asset_path.empty() || args.output_path.empty()) throw std::runtime_error("required args: --asset --output");
    return args;
}

std::vector<double> stage_params(const nmpc_bench::DrivingTrackAsset& asset, size_t idx, double target_speed) {
    return {asset.x[idx], asset.y[idx], asset.psi[idx], target_speed, asset.nx[idx], asset.ny[idx], asset.w_left[idx], asset.w_right[idx]};
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        const auto asset = nmpc_bench::load_driving_track(args.asset_path);
        const size_t stride = std::max<size_t>(1, static_cast<size_t>(std::llround(args.target_speed * args.dt / std::max(asset.avg_ds, 1e-6))));
        Function solver = build_solver(args.horizon, args.dt);

        std::vector<double> current = {asset.x.front(), asset.y.front(), asset.psi.front(), args.target_speed, 0.0};
        size_t ref_idx = 0;
        std::vector<double> last_w(static_cast<size_t>((args.horizon + 1) * kNx + args.horizon * kNu + args.horizon * kNh), 0.0);
        for (int k = 0; k <= args.horizon; ++k) {
            const size_t idx = (ref_idx + static_cast<size_t>(k) * stride) % asset.x.size();
            const auto pk = stage_params(asset, idx, args.target_speed);
            std::copy(pk.begin(), pk.begin() + 5, last_w.begin() + static_cast<size_t>(k * kNx));
        }

        std::vector<double> lbx(last_w.size(), -std::numeric_limits<double>::infinity());
        std::vector<double> ubx(last_w.size(), std::numeric_limits<double>::infinity());
        for (int k = 0; k <= args.horizon; ++k) {
            lbx[static_cast<size_t>(k * kNx + 3)] = kVMin;
            ubx[static_cast<size_t>(k * kNx + 3)] = kVMax;
            lbx[static_cast<size_t>(k * kNx + 4)] = kDeltaMin;
            ubx[static_cast<size_t>(k * kNx + 4)] = kDeltaMax;
        }
        const int u_offset = (args.horizon + 1) * kNx;
        for (int k = 0; k < args.horizon; ++k) {
            lbx[static_cast<size_t>(u_offset + k * kNu + 0)] = -15.0;
            lbx[static_cast<size_t>(u_offset + k * kNu + 1)] = -6.0;
            ubx[static_cast<size_t>(u_offset + k * kNu + 0)] = 15.0;
            ubx[static_cast<size_t>(u_offset + k * kNu + 1)] = 6.0;
        }
        const int s_offset = u_offset + args.horizon * kNu;
        for (int k = 0; k < args.horizon; ++k) {
            for (int i = 0; i < kNh; ++i) lbx[static_cast<size_t>(s_offset + k * kNh + i)] = 0.0;
        }

        std::vector<double> lbg(static_cast<size_t>(kNx + args.horizon * (kNx + kNh)), 0.0);
        std::vector<double> ubg = lbg;
        size_t cursor = static_cast<size_t>(kNx);
        for (int k = 0; k < args.horizon; ++k) {
            for (int i = 0; i < kNx; ++i) {
                lbg[cursor + static_cast<size_t>(i)] = 0.0;
                ubg[cursor + static_cast<size_t>(i)] = 0.0;
            }
            cursor += static_cast<size_t>(kNx);
            for (int i = 0; i < kNh; ++i) {
                lbg[cursor + static_cast<size_t>(i)] = -std::numeric_limits<double>::infinity();
                ubg[cursor + static_cast<size_t>(i)] = 0.0;
            }
            cursor += static_cast<size_t>(kNh);
        }

        std::ofstream out(args.output_path);
        out << "step,status,time_ms,iterations,x,y,psi,v,delta,tracking_error,max_constraint_violation,ref_index\n";
        std::vector<double> times;
        int success = 0;

        for (int step = 0; step < args.steps; ++step) {
            std::vector<double> params;
            params.reserve(static_cast<size_t>(kNx + (args.horizon + 1) * kNp));
            params.insert(params.end(), current.begin(), current.end());
            for (int k = 0; k <= args.horizon; ++k) {
                const size_t idx = (ref_idx + static_cast<size_t>(k) * stride) % asset.x.size();
                const auto pk = stage_params(asset, idx, args.target_speed);
                params.insert(params.end(), pk.begin(), pk.end());
            }

            const auto start = std::chrono::steady_clock::now();
            DMDict arg;
            arg["x0"] = DM(last_w);
            arg["lbx"] = DM(lbx);
            arg["ubx"] = DM(ubx);
            arg["lbg"] = DM(lbg);
            arg["ubg"] = DM(ubg);
            arg["p"] = DM(params);
            DMDict sol = solver(arg);
            const auto end = std::chrono::steady_clock::now();
            const double time_ms = std::chrono::duration<double, std::milli>(end - start).count();
            const Dict stats = solver.stats();
            const bool ok = success_status(stats);
            if (ok) success++;
            times.push_back(time_ms);

            const std::vector<double> solution = std::vector<double>(static_cast<std::vector<double>>(sol.at("x")));
            last_w = solution;
            const int iterations = static_cast<int>(stats.at("iter_count"));
            const std::vector<double> u0 = {solution[static_cast<size_t>(u_offset)], solution[static_cast<size_t>(u_offset + 1)]};
            const double tracking_error = std::hypot(current[0] - asset.x[ref_idx], current[1] - asset.y[ref_idx]);
            const auto pk0 = stage_params(asset, ref_idx, args.target_speed);
            const double lateral = pk0[4] * (current[0] - pk0[0]) + pk0[5] * (current[1] - pk0[1]);
            const double max_viol = std::max({0.0, lateral - pk0[6], -lateral - pk0[7], current[3] - kVMax, kVMin - current[3], current[4] - kDeltaMax, kDeltaMin - current[4]});

            out << step << ',' << (ok ? 0 : 1) << ',' << std::setprecision(17) << time_ms << ',' << iterations << ','
                << current[0] << ',' << current[1] << ',' << current[2] << ',' << current[3] << ',' << current[4] << ','
                << tracking_error << ',' << max_viol << ',' << ref_idx << '\n';

            current[0] += current[3] * std::cos(current[2]) * args.dt;
            current[1] += current[3] * std::sin(current[2]) * args.dt;
            current[2] = nmpc_bench::wrap_angle(current[2] + current[3] * std::tan(current[4]) / kWheelbase * args.dt);
            current[3] = std::clamp(current[3] + u0[0] * args.dt, kVMin, kVMax);
            current[4] = std::clamp(current[4] + u0[1] * args.dt, kDeltaMin, kDeltaMax);
            ref_idx = nmpc_bench::nearest_cyclic_index(asset, current[0], current[1], ref_idx + stride, 30 + 4 * stride);
        }
        std::cout << "steps=" << args.steps << " success=" << success
                  << " median_ms=" << percentile(times, 0.5)
                  << " p95_ms=" << percentile(times, 0.95)
                  << " max_ms=" << (times.empty() ? 0.0 : *std::max_element(times.begin(), times.end()))
                  << " output=" << args.output_path << "\n";
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "casadi kinematic_bicycle_benchmark error: " << exc.what() << "\n";
        return 1;
    }
}
