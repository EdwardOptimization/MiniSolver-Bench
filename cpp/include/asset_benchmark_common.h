#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace nmpc_bench {

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

struct DrivingTrackSample {
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;
    double nx = 0.0;
    double ny = 0.0;
    double w_left = 0.0;
    double w_right = 0.0;
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

inline std::vector<std::string> split_csv_line(const std::string& line) {
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

inline double wrap_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline DrivingTrackAsset load_driving_track(const std::string& path) {
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

inline RobotReferenceAsset load_robot_reference(const std::string& path) {
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

inline size_t nearest_cyclic_index(const DrivingTrackAsset& asset, double x, double y, size_t anchor, size_t window) {
    const size_t n = asset.x.size();
    size_t best = anchor % n;
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

inline DrivingTrackSample sample_driving_track(const DrivingTrackAsset& asset, double s_query) {
    const size_t n = asset.s.size();
    if (n == 0) {
        throw std::runtime_error("cannot sample empty driving asset");
    }
    if (n == 1) {
        return DrivingTrackSample{
            asset.x[0], asset.y[0], asset.psi[0], asset.nx[0], asset.ny[0], asset.w_left[0], asset.w_right[0]
        };
    }

    const double seam_ds = std::hypot(asset.x.front() - asset.x.back(), asset.y.front() - asset.y.back());
    const double period = asset.s.back() + ((seam_ds > 1e-9) ? seam_ds : 0.0);
    double s_wrapped = std::fmod(s_query, period);
    if (s_wrapped < 0.0) s_wrapped += period;

    size_t hi = static_cast<size_t>(std::upper_bound(asset.s.begin(), asset.s.end(), s_wrapped) - asset.s.begin());
    size_t lo = (hi == 0) ? (n - 1) : (hi - 1);
    if (hi >= n) hi = 0;

    double s0 = asset.s[lo];
    double s1 = (hi == 0) ? period : asset.s[hi];
    double alpha = 0.0;
    if (s1 > s0 + 1e-12) {
        alpha = (s_wrapped - s0) / (s1 - s0);
    }
    alpha = std::clamp(alpha, 0.0, 1.0);

    auto lerp = [&](double a, double b) { return a * (1.0 - alpha) + b * alpha; };

    double psi0 = asset.psi[lo];
    double psi1 = asset.psi[hi];
    double dpsi = wrap_angle(psi1 - psi0);
    double psi = wrap_angle(psi0 + alpha * dpsi);

    DrivingTrackSample sample;
    sample.x = lerp(asset.x[lo], asset.x[hi]);
    sample.y = lerp(asset.y[lo], asset.y[hi]);
    sample.psi = psi;
    sample.nx = -std::sin(psi);
    sample.ny = std::cos(psi);
    sample.w_left = lerp(asset.w_left[lo], asset.w_left[hi]);
    sample.w_right = lerp(asset.w_right[lo], asset.w_right[hi]);
    return sample;
}

}  // namespace nmpc_bench
