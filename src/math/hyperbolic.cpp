#include "math/hyperbolic.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace hyper::math {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double acosh_checked(double value) {
    if (!std::isfinite(value) || value < 1.0 - kDefaultTolerance) {
        throw std::domain_error("hyperbolic acosh input must be >= 1");
    }

    return std::acosh(std::max(1.0, value));
}

void require_finite(const Vec3& v, const char* name) {
    if (!is_finite(v)) {
        throw std::invalid_argument(std::string{name} + " must be finite");
    }
}

void require_hyperboloid_point(const Vec3& p, const char* name) {
    require_finite(p, name);

    if (!is_on_hyperboloid(p)) {
        throw std::domain_error(std::string{name} + " must be on the future hyperboloid sheet");
    }
}

double cot(double value) {
    return std::cos(value) / std::sin(value);
}

} // namespace

Vec3 origin() {
    return Vec3{1.0, 0.0, 0.0};
}

Vec3 point_from_polar(double radius, double angle_radians) {
    if (!std::isfinite(radius) || !std::isfinite(angle_radians) || radius < 0.0) {
        throw std::invalid_argument("polar hyperbolic coordinates require finite radius >= 0");
    }

    const double sinh_radius = std::sinh(radius);
    return Vec3{
        std::cosh(radius),
        sinh_radius * std::cos(angle_radians),
        sinh_radius * std::sin(angle_radians),
    };
}

double minkowski_dot(const Vec3& a, const Vec3& b) {
    return -a.t * b.t + a.x * b.x + a.y * b.y;
}

double minkowski_norm_squared(const Vec3& v) {
    return minkowski_dot(v, v);
}

bool is_finite(const Vec2& v) {
    return std::isfinite(v.x) && std::isfinite(v.y);
}

bool is_finite(const Vec3& v) {
    return std::isfinite(v.t) && std::isfinite(v.x) && std::isfinite(v.y);
}

bool is_timelike(const Vec3& v, double tolerance) {
    return is_finite(v) && minkowski_norm_squared(v) < -tolerance;
}

bool is_on_hyperboloid(const Vec3& p, double tolerance) {
    return is_finite(p) && p.t > 0.0 && std::abs(minkowski_norm_squared(p) + 1.0) <= tolerance;
}

Vec3 hyperboloid_normalize(const Vec3& v) {
    require_finite(v, "vector");

    const double norm_squared = minkowski_norm_squared(v);
    if (norm_squared >= -kDefaultTolerance) {
        throw std::domain_error("hyperboloid normalization requires a timelike vector");
    }

    const double scale = 1.0 / std::sqrt(-norm_squared);
    Vec3 normalized{v.t * scale, v.x * scale, v.y * scale};

    if (normalized.t < 0.0) {
        normalized.t = -normalized.t;
        normalized.x = -normalized.x;
        normalized.y = -normalized.y;
    }

    return normalized;
}

double intrinsic_distance(const Vec3& a, const Vec3& b) {
    require_hyperboloid_point(a, "first point");
    require_hyperboloid_point(b, "second point");

    return acosh_checked(-minkowski_dot(a, b));
}

Vec2 project_to_poincare_disk(const Vec3& p) {
    require_hyperboloid_point(p, "point");
    return Vec2{p.x / (p.t + 1.0), p.y / (p.t + 1.0)};
}

Vec2 project_to_klein_disk(const Vec3& p) {
    require_hyperboloid_point(p, "point");
    return Vec2{p.x / p.t, p.y / p.t};
}

bool is_hyperbolic_tiling(int p, int q) {
    return p >= 3 && q >= 3 && (p - 2) * (q - 2) > 4;
}

RegularTilingMetrics regular_tiling_metrics(RegularTilingParameters params) {
    if (!is_hyperbolic_tiling(params.p, params.q)) {
        throw std::invalid_argument("regular tiling must satisfy p >= 3, q >= 3, and (p - 2)(q - 2) > 4");
    }

    const double p_angle = kPi / static_cast<double>(params.p);
    const double q_angle = kPi / static_cast<double>(params.q);

    const double half_edge = acosh_checked(std::cos(p_angle) / std::sin(q_angle));
    const double inradius = acosh_checked(std::cos(q_angle) / std::sin(p_angle));
    const double circumradius = acosh_checked(cot(p_angle) * cot(q_angle));

    return RegularTilingMetrics{
        2.0 * half_edge,
        inradius,
        circumradius,
    };
}

} // namespace hyper::math
