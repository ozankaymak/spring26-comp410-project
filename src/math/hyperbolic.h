#pragma once

namespace hyper::math {

constexpr double kCurvature = -1.0;
constexpr double kDefaultTolerance = 1.0e-9;

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Vec3 {
    double t = 0.0;
    double x = 0.0;
    double y = 0.0;
};

struct RegularTilingParameters {
    int p = 4;
    int q = 6;
};

struct RegularTilingMetrics {
    double edge_length = 0.0;
    double inradius = 0.0;
    double circumradius = 0.0;
};

// Points use the hyperboloid model with Minkowski signature (-, +, +).
// Valid positions satisfy minkowski_norm_squared(p) == -1 and p.t > 0.
Vec3 origin();
Vec3 point_from_polar(double radius, double angle_radians);

double minkowski_dot(const Vec3& a, const Vec3& b);
double minkowski_norm_squared(const Vec3& v);

bool is_finite(const Vec2& v);
bool is_finite(const Vec3& v);
bool is_timelike(const Vec3& v, double tolerance = kDefaultTolerance);
bool is_on_hyperboloid(const Vec3& p, double tolerance = kDefaultTolerance);

// Normalizes any finite timelike vector onto the future hyperboloid sheet.
Vec3 hyperboloid_normalize(const Vec3& v);

// Requires both inputs to already be valid future-sheet hyperboloid points.
double intrinsic_distance(const Vec3& a, const Vec3& b);

Vec2 project_to_poincare_disk(const Vec3& p);
Vec2 project_to_klein_disk(const Vec3& p);

bool is_hyperbolic_tiling(int p, int q);
RegularTilingMetrics regular_tiling_metrics(RegularTilingParameters params);

} // namespace hyper::math
