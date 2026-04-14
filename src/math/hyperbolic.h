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

struct Mat3 {
    // Row-major transform acting on column vectors in (t, x, y) order.
    double m[3][3] = {
        {1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {0.0, 0.0, 1.0},
    };
};

struct CameraFrame {
    // position is timelike; forward and right are spacelike tangent axes.
    Vec3 position{1.0, 0.0, 0.0};
    Vec3 forward{0.0, 1.0, 0.0};
    Vec3 right{0.0, 0.0, 1.0};
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
bool is_finite(const Mat3& m);
bool is_timelike(const Vec3& v, double tolerance = kDefaultTolerance);
bool is_on_hyperboloid(const Vec3& p, double tolerance = kDefaultTolerance);
bool is_lorentz_isometry(const Mat3& transform, double tolerance = kDefaultTolerance);
bool is_orthonormal_frame(const CameraFrame& frame, double tolerance = kDefaultTolerance);

// Normalizes any finite timelike vector onto the future hyperboloid sheet.
Vec3 hyperboloid_normalize(const Vec3& v);

// Requires both inputs to already be valid future-sheet hyperboloid points.
double intrinsic_distance(const Vec3& a, const Vec3& b);

Vec2 project_to_poincare_disk(const Vec3& p);
Vec2 project_to_klein_disk(const Vec3& p);

Mat3 identity_isometry();

// Builds a local translation on H^2. The vector length is the hyperbolic
// distance moved, and its direction is measured in the current tangent frame.
Mat3 lorentz_boost(Vec2 local_delta);
Mat3 compose_isometries(const Mat3& a, const Mat3& b);

// Lorentz inverse of a metric-preserving transform.
Mat3 inverse_isometry(const Mat3& transform);
Vec3 apply_isometry(const Mat3& transform, const Vec3& v);
CameraFrame apply_isometry(const Mat3& transform, const CameraFrame& frame);

CameraFrame canonical_frame();

// Minkowski Gram-Schmidt cleanup for accumulated floating-point drift.
CameraFrame orthonormalize_frame(const CameraFrame& frame);

// local_delta.x moves along frame.forward, local_delta.y moves along frame.right.
CameraFrame move_frame(const CameraFrame& frame, Vec2 local_delta);
Mat3 frame_to_isometry(const CameraFrame& frame);

bool is_hyperbolic_tiling(int p, int q);
RegularTilingMetrics regular_tiling_metrics(RegularTilingParameters params);

} // namespace hyper::math
