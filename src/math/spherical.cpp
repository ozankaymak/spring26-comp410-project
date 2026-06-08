#include "math/spherical.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace hyper::math::sphere {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double acos_checked(double value) {
    if (!std::isfinite(value)) {
        throw std::domain_error("spherical acos input must be finite");
    }

    return std::acos(std::clamp(value, -1.0, 1.0));
}

void require_finite(const Vec3& v, const char* name) {
    if (!is_finite(v)) {
        throw std::invalid_argument(std::string{name} + " must be finite");
    }
}

void require_sphere_point(const Vec3& p, const char* name) {
    require_finite(p, name);

    if (!is_on_sphere(p)) {
        throw std::domain_error(std::string{name} + " must lie on the unit sphere");
    }
}

void require_finite_matrix(const Mat3& matrix, const char* name) {
    if (!is_finite(matrix)) {
        throw std::invalid_argument(std::string{name} + " must be finite");
    }
}

void require_isometry(const Mat3& transform, const char* name) {
    require_finite_matrix(transform, name);

    if (!is_rotation(transform)) {
        throw std::domain_error(std::string{name} + " must preserve the spherical metric");
    }
}

double cot(double value) {
    return std::cos(value) / std::sin(value);
}

double tangent_length(const Vec2& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

Vec3 subtract(const Vec3& a, const Vec3& b) {
    return Vec3{a.t - b.t, a.x - b.x, a.y - b.y};
}

Vec3 scale(const Vec3& v, double value) {
    return Vec3{v.t * value, v.x * value, v.y * value};
}

Vec3 column(const Mat3& matrix, int index) {
    return Vec3{matrix.m[0][index], matrix.m[1][index], matrix.m[2][index]};
}

// Removes the component of v along the (unit) position, leaving a tangent
// vector. On the sphere the surface normal is the position itself, so the
// tangent space is the Euclidean orthogonal complement of p.
Vec3 tangent_projection(const Vec3& v, const Vec3& position) {
    return subtract(v, scale(position, euclidean_dot(v, position)));
}

Vec3 remove_axis(const Vec3& v, const Vec3& axis) {
    return subtract(v, scale(axis, euclidean_dot(v, axis)));
}

Vec3 normalize_tangent(const Vec3& v, const char* name) {
    require_finite(v, name);

    const double norm_squared = euclidean_norm_squared(v);
    if (norm_squared <= kDefaultTolerance) {
        throw std::domain_error(std::string{name} + " must be a non-degenerate tangent");
    }

    return scale(v, 1.0 / std::sqrt(norm_squared));
}

Vec3 fallback_tangent(const Vec3& position, const Vec3* avoid) {
    const std::array<Vec3, 3> candidates{
        Vec3{0.0, 1.0, 0.0},
        Vec3{0.0, 0.0, 1.0},
        Vec3{1.0, 0.0, 0.0},
    };

    Vec3 best{};
    double best_norm = -1.0;

    for (const Vec3& candidate : candidates) {
        Vec3 tangent = tangent_projection(candidate, position);
        if (avoid != nullptr) {
            tangent = remove_axis(tangent, *avoid);
        }

        const double norm = euclidean_norm_squared(tangent);
        if (norm > best_norm) {
            best = tangent;
            best_norm = norm;
        }
    }

    if (best_norm <= kDefaultTolerance) {
        throw std::domain_error("could not construct a tangent fallback");
    }

    return normalize_tangent(best, "fallback tangent");
}

} // namespace

double euclidean_dot(const Vec3& a, const Vec3& b) {
    return a.t * b.t + a.x * b.x + a.y * b.y;
}

double euclidean_norm_squared(const Vec3& v) {
    return euclidean_dot(v, v);
}

Vec3 origin() {
    return Vec3{1.0, 0.0, 0.0};
}

Vec3 point_from_polar(double radius, double angle_radians) {
    if (!std::isfinite(radius) || !std::isfinite(angle_radians) || radius < 0.0) {
        throw std::invalid_argument("polar spherical coordinates require finite radius >= 0");
    }

    const double sin_radius = std::sin(radius);
    return Vec3{
        std::cos(radius),
        sin_radius * std::cos(angle_radians),
        sin_radius * std::sin(angle_radians),
    };
}

bool is_on_sphere(const Vec3& p, double tolerance) {
    return is_finite(p) && std::abs(euclidean_norm_squared(p) - 1.0) <= tolerance;
}

bool is_rotation(const Mat3& transform, double tolerance) {
    if (!is_finite(transform)) {
        return false;
    }

    // An orthogonal matrix has columns whose Euclidean inner products reproduce
    // the identity. This is the R^T R = I check written directly.
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            const double expected = i == j ? 1.0 : 0.0;
            if (std::abs(euclidean_dot(column(transform, i), column(transform, j)) - expected) > tolerance) {
                return false;
            }
        }
    }

    return true;
}

bool is_orthonormal_frame(const CameraFrame& frame, double tolerance) {
    return is_on_sphere(frame.position, tolerance) &&
           std::abs(euclidean_dot(frame.position, frame.forward)) <= tolerance &&
           std::abs(euclidean_dot(frame.position, frame.right)) <= tolerance &&
           std::abs(euclidean_dot(frame.forward, frame.right)) <= tolerance &&
           std::abs(euclidean_norm_squared(frame.forward) - 1.0) <= tolerance &&
           std::abs(euclidean_norm_squared(frame.right) - 1.0) <= tolerance;
}

Vec3 sphere_normalize(const Vec3& v) {
    require_finite(v, "vector");

    const double norm_squared = euclidean_norm_squared(v);
    if (norm_squared <= kDefaultTolerance) {
        throw std::domain_error("sphere normalization requires a non-zero vector");
    }

    return scale(v, 1.0 / std::sqrt(norm_squared));
}

double intrinsic_distance(const Vec3& a, const Vec3& b) {
    require_sphere_point(a, "first point");
    require_sphere_point(b, "second point");

    return acos_checked(euclidean_dot(a, b));
}

Vec3 geodesic_lerp(const Vec3& a, const Vec3& b, double t) {
    const double d = sphere::intrinsic_distance(a, b);
    if (d < 1e-12) {
        return a;
    }

    const double sin_d = std::sin(d);
    if (std::abs(sin_d) < 1e-12) {
        // Antipodal points have no unique great circle; fall back to a.
        return a;
    }

    const double sa = std::sin((1.0 - t) * d) / sin_d;
    const double sb = std::sin(t * d) / sin_d;

    return sphere_normalize(Vec3{
        sa * a.t + sb * b.t,
        sa * a.x + sb * b.x,
        sa * a.y + sb * b.y,
    });
}

Vec2 project_to_stereographic_disk(const Vec3& p) {
    require_sphere_point(p, "point");
    return Vec2{p.x / (p.t + 1.0), p.y / (p.t + 1.0)};
}

Vec2 project_to_gnomonic_disk(const Vec3& p) {
    require_sphere_point(p, "point");
    return Vec2{p.x / p.t, p.y / p.t};
}

Mat3 identity_isometry() {
    return Mat3{};
}

Mat3 spherical_rotation(Vec2 local_delta) {
    if (!is_finite(local_delta)) {
        throw std::invalid_argument("rotation tangent vector must be finite");
    }

    // The tangent-vector norm is exactly the angular distance for this local
    // move. The unit vector (nx, ny) selects the tangent direction.
    const double distance = tangent_length(local_delta);
    if (distance <= kDefaultTolerance) {
        return identity_isometry();
    }

    const double nx = local_delta.x / distance;
    const double ny = local_delta.y / distance;

    // Rotation by `distance` in the plane spanned by the time axis and the
    // chosen tangent direction. This is the SO(3) counterpart of the Lorentz
    // boost: cosh/sinh become cos/sin and the time/space coupling picks up the
    // antisymmetric sign of a rotation.
    const double c = std::cos(distance);
    const double s = std::sin(distance);
    const double spatial = c - 1.0;

    return Mat3{{
        {c, -s * nx, -s * ny},
        {s * nx, 1.0 + spatial * nx * nx, spatial * nx * ny},
        {s * ny, spatial * nx * ny, 1.0 + spatial * ny * ny},
    }};
}

Mat3 compose_isometries(const Mat3& a, const Mat3& b) {
    require_finite_matrix(a, "first transform");
    require_finite_matrix(b, "second transform");

    Mat3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            result.m[row][col] = 0.0;
            for (int k = 0; k < 3; ++k) {
                result.m[row][col] += a.m[row][k] * b.m[k][col];
            }
        }
    }

    return result;
}

Mat3 inverse_isometry(const Mat3& transform) {
    require_isometry(transform, "transform");

    // For an orthogonal matrix the inverse is the transpose.
    Mat3 result{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            result.m[row][col] = transform.m[col][row];
        }
    }

    return result;
}

Vec3 apply_isometry(const Mat3& transform, const Vec3& v) {
    require_finite_matrix(transform, "transform");
    require_finite(v, "vector");

    return Vec3{
        transform.m[0][0] * v.t + transform.m[0][1] * v.x + transform.m[0][2] * v.y,
        transform.m[1][0] * v.t + transform.m[1][1] * v.x + transform.m[1][2] * v.y,
        transform.m[2][0] * v.t + transform.m[2][1] * v.x + transform.m[2][2] * v.y,
    };
}

CameraFrame apply_isometry(const Mat3& transform, const CameraFrame& frame) {
    require_isometry(transform, "transform");
    return sphere::orthonormalize_frame(CameraFrame{
        sphere::apply_isometry(transform, frame.position),
        sphere::apply_isometry(transform, frame.forward),
        sphere::apply_isometry(transform, frame.right),
    });
}

CameraFrame canonical_frame() {
    return CameraFrame{origin(), Vec3{0.0, 1.0, 0.0}, Vec3{0.0, 0.0, 1.0}};
}

CameraFrame orthonormalize_frame(const CameraFrame& frame) {
    CameraFrame result{};
    result.position = sphere_normalize(frame.position);

    // Euclidean Gram-Schmidt: project tangent axes off the position normal
    // first, then make right orthogonal to forward.
    try {
        result.forward = normalize_tangent(tangent_projection(frame.forward, result.position), "frame forward");
    } catch (const std::domain_error&) {
        result.forward = fallback_tangent(result.position, nullptr);
    }

    Vec3 right = tangent_projection(frame.right, result.position);
    right = remove_axis(right, result.forward);

    try {
        result.right = normalize_tangent(right, "frame right");
    } catch (const std::domain_error&) {
        result.right = fallback_tangent(result.position, &result.forward);
    }

    return result;
}

CameraFrame move_frame(const CameraFrame& frame, Vec2 local_delta) {
    const CameraFrame clean = sphere::orthonormalize_frame(frame);

    // frame_to_isometry maps the canonical basis to the current frame. Right
    // multiplication applies the rotation in the camera-local tangent basis.
    const Mat3 transform =
        sphere::compose_isometries(sphere::frame_to_isometry(clean), spherical_rotation(local_delta));

    return sphere::orthonormalize_frame(CameraFrame{
        column(transform, 0),
        column(transform, 1),
        column(transform, 2),
    });
}

Mat3 frame_to_isometry(const CameraFrame& frame) {
    const CameraFrame clean = sphere::orthonormalize_frame(frame);

    return Mat3{{
        {clean.position.t, clean.forward.t, clean.right.t},
        {clean.position.x, clean.forward.x, clean.right.x},
        {clean.position.y, clean.forward.y, clean.right.y},
    }};
}

bool is_spherical_tiling(int p, int q) {
    return p >= 3 && q >= 3 && (p - 2) * (q - 2) < 4;
}

RegularTilingMetrics regular_tiling_metrics(RegularTilingParameters params) {
    if (!is_spherical_tiling(params.p, params.q)) {
        throw std::invalid_argument("regular spherical tiling must satisfy p >= 3, q >= 3, and (p - 2)(q - 2) < 4");
    }

    const double p_angle = kPi / static_cast<double>(params.p);
    const double q_angle = kPi / static_cast<double>(params.q);

    // Same right-triangle relations as the hyperbolic case, with the spherical
    // law of cosines (arccos) replacing the hyperbolic one (arcosh).
    const double half_edge = acos_checked(std::cos(p_angle) / std::sin(q_angle));
    const double inradius = acos_checked(std::cos(q_angle) / std::sin(p_angle));
    const double circumradius = acos_checked(cot(p_angle) * cot(q_angle));

    return RegularTilingMetrics{
        2.0 * half_edge,
        inradius,
        circumradius,
    };
}

} // namespace hyper::math::sphere
