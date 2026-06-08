#pragma once

#include "math/geometry_mode.h"
#include "math/hyperbolic.h"

// Spherical analogue of the hyperbolic geometry layer.
//
// This module mirrors the public surface of hyperbolic.h but for the unit
// 2-sphere instead of the hyperboloid. The hyperbolic API in hyperbolic.h is
// left completely untouched; the two live side by side and share the plain
// data types (Vec2, Vec3, Mat3, CameraFrame, RegularTiling*) so that the
// renderer and tiling layers can dispatch between them at runtime.
//
// Points use the embedded model with the Euclidean signature (+, +, +):
//
//     S^2 = { (t, x, y) in R^3 : t^2 + x^2 + y^2 = 1 }
//
// The "origin" is (1, 0, 0). The first coordinate t plays the role that the
// timelike coordinate plays on the hyperboloid, which keeps the embedding,
// projections, and camera frames structurally identical to the hyperbolic
// path. Spherical isometries are rotations of SO(3); the metric is the
// ordinary dot product and the distance between two points is the angle
// between them.
namespace hyper::math::sphere {

// Euclidean inner product on R^3, the spherical counterpart of minkowski_dot.
double euclidean_dot(const Vec3& a, const Vec3& b);
double euclidean_norm_squared(const Vec3& v);

// (1, 0, 0): the spherical origin.
Vec3 origin();

// Geodesic-polar point at angular radius `radius` and bearing `angle_radians`.
//   (cos r, sin r cos a, sin r sin a)
Vec3 point_from_polar(double radius, double angle_radians);

bool is_on_sphere(const Vec3& p, double tolerance = kDefaultTolerance);
bool is_rotation(const Mat3& transform, double tolerance = kDefaultTolerance);
bool is_orthonormal_frame(const CameraFrame& frame, double tolerance = kDefaultTolerance);

// Projects any finite non-zero vector radially onto the unit sphere.
Vec3 sphere_normalize(const Vec3& v);

// Great-circle distance: arccos of the clamped dot product. Both inputs must
// already lie on the unit sphere.
double intrinsic_distance(const Vec3& a, const Vec3& b);

// Spherical linear interpolation (slerp) along the great circle from a to b.
Vec3 geodesic_lerp(const Vec3& a, const Vec3& b, double t);

// Stereographic projection from the antipole, the spherical analogue of the
// Poincare disk map: (x, y) / (t + 1).
Vec2 project_to_stereographic_disk(const Vec3& p);

// Gnomonic projection from the sphere centre, the spherical analogue of the
// Klein disk map: (x, y) / t. Only valid on the near hemisphere (t > 0).
Vec2 project_to_gnomonic_disk(const Vec3& p);

Mat3 identity_isometry();

// Builds a local rotation that translates the origin by `local_delta` along the
// current tangent frame. The vector length is the angular distance moved and
// its direction is the bearing within the tangent plane. This is the spherical
// counterpart of lorentz_boost.
Mat3 spherical_rotation(Vec2 local_delta);
Mat3 compose_isometries(const Mat3& a, const Mat3& b);

// For a rotation matrix the inverse is just the transpose.
Mat3 inverse_isometry(const Mat3& transform);
Vec3 apply_isometry(const Mat3& transform, const Vec3& v);
CameraFrame apply_isometry(const Mat3& transform, const CameraFrame& frame);

CameraFrame canonical_frame();

// Euclidean Gram-Schmidt cleanup for accumulated floating-point drift.
CameraFrame orthonormalize_frame(const CameraFrame& frame);

// local_delta.x moves along frame.forward, local_delta.y moves along frame.right.
CameraFrame move_frame(const CameraFrame& frame, Vec2 local_delta);
Mat3 frame_to_isometry(const CameraFrame& frame);

// A regular {p, q} tiling closes up on the sphere when (p - 2)(q - 2) < 4.
bool is_spherical_tiling(int p, int q);
RegularTilingMetrics regular_tiling_metrics(RegularTilingParameters params);

} // namespace hyper::math::sphere
