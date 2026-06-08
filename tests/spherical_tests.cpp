#include "math/spherical.h"
#include "test_support.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

namespace sphere = hyper::math::sphere;

using hyper::test::require;
using hyper::test::require_close;
using hyper::test::require_throws;

void require_vec_close(const hyper::math::Vec3& actual, const hyper::math::Vec3& expected,
                       const char* message) {
    require_close(actual.t, expected.t, std::string(message) + " t");
    require_close(actual.x, expected.x, std::string(message) + " x");
    require_close(actual.y, expected.y, std::string(message) + " y");
}

void require_matrix_close(const hyper::math::Mat3& actual, const hyper::math::Mat3& expected,
                          const char* message) {
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            require_close(actual.m[row][col], expected.m[row][col],
                          std::string(message) + " [" + std::to_string(row) + "," + std::to_string(col) + "]");
        }
    }
}

double euclidean_norm(const hyper::math::Vec2& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

void test_sphere_invariants() {
    const hyper::math::Vec3 o = sphere::origin();
    require_close(sphere::euclidean_dot(o, o), 1.0, "origin has unit norm");
    require(sphere::is_on_sphere(o), "origin lies on the unit sphere");
    require_close(sphere::intrinsic_distance(o, o), 0.0, "origin distance to itself is zero");

    const hyper::math::Vec3 p = sphere::point_from_polar(1.25, 0.4);
    require(sphere::is_on_sphere(p), "polar point lies on the sphere");
    require_close(sphere::intrinsic_distance(o, p), 1.25, "origin-to-polar distance equals radius");
    require_close(sphere::intrinsic_distance(p, o), 1.25, "intrinsic distance is symmetric");

    const hyper::math::Vec3 a{0.5, 0.5, 0.7071067811865476};
    const hyper::math::Vec3 b{0.6, -0.3, 0.74161984870956};
    require_close(sphere::euclidean_dot(a, b), sphere::euclidean_dot(b, a),
                  "Euclidean dot is symmetric");
}

void test_distance_follows_spherical_law_of_cosines() {
    const double r0 = 0.9;
    const double r1 = 1.4;
    const double a0 = 0.35;
    const double a1 = 1.1;
    const double angle_delta = a0 - a1;

    const hyper::math::Vec3 p0 = sphere::point_from_polar(r0, a0);
    const hyper::math::Vec3 p1 = sphere::point_from_polar(r1, a1);

    const double expected = std::acos(std::cos(r0) * std::cos(r1) +
                                      std::sin(r0) * std::sin(r1) * std::cos(angle_delta));
    require_close(sphere::intrinsic_distance(p0, p1), expected,
                  "distance follows the spherical polar law of cosines");
    require_close(sphere::intrinsic_distance(p1, p0), expected,
                  "distance formula is order independent");
}

void test_classification_and_validation() {
    const double infinity = std::numeric_limits<double>::infinity();

    require(!sphere::is_on_sphere(hyper::math::Vec3{2.0, 0.0, 0.0}), "wrong-norm point is rejected");
    require(!sphere::is_on_sphere(hyper::math::Vec3{0.0, 0.0, 0.0}), "zero vector is not on the sphere");
    require(sphere::is_on_sphere(hyper::math::Vec3{0.0, 1.0, 0.0}), "equatorial point is on the sphere");

    require_throws<std::invalid_argument>(
        [] { (void)sphere::point_from_polar(-0.1, 0.0); },
        "negative polar radius is rejected");
    require_throws<std::invalid_argument>(
        [infinity] { (void)sphere::point_from_polar(1.0, infinity); },
        "non-finite polar angle is rejected");
}

void test_normalization() {
    const hyper::math::Vec3 raw{3.0, 0.0, 0.0};
    const hyper::math::Vec3 normalized = sphere::sphere_normalize(raw);
    require_close(normalized.t, 1.0, "normalization rescales onto the sphere");
    require(sphere::is_on_sphere(normalized), "normalized vector is on the sphere");

    const hyper::math::Vec3 tilted{2.0, 1.0, 0.5};
    const hyper::math::Vec3 tilted_normalized = sphere::sphere_normalize(tilted);
    require(sphere::is_on_sphere(tilted_normalized), "tilted vector normalizes onto the sphere");
    require_close(tilted_normalized.x / tilted_normalized.t, tilted.x / tilted.t,
                  "normalization preserves direction ratios");

    require_throws<std::domain_error>(
        [] { (void)sphere::sphere_normalize(hyper::math::Vec3{0.0, 0.0, 0.0}); },
        "zero vector cannot be normalized onto the sphere");
    require_throws<std::invalid_argument>(
        [] {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            (void)sphere::sphere_normalize(hyper::math::Vec3{nan, 0.0, 0.0});
        },
        "non-finite vector cannot be normalized");
}

void test_projections() {
    const double radius = 1.2;
    const double angle = 0.75;
    const hyper::math::Vec3 p = sphere::point_from_polar(radius, angle);

    const hyper::math::Vec2 stereographic = sphere::project_to_stereographic_disk(p);
    const hyper::math::Vec2 gnomonic = sphere::project_to_gnomonic_disk(p);

    require_close(euclidean_norm(stereographic), std::tan(radius / 2.0),
                  "stereographic radius matches tan(r/2)");
    require_close(euclidean_norm(gnomonic), std::tan(radius), "gnomonic radius matches tan(r)");
    require_close(stereographic.y / stereographic.x, std::tan(angle), "stereographic keeps angle");
    require_close(gnomonic.y / gnomonic.x, std::tan(angle), "gnomonic keeps angle");

    const hyper::math::Vec2 stereographic_origin = sphere::project_to_stereographic_disk(sphere::origin());
    require_close(euclidean_norm(stereographic_origin), 0.0, "origin projects to disk centre");

    require_throws<std::domain_error>(
        [] { (void)sphere::project_to_stereographic_disk(hyper::math::Vec3{2.0, 0.0, 0.0}); },
        "stereographic projection rejects off-sphere point");
}

void test_regular_tiling_metrics() {
    require(sphere::is_spherical_tiling(4, 3), "{4,3} cube is spherical");
    require(sphere::is_spherical_tiling(3, 3), "{3,3} tetrahedron is spherical");
    require(sphere::is_spherical_tiling(3, 4), "{3,4} octahedron is spherical");
    require(sphere::is_spherical_tiling(5, 3), "{5,3} dodecahedron is spherical");
    require(sphere::is_spherical_tiling(3, 5), "{3,5} icosahedron is spherical");
    require(!sphere::is_spherical_tiling(4, 4), "{4,4} is Euclidean, not spherical");
    require(!sphere::is_spherical_tiling(4, 6), "{4,6} is hyperbolic, not spherical");
    require(!sphere::is_spherical_tiling(2, 3), "p must be at least 3");

    const hyper::math::RegularTilingMetrics cube =
        sphere::regular_tiling_metrics(hyper::math::RegularTilingParameters{4, 3});
    require_close(cube.edge_length, 1.2309594173407747, "{4,3} edge length");
    require_close(cube.inradius, 0.7853981633974483, "{4,3} inradius is pi/4");
    require_close(cube.circumradius, 0.9553166181245093, "{4,3} circumradius");
    require(cube.edge_length > cube.inradius, "edge length exceeds inradius for {4,3}");
    require(cube.circumradius > cube.inradius, "circumradius exceeds inradius for {4,3}");

    require_throws<std::invalid_argument>(
        [] { (void)sphere::regular_tiling_metrics(hyper::math::RegularTilingParameters{4, 6}); },
        "hyperbolic tiling metrics are rejected by the spherical path");
}

void test_rotation_preserves_metric() {
    const hyper::math::Mat3 rotation = sphere::spherical_rotation(hyper::math::Vec2{0.6, 0.2});

    require(sphere::is_rotation(rotation), "rotation is a spherical isometry");

    const hyper::math::Vec3 a = sphere::point_from_polar(0.5, 0.2);
    const hyper::math::Vec3 b = sphere::point_from_polar(0.9, -0.4);
    const hyper::math::Vec3 moved_a = sphere::apply_isometry(rotation, a);
    const hyper::math::Vec3 moved_b = sphere::apply_isometry(rotation, b);

    require(sphere::is_on_sphere(moved_a), "rotation maps points to the sphere");
    require_close(sphere::euclidean_dot(moved_a, moved_b), sphere::euclidean_dot(a, b),
                  "rotation preserves the Euclidean dot");
    require_close(sphere::intrinsic_distance(moved_a, moved_b), sphere::intrinsic_distance(a, b),
                  "rotation preserves intrinsic distance");
}

void test_rotation_distance_and_direction() {
    const hyper::math::Mat3 rotation = sphere::spherical_rotation(hyper::math::Vec2{0.3, 0.4});
    const hyper::math::Vec3 moved_origin = sphere::apply_isometry(rotation, sphere::origin());

    require_close(sphere::intrinsic_distance(sphere::origin(), moved_origin), 0.5,
                  "rotation length equals tangent displacement length");
    require_close(moved_origin.x / moved_origin.y, 0.3 / 0.4, "rotation keeps the local tangent direction");

    require_matrix_close(sphere::spherical_rotation(hyper::math::Vec2{}), sphere::identity_isometry(),
                         "zero rotation is identity");
    require_throws<std::invalid_argument>(
        [] {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            (void)sphere::spherical_rotation(hyper::math::Vec2{nan, 0.0});
        },
        "non-finite rotation input is rejected");
}

void test_isometry_inverse() {
    const hyper::math::Mat3 rotation = sphere::spherical_rotation(hyper::math::Vec2{0.7, -0.2});
    const hyper::math::Mat3 inverse = sphere::inverse_isometry(rotation);
    const hyper::math::Mat3 round_trip = sphere::compose_isometries(inverse, rotation);

    require(sphere::is_rotation(inverse), "inverse is also a spherical isometry");
    require_matrix_close(round_trip, sphere::identity_isometry(), "inverse times rotation is identity");

    const hyper::math::Vec3 p = sphere::point_from_polar(0.8, 1.1);
    require_vec_close(sphere::apply_isometry(inverse, sphere::apply_isometry(rotation, p)), p,
                      "inverse returns transformed point");

    hyper::math::Mat3 invalid = sphere::identity_isometry();
    invalid.m[0][0] = 2.0;
    require_throws<std::domain_error>(
        [invalid] { (void)sphere::inverse_isometry(invalid); },
        "inverse rejects non-isometry matrices");
}

void test_frame_orthonormalization() {
    const hyper::math::CameraFrame noisy{
        hyper::math::Vec3{1.0001, 0.01, 0.0},
        hyper::math::Vec3{0.2, 1.1, 0.1},
        hyper::math::Vec3{-0.1, 0.2, 0.9},
    };

    const hyper::math::CameraFrame clean = sphere::orthonormalize_frame(noisy);
    require(sphere::is_orthonormal_frame(clean), "frame cleanup restores orthonormal invariants");
    require(sphere::is_rotation(sphere::frame_to_isometry(clean)),
            "clean frame can be used as an isometry basis");

    const hyper::math::CameraFrame degenerate{
        sphere::point_from_polar(0.4, 0.3),
        hyper::math::Vec3{0.0, 0.0, 0.0},
        hyper::math::Vec3{0.0, 0.0, 0.0},
    };
    require(sphere::is_orthonormal_frame(sphere::orthonormalize_frame(degenerate)),
            "frame cleanup can rebuild missing tangent axes");
}

void test_frame_movement() {
    const hyper::math::CameraFrame start = sphere::canonical_frame();
    const hyper::math::CameraFrame moved = sphere::move_frame(start, hyper::math::Vec2{0.6, 0.0});

    require(sphere::is_orthonormal_frame(moved), "moved frame remains orthonormal");
    require_close(sphere::intrinsic_distance(start.position, moved.position), 0.6,
                  "frame movement length matches local input");
    require_vec_close(moved.position, hyper::math::Vec3{std::cos(0.6), std::sin(0.6), 0.0},
                      "forward frame move position");
    require_vec_close(moved.forward, hyper::math::Vec3{-std::sin(0.6), std::cos(0.6), 0.0},
                      "forward frame move tangent axis");
    require_vec_close(moved.right, hyper::math::Vec3{0.0, 0.0, 1.0}, "right axis is parallel transported");

    const hyper::math::CameraFrame back = sphere::move_frame(moved, hyper::math::Vec2{-0.6, 0.0});
    require_vec_close(back.position, start.position, "moving back returns to start position");
    require(sphere::is_orthonormal_frame(back), "round-trip frame remains orthonormal");
}

} // namespace

int main() {
    return hyper::test::run("spherical_tests", [] {
        test_sphere_invariants();
        test_distance_follows_spherical_law_of_cosines();
        test_classification_and_validation();
        test_normalization();
        test_projections();
        test_regular_tiling_metrics();
        test_rotation_preserves_metric();
        test_rotation_distance_and_direction();
        test_isometry_inverse();
        test_frame_orthonormalization();
        test_frame_movement();
    });
}
