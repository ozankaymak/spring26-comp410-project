#include "math/hyperbolic.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

constexpr double kTolerance = 1.0e-9;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_close(double actual, double expected, const std::string& message) {
    if (std::abs(actual - expected) > kTolerance) {
        throw std::runtime_error(message + ": expected " + std::to_string(expected) +
                                 ", got " + std::to_string(actual));
    }
}

template <typename Exception, typename Fn>
void require_throws(Fn&& fn, const std::string& message) {
    bool threw = false;
    try {
        fn();
    } catch (const Exception&) {
        threw = true;
    }

    require(threw, message);
}

double euclidean_norm(const hyper::math::Vec2& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

void test_hyperboloid_invariants() {
    require_close(hyper::math::kCurvature, -1.0, "curvature is fixed to -1");

    const hyper::math::Vec3 o = hyper::math::origin();
    require_close(hyper::math::minkowski_dot(o, o), -1.0, "origin has unit timelike norm");
    require(hyper::math::is_on_hyperboloid(o), "origin is on the future hyperboloid sheet");
    require_close(hyper::math::intrinsic_distance(o, o), 0.0, "origin distance to itself is zero");

    const hyper::math::Vec3 p = hyper::math::point_from_polar(1.25, 0.4);
    require(hyper::math::is_on_hyperboloid(p), "polar point lies on the future sheet");
    require_close(hyper::math::intrinsic_distance(o, p), 1.25, "origin-to-polar distance equals radius");
    require_close(hyper::math::intrinsic_distance(p, o), 1.25, "intrinsic distance is symmetric");

    const hyper::math::Vec3 a{2.0, 1.0, 0.5};
    const hyper::math::Vec3 b{1.5, -0.25, 0.75};
    require_close(hyper::math::minkowski_dot(a, b), hyper::math::minkowski_dot(b, a),
                  "Minkowski dot is symmetric");
}

void test_distance_formula_between_polar_points() {
    const double r0 = 0.9;
    const double r1 = 1.4;
    const double a0 = 0.35;
    const double a1 = 1.1;
    const double angle_delta = a0 - a1;

    const hyper::math::Vec3 p0 = hyper::math::point_from_polar(r0, a0);
    const hyper::math::Vec3 p1 = hyper::math::point_from_polar(r1, a1);

    const double expected = std::acosh(std::cosh(r0) * std::cosh(r1) -
                                      std::sinh(r0) * std::sinh(r1) * std::cos(angle_delta));
    require_close(hyper::math::intrinsic_distance(p0, p1), expected,
                  "distance follows the hyperbolic polar law of cosines");
    require_close(hyper::math::intrinsic_distance(p1, p0), expected,
                  "distance formula is order independent");
}

void test_classification_and_validation() {
    const double infinity = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    require(hyper::math::is_finite(hyper::math::Vec2{1.0, -2.0}), "finite Vec2 is accepted");
    require(!hyper::math::is_finite(hyper::math::Vec2{infinity, 0.0}), "infinite Vec2 is rejected");
    require(hyper::math::is_finite(hyper::math::Vec3{1.0, 2.0, 3.0}), "finite Vec3 is accepted");
    require(!hyper::math::is_finite(hyper::math::Vec3{1.0, nan, 3.0}), "NaN Vec3 is rejected");

    require(hyper::math::is_timelike(hyper::math::Vec3{2.0, 1.0, 0.0}), "negative norm vector is timelike");
    require(!hyper::math::is_timelike(hyper::math::Vec3{1.0, 1.0, 0.0}), "lightlike vector is not timelike");
    require(!hyper::math::is_timelike(hyper::math::Vec3{1.0, 2.0, 0.0}), "spacelike vector is not timelike");

    require(!hyper::math::is_on_hyperboloid(hyper::math::Vec3{-1.0, 0.0, 0.0}),
            "past sheet point is rejected");
    require(!hyper::math::is_on_hyperboloid(hyper::math::Vec3{2.0, 0.0, 0.0}),
            "incorrect norm is rejected");

    require_throws<std::invalid_argument>(
        [] { (void)hyper::math::point_from_polar(-0.1, 0.0); },
        "negative polar radius is rejected");
    require_throws<std::invalid_argument>(
        [infinity] { (void)hyper::math::point_from_polar(1.0, infinity); },
        "non-finite polar angle is rejected");
}

void test_normalization() {
    const hyper::math::Vec3 raw{3.0, 0.0, 0.0};
    const hyper::math::Vec3 normalized = hyper::math::hyperboloid_normalize(raw);
    require_close(normalized.t, 1.0, "normalization rescales time coordinate");
    require_close(normalized.x, 0.0, "normalization preserves x direction");
    require_close(normalized.y, 0.0, "normalization preserves y direction");
    require(hyper::math::is_on_hyperboloid(normalized), "normalized vector is on hyperboloid");

    const hyper::math::Vec3 past{-2.0, 0.0, 0.0};
    require(hyper::math::hyperboloid_normalize(past).t > 0.0, "normalization selects future sheet");

    const hyper::math::Vec3 tilted{2.0, 1.0, 0.0};
    const hyper::math::Vec3 tilted_normalized = hyper::math::hyperboloid_normalize(tilted);
    require(hyper::math::is_on_hyperboloid(tilted_normalized), "tilted timelike vector normalizes");
    require_close(tilted_normalized.x / tilted_normalized.t, tilted.x / tilted.t,
                  "normalization preserves direction ratios");

    require_throws<std::domain_error>(
        [] { (void)hyper::math::hyperboloid_normalize(hyper::math::Vec3{0.0, 1.0, 0.0}); },
        "spacelike vector cannot be normalized onto the hyperboloid");
    require_throws<std::domain_error>(
        [] { (void)hyper::math::hyperboloid_normalize(hyper::math::Vec3{1.0, 1.0, 0.0}); },
        "lightlike vector cannot be normalized onto the hyperboloid");
    require_throws<std::invalid_argument>(
        [] {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            (void)hyper::math::hyperboloid_normalize(hyper::math::Vec3{nan, 0.0, 0.0});
        },
        "non-finite vector cannot be normalized");
}

void test_projections() {
    const double radius = 1.2;
    const double angle = 0.75;
    const hyper::math::Vec3 p = hyper::math::point_from_polar(radius, angle);

    const hyper::math::Vec2 poincare = hyper::math::project_to_poincare_disk(p);
    const hyper::math::Vec2 klein = hyper::math::project_to_klein_disk(p);

    require_close(euclidean_norm(poincare), std::tanh(radius / 2.0), "Poincare radius matches tanh(r/2)");
    require_close(euclidean_norm(klein), std::tanh(radius), "Klein radius matches tanh(r)");
    require_close(poincare.y / poincare.x, std::tan(angle), "Poincare projection keeps angle");
    require_close(klein.y / klein.x, std::tan(angle), "Klein projection keeps angle");
    require(euclidean_norm(poincare) < 1.0, "Poincare projection stays inside unit disk");
    require(euclidean_norm(klein) < 1.0, "Klein projection stays inside unit disk");

    const hyper::math::Vec2 poincare_origin = hyper::math::project_to_poincare_disk(hyper::math::origin());
    const hyper::math::Vec2 klein_origin = hyper::math::project_to_klein_disk(hyper::math::origin());
    require_close(euclidean_norm(poincare_origin), 0.0, "origin projects to Poincare disk center");
    require_close(euclidean_norm(klein_origin), 0.0, "origin projects to Klein disk center");

    require_throws<std::domain_error>(
        [] { (void)hyper::math::project_to_poincare_disk(hyper::math::Vec3{2.0, 0.0, 0.0}); },
        "Poincare projection rejects invalid hyperboloid point");
    require_throws<std::domain_error>(
        [] { (void)hyper::math::project_to_klein_disk(hyper::math::Vec3{2.0, 0.0, 0.0}); },
        "Klein projection rejects invalid hyperboloid point");
}

void test_regular_tiling_metrics() {
    const hyper::math::RegularTilingMetrics metrics =
        hyper::math::regular_tiling_metrics(hyper::math::RegularTilingParameters{4, 6});

    require(hyper::math::is_hyperbolic_tiling(4, 6), "{4,6} is hyperbolic");
    require(hyper::math::is_hyperbolic_tiling(3, 7), "{3,7} is hyperbolic");
    require(!hyper::math::is_hyperbolic_tiling(4, 4), "{4,4} is Euclidean, not hyperbolic");
    require(!hyper::math::is_hyperbolic_tiling(2, 7), "p must be at least 3");
    require(!hyper::math::is_hyperbolic_tiling(7, 2), "q must be at least 3");
    require_close(metrics.edge_length, 1.762747174039086, "{4,6} edge length");
    require_close(metrics.inradius, 0.6584789484624083, "{4,6} inradius");
    require_close(metrics.circumradius, 1.1462158347805889, "{4,6} circumradius");
    require(metrics.edge_length > metrics.inradius, "edge length is larger than inradius for {4,6}");
    require(metrics.circumradius > metrics.inradius, "circumradius is larger than inradius for {4,6}");

    require_throws<std::invalid_argument>(
        [] { (void)hyper::math::regular_tiling_metrics(hyper::math::RegularTilingParameters{4, 4}); },
        "Euclidean tiling metrics are rejected");
    require_throws<std::invalid_argument>(
        [] { (void)hyper::math::regular_tiling_metrics(hyper::math::RegularTilingParameters{2, 7}); },
        "invalid p value is rejected");
    require_throws<std::invalid_argument>(
        [] { (void)hyper::math::regular_tiling_metrics(hyper::math::RegularTilingParameters{7, 2}); },
        "invalid q value is rejected");
}

} // namespace

int main() {
    try {
        test_hyperboloid_invariants();
        test_distance_formula_between_polar_points();
        test_classification_and_validation();
        test_normalization();
        test_projections();
        test_regular_tiling_metrics();
    } catch (const std::exception& error) {
        std::cerr << "math_tests failed: " << error.what() << '\n';
        return 1;
    }

    return 0;
}
