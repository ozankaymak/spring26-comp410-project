#pragma once

namespace hyper::math {

// Selects which curved-space model the runtime is operating in. The hyperbolic
// path uses the hyperboloid model (negative curvature); the spherical path uses
// the 2-sphere model (positive curvature). The two share storage types but use
// different metrics, isometries, and projections.
enum class GeometryMode {
    Hyperbolic,
    Spherical,
};

constexpr const char* geometry_mode_name(GeometryMode mode) {
    return mode == GeometryMode::Spherical ? "Spherical" : "Hyperbolic";
}

} // namespace hyper::math
