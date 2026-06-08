#include "tiling_core.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace hyper::tiling {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double coordinate_at(const math::Vec3& v, int index) {
    if (index == 0) {
        return v.t;
    }
    if (index == 1) {
        return v.x;
    }
    return v.y;
}

double metric_sign(int index, math::GeometryMode mode) {
    if (mode == math::GeometryMode::Spherical) {
        return 1.0;
    }
    return index == 0 ? -1.0 : 1.0;
}

// --- Geometry-mode dispatch -------------------------------------------------
// The tiling logic is shared; only the underlying manifold operations differ.
// Isometry composition and matrix/vector application are metric-agnostic and
// reuse the hyperbolic implementations for both modes.

bool is_spherical(math::GeometryMode mode) {
    return mode == math::GeometryMode::Spherical;
}

math::Vec3 geo_normalize(const math::Vec3& v, math::GeometryMode mode) {
    return is_spherical(mode) ? math::sphere::sphere_normalize(v) : math::hyperboloid_normalize(v);
}

double geo_distance(const math::Vec3& a, const math::Vec3& b, math::GeometryMode mode) {
    return is_spherical(mode) ? math::sphere::intrinsic_distance(a, b) : math::intrinsic_distance(a, b);
}

// Proximity test used to merge coincident tile centers during generation.
// The spherical path must NOT use intrinsic_distance here: acos(dot) loses all
// precision near zero (acos(1 - eps) ~ sqrt(2 eps)), so genuinely identical
// centers read as ~1.5e-8 apart and never merge, which leaves the antipodal
// tile duplicated or, with a looser angular tolerance, corrupts the BFS
// topology and drops a face. The chordal (Euclidean) distance stays accurate
// down to machine precision, and since distinct face centers are far apart the
// same tolerance separates true duplicates cleanly.
double geo_center_gap(const math::Vec3& a, const math::Vec3& b, math::GeometryMode mode) {
    if (is_spherical(mode)) {
        const double dt = a.t - b.t;
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        return std::sqrt(dt * dt + dx * dx + dy * dy);
    }
    return math::intrinsic_distance(a, b);
}

math::Vec3 geo_point_from_polar(double radius, double angle, math::GeometryMode mode) {
    return is_spherical(mode) ? math::sphere::point_from_polar(radius, angle)
                              : math::point_from_polar(radius, angle);
}

math::RegularTilingMetrics geo_metrics(math::RegularTilingParameters parameters, math::GeometryMode mode) {
    return is_spherical(mode) ? math::sphere::regular_tiling_metrics(parameters)
                              : math::regular_tiling_metrics(parameters);
}

math::Mat3 geo_inverse(const math::Mat3& transform, math::GeometryMode mode) {
    return is_spherical(mode) ? math::sphere::inverse_isometry(transform)
                              : math::inverse_isometry(transform);
}

// Crossing detection projects to the model where geodesic polygon edges become
// straight segments: Klein for hyperbolic, gnomonic for spherical.
math::Vec2 geo_project_straight_edges(const math::Vec3& p, math::GeometryMode mode) {
    return is_spherical(mode) ? math::sphere::project_to_gnomonic_disk(p)
                              : math::project_to_klein_disk(p);
}

// Applying an isometry to a camera frame re-orthonormalizes under the active
// metric, so this one must dispatch (the others are pure matrix arithmetic).
math::CameraFrame geo_apply_frame(const math::Mat3& transform, const math::CameraFrame& frame,
                                  math::GeometryMode mode) {
    return is_spherical(mode) ? math::sphere::apply_isometry(transform, frame)
                              : math::apply_isometry(transform, frame);
}

math::Mat3 reflection_across_side(double inradius, double angle, math::GeometryMode mode) {
    // The mirror is the polygon side, a geodesic at distance `inradius` from the
    // tile centre. Its pole is the unit normal of the reflecting hyperplane.
    const math::Vec3 normal =
        is_spherical(mode)
            ? math::Vec3{std::sin(inradius), -std::cos(inradius) * std::cos(angle),
                         -std::cos(inradius) * std::sin(angle)}
            : math::Vec3{std::sinh(inradius), std::cosh(inradius) * std::cos(angle),
                         std::cosh(inradius) * std::sin(angle)};

    math::Mat3 reflection{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const double identity = row == col ? 1.0 : 0.0;
            reflection.m[row][col] = identity - 2.0 * coordinate_at(normal, row) *
                                                    metric_sign(col, mode) * coordinate_at(normal, col);
        }
    }

    return reflection;
}

std::vector<math::Mat3> side_reflections(math::RegularTilingParameters parameters,
                                         const math::RegularTilingMetrics& metrics,
                                         math::GeometryMode mode) {
    std::vector<math::Mat3> reflections;
    reflections.reserve(static_cast<std::size_t>(parameters.p));

    for (int side = 0; side < parameters.p; ++side) {
        const double angle = (static_cast<double>(side) + 0.5) * 2.0 * kPi / static_cast<double>(parameters.p);
        reflections.push_back(reflection_across_side(metrics.inradius, angle, mode));
    }

    return reflections;
}

math::Vec3 tile_center_from_transform(const math::Mat3& transform, math::GeometryMode mode) {
    return geo_normalize(math::apply_isometry(transform, math::origin()), mode);
}

double cross2d(const math::Vec2& a, const math::Vec2& b) {
    return a.x * b.y - a.y * b.x;
}

int find_existing_tile(const std::vector<Tile>& tiles, const math::Vec3& center, double tolerance,
                       math::GeometryMode mode) {
    for (const Tile& tile : tiles) {
        if (geo_center_gap(tile.center, center, mode) <= tolerance) {
            return tile.id;
        }
    }

    return -1;
}

} // namespace

std::vector<math::Vec3> make_base_polygon_vertices(math::RegularTilingParameters parameters,
                                                   math::GeometryMode mode) {
    const math::RegularTilingMetrics metrics = geo_metrics(parameters, mode);

    std::vector<math::Vec3> vertices;
    vertices.reserve(static_cast<std::size_t>(parameters.p));

    for (int i = 0; i < parameters.p; ++i) {
        const double angle = static_cast<double>(i) * 2.0 * kPi / static_cast<double>(parameters.p);
        vertices.push_back(geo_point_from_polar(metrics.circumradius, angle, mode));
    }

    return vertices;
}

TilingPatch generate_tiling_patch(math::RegularTilingParameters parameters, int max_depth,
                                  double duplicate_tolerance, math::GeometryMode mode) {
    if (max_depth < 0) {
        throw std::invalid_argument("tiling depth must be non-negative");
    }
    if (duplicate_tolerance <= 0.0) {
        throw std::invalid_argument("duplicate tolerance must be positive");
    }

    TilingPatch patch;
    patch.mode = mode;
    patch.parameters = parameters;
    patch.metrics = geo_metrics(parameters, mode);
    patch.base_polygon_vertices = make_base_polygon_vertices(parameters, mode);

    const std::vector<math::Mat3> reflections = side_reflections(parameters, patch.metrics, mode);

    Tile root;
    root.id = 0;
    root.neighbors.assign(static_cast<std::size_t>(parameters.p), -1);
    patch.tiles.push_back(root);

    for (std::size_t index = 0; index < patch.tiles.size(); ++index) {
        if (patch.tiles[index].depth >= max_depth) {
            continue;
        }

        for (int side = 0; side < parameters.p; ++side) {
            if (patch.tiles[index].neighbors[static_cast<std::size_t>(side)] >= 0) {
                continue;
            }

            const math::Mat3 neighbor_transform =
                math::compose_isometries(patch.tiles[index].transform, reflections[static_cast<std::size_t>(side)]);
            const math::Vec3 neighbor_center = tile_center_from_transform(neighbor_transform, mode);
            int neighbor_id = find_existing_tile(patch.tiles, neighbor_center, duplicate_tolerance, mode);

            if (neighbor_id < 0) {
                neighbor_id = static_cast<int>(patch.tiles.size());

                Tile neighbor;
                neighbor.id = neighbor_id;
                neighbor.depth = patch.tiles[index].depth + 1;
                neighbor.parent = patch.tiles[index].id;
                neighbor.parent_side = side;
                neighbor.transform = neighbor_transform;
                neighbor.center = neighbor_center;
                neighbor.neighbors.assign(static_cast<std::size_t>(parameters.p), -1);
                neighbor.neighbors[static_cast<std::size_t>(side)] = patch.tiles[index].id;

                patch.tiles.push_back(neighbor);
            }

            patch.tiles[index].neighbors[static_cast<std::size_t>(side)] = neighbor_id;

            // Reciprocal back-link. This is only valid when the neighbour's
            // shared side carries the same index as ours, which holds when the
            // neighbour was just created from this tile (its frame is exactly
            // our transform composed with reflections[side]). When the neighbour
            // already existed it was reached via a different group element that
            // differs by the face's rotational stabiliser, so its local side
            // indexing is rotated and writing index `side` would corrupt the
            // table. On a closed spherical patch that corruption clobbers the
            // antipodal tile's generator and leaves a hole, so there we skip the
            // assumption entirely and let every tile fill its own forward links
            // (each tile is processed, so the reciprocal link is still created
            // from the neighbour's own correct frame). The hyperbolic path keeps
            // its original behaviour.
            if (!is_spherical(mode) &&
                patch.tiles[static_cast<std::size_t>(neighbor_id)].neighbors[static_cast<std::size_t>(side)] < 0) {
                patch.tiles[static_cast<std::size_t>(neighbor_id)].neighbors[static_cast<std::size_t>(side)] =
                    patch.tiles[index].id;
            }
        }
    }

    return patch;
}

bool has_duplicate_centers(const TilingPatch& patch, double tolerance) {
    if (tolerance <= 0.0) {
        throw std::invalid_argument("duplicate tolerance must be positive");
    }

    for (std::size_t i = 0; i < patch.tiles.size(); ++i) {
        for (std::size_t j = i + 1; j < patch.tiles.size(); ++j) {
            if (geo_center_gap(patch.tiles[i].center, patch.tiles[j].center, patch.mode) <= tolerance) {
                return true;
            }
        }
    }

    return false;
}

int locate_crossed_side(const TilingPatch& patch, const math::Vec3& tile_local_position,
                        double tolerance) {
    if (tolerance < 0.0) {
        throw std::invalid_argument("crossing tolerance must be non-negative");
    }
    if (patch.base_polygon_vertices.size() < 3) {
        return -1;
    }

    const math::Vec2 point = geo_project_straight_edges(tile_local_position, patch.mode);
    double worst_margin = -tolerance;
    int crossed_side = -1;

    for (std::size_t side = 0; side < patch.base_polygon_vertices.size(); ++side) {
        const math::Vec2 a = geo_project_straight_edges(patch.base_polygon_vertices[side], patch.mode);
        const math::Vec2 b = geo_project_straight_edges(
            patch.base_polygon_vertices[(side + 1U) % patch.base_polygon_vertices.size()], patch.mode);
        const math::Vec2 edge{b.x - a.x, b.y - a.y};
        const math::Vec2 relative{point.x - a.x, point.y - a.y};

        // Base vertices are generated counter-clockwise, so points inside the
        // convex Klein-projected polygon stay on the left side of every edge.
        const double margin = cross2d(edge, relative);
        if (margin < worst_margin) {
            worst_margin = margin;
            crossed_side = static_cast<int>(side);
        }
    }

    return crossed_side;
}

int find_current_tile(const TilingPatch& patch, const math::Vec3& position) {
    for (const Tile& tile : patch.tiles) {
        const math::Mat3 inverse_transform = geo_inverse(tile.transform, patch.mode);
        const math::Vec3 local_position = math::apply_isometry(inverse_transform, position);
        if (locate_crossed_side(patch, local_position) < 0) {
            return tile.id;
        }
    }

    int closest_tile = -1;
    double min_distance = std::numeric_limits<double>::max();

    for (const Tile& tile : patch.tiles) {
        const double dist = geo_distance(tile.center, position, patch.mode);
        if (dist < min_distance) {
            min_distance = dist;
            closest_tile = tile.id;
        }
    }

    return closest_tile;
}

math::CameraFrame global_frame_from_tile(const math::CameraFrame& tile_local_frame, const Tile& tile,
                                         math::GeometryMode mode) {
    return geo_apply_frame(tile.transform, tile_local_frame, mode);
}

math::CameraFrame rebase_frame_to_tile(const math::CameraFrame& frame, const Tile& tile,
                                       math::GeometryMode mode) {
    // Apply the inverse of the tile's transform to the frame
    const math::Mat3 inverse_transform = geo_inverse(tile.transform, mode);
    return geo_apply_frame(inverse_transform, frame, mode);
}

bool rebase_frame_across_edges(const TilingPatch& patch, int& current_tile_id,
                               math::CameraFrame& tile_local_frame, int max_crossings) {
    if (max_crossings < 0) {
        throw std::invalid_argument("max crossings must be non-negative");
    }
    if (current_tile_id < 0 || current_tile_id >= static_cast<int>(patch.tiles.size())) {
        return false;
    }

    int tile_id = current_tile_id;
    math::CameraFrame frame = tile_local_frame;

    for (int crossing = 0; crossing < max_crossings; ++crossing) {
        const int crossed_side = locate_crossed_side(patch, frame.position);
        if (crossed_side < 0) {
            current_tile_id = tile_id;
            tile_local_frame = frame;
            return true;
        }

        const Tile& tile = patch.tiles[static_cast<std::size_t>(tile_id)];
        if (crossed_side >= static_cast<int>(tile.neighbors.size())) {
            return false;
        }

        const int next_tile_id = tile.neighbors[static_cast<std::size_t>(crossed_side)];
        if (next_tile_id < 0 || next_tile_id >= static_cast<int>(patch.tiles.size())) {
            return false;
        }

        const math::CameraFrame global_frame = global_frame_from_tile(frame, tile, patch.mode);
        frame = rebase_frame_to_tile(global_frame, patch.tiles[static_cast<std::size_t>(next_tile_id)], patch.mode);
        tile_id = next_tile_id;
    }

    if (locate_crossed_side(patch, frame.position) < 0) {
        current_tile_id = tile_id;
        tile_local_frame = frame;
        return true;
    }

    return false;
}

} // namespace hyper::tiling
