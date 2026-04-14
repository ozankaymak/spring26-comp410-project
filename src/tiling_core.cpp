#include "tiling_core.h"

#include <cmath>
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

double metric_sign(int index) {
    return index == 0 ? -1.0 : 1.0;
}

math::Mat3 reflection_across_side(double inradius, double angle) {
    const math::Vec3 normal{
        std::sinh(inradius),
        std::cosh(inradius) * std::cos(angle),
        std::cosh(inradius) * std::sin(angle),
    };

    math::Mat3 reflection{};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            const double identity = row == col ? 1.0 : 0.0;
            reflection.m[row][col] =
                identity - 2.0 * coordinate_at(normal, row) * metric_sign(col) * coordinate_at(normal, col);
        }
    }

    return reflection;
}

std::vector<math::Mat3> side_reflections(math::RegularTilingParameters parameters,
                                         const math::RegularTilingMetrics& metrics) {
    std::vector<math::Mat3> reflections;
    reflections.reserve(static_cast<std::size_t>(parameters.p));

    for (int side = 0; side < parameters.p; ++side) {
        const double angle = (static_cast<double>(side) + 0.5) * 2.0 * kPi / static_cast<double>(parameters.p);
        reflections.push_back(reflection_across_side(metrics.inradius, angle));
    }

    return reflections;
}

math::Vec3 tile_center_from_transform(const math::Mat3& transform) {
    return math::hyperboloid_normalize(math::apply_isometry(transform, math::origin()));
}

int find_existing_tile(const std::vector<Tile>& tiles, const math::Vec3& center, double tolerance) {
    for (const Tile& tile : tiles) {
        if (math::intrinsic_distance(tile.center, center) <= tolerance) {
            return tile.id;
        }
    }

    return -1;
}

} // namespace

std::vector<math::Vec3> make_base_polygon_vertices(math::RegularTilingParameters parameters) {
    const math::RegularTilingMetrics metrics = math::regular_tiling_metrics(parameters);

    std::vector<math::Vec3> vertices;
    vertices.reserve(static_cast<std::size_t>(parameters.p));

    for (int i = 0; i < parameters.p; ++i) {
        const double angle = static_cast<double>(i) * 2.0 * kPi / static_cast<double>(parameters.p);
        vertices.push_back(math::point_from_polar(metrics.circumradius, angle));
    }

    return vertices;
}

TilingPatch generate_tiling_patch(math::RegularTilingParameters parameters, int max_depth,
                                  double duplicate_tolerance) {
    if (max_depth < 0) {
        throw std::invalid_argument("tiling depth must be non-negative");
    }
    if (duplicate_tolerance <= 0.0) {
        throw std::invalid_argument("duplicate tolerance must be positive");
    }

    TilingPatch patch;
    patch.parameters = parameters;
    patch.metrics = math::regular_tiling_metrics(parameters);
    patch.base_polygon_vertices = make_base_polygon_vertices(parameters);

    const std::vector<math::Mat3> reflections = side_reflections(parameters, patch.metrics);

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
            const math::Vec3 neighbor_center = tile_center_from_transform(neighbor_transform);
            int neighbor_id = find_existing_tile(patch.tiles, neighbor_center, duplicate_tolerance);

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
            if (patch.tiles[static_cast<std::size_t>(neighbor_id)].neighbors[static_cast<std::size_t>(side)] < 0) {
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
            if (math::intrinsic_distance(patch.tiles[i].center, patch.tiles[j].center) <= tolerance) {
                return true;
            }
        }
    }

    return false;
}

int find_current_tile(const TilingPatch& patch, const math::Vec3& position) {
    int closest_tile = -1;
    double min_distance = std::numeric_limits<double>::max();

    for (const Tile& tile : patch.tiles) {
        const double dist = math::intrinsic_distance(tile.center, position);
        if (dist < min_distance) {
            min_distance = dist;
            closest_tile = tile.id;
        }
    }

    return closest_tile;
}

math::CameraFrame rebase_frame_to_tile(const math::CameraFrame& frame, const Tile& tile) {
    // Apply the inverse of the tile's transform to the frame
    const math::Mat3 inverse_transform = math::inverse_isometry(tile.transform);
    return math::apply_isometry(inverse_transform, frame);
}

} // namespace hyper::tiling

