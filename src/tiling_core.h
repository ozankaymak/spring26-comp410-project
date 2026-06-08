#pragma once

#include "math/geometry_mode.h"
#include "math/hyperbolic.h"
#include "math/spherical.h"

#include <vector>

namespace hyper::tiling {

struct Tile {
    int id = 0;
    int depth = 0;
    int parent = -1;
    int parent_side = -1;
    math::Mat3 transform{};
    math::Vec3 center{1.0, 0.0, 0.0};
    std::vector<int> neighbors;
};

struct TilingPatch {
    math::GeometryMode mode = math::GeometryMode::Hyperbolic;
    math::RegularTilingParameters parameters{};
    math::RegularTilingMetrics metrics{};
    std::vector<math::Vec3> base_polygon_vertices;
    std::vector<Tile> tiles;
};

std::vector<math::Vec3> make_base_polygon_vertices(
    math::RegularTilingParameters parameters,
    math::GeometryMode mode = math::GeometryMode::Hyperbolic);
TilingPatch generate_tiling_patch(math::RegularTilingParameters parameters, int max_depth,
                                  double duplicate_tolerance = 1.0e-6,
                                  math::GeometryMode mode = math::GeometryMode::Hyperbolic);
bool has_duplicate_centers(const TilingPatch& patch, double tolerance);

// Find the tile that contains the given position in the tiling patch.
int find_current_tile(const TilingPatch& patch, const math::Vec3& position);

// Returns the crossed base-polygon side for a tile-local position, or -1 when
// the position is still inside the current tile.
int locate_crossed_side(const TilingPatch& patch, const math::Vec3& tile_local_position,
                        double tolerance = 1.0e-10);

// Converts a tile-local camera frame into global patch coordinates.
math::CameraFrame global_frame_from_tile(const math::CameraFrame& tile_local_frame, const Tile& tile,
                                         math::GeometryMode mode = math::GeometryMode::Hyperbolic);

// Rebase a global camera frame to be relative to the given tile.
math::CameraFrame rebase_frame_to_tile(const math::CameraFrame& frame, const Tile& tile,
                                       math::GeometryMode mode = math::GeometryMode::Hyperbolic);

// Rebase a tile-local camera frame across any crossed tile edges.
bool rebase_frame_across_edges(const TilingPatch& patch, int& current_tile_id,
                               math::CameraFrame& tile_local_frame, int max_crossings = 8);

} // namespace hyper::tiling
