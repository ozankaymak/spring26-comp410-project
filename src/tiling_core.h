#pragma once

#include "math/hyperbolic.h"

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
    math::RegularTilingParameters parameters{};
    math::RegularTilingMetrics metrics{};
    std::vector<math::Vec3> base_polygon_vertices;
    std::vector<Tile> tiles;
};

std::vector<math::Vec3> make_base_polygon_vertices(math::RegularTilingParameters parameters);
TilingPatch generate_tiling_patch(math::RegularTilingParameters parameters, int max_depth,
                                  double duplicate_tolerance = 1.0e-6);
bool has_duplicate_centers(const TilingPatch& patch, double tolerance);

} // namespace hyper::tiling

