#pragma once

#include "tiling_core.h"
#include "math/hyperbolic.h"

#include <glm/glm.hpp>
#include <vector>

namespace hyper::tiling {

using MinimapPolyline = std::vector<glm::vec2>;

// Collect tile indices within hyperbolic radius of center.
void collect_minimap_candidate_tiles(const TilingPatch& patch,
                                     const math::Vec3& center,
                                     double radius,
                                     std::vector<int>& out);

// Project candidate tile centers to 2D Poincaré-disk coordinates.
// minimap_view: inverse of the camera isometry (brings world into camera-local frame).
void collect_minimap_points_from_tiles(const TilingPatch& patch,
                                       const std::vector<int>& tile_indices,
                                       const math::Mat3& minimap_view,
                                       std::vector<glm::vec2>& out,
                                       int max_points = 2000);

// Collect projected tile boundary edges for candidate tiles.
void collect_minimap_edges_from_tiles(const TilingPatch& patch,
                                      const std::vector<int>& tile_indices,
                                      const math::Mat3& minimap_view,
                                      std::vector<MinimapPolyline>& out,
                                      int max_edges = 5000,
                                      int segments = 8);

// Convenience: collect points and edges within radius in one call.
void collect_minimap(const TilingPatch& patch,
                     const math::Vec3& center,
                     double radius,
                     const math::Mat3& minimap_view,
                     std::vector<glm::vec2>& out_points,
                     std::vector<MinimapPolyline>& out_edges,
                     int max_points = 2000,
                     int max_edges = 5000,
                     int segments = 8);

} // namespace hyper::tiling
