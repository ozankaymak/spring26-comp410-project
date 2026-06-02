#include "test_support.h"
#include "tiling_core.h"
#include "tiling_minimap.h"

#include <cmath>
#include <stdexcept>
#include <string>

namespace {

using hyper::test::require;
using hyper::test::require_close;
using hyper::test::require_throws;

void test_base_polygon_vertices() {
    const hyper::math::RegularTilingParameters parameters{4, 6};
    const hyper::math::RegularTilingMetrics metrics = hyper::math::regular_tiling_metrics(parameters);
    const std::vector<hyper::math::Vec3> vertices = hyper::tiling::make_base_polygon_vertices(parameters);

    require(vertices.size() == 4, "base polygon has four vertices");

    for (const hyper::math::Vec3& vertex : vertices) {
        require(hyper::math::is_on_hyperboloid(vertex), "base vertex is on the hyperboloid");
        require_close(hyper::math::intrinsic_distance(hyper::math::origin(), vertex), metrics.circumradius,
                      "base vertex lies at the circumradius");
    }

    for (std::size_t i = 0; i < vertices.size(); ++i) {
        const hyper::math::Vec3& a = vertices[i];
        const hyper::math::Vec3& b = vertices[(i + 1) % vertices.size()];
        require_close(hyper::math::intrinsic_distance(a, b), metrics.edge_length, "base edge length is regular");
    }
}

void test_patch_depth_and_centers() {
    const hyper::tiling::TilingPatch depth0 =
        hyper::tiling::generate_tiling_patch(hyper::math::RegularTilingParameters{4, 6}, 0);
    const hyper::tiling::TilingPatch depth1 =
        hyper::tiling::generate_tiling_patch(hyper::math::RegularTilingParameters{4, 6}, 1);
    const hyper::tiling::TilingPatch depth2 =
        hyper::tiling::generate_tiling_patch(hyper::math::RegularTilingParameters{4, 6}, 2);

    require(depth0.tiles.size() == 1, "depth zero contains only the root tile");
    require(depth1.tiles.size() == 5, "depth one contains root plus four neighbors");
    require(depth2.tiles.size() > depth1.tiles.size(), "depth two expands the patch");

    for (const hyper::tiling::Tile& tile : depth2.tiles) {
        require(tile.id >= 0, "tile id is assigned");
        require(tile.depth >= 0 && tile.depth <= 2, "tile depth stays within requested limit");
        require(tile.neighbors.size() == 4, "square tiling stores four side slots");
        require(hyper::math::is_on_hyperboloid(tile.center), "tile center is on the hyperboloid");
    }

    require(!hyper::tiling::has_duplicate_centers(depth2, 1.0e-7), "depth two has no duplicate centers");
}

void test_neighbor_centers_are_one_tile_step_away() {
    const hyper::math::RegularTilingParameters parameters{4, 6};
    const hyper::math::RegularTilingMetrics metrics = hyper::math::regular_tiling_metrics(parameters);
    const hyper::tiling::TilingPatch patch = hyper::tiling::generate_tiling_patch(parameters, 1);
    const hyper::tiling::Tile& root = patch.tiles[0];

    for (int neighbor_id : root.neighbors) {
        require(neighbor_id > 0, "root side has a generated neighbor");
        const hyper::tiling::Tile& neighbor = patch.tiles[static_cast<std::size_t>(neighbor_id)];
        require_close(hyper::math::intrinsic_distance(root.center, neighbor.center), 2.0 * metrics.inradius,
                      "neighbor center is across one tile side");
    }
}

void test_minimap_collects_tile_boundary_edges() {
    const hyper::tiling::TilingPatch patch =
        hyper::tiling::generate_tiling_patch(hyper::math::RegularTilingParameters{4, 6}, 0);

    std::vector<glm::vec2> points;
    std::vector<hyper::tiling::MinimapPolyline> edges;
    hyper::tiling::collect_minimap(patch,
                                   hyper::math::origin(),
                                   0.1,
                                   hyper::math::identity_isometry(),
                                   points,
                                   edges,
                                   10,
                                   10,
                                   8);

    require(points.size() == 1, "depth-zero minimap collects the root tile center");

    int edge_count = 0;
    for (const hyper::tiling::MinimapPolyline& edge : edges) {
        if (edge.empty()) {
            continue;
        }

        require(edge.size() >= 2, "minimap edge contains a drawable polyline");
        for (const glm::vec2& point : edge) {
            require(point.x * point.x + point.y * point.y < 1.0F,
                    "minimap edge point stays inside the Poincare disk");
        }
        ++edge_count;
    }

    require(edge_count == 4, "depth-zero square minimap has four tile boundary edges");
}

int count_tiles_at_root_vertex(hyper::math::RegularTilingParameters parameters, int depth) {
    const hyper::tiling::TilingPatch patch = hyper::tiling::generate_tiling_patch(parameters, depth);
    const hyper::math::Vec3 target = patch.base_polygon_vertices[0];

    int count = 0;
    for (const hyper::tiling::Tile& tile : patch.tiles) {
        bool touches_target = false;
        for (const hyper::math::Vec3& local_vertex : patch.base_polygon_vertices) {
            const hyper::math::Vec3 global_vertex = hyper::math::apply_isometry(tile.transform, local_vertex);
            if (hyper::math::intrinsic_distance(target, global_vertex) < 1.0e-5) {
                touches_target = true;
            }
        }
        if (touches_target) {
            ++count;
        }
    }

    return count;
}

void test_corner_tile_count_matches_q() {
    require(count_tiles_at_root_vertex(hyper::math::RegularTilingParameters{4, 6}, 5) == 6,
            "{4,6} has six squares around an interior corner");
    require(count_tiles_at_root_vertex(hyper::math::RegularTilingParameters{3, 7}, 6) == 7,
            "{3,7} has seven triangles around an interior corner");
}

void test_patch_rejects_invalid_inputs() {
    require_throws<std::invalid_argument>(
        [] { (void)hyper::tiling::generate_tiling_patch(hyper::math::RegularTilingParameters{4, 6}, -1); },
        "negative depth is rejected");
    require_throws<std::invalid_argument>(
        [] { (void)hyper::tiling::generate_tiling_patch(hyper::math::RegularTilingParameters{4, 4}, 1); },
        "non-hyperbolic parameters are rejected");
    require_throws<std::invalid_argument>(
        [] {
            (void)hyper::tiling::generate_tiling_patch(hyper::math::RegularTilingParameters{4, 6}, 1, 0.0);
        },
        "non-positive duplicate tolerance is rejected");
}

void test_find_current_tile_and_rebasing() {
    const hyper::math::RegularTilingParameters parameters{4, 6};
    const hyper::math::RegularTilingMetrics metrics = hyper::math::regular_tiling_metrics(parameters);
    const hyper::tiling::TilingPatch patch = hyper::tiling::generate_tiling_patch(parameters, 1);
    const hyper::math::CameraFrame canonical = hyper::math::canonical_frame();

    // Should find root tile for origin
    const int root_tile = hyper::tiling::find_current_tile(patch, canonical.position);
    require(root_tile == 0, "origin is in root tile");

    // Move through side 0, whose outward normal sits halfway between the first
    // two base-polygon vertices.
    const double side_crossing_distance = metrics.inradius + 0.2;
    const double diagonal = side_crossing_distance / std::sqrt(2.0);
    const hyper::math::CameraFrame moved = hyper::math::move_frame(canonical, hyper::math::Vec2{diagonal, diagonal});
    require(hyper::tiling::locate_crossed_side(patch, moved.position) == 0,
            "moved position crossed root side zero");

    const int neighbor_tile = hyper::tiling::find_current_tile(patch, moved.position);
    require(neighbor_tile != 0, "moved position is in a different tile");

    int current_tile = 0;
    hyper::math::CameraFrame rebased = moved;
    require(hyper::tiling::rebase_frame_across_edges(patch, current_tile, rebased),
            "crossed frame rebases through linked neighbor");
    require(current_tile == neighbor_tile, "rebase updates current tile");
    require(hyper::tiling::locate_crossed_side(patch, rebased.position) < 0,
            "rebased frame is local to the destination tile");

    const hyper::math::CameraFrame rebased_global =
        hyper::tiling::global_frame_from_tile(rebased, patch.tiles[static_cast<std::size_t>(current_tile)]);
    require_close(hyper::math::intrinsic_distance(moved.position, rebased_global.position), 0.0,
                  "rebasing preserves global position", 1.0e-7);

    const hyper::tiling::TilingPatch boundary_patch = hyper::tiling::generate_tiling_patch(parameters, 0);
    int boundary_tile = 0;
    hyper::math::CameraFrame boundary_frame = moved;
    require(!hyper::tiling::rebase_frame_across_edges(boundary_patch, boundary_tile, boundary_frame),
            "crossing beyond the generated patch is rejected");
    require(boundary_tile == 0, "failed boundary rebase preserves current tile");
}

} // namespace

int main() {
    return hyper::test::run("tiling_tests", [] {
        test_base_polygon_vertices();
        test_patch_depth_and_centers();
        test_neighbor_centers_are_one_tile_step_away();
        test_minimap_collects_tile_boundary_edges();
        test_corner_tile_count_matches_q();
        test_patch_rejects_invalid_inputs();
        test_find_current_tile_and_rebasing();
    });
}
