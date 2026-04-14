#include "test_support.h"
#include "tiling_core.h"

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

} // namespace

int main() {
    return hyper::test::run("tiling_tests", [] {
        test_base_polygon_vertices();
        test_patch_depth_and_centers();
        test_neighbor_centers_are_one_tile_step_away();
        test_patch_rejects_invalid_inputs();
    });
}

