#include "tiling_minimap.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <unordered_set>

namespace hyper::tiling {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Apply the minimap view isometry then project to the Poincaré disk.
glm::dvec2 project_to_minimap(const math::Vec3& world_pos, const math::Mat3& minimap_view) {
    const math::Vec3 local = math::apply_isometry(minimap_view, world_pos);
    const math::Vec2 p = math::project_to_poincare_disk(local);
    return {p.x, p.y};
}

std::uint64_t vertex_key(const math::Vec3& world_pos) {
    constexpr double kQuantizeScale = 1.0e8;
    const math::Vec2 p = math::project_to_poincare_disk(world_pos);
    const auto qx = static_cast<std::int32_t>(std::llround(p.x * kQuantizeScale));
    const auto qy = static_cast<std::int32_t>(std::llround(p.y * kQuantizeScale));
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(qx)) << 32U) |
           static_cast<std::uint32_t>(qy);
}

struct EdgeKey {
    std::uint64_t a = 0;
    std::uint64_t b = 0;

    bool operator==(const EdgeKey& other) const {
        return a == other.a && b == other.b;
    }
};

struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey& key) const {
        return static_cast<std::size_t>(key.a ^ (key.b + 0x9e3779b97f4a7c15ULL + (key.a << 6U) + (key.a >> 2U)));
    }
};

EdgeKey edge_key(const math::Vec3& a, const math::Vec3& b) {
    const std::uint64_t key_a = vertex_key(a);
    const std::uint64_t key_b = vertex_key(b);
    return key_a < key_b ? EdgeKey{key_a, key_b} : EdgeKey{key_b, key_a};
}

struct MinimapEdge {
    math::Vec3 a{};
    math::Vec3 b{};
};

} // namespace

void collect_minimap_candidate_tiles(const TilingPatch& patch,
                                     const math::Vec3& center,
                                     double radius,
                                     std::vector<int>& out) {
    out.clear();
    for (int i = 0; i < static_cast<int>(patch.tiles.size()); ++i) {
        if (math::intrinsic_distance(center, patch.tiles[static_cast<std::size_t>(i)].center) <= radius) {
            out.push_back(i);
        }
    }
}

void collect_minimap_points_from_tiles(const TilingPatch& patch,
                                       const std::vector<int>& tile_indices,
                                       const math::Mat3& minimap_view,
                                       std::vector<glm::vec2>& out,
                                       int max_points) {
    out.clear();
    if (max_points <= 0) return;
    const int count = std::min(max_points, static_cast<int>(tile_indices.size()));
    out.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const glm::dvec2 p = project_to_minimap(
            patch.tiles[static_cast<std::size_t>(tile_indices[static_cast<std::size_t>(i)])].center,
            minimap_view);
        out.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
    }
}

void collect_minimap_edges_from_tiles(const TilingPatch& patch,
                                      const std::vector<int>& tile_indices,
                                      const math::Mat3& minimap_view,
                                      std::vector<MinimapPolyline>& out,
                                      int max_edges,
                                      int segments) {
    if (max_edges <= 0 || tile_indices.empty() || patch.base_polygon_vertices.empty()) {
        for (MinimapPolyline& pl : out) pl.clear();
        return;
    }

    std::unordered_set<EdgeKey, EdgeKeyHash> seen;
    std::vector<MinimapEdge> edge_list;
    edge_list.reserve(static_cast<std::size_t>(max_edges));

    bool limit_hit = false;
    for (int tile_idx : tile_indices) {
        if (limit_hit) break;
        if (tile_idx < 0 || tile_idx >= static_cast<int>(patch.tiles.size())) continue;
        const Tile& tile = patch.tiles[static_cast<std::size_t>(tile_idx)];
        for (std::size_t side = 0; side < patch.base_polygon_vertices.size(); ++side) {
            const std::size_t next_side = (side + 1U) % patch.base_polygon_vertices.size();
            const math::Vec3 a = math::apply_isometry(tile.transform, patch.base_polygon_vertices[side]);
            const math::Vec3 b = math::apply_isometry(tile.transform, patch.base_polygon_vertices[next_side]);
            const EdgeKey key = edge_key(a, b);
            if (seen.insert(key).second) {
                edge_list.push_back(MinimapEdge{a, b});
                if (static_cast<int>(edge_list.size()) >= max_edges) {
                    limit_hit = true;
                    break;
                }
            }
        }
    }

    const int detail_segments = std::max(1, segments);
    const double max_angle_step =
        std::clamp(0.12 / std::sqrt(static_cast<double>(detail_segments)), 0.012, 0.12);

    const std::size_t needed = edge_list.size();
    while (out.size() < needed) out.emplace_back();

    for (std::size_t ei = 0; ei < needed; ++ei) {
        MinimapPolyline& polyline = out[ei];
        polyline.clear();

        const glm::dvec2 pa = project_to_minimap(edge_list[ei].a, minimap_view);
        const glm::dvec2 pb = project_to_minimap(edge_list[ei].b, minimap_view);

        auto push_point = [&](const glm::dvec2& p) {
            polyline.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
        };

        // In the Poincaré disk, hyperbolic geodesics are circular arcs perpendicular
        // to the boundary. Solve for the arc through pa and pb.
        const double det = pa.x * pb.y - pa.y * pb.x;
        if (std::abs(det) < 1e-10) {
            // Points are collinear through origin — straight-line geodesic.
            push_point(pa);
            push_point(pb);
            continue;
        }

        const double rhs_a = 0.5 * (glm::dot(pa, pa) + 1.0);
        const double rhs_b = 0.5 * (glm::dot(pb, pb) + 1.0);
        const glm::dvec2 center_2d((rhs_a * pb.y - rhs_b * pa.y) / det,
                                   (pa.x * rhs_b - pb.x * rhs_a) / det);
        const double radius2 = glm::dot(center_2d, center_2d) - 1.0;

        if (radius2 <= 1e-12) {
            push_point(pa);
            push_point(pb);
            continue;
        }

        const double radius_2d = std::sqrt(radius2);
        const double theta_a = std::atan2(pa.y - center_2d.y, pa.x - center_2d.x);
        const double theta_b = std::atan2(pb.y - center_2d.y, pb.x - center_2d.x);

        auto wrap_angle = [](double angle) {
            while (angle <= -kPi) angle += 2.0 * kPi;
            while (angle > kPi)  angle -= 2.0 * kPi;
            return angle;
        };

        const double delta_short = wrap_angle(theta_b - theta_a);
        const double sign = (delta_short == 0.0) ? 1.0 : delta_short;
        const double delta_long = delta_short - std::copysign(2.0 * kPi, sign);

        auto arc_mid_r2 = [&](double delta) {
            const double angle = theta_a + 0.5 * delta;
            const glm::dvec2 mid(center_2d.x + radius_2d * std::cos(angle),
                                 center_2d.y + radius_2d * std::sin(angle));
            return glm::dot(mid, mid);
        };

        // Choose the shorter arc that stays inside the disk.
        const double delta =
            (arc_mid_r2(delta_short) <= arc_mid_r2(delta_long)) ? delta_short : delta_long;
        const int arc_steps =
            std::max(1, static_cast<int>(std::ceil(std::abs(delta) / max_angle_step)));

        polyline.reserve(static_cast<std::size_t>(arc_steps + 1));
        for (int i = 0; i <= arc_steps; ++i) {
            const double t = static_cast<double>(i) / arc_steps;
            const double angle = theta_a + delta * t;
            glm::dvec2 p(center_2d.x + radius_2d * std::cos(angle),
                         center_2d.y + radius_2d * std::sin(angle));
            const double r2 = glm::dot(p, p);
            if (r2 >= 1.0) {
                p *= 0.999999 / std::sqrt(std::max(r2, 1e-30));
            }
            push_point(p);
        }
    }

    for (std::size_t i = needed; i < out.size(); ++i) {
        out[i].clear();
    }
}

void collect_minimap(const TilingPatch& patch,
                     const math::Vec3& center,
                     double radius,
                     const math::Mat3& minimap_view,
                     std::vector<glm::vec2>& out_points,
                     std::vector<MinimapPolyline>& out_edges,
                     int max_points,
                     int max_edges,
                     int segments) {
    std::vector<int> tile_indices;
    collect_minimap_candidate_tiles(patch, center, radius, tile_indices);
    collect_minimap_points_from_tiles(patch, tile_indices, minimap_view, out_points, max_points);
    collect_minimap_edges_from_tiles(patch, tile_indices, minimap_view, out_edges, max_edges, segments);
}

} // namespace hyper::tiling
