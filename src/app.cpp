#include "app.h"

#include "math/geometry_mode.h"
#include "math/hyperbolic.h"
#include "math/spherical.h"
#include "mesh.h"
#include "shader.h"
#include "tiling_core.h"
#include "tiling_minimap.h"

#include <chrono>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace hyper {
namespace {

constexpr int kProgressDemoTilingDepth = 5;
constexpr float kMinMoveSpeed = 0.25F;
constexpr float kMaxMoveSpeed = 8.0F;
constexpr float kMinEyeHeight = 0.08F;
constexpr float kMaxEyeHeight = 1.75F;
constexpr float kEyeLiftSpeed = 0.95F;
constexpr float kBaseFovDegrees = 74.0F;
constexpr float kMinFovDegrees = 34.0F;
constexpr float kMaxFovDegrees = 100.0F;
constexpr float kMinViewZoom = kBaseFovDegrees / kMaxFovDegrees;
constexpr float kMaxViewZoom = kBaseFovDegrees / kMinFovDegrees;

struct GlfwContext {
    GlfwContext() {
        if (glfwInit() != GLFW_TRUE) {
            throw std::runtime_error("failed to initialize GLFW");
        }
    }

    ~GlfwContext() {
        glfwTerminate();
    }
};

struct RenderSettings {
    math::GeometryMode geometry_mode = math::GeometryMode::Hyperbolic;
    math::RegularTilingParameters tiling_parameters{4, 6};
    int tiling_depth = kProgressDemoTilingDepth;
    bool show_grid = true;
    bool show_wireframe = false;
    bool show_houses = true;
    bool show_lasers = true;
    bool show_debug_ui = true;
    bool show_minimap = true;
    float minimap_radius = 4.5F;
    int minimap_max_points = 1200;
    int minimap_max_edges = 3000;
    int minimap_edge_segments = 8;
    glm::vec3 atmosphere_color{0.52F, 0.76F, 0.90F};
    int edge_segments = 8;
    int radial_bands = 2;
};

// --- Geometry-mode dispatch -------------------------------------------------
// Matrix/vector application and isometry composition are metric-agnostic and
// reuse the hyperbolic implementations; only the operations below differ.
bool geo_is_spherical(math::GeometryMode mode) {
    return mode == math::GeometryMode::Spherical;
}

math::Vec3 geo_normalize(const math::Vec3& v, math::GeometryMode mode) {
    return geo_is_spherical(mode) ? math::sphere::sphere_normalize(v) : math::hyperboloid_normalize(v);
}

double geo_distance(const math::Vec3& a, const math::Vec3& b, math::GeometryMode mode) {
    return geo_is_spherical(mode) ? math::sphere::intrinsic_distance(a, b) : math::intrinsic_distance(a, b);
}

math::Vec3 geo_geodesic_lerp(const math::Vec3& a, const math::Vec3& b, double t, math::GeometryMode mode) {
    return geo_is_spherical(mode) ? math::sphere::geodesic_lerp(a, b, t) : math::geodesic_lerp(a, b, t);
}

math::CameraFrame geo_orthonormalize(const math::CameraFrame& frame, math::GeometryMode mode) {
    return geo_is_spherical(mode) ? math::sphere::orthonormalize_frame(frame)
                                  : math::orthonormalize_frame(frame);
}

math::Mat3 geo_frame_to_isometry(const math::CameraFrame& frame, math::GeometryMode mode) {
    return geo_is_spherical(mode) ? math::sphere::frame_to_isometry(frame)
                                  : math::frame_to_isometry(frame);
}

math::Mat3 geo_inverse(const math::Mat3& m, math::GeometryMode mode) {
    return geo_is_spherical(mode) ? math::sphere::inverse_isometry(m) : math::inverse_isometry(m);
}

math::Mat3 geo_local_translation(math::Vec2 delta, math::GeometryMode mode) {
    return geo_is_spherical(mode) ? math::sphere::spherical_rotation(delta) : math::lorentz_boost(delta);
}

math::CameraFrame geo_move_frame(const math::CameraFrame& frame, math::Vec2 delta, math::GeometryMode mode) {
    return geo_is_spherical(mode) ? math::sphere::move_frame(frame, delta) : math::move_frame(frame, delta);
}

math::Vec2 geo_project_conformal_disk(const math::Vec3& p, math::GeometryMode mode) {
    return geo_is_spherical(mode) ? math::sphere::project_to_stereographic_disk(p)
                                  : math::project_to_poincare_disk(p);
}

// Spherical patches close up, so they need enough BFS depth to wrap the sphere.
int geo_tiling_depth(math::GeometryMode mode) {
    return geo_is_spherical(mode) ? 8 : kProgressDemoTilingDepth;
}

math::CameraFrame display_aligned_frame(math::GeometryMode mode = math::GeometryMode::Hyperbolic) {
    return geo_orthonormalize(math::CameraFrame{
                                  math::origin(),
                                  math::Vec3{0.0, 0.0, 1.0},
                                  math::Vec3{0.0, 1.0, 0.0},
                              },
                              mode);
}

struct CameraState {
    math::CameraFrame frame = display_aligned_frame();
    float zoom = 1.0F;
    float pitch = -0.28F;
    float eye_height = 0.32F;
    float move_speed = 1.0F;
    int current_tile_id = 0;
};

bool g_cursor_captured = true;
bool g_first_mouse = true;
double g_last_mouse_x = 0;
double g_last_mouse_y = 0;

struct DebugInputState {
    bool f1_down = false;
    bool preset1_down = false;
    bool preset2_down = false;
    bool preset3_down = false;
    bool preset4_down = false;
    bool preset5_down = false;
    bool preset6_down = false;
    bool preset7_down = false;
    bool preset8_down = false;
    bool g_down = false;
    bool f_down = false;
    bool c_down = false;
    bool v_down = false;
    bool b_down = false;
    bool n_down = false;
    bool m_down = false;
    bool h_down = false;
    bool l_down = false;
};

struct OverlayVertex {
    glm::vec2 position{};
    glm::vec4 color{};
};

struct TilingPreset {
    int key = 0;
    math::RegularTilingParameters parameters{};
    const char* label = "";
    math::GeometryMode mode = math::GeometryMode::Hyperbolic;
};

struct HousePropMeshes {
    MeshData solid;
    MeshData outline;
};

math::CameraFrame global_camera_frame(const CameraState& camera, const tiling::TilingPatch& patch);
glm::vec4 h2_point_to_h3(const math::Vec3& point);
void append_vertex(MeshData& mesh,
                   const glm::vec4& position,
                   const glm::vec3& normal,
                   const glm::vec3& color);
MeshData make_curved_tile_mesh(const tiling::TilingPatch& patch, int radial_bands, int edge_segments);
MeshData make_grid_mesh(const tiling::TilingPatch& patch, int edge_segments);
HousePropMeshes make_house_prop_meshes(const tiling::TilingPatch& patch);
MeshData make_laser_prop_mesh(const tiling::TilingPatch& patch);
glm::mat4 h3_lorentz_boost(const glm::vec3& direction, float distance);
glm::mat4 h3_spherical_rotation(const glm::vec3& direction, float distance);
glm::mat4 h3_rotation_xz(float angle);

glm::vec2 clamp_to_disk(const glm::vec2& p) {
    constexpr float kClipRadius = 0.9995F;
    const float r2 = glm::dot(p, p);
    if (r2 <= kClipRadius * kClipRadius || r2 <= 0.0F) {
        return p;
    }

    return p * (kClipRadius / std::sqrt(r2));
}

bool clip_segment_to_unit_disk(const glm::vec2& a,
                               const glm::vec2& b,
                               glm::vec2& out_a,
                               glm::vec2& out_b) {
    const glm::vec2 d = b - a;
    const float aa = glm::dot(d, d);
    if (aa < 1.0e-12F) {
        if (glm::dot(a, a) > 1.0F) {
            return false;
        }
        out_a = clamp_to_disk(a);
        out_b = clamp_to_disk(b);
        return true;
    }

    const float bb = 2.0F * glm::dot(a, d);
    const float cc = glm::dot(a, a) - 1.0F;
    const float discriminant = bb * bb - 4.0F * aa * cc;
    if (discriminant < 0.0F) {
        return false;
    }

    const float root = std::sqrt(std::max(discriminant, 0.0F));
    float t0 = (-bb - root) / (2.0F * aa);
    float t1 = (-bb + root) / (2.0F * aa);
    if (t0 > t1) {
        std::swap(t0, t1);
    }

    const float enter = std::max(0.0F, t0);
    const float exit = std::min(1.0F, t1);
    if (enter > exit) {
        return false;
    }

    out_a = clamp_to_disk(a + d * enter);
    out_b = clamp_to_disk(a + d * exit);
    return true;
}

glm::vec2 project_minimap_point(const math::Vec3& world_pos, const math::Mat3& minimap_view,
                                math::GeometryMode mode) {
    const math::Vec3 local = geo_normalize(math::apply_isometry(minimap_view, world_pos), mode);
    const math::Vec2 projected = geo_project_conformal_disk(local, mode);
    return clamp_to_disk(glm::vec2{static_cast<float>(projected.x), static_cast<float>(projected.y)});
}

math::Vec3 frame_offset_point(const math::CameraFrame& frame, double forward_distance,
                              math::GeometryMode mode) {
    const math::Vec3 local_offset =
        math::apply_isometry(geo_local_translation(math::Vec2{forward_distance, 0.0}, mode), math::origin());
    return geo_normalize(math::apply_isometry(geo_frame_to_isometry(frame, mode), local_offset), mode);
}

double generated_map_radius(const tiling::TilingPatch& patch) {
    double radius = 0.0;
    const math::Vec3 origin = math::origin();
    for (const tiling::Tile& tile : patch.tiles) {
        radius = std::max(radius, geo_distance(origin, tile.center, patch.mode));
    }
    return radius + 1.0e-6;
}

struct LaserPath {
    math::Vec3 start{};
    math::Vec3 end{};
    glm::vec3 color{};
};

constexpr std::size_t kMinimumLaserTurnCount = 3;

bool finite_disk_point(const math::Vec2& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

bool supports_laser_props(const tiling::TilingPatch& patch) {
    return !patch.tiles.empty() &&
           patch.parameters.p >= 3 &&
           patch.parameters.q >= static_cast<int>(kMinimumLaserTurnCount);
}

std::size_t laser_turn_count(const tiling::TilingPatch& patch) {
    return static_cast<std::size_t>(std::max(patch.parameters.q, 0));
}

bool has_tile_id(const std::vector<int>& tile_ids, int id) {
    return std::find(tile_ids.begin(), tile_ids.end(), id) != tile_ids.end();
}

void add_unique_tile_id(std::vector<int>& tile_ids, int id) {
    if (id >= 0 && !has_tile_id(tile_ids, id)) {
        tile_ids.push_back(id);
    }
}

bool route_contains_tile(const std::vector<int>& route, int tile_id) {
    return std::find(route.begin(), route.end(), tile_id) != route.end();
}

bool tiles_are_neighbors(const tiling::TilingPatch& patch, int a, int b) {
    if (a < 0 || a >= static_cast<int>(patch.tiles.size())) {
        return false;
    }

    const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(a)];
    return std::find(tile.neighbors.begin(), tile.neighbors.end(), b) != tile.neighbors.end();
}

math::Vec3 tile_vertex(const tiling::TilingPatch& patch, const tiling::Tile& tile, std::size_t vertex_index) {
    return geo_normalize(
        math::apply_isometry(tile.transform, patch.base_polygon_vertices[vertex_index]),
        patch.mode);
}

bool points_coincide(const math::Vec3& a, const math::Vec3& b, math::GeometryMode mode) {
    try {
        return geo_distance(a, b, mode) <= 1.0e-5;
    } catch (const std::exception&) {
        return false;
    }
}

bool tile_touches_vertex(const tiling::TilingPatch& patch, int tile_id, const math::Vec3& target_vertex) {
    if (tile_id < 0 ||
        tile_id >= static_cast<int>(patch.tiles.size()) ||
        patch.base_polygon_vertices.empty()) {
        return false;
    }

    const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(tile_id)];
    for (std::size_t i = 0; i < patch.base_polygon_vertices.size(); ++i) {
        if (points_coincide(tile_vertex(patch, tile, i), target_vertex, patch.mode)) {
            return true;
        }
    }
    return false;
}

bool extend_vertex_laser_loop(const tiling::TilingPatch& patch,
                              const math::Vec3& target_vertex,
                              std::vector<int>& route,
                              std::size_t turn_count) {
    if (route.size() == turn_count) {
        return tiles_are_neighbors(patch, route.back(), route.front());
    }

    const int current_tile = route.back();
    if (current_tile < 0 || current_tile >= static_cast<int>(patch.tiles.size())) {
        return false;
    }

    const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(current_tile)];
    for (int next_tile : tile.neighbors) {
        if (next_tile < 0 ||
            route_contains_tile(route, next_tile) ||
            !tile_touches_vertex(patch, next_tile, target_vertex)) {
            continue;
        }

        route.push_back(next_tile);
        if (extend_vertex_laser_loop(patch, target_vertex, route, turn_count)) {
            return true;
        }
        route.pop_back();
    }

    return false;
}

int best_tile_in_disk_direction(const tiling::TilingPatch& patch, const glm::dvec2& direction) {
    if (patch.tiles.size() <= 1) {
        return -1;
    }

    const double direction_length = glm::length(direction);
    if (direction_length <= 1.0e-9) {
        return -1;
    }

    const glm::dvec2 unit_direction = direction / direction_length;
    double best_score = -std::numeric_limits<double>::infinity();
    int best_id = -1;

    for (const tiling::Tile& tile : patch.tiles) {
        if (tile.id == 0) {
            continue;
        }

        math::Vec2 projected{};
        try {
            projected = geo_project_conformal_disk(tile.center, patch.mode);
        } catch (const std::exception&) {
            continue;
        }
        if (!finite_disk_point(projected)) {
            continue;
        }

        const glm::dvec2 p{projected.x, projected.y};
        const double length = glm::length(p);
        if (length <= 1.0e-8) {
            continue;
        }

        const double score = glm::dot(p / length, unit_direction) + 0.30 * length;
        if (score > best_score) {
            best_score = score;
            best_id = tile.id;
        }
    }

    return best_id;
}

std::vector<int> choose_showcase_tiles(const tiling::TilingPatch& patch, std::size_t max_count) {
    std::vector<int> selected;
    if (patch.tiles.empty() || max_count == 0) {
        return selected;
    }

    const tiling::Tile& root = patch.tiles.front();
    for (int neighbor_id : root.neighbors) {
        add_unique_tile_id(selected, neighbor_id);
        if (selected.size() >= max_count) {
            return selected;
        }
    }

    constexpr std::array<glm::dvec2, 6> kDirections{{
        glm::dvec2{0.0, 1.0},
        glm::dvec2{1.0, 0.0},
        glm::dvec2{0.0, -1.0},
        glm::dvec2{-1.0, 0.0},
        glm::dvec2{0.72, 0.72},
        glm::dvec2{-0.72, 0.72},
    }};

    for (const glm::dvec2& direction : kDirections) {
        add_unique_tile_id(selected, best_tile_in_disk_direction(patch, direction));
        if (selected.size() >= max_count) {
            return selected;
        }
    }

    for (std::size_t i = 1; i < patch.tiles.size() && selected.size() < max_count; ++i) {
        add_unique_tile_id(selected, patch.tiles[i].id);
    }

    return selected;
}

std::vector<int> choose_laser_anchor_tiles(const tiling::TilingPatch& patch) {
    std::vector<int> anchors;
    if (!supports_laser_props(patch) || patch.tiles.empty() || patch.base_polygon_vertices.empty()) {
        return anchors;
    }

    const std::size_t turn_count = laser_turn_count(patch);
    if (turn_count < kMinimumLaserTurnCount) {
        return anchors;
    }

    // A laser loop around a tiling vertex has q one-tile segments. Consecutive
    // tiles meet at the same regular p-gon vertex, so each turn is 360 / p
    // degrees: {4,6} gives six 90-degree turns and {4,5} gives five.
    const tiling::Tile& root = patch.tiles.front();
    for (std::size_t vertex_index = 0; vertex_index < patch.base_polygon_vertices.size(); ++vertex_index) {
        anchors.assign(1U, 0);
        const math::Vec3 target_vertex = tile_vertex(patch, root, vertex_index);
        if (extend_vertex_laser_loop(patch, target_vertex, anchors, turn_count)) {
            return anchors;
        }
    }

    anchors.clear();
    return anchors;
}

std::vector<LaserPath> make_laser_paths(const tiling::TilingPatch& patch) {
    std::vector<LaserPath> paths;
    if (!supports_laser_props(patch)) {
        return paths;
    }

    const std::vector<int> anchors = choose_laser_anchor_tiles(patch);
    const std::size_t turn_count = laser_turn_count(patch);
    if (anchors.size() < turn_count) {
        return paths;
    }

    constexpr glm::vec3 kLaserColor{1.00F, 0.24F, 0.58F};

    const std::size_t path_count = std::min(anchors.size(), turn_count);
    for (std::size_t i = 0; i < path_count; ++i) {
        const int a = anchors[i];
        const int b = anchors[(i + 1) % path_count];
        if (a < 0 || b < 0 || a == b ||
            a >= static_cast<int>(patch.tiles.size()) ||
            b >= static_cast<int>(patch.tiles.size())) {
            continue;
        }

        double distance = 0.0;
        try {
            distance = geo_distance(patch.tiles[static_cast<std::size_t>(a)].center,
                                    patch.tiles[static_cast<std::size_t>(b)].center,
                                    patch.mode);
        } catch (const std::exception&) {
            continue;
        }
        if (!std::isfinite(distance) || distance <= 1.0e-5 ||
            (geo_is_spherical(patch.mode) && distance >= 3.12)) {
            continue;
        }

        paths.push_back(LaserPath{
            patch.tiles[static_cast<std::size_t>(a)].center,
            patch.tiles[static_cast<std::size_t>(b)].center,
            kLaserColor,
        });
    }

    return paths;
}

void collect_laser_minimap_edges(const tiling::TilingPatch& patch,
                                 const math::Mat3& minimap_view,
                                 std::vector<tiling::MinimapPolyline>& edges) {
    edges.clear();
    constexpr int kSegments = 40;
    const std::vector<LaserPath> paths = make_laser_paths(patch);
    if (paths.empty()) {
        return;
    }

    tiling::MinimapPolyline polyline;
    polyline.reserve(paths.size() * static_cast<std::size_t>(kSegments) + 1U);

    try {
        for (const LaserPath& path : paths) {
            for (int segment = 0; segment <= kSegments; ++segment) {
                if (!polyline.empty() && segment == 0) {
                    continue;
                }
                const double t = static_cast<double>(segment) / static_cast<double>(kSegments);
                const math::Vec3 point = geo_geodesic_lerp(path.start, path.end, t, patch.mode);
                polyline.push_back(project_minimap_point(point, minimap_view, patch.mode));
            }
        }
    } catch (const std::exception&) {
        polyline.clear();
    }

    if (polyline.size() >= 2) {
        edges.push_back(polyline);
    }
}

glm::vec3 active_sky_color(const RenderSettings& settings) {
    return settings.atmosphere_color;
}

constexpr std::array<TilingPreset, 8> kTilingPresets{{
    TilingPreset{GLFW_KEY_1, math::RegularTilingParameters{4, 6}, "1 {4,6} 6 SQUARES", math::GeometryMode::Hyperbolic},
    TilingPreset{GLFW_KEY_2, math::RegularTilingParameters{3, 7}, "2 {3,7} 7 TRIANGLES", math::GeometryMode::Hyperbolic},
    TilingPreset{GLFW_KEY_3, math::RegularTilingParameters{5, 4}, "3 {5,4} 4 PENTAGONS", math::GeometryMode::Hyperbolic},
    TilingPreset{GLFW_KEY_4, math::RegularTilingParameters{7, 3}, "4 {7,3} 3 HEPTAGONS", math::GeometryMode::Hyperbolic},
    TilingPreset{GLFW_KEY_5, math::RegularTilingParameters{4, 3}, "5 {4,3} SPHERE CUBE", math::GeometryMode::Spherical},
    TilingPreset{GLFW_KEY_6, math::RegularTilingParameters{3, 4}, "6 {3,4} SPHERE OCTA", math::GeometryMode::Spherical},
    TilingPreset{GLFW_KEY_7, math::RegularTilingParameters{5, 3}, "7 {5,3} SPHERE DODECA", math::GeometryMode::Spherical},
    TilingPreset{GLFW_KEY_8, math::RegularTilingParameters{3, 5}, "8 {3,5} SPHERE ICOSA", math::GeometryMode::Spherical},
}};

void framebuffer_size_callback(GLFWwindow*, int width, int height) {
    glViewport(0, 0, width, height);
}

std::filesystem::path resolve_shader_path(const std::string& filename) {
    const std::filesystem::path direct = std::filesystem::path{"shaders"} / filename;
    if (std::filesystem::exists(direct)) {
        return direct;
    }

    const std::filesystem::path from_build = std::filesystem::path{".."} / "shaders" / filename;
    if (std::filesystem::exists(from_build)) {
        return from_build;
    }

    return direct;
}

std::string overlay_shader_log(GLuint shader) {
    GLint length = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
    if (length <= 1) {
        return {};
    }

    std::vector<char> buffer(static_cast<std::size_t>(length));
    glGetShaderInfoLog(shader, length, nullptr, buffer.data());
    return std::string(buffer.data());
}

std::string overlay_program_log(GLuint program) {
    GLint length = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
    if (length <= 1) {
        return {};
    }

    std::vector<char> buffer(static_cast<std::size_t>(length));
    glGetProgramInfoLog(program, length, nullptr, buffer.data());
    return std::string(buffer.data());
}

GLuint compile_overlay_shader(GLenum type, const char* source) {
    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok != GL_TRUE) {
        const std::string log = overlay_shader_log(shader);
        glDeleteShader(shader);
        throw std::runtime_error("failed to compile debug overlay shader: " + log);
    }

    return shader;
}

std::array<std::uint8_t, 7> glyph_rows(char ch) {
    switch (static_cast<char>(std::toupper(static_cast<unsigned char>(ch)))) {
    case 'A': return {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    case 'B': return {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E};
    case 'C': return {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E};
    case 'D': return {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E};
    case 'E': return {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F};
    case 'F': return {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10};
    case 'G': return {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F};
    case 'H': return {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11};
    case 'I': return {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E};
    case 'J': return {0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0E};
    case 'K': return {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11};
    case 'L': return {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F};
    case 'M': return {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11};
    case 'N': return {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11};
    case 'O': return {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    case 'P': return {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10};
    case 'Q': return {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D};
    case 'R': return {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11};
    case 'S': return {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E};
    case 'T': return {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
    case 'U': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E};
    case 'V': return {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04};
    case 'W': return {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11};
    case 'X': return {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11};
    case 'Y': return {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04};
    case 'Z': return {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F};
    case '0': return {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E};
    case '1': return {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E};
    case '2': return {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F};
    case '3': return {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E};
    case '4': return {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02};
    case '5': return {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E};
    case '6': return {0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E};
    case '7': return {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08};
    case '8': return {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E};
    case '9': return {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E};
    case '.': return {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C};
    case ':': return {0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00};
    case '-': return {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00};
    case '+': return {0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00};
    case '/': return {0x01, 0x01, 0x02, 0x04, 0x08, 0x10, 0x10};
    case '[': return {0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E};
    case ']': return {0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E};
    case '|': return {0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04};
    case '=': return {0x00, 0x00, 0x1F, 0x00, 0x1F, 0x00, 0x00};
    default: return {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    }
}

class DebugOverlay {
    struct MinimapWindow {
        glm::vec2 position{};
        float size = 150.0F;
        bool initialized = false;
    };

public:
    ~DebugOverlay() {
        shutdown();
    }

    void init() {
        constexpr const char* vertex_source = R"glsl(
#version 410 core
layout(location = 0) in vec2 aPosition;
layout(location = 1) in vec4 aColor;
uniform vec2 uViewport;
out vec4 vColor;
void main() {
    vec2 ndc = vec2((aPosition.x / uViewport.x) * 2.0 - 1.0,
                    1.0 - (aPosition.y / uViewport.y) * 2.0);
    gl_Position = vec4(ndc, 0.0, 1.0);
    vColor = aColor;
}
)glsl";

        constexpr const char* fragment_source = R"glsl(
#version 410 core
in vec4 vColor;
out vec4 FragColor;
void main() {
    FragColor = vColor;
}
)glsl";

        const GLuint vertex_shader = compile_overlay_shader(GL_VERTEX_SHADER, vertex_source);
        const GLuint fragment_shader = compile_overlay_shader(GL_FRAGMENT_SHADER, fragment_source);

        program_ = glCreateProgram();
        glAttachShader(program_, vertex_shader);
        glAttachShader(program_, fragment_shader);
        glLinkProgram(program_);

        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);

        GLint ok = GL_FALSE;
        glGetProgramiv(program_, GL_LINK_STATUS, &ok);
        if (ok != GL_TRUE) {
            const std::string log = overlay_program_log(program_);
            glDeleteProgram(program_);
            program_ = 0;
            throw std::runtime_error("failed to link debug overlay shader: " + log);
        }

        glGenVertexArrays(1, &vao_);
        glGenBuffers(1, &vbo_);

        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(OverlayVertex),
                              reinterpret_cast<void*>(offsetof(OverlayVertex, position)));
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(OverlayVertex),
                              reinterpret_cast<void*>(offsetof(OverlayVertex, color)));
        glBindVertexArray(0);
    }

    void shutdown() {
        if (vbo_ != 0) {
            glDeleteBuffers(1, &vbo_);
            vbo_ = 0;
        }
        if (vao_ != 0) {
            glDeleteVertexArrays(1, &vao_);
            vao_ = 0;
        }
        if (program_ != 0) {
            glDeleteProgram(program_);
            program_ = 0;
        }
    }

    void draw(int width,
              int height,
              const RenderSettings& settings,
              const CameraState& camera,
              const tiling::TilingPatch& patch) {
        if (program_ == 0 || width <= 0 || height <= 0) {
            return;
        }

        vertices_.clear();
        add_crosshair(static_cast<float>(width), static_cast<float>(height));
        if (settings.show_debug_ui) {
            add_panel(settings, camera, patch);
        } else {
            add_rect(10.0F, 10.0F, 138.0F, 24.0F, glm::vec4{0.02F, 0.03F, 0.04F, 0.72F});
            add_text(18.0F, 18.0F, "F1 DEBUG UI", 2.0F, glm::vec4{0.88F, 0.94F, 1.00F, 0.92F});
        }
        if (settings.show_minimap) {
            add_minimap(static_cast<float>(width), static_cast<float>(height), settings, camera, patch);
        }

        if (vertices_.empty()) {
            return;
        }

        glDisable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glUseProgram(program_);
        glUniform2f(glGetUniformLocation(program_, "uViewport"),
                    static_cast<float>(width),
                    static_cast<float>(height));

        glBindVertexArray(vao_);
        glBindBuffer(GL_ARRAY_BUFFER, vbo_);
        glBufferData(GL_ARRAY_BUFFER,
                     static_cast<GLsizeiptr>(vertices_.size() * sizeof(OverlayVertex)),
                     vertices_.data(),
                     GL_STREAM_DRAW);
        glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices_.size()));
        glBindVertexArray(0);
    }

private:
    void add_rect(float x, float y, float w, float h, const glm::vec4& color) {
        const OverlayVertex a{{x, y}, color};
        const OverlayVertex b{{x + w, y}, color};
        const OverlayVertex c{{x + w, y + h}, color};
        const OverlayVertex d{{x, y + h}, color};
        vertices_.push_back(a);
        vertices_.push_back(b);
        vertices_.push_back(c);
        vertices_.push_back(a);
        vertices_.push_back(c);
        vertices_.push_back(d);
    }

    void add_triangle(const glm::vec2& a,
                      const glm::vec2& b,
                      const glm::vec2& c,
                      const glm::vec4& color) {
        vertices_.push_back(OverlayVertex{a, color});
        vertices_.push_back(OverlayVertex{b, color});
        vertices_.push_back(OverlayVertex{c, color});
    }

    void add_line_segment(const glm::vec2& a,
                          const glm::vec2& b,
                          float thickness,
                          const glm::vec4& color) {
        const glm::vec2 d = b - a;
        const float len2 = glm::dot(d, d);
        if (len2 <= 1.0e-6F) {
            return;
        }

        const glm::vec2 unit = d / std::sqrt(len2);
        const glm::vec2 normal{-unit.y, unit.x};
        const glm::vec2 offset = normal * (0.5F * thickness);
        const glm::vec2 p0 = a + offset;
        const glm::vec2 p1 = b + offset;
        const glm::vec2 p2 = b - offset;
        const glm::vec2 p3 = a - offset;
        add_triangle(p0, p1, p2, color);
        add_triangle(p0, p2, p3, color);
    }

    void add_circle_filled(const glm::vec2& center,
                           float radius,
                           int segments,
                           const glm::vec4& color) {
        const int count = std::max(8, segments);
        constexpr float kTwoPi = 6.28318530717958647692F;
        for (int i = 0; i < count; ++i) {
            const float a0 = kTwoPi * static_cast<float>(i) / static_cast<float>(count);
            const float a1 = kTwoPi * static_cast<float>(i + 1) / static_cast<float>(count);
            add_triangle(center,
                         center + radius * glm::vec2{std::cos(a0), std::sin(a0)},
                         center + radius * glm::vec2{std::cos(a1), std::sin(a1)},
                         color);
        }
    }

    void add_circle_outline(const glm::vec2& center,
                            float radius,
                            int segments,
                            float thickness,
                            const glm::vec4& color) {
        const int count = std::max(8, segments);
        constexpr float kTwoPi = 6.28318530717958647692F;
        for (int i = 0; i < count; ++i) {
            const float a0 = kTwoPi * static_cast<float>(i) / static_cast<float>(count);
            const float a1 = kTwoPi * static_cast<float>(i + 1) / static_cast<float>(count);
            add_line_segment(center + radius * glm::vec2{std::cos(a0), std::sin(a0)},
                             center + radius * glm::vec2{std::cos(a1), std::sin(a1)},
                             thickness,
                             color);
        }
    }

    static float minimap_window_width(const MinimapWindow& window) {
        return window.size + 16.0F;
    }

    static float minimap_window_height(const MinimapWindow& window) {
        return window.size + 38.0F;
    }

    static constexpr float kMinimapStackGap = 12.0F;
    static constexpr float kMinimapMargin = 16.0F;
    static constexpr float kMinimapViewportScale = 0.26F;

    static float minimap_view_size(float width, float height) {
        const float desired_size = std::max(0.0F, std::min(width, height) * kMinimapViewportScale);
        const float max_width_size = std::max(0.0F, width - 16.0F);
        const float max_stacked_size = std::max(0.0F, (height - kMinimapStackGap) * 0.5F - 38.0F);
        return std::min(desired_size, std::min(max_width_size, max_stacked_size));
    }

    void sync_minimap_stack(float width, float height) {
        const float side = minimap_view_size(width, height);
        static_window_.size = side;
        dynamic_window_.size = side;

        const float stack_width = std::max(minimap_window_width(static_window_), minimap_window_width(dynamic_window_));
        const float stack_height =
            minimap_window_height(static_window_) + kMinimapStackGap + minimap_window_height(dynamic_window_);

        static_window_.position.x = std::max(0.0F, width - kMinimapMargin - stack_width);
        static_window_.position.y = std::min(kMinimapMargin, std::max(0.0F, height - stack_height));

        dynamic_window_.position.x = static_window_.position.x;
        dynamic_window_.position.y =
            static_window_.position.y + minimap_window_height(static_window_) + kMinimapStackGap;
    }

    void ensure_minimap_windows(float width, float height) {
        if (static_window_.initialized && dynamic_window_.initialized) {
            sync_minimap_stack(width, height);
            return;
        }

        static_window_.initialized = true;
        dynamic_window_.initialized = true;
        sync_minimap_stack(width, height);
    }

    void add_text(float x, float y, const std::string& text, float scale, const glm::vec4& color) {
        float cursor_x = x;
        for (char ch : text) {
            if (ch == '\n') {
                cursor_x = x;
                y += 8.0F * scale;
                continue;
            }
            if (ch != ' ') {
                const auto rows = glyph_rows(ch);
                for (int row = 0; row < 7; ++row) {
                    for (int col = 0; col < 5; ++col) {
                        if ((rows[static_cast<std::size_t>(row)] & (1U << (4 - col))) != 0U) {
                            add_rect(cursor_x + static_cast<float>(col) * scale,
                                     y + static_cast<float>(row) * scale,
                                     scale,
                                     scale,
                                     color);
                        }
                    }
                }
            }
            cursor_x += 6.0F * scale;
        }
    }

    void add_crosshair(float width, float height) {
        const glm::vec4 color{1.0F, 1.0F, 1.0F, 0.78F};
        const float cx = width * 0.5F;
        const float cy = height * 0.5F;
        add_rect(cx - 9.0F, cy - 1.0F, 18.0F, 2.0F, color);
        add_rect(cx - 1.0F, cy - 9.0F, 2.0F, 18.0F, color);
    }

    void ensure_static_minimap_cache(const RenderSettings& settings, const tiling::TilingPatch& patch) {
        if (static_minimap_cache_valid_ &&
            static_minimap_parameters_.p == settings.tiling_parameters.p &&
            static_minimap_parameters_.q == settings.tiling_parameters.q &&
            static_minimap_depth_ == settings.tiling_depth &&
            static_minimap_tile_count_ == patch.tiles.size() &&
            static_minimap_max_points_ == settings.minimap_max_points &&
            static_minimap_max_edges_ == settings.minimap_max_edges &&
            static_minimap_edge_segments_ == settings.minimap_edge_segments) {
            return;
        }

        tiling::collect_minimap(patch,
                                math::origin(),
                                generated_map_radius(patch),
                                math::identity_isometry(),
                                static_minimap_points_,
                                static_minimap_edges_,
                                settings.minimap_max_points,
                                settings.minimap_max_edges,
                                settings.minimap_edge_segments);

        static_minimap_parameters_ = settings.tiling_parameters;
        static_minimap_depth_ = settings.tiling_depth;
        static_minimap_tile_count_ = patch.tiles.size();
        static_minimap_max_points_ = settings.minimap_max_points;
        static_minimap_max_edges_ = settings.minimap_max_edges;
        static_minimap_edge_segments_ = settings.minimap_edge_segments;
        static_minimap_cache_valid_ = true;
    }

    void add_minimap_window_frame(const MinimapWindow& window, const std::string& title) {
        const float w = minimap_window_width(window);
        const float h = minimap_window_height(window);
        const glm::vec2 p = window.position;
        const glm::vec4 shadow{0.0F, 0.0F, 0.0F, 0.26F};
        const glm::vec4 body{0.015F, 0.025F, 0.026F, 0.86F};
        const glm::vec4 title_bar{0.08F, 0.16F, 0.18F, 0.94F};
        const glm::vec4 border{0.64F, 0.82F, 0.88F, 0.48F};
        const glm::vec4 title_color{0.90F, 0.96F, 0.98F, 0.92F};

        add_rect(p.x + 4.0F, p.y + 5.0F, w, h, shadow);
        add_rect(p.x, p.y, w, h, body);
        add_rect(p.x, p.y, w, 24.0F, title_bar);
        add_text(p.x + 10.0F, p.y + 7.0F, title, 1.35F, title_color);

        add_line_segment(p, p + glm::vec2{w, 0.0F}, 1.0F, border);
        add_line_segment(p + glm::vec2{w, 0.0F}, p + glm::vec2{w, h}, 1.0F, border);
        add_line_segment(p + glm::vec2{w, h}, p + glm::vec2{0.0F, h}, 1.0F, border);
        add_line_segment(p + glm::vec2{0.0F, h}, p, 1.0F, border);
    }

    void add_minimap_view(const MinimapWindow& window,
                          const std::vector<glm::vec2>& points,
                          const std::vector<tiling::MinimapPolyline>& edges,
                          const std::vector<tiling::MinimapPolyline>& laser_edges,
                          const glm::vec2& origin_marker,
                          const glm::vec2& camera_marker,
                          const glm::vec2& forward_marker,
                          bool camera_local_axes,
                          const std::string& label) {
        add_minimap_window_frame(window, label);
        const glm::vec2 center =
            window.position + glm::vec2{8.0F + window.size * 0.5F, 30.0F + window.size * 0.5F};
        const float radius = window.size * 0.5F;
        const float scale = radius * 0.90F;
        const glm::vec4 bg{0.02F, 0.035F, 0.035F, 0.76F};
        const glm::vec4 rim{0.82F, 0.92F, 0.96F, 0.36F};
        const glm::vec4 edge{0.68F, 0.82F, 0.90F, 0.43F};
        const glm::vec4 laser_glow{1.00F, 0.20F, 0.56F, 0.58F};
        const glm::vec4 laser_core{1.00F, 0.96F, 0.98F, 0.88F};
        const glm::vec4 dot{0.32F, 0.72F, 0.96F, 0.80F};
        const glm::vec4 camera_color{0.98F, 0.92F, 0.18F, 0.96F};
        const glm::vec4 origin_color{0.90F, 0.96F, 1.0F, 0.52F};

        add_circle_filled(center, radius, 48, bg);
        add_circle_outline(center, radius, 72, 1.5F, rim);

        auto map_point = [&](const glm::vec2& p) {
            const glm::vec2 oriented = camera_local_axes ? glm::vec2{p.y, p.x} : p;
            return glm::vec2{center.x + oriented.x * scale, center.y - oriented.y * scale};
        };

        for (const tiling::MinimapPolyline& polyline : edges) {
            if (polyline.size() < 2) {
                continue;
            }
            for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
                glm::vec2 a{};
                glm::vec2 b{};
                if (clip_segment_to_unit_disk(polyline[i], polyline[i + 1], a, b)) {
                    add_line_segment(map_point(a), map_point(b), 1.1F, edge);
                }
            }
        }

        for (const tiling::MinimapPolyline& polyline : laser_edges) {
            if (polyline.size() < 2) {
                continue;
            }
            for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
                glm::vec2 a{};
                glm::vec2 b{};
                if (clip_segment_to_unit_disk(polyline[i], polyline[i + 1], a, b)) {
                    add_line_segment(map_point(a), map_point(b), 3.6F, laser_glow);
                    add_line_segment(map_point(a), map_point(b), 1.3F, laser_core);
                }
            }
        }

        for (const glm::vec2& p : points) {
            if (glm::dot(p, p) >= 1.0F) {
                continue;
            }
            const glm::vec2 screen = map_point(p);
            add_rect(screen.x - 1.5F, screen.y - 1.5F, 3.0F, 3.0F, dot);
        }

        if (glm::dot(origin_marker, origin_marker) < 1.0F) {
            const glm::vec2 origin = map_point(origin_marker);
            add_line_segment(origin + glm::vec2{-4.0F, 0.0F}, origin + glm::vec2{4.0F, 0.0F}, 1.0F, origin_color);
            add_line_segment(origin + glm::vec2{0.0F, -4.0F}, origin + glm::vec2{0.0F, 4.0F}, 1.0F, origin_color);
        }

        if (glm::dot(camera_marker, camera_marker) < 1.0F) {
            const glm::vec2 camera = map_point(camera_marker);
            glm::vec2 clipped_a{};
            glm::vec2 clipped_b{};
            if (clip_segment_to_unit_disk(camera_marker, forward_marker, clipped_a, clipped_b)) {
                add_line_segment(map_point(clipped_a), map_point(clipped_b), 2.0F, camera_color);
            }
            add_rect(camera.x - 3.0F, camera.y - 3.0F, 6.0F, 6.0F, camera_color);
        }
    }

    void add_minimap(float width,
                     float height,
                     const RenderSettings& settings,
                     const CameraState& camera,
                     const tiling::TilingPatch& patch) {
        if (patch.tiles.empty() || width <= 0.0F || height <= 0.0F) {
            return;
        }

        ensure_static_minimap_cache(settings, patch);
        ensure_minimap_windows(width, height);

        const math::CameraFrame global_frame = global_camera_frame(camera, patch);
        const math::Mat3 static_view = math::identity_isometry();
        const math::Mat3 dynamic_view =
            geo_inverse(geo_frame_to_isometry(global_frame, patch.mode), patch.mode);
        const math::Vec3 forward_point = frame_offset_point(global_frame, 0.35, patch.mode);

        const double dynamic_query_radius =
            static_cast<double>(settings.minimap_radius) + patch.metrics.circumradius;
        tiling::collect_minimap(patch,
                                global_frame.position,
                                dynamic_query_radius,
                                dynamic_view,
                                dynamic_minimap_points_,
                                dynamic_minimap_edges_,
                                settings.minimap_max_points,
                                settings.minimap_max_edges,
                                settings.minimap_edge_segments);

        if (settings.show_lasers) {
            collect_laser_minimap_edges(patch, static_view, static_laser_minimap_edges_);
            collect_laser_minimap_edges(patch, dynamic_view, dynamic_laser_minimap_edges_);
        } else {
            static_laser_minimap_edges_.clear();
            dynamic_laser_minimap_edges_.clear();
        }

        add_minimap_view(static_window_,
                         static_minimap_points_,
                         static_minimap_edges_,
                         static_laser_minimap_edges_,
                         project_minimap_point(math::origin(), static_view, patch.mode),
                         project_minimap_point(global_frame.position, static_view, patch.mode),
                         project_minimap_point(forward_point, static_view, patch.mode),
                         false,
                         "STATIC MINIMAP");
        add_minimap_view(dynamic_window_,
                         dynamic_minimap_points_,
                         dynamic_minimap_edges_,
                         dynamic_laser_minimap_edges_,
                         project_minimap_point(math::origin(), dynamic_view, patch.mode),
                         project_minimap_point(global_frame.position, dynamic_view, patch.mode),
                         project_minimap_point(forward_point, dynamic_view, patch.mode),
                         true,
                         "LOCAL MINIMAP");
    }

    void add_panel(const RenderSettings& settings,
                   const CameraState& camera,
                   const tiling::TilingPatch& patch) {
        const math::CameraFrame global_frame = global_camera_frame(camera, patch);
        const double origin_distance = geo_distance(math::origin(), global_frame.position, patch.mode);
        const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(camera.current_tile_id)];

        add_rect(10.0F, 10.0F, 392.0F, 398.0F, glm::vec4{0.02F, 0.03F, 0.04F, 0.78F});
        add_rect(10.0F, 10.0F, 392.0F, 30.0F, glm::vec4{0.12F, 0.18F, 0.20F, 0.92F});
        add_text(20.0F, 19.0F, "DEBUG UI", 2.0F, glm::vec4{0.94F, 0.98F, 1.0F, 0.98F});

        float y = 52.0F;
        constexpr float line_height = 18.0F;
        const glm::vec4 label{0.78F, 0.88F, 0.92F, 0.96F};
        const glm::vec4 value{0.96F, 0.96F, 0.82F, 0.96F};

        auto line = [&](const std::string& text, const glm::vec4& color) {
            add_text(22.0F, y, text, 2.0F, color);
            y += line_height;
        };

        {
            std::ostringstream text;
            text << math::geometry_mode_name(patch.mode) << " {" << patch.parameters.p << ","
                 << patch.parameters.q << "}";
            line(text.str(), value);
        }
        {
            std::ostringstream text;
            text << patch.parameters.p << " SIDES | " << patch.parameters.q << " AT CORNER";
            line(text.str(), label);
        }
        line("1-4 HYPERBOLIC TILINGS", label);
        line("5-8 SPHERICAL TILINGS", label);
        {
            std::ostringstream text;
            text << "[/] SPEED " << std::fixed << std::setprecision(2) << camera.move_speed;
            line(text.str(), value);
        }
        {
            std::ostringstream text;
            text << "Q/E VIEW ZOOM " << std::fixed << std::setprecision(2) << camera.zoom;
            line(text.str(), value);
        }
        {
            std::ostringstream text;
            text << "SPACE/SHIFT HEIGHT " << std::fixed << std::setprecision(2) << camera.eye_height;
            line(text.str(), value);
        }
        {
            std::ostringstream text;
            text << "C/V EDGE SEG " << settings.edge_segments;
            line(text.str(), value);
        }
        {
            std::ostringstream text;
            text << "B/N RADIAL BANDS " << settings.radial_bands;
            line(text.str(), value);
        }

        line(std::string{"G GRID "} + (settings.show_grid ? "ON" : "OFF"), label);
        line(std::string{"F WIREFRAME "} + (settings.show_wireframe ? "ON" : "OFF"), label);
        line(std::string{"M MINIMAP "} + (settings.show_minimap ? "ON" : "OFF"), label);
        line(std::string{"H HOUSES "} + (settings.show_houses ? "ON" : "OFF"), label);
        if (supports_laser_props(patch)) {
            line(std::string{"L LASERS "} + (settings.show_lasers ? "ON" : "OFF"), label);
        } else {
            line("L LASERS N/A", label);
        }
        line(std::string{"ESC CURSOR "} + (g_cursor_captured ? "CAPTURED" : "FREE"), label);

        {
            std::ostringstream text;
            text << "TILE " << camera.current_tile_id << " DEPTH " << tile.depth;
            line(text.str(), value);
        }
        {
            std::ostringstream text;
            text << "DIST " << std::fixed << std::setprecision(2) << origin_distance
                 << " TILES " << patch.tiles.size();
            line(text.str(), value);
        }

        add_text(22.0F, 380.0F, "F1 HIDE PANEL", 2.0F, glm::vec4{0.58F, 0.70F, 0.74F, 0.95F});
    }

    GLuint program_ = 0;
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    bool static_minimap_cache_valid_ = false;
    math::RegularTilingParameters static_minimap_parameters_{};
    int static_minimap_depth_ = -1;
    std::size_t static_minimap_tile_count_ = 0;
    int static_minimap_max_points_ = -1;
    int static_minimap_max_edges_ = -1;
    int static_minimap_edge_segments_ = -1;
    std::vector<glm::vec2> static_minimap_points_;
    std::vector<tiling::MinimapPolyline> static_minimap_edges_;
    std::vector<glm::vec2> dynamic_minimap_points_;
    std::vector<tiling::MinimapPolyline> dynamic_minimap_edges_;
    std::vector<tiling::MinimapPolyline> static_laser_minimap_edges_;
    std::vector<tiling::MinimapPolyline> dynamic_laser_minimap_edges_;
    MinimapWindow static_window_;
    MinimapWindow dynamic_window_;
    std::vector<OverlayVertex> vertices_;
};

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    (void)scancode;
    (void)mods;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        g_cursor_captured = !g_cursor_captured;
        glfwSetInputMode(window, GLFW_CURSOR, g_cursor_captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        g_first_mouse = true;
    }
}

bool consume_key_press(GLFWwindow* window, int key, bool& was_down) {
    const bool down = glfwGetKey(window, key) == GLFW_PRESS;
    const bool pressed = down && !was_down;
    was_down = down;
    return pressed;
}

void upload_optional_mesh(Mesh& mesh, const MeshData& data) {
    if (data.vertices.empty() || data.indices.empty()) {
        mesh.reset();
        return;
    }
    mesh.upload(data);
}

void rebuild_tiling(RenderSettings& settings,
                    tiling::TilingPatch& patch,
                    Mesh& center_mesh,
                    Mesh& grid_mesh,
                    Mesh& house_solid_mesh,
                    Mesh& house_outline_mesh,
                    Mesh& laser_mesh,
                    CameraState& camera) {
    settings.tiling_depth = geo_tiling_depth(settings.geometry_mode);
    patch = tiling::generate_tiling_patch(settings.tiling_parameters, settings.tiling_depth, 1.0e-6,
                                          settings.geometry_mode);
    center_mesh.upload(make_curved_tile_mesh(patch, settings.radial_bands, settings.edge_segments));
    grid_mesh.upload(make_grid_mesh(patch, settings.edge_segments));
    const HousePropMeshes house_meshes = make_house_prop_meshes(patch);
    upload_optional_mesh(house_solid_mesh, house_meshes.solid);
    upload_optional_mesh(house_outline_mesh, house_meshes.outline);
    upload_optional_mesh(laser_mesh, make_laser_prop_mesh(patch));
    camera.frame = display_aligned_frame(settings.geometry_mode);
    camera.pitch = -0.28F;
    camera.eye_height = 0.32F;
    camera.current_tile_id = 0;
}

bool apply_tiling_preset(RenderSettings& settings, math::RegularTilingParameters parameters,
                         math::GeometryMode mode) {
    const bool valid = geo_is_spherical(mode) ? math::sphere::is_spherical_tiling(parameters.p, parameters.q)
                                              : math::is_hyperbolic_tiling(parameters.p, parameters.q);
    if (!valid) {
        return false;
    }
    if (settings.geometry_mode == mode && settings.tiling_parameters.p == parameters.p &&
        settings.tiling_parameters.q == parameters.q) {
        return false;
    }

    settings.geometry_mode = mode;
    settings.tiling_parameters = parameters;
    return true;
}

bool process_debug_input(GLFWwindow* window,
                         RenderSettings& settings,
                         DebugInputState& input,
                         tiling::TilingPatch& patch,
                         Mesh& center_mesh,
                         Mesh& grid_mesh,
                         Mesh& house_solid_mesh,
                         Mesh& house_outline_mesh,
                         Mesh& laser_mesh,
                         CameraState& camera) {
    bool rebuild_meshes = false;
    bool rebuild_patch = false;

    if (consume_key_press(window, GLFW_KEY_F1, input.f1_down)) {
        settings.show_debug_ui = !settings.show_debug_ui;
    }
    if (consume_key_press(window, GLFW_KEY_1, input.preset1_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[0].parameters, kTilingPresets[0].mode);
    }
    if (consume_key_press(window, GLFW_KEY_2, input.preset2_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[1].parameters, kTilingPresets[1].mode);
    }
    if (consume_key_press(window, GLFW_KEY_3, input.preset3_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[2].parameters, kTilingPresets[2].mode);
    }
    if (consume_key_press(window, GLFW_KEY_4, input.preset4_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[3].parameters, kTilingPresets[3].mode);
    }
    if (consume_key_press(window, GLFW_KEY_5, input.preset5_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[4].parameters, kTilingPresets[4].mode);
    }
    if (consume_key_press(window, GLFW_KEY_6, input.preset6_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[5].parameters, kTilingPresets[5].mode);
    }
    if (consume_key_press(window, GLFW_KEY_7, input.preset7_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[6].parameters, kTilingPresets[6].mode);
    }
    if (consume_key_press(window, GLFW_KEY_8, input.preset8_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[7].parameters, kTilingPresets[7].mode);
    }
    if (consume_key_press(window, GLFW_KEY_G, input.g_down)) {
        settings.show_grid = !settings.show_grid;
    }
    if (consume_key_press(window, GLFW_KEY_F, input.f_down)) {
        settings.show_wireframe = !settings.show_wireframe;
    }
    if (consume_key_press(window, GLFW_KEY_M, input.m_down)) {
        settings.show_minimap = !settings.show_minimap;
    }
    if (consume_key_press(window, GLFW_KEY_H, input.h_down)) {
        settings.show_houses = !settings.show_houses;
    }
    if (consume_key_press(window, GLFW_KEY_L, input.l_down)) {
        settings.show_lasers = !settings.show_lasers;
    }
    if (consume_key_press(window, GLFW_KEY_C, input.c_down)) {
        settings.edge_segments = std::max(1, settings.edge_segments - 1);
        rebuild_meshes = true;
    }
    if (consume_key_press(window, GLFW_KEY_V, input.v_down)) {
        settings.edge_segments = std::min(24, settings.edge_segments + 1);
        rebuild_meshes = true;
    }
    if (consume_key_press(window, GLFW_KEY_B, input.b_down)) {
        settings.radial_bands = std::max(1, settings.radial_bands - 1);
        rebuild_meshes = true;
    }
    if (consume_key_press(window, GLFW_KEY_N, input.n_down)) {
        settings.radial_bands = std::min(8, settings.radial_bands + 1);
        rebuild_meshes = true;
    }

    if (rebuild_patch) {
        rebuild_tiling(settings,
                       patch,
                       center_mesh,
                       grid_mesh,
                       house_solid_mesh,
                       house_outline_mesh,
                       laser_mesh,
                       camera);
        return true;
    }

    if (rebuild_meshes) {
        center_mesh.upload(make_curved_tile_mesh(patch, settings.radial_bands, settings.edge_segments));
        grid_mesh.upload(make_grid_mesh(patch, settings.edge_segments));
    }

    return rebuild_meshes;
}

void process_input(GLFWwindow* window, CameraState& camera, const tiling::TilingPatch& patch, float dt) {
    if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_MINUS) == GLFW_PRESS) {
        camera.move_speed = glm::max(camera.move_speed - 1.5F * dt, kMinMoveSpeed);
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_EQUAL) == GLFW_PRESS) {
        camera.move_speed = glm::min(camera.move_speed + 1.5F * dt, kMaxMoveSpeed);
    }

    const double move_speed = static_cast<double>(camera.move_speed) * static_cast<double>(dt);
    math::Vec2 local_delta{};

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        local_delta.y -= move_speed;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        local_delta.y += move_speed;
    }
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        local_delta.x += move_speed;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        local_delta.x -= move_speed;
    }

    if (local_delta.x != 0.0 || local_delta.y != 0.0) {
        math::CameraFrame moved_frame = geo_move_frame(camera.frame, local_delta, patch.mode);
        int moved_tile_id = camera.current_tile_id;

        if (tiling::rebase_frame_across_edges(patch, moved_tile_id, moved_frame)) {
            camera.frame = moved_frame;
            camera.current_tile_id = moved_tile_id;
        }
    }

    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        camera.zoom = glm::min(camera.zoom + dt, kMaxViewZoom);
    }
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        camera.zoom = glm::max(camera.zoom - dt, kMinViewZoom);
    }
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        camera.eye_height = glm::min(camera.eye_height + kEyeLiftSpeed * dt, kMaxEyeHeight);
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS) {
        camera.eye_height = glm::max(camera.eye_height - kEyeLiftSpeed * dt, kMinEyeHeight);
    }

    if (g_cursor_captured) {
        double mx = 0, my = 0;
        glfwGetCursorPos(window, &mx, &my);
        if (g_first_mouse) {
            g_last_mouse_x = mx;
            g_last_mouse_y = my;
            g_first_mouse = false;
        }
        float dx = static_cast<float>(mx - g_last_mouse_x);
        float dy = static_cast<float>(my - g_last_mouse_y);
        g_last_mouse_x = mx;
        g_last_mouse_y = my;
        
        if (dx != 0.0f || dy != 0.0f) {
            float sensitivity = 0.002f;
            float yaw = dx * sensitivity;
            camera.pitch = glm::clamp(camera.pitch + dy * sensitivity, -1.35F, 1.20F);
            
            math::Vec3 f = camera.frame.forward;
            math::Vec3 r = camera.frame.right;
            camera.frame.forward = math::Vec3{
                f.t * std::cos(yaw) + r.t * std::sin(yaw),
                f.x * std::cos(yaw) + r.x * std::sin(yaw),
                f.y * std::cos(yaw) + r.y * std::sin(yaw)
            };
            camera.frame.right = math::Vec3{
                f.t * -std::sin(yaw) + r.t * std::cos(yaw),
                f.x * -std::sin(yaw) + r.x * std::cos(yaw),
                f.y * -std::sin(yaw) + r.y * std::cos(yaw)
            };
            
            camera.frame = geo_orthonormalize(camera.frame, patch.mode);
        }
    }
}

MeshData make_curved_tile_mesh(const tiling::TilingPatch& patch, int radial_bands, int edge_segments) {
    MeshData mesh;
    edge_segments = std::max(1, edge_segments);
    radial_bands = std::max(1, radial_bands);

    const glm::vec3 floor_normal{0.0F, 1.0F, 0.0F};

    for (const tiling::Tile& tile : patch.tiles) {
        const float depth_factor = static_cast<float>(tile.depth) / 4.0F;
        const glm::vec3 color{
            tile.depth == 0 ? 0.82F : 0.30F + 0.08F * depth_factor,
            tile.depth == 0 ? 0.76F : 0.62F - 0.05F * depth_factor,
            tile.depth == 0 ? 0.38F : 0.78F,
        };

        const unsigned int center_idx = static_cast<unsigned int>(mesh.vertices.size());
        append_vertex(mesh, h2_point_to_h3(tile.center), floor_normal, color);

        std::vector<math::Vec3> boundary_ring;
        boundary_ring.reserve(patch.base_polygon_vertices.size() * static_cast<size_t>(edge_segments));

        for (size_t i = 0; i < patch.base_polygon_vertices.size(); ++i) {
            const size_t next_i = (i + 1) % patch.base_polygon_vertices.size();
            const math::Vec3 a = math::apply_isometry(tile.transform, patch.base_polygon_vertices[i]);
            const math::Vec3 b = math::apply_isometry(tile.transform, patch.base_polygon_vertices[next_i]);

            for (int s = 0; s < edge_segments; ++s) {
                boundary_ring.push_back(
                    geo_geodesic_lerp(a, b, static_cast<double>(s) / edge_segments, patch.mode));
            }
        }

        const int rc = static_cast<int>(boundary_ring.size());

        for (int band = 1; band <= radial_bands; ++band) {
            double t = static_cast<double>(band) / radial_bands;
            for (const math::Vec3& boundary_pos : boundary_ring) {
                math::Vec3 hv = geo_geodesic_lerp(tile.center, boundary_pos, t, patch.mode);
                append_vertex(mesh, h2_point_to_h3(hv), floor_normal, color);
            }
        }

        for (int i = 0; i < rc; ++i) {
            mesh.indices.push_back(center_idx);
            mesh.indices.push_back(center_idx + 1 + static_cast<unsigned int>(i));
            mesh.indices.push_back(center_idx + 1 + static_cast<unsigned int>((i + 1) % rc));
        }

        for (int band = 1; band < radial_bands; ++band) {
            unsigned int inner_start = center_idx + 1 + static_cast<unsigned int>((band - 1) * rc);
            unsigned int outer_start = center_idx + 1 + static_cast<unsigned int>(band * rc);
            for (int i = 0; i < rc; ++i) {
                unsigned int inner0 = inner_start + static_cast<unsigned int>(i);
                unsigned int inner1 = inner_start + static_cast<unsigned int>((i + 1) % rc);
                unsigned int outer0 = outer_start + static_cast<unsigned int>(i);
                unsigned int outer1 = outer_start + static_cast<unsigned int>((i + 1) % rc);

                mesh.indices.push_back(inner0);
                mesh.indices.push_back(outer0);
                mesh.indices.push_back(outer1);

                mesh.indices.push_back(inner0);
                mesh.indices.push_back(outer1);
                mesh.indices.push_back(inner1);
            }
        }
    }

    return mesh;
}

MeshData make_grid_mesh(const tiling::TilingPatch& patch, int edge_segments) {
    MeshData mesh;
    edge_segments = std::max(1, edge_segments);
    const glm::vec3 grid_color{0.9F, 0.9F, 0.9F};

    for (const tiling::Tile& tile : patch.tiles) {
        for (size_t i = 0; i < patch.base_polygon_vertices.size(); ++i) {
            const size_t next_i = (i + 1) % patch.base_polygon_vertices.size();
            const math::Vec3 a = math::apply_isometry(tile.transform, patch.base_polygon_vertices[i]);
            const math::Vec3 b = math::apply_isometry(tile.transform, patch.base_polygon_vertices[next_i]);

            unsigned int base = static_cast<unsigned int>(mesh.vertices.size());
            for (int s = 0; s <= edge_segments; ++s) {
                math::Vec3 p = geo_geodesic_lerp(a, b, static_cast<double>(s) / edge_segments, patch.mode);
                append_vertex(mesh, h2_point_to_h3(p), glm::vec3{0.0F, 1.0F, 0.0F}, grid_color);
                if (s > 0) {
                    mesh.indices.push_back(base + s - 1);
                    mesh.indices.push_back(base + s);
                }
            }
        }
    }

    return mesh;
}

glm::vec4 h2_point_to_h3(const math::Vec3& point) {
    return glm::vec4{
        static_cast<float>(point.y),
        0.0F,
        static_cast<float>(point.x),
        static_cast<float>(point.t),
    };
}

glm::vec4 h2_tangent_to_h3(const math::Vec3& vector) {
    return glm::vec4{
        static_cast<float>(vector.y),
        0.0F,
        static_cast<float>(vector.x),
        static_cast<float>(vector.t),
    };
}

void append_vertex(MeshData& mesh,
                   const glm::vec4& position,
                   const glm::vec3& normal,
                   const glm::vec3& color) {
    mesh.vertices.push_back(Vertex{position, normal, color});
}

glm::mat4 h3_lorentz_boost(const glm::vec3& direction, float distance) {
    const float length = glm::length(direction);
    if (length <= 1.0e-6F || std::abs(distance) <= 1.0e-6F) {
        return glm::mat4{1.0F};
    }

    const glm::vec3 n = direction / length;
    const float c = std::cosh(distance);
    const float s = std::sinh(distance);
    glm::mat4 matrix{1.0F};

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            matrix[col][row] = (row == col ? 1.0F : 0.0F) + (c - 1.0F) * n[row] * n[col];
        }
    }

    for (int i = 0; i < 3; ++i) {
        matrix[3][i] = s * n[i];
        matrix[i][3] = s * n[i];
    }
    matrix[3][3] = c;
    return matrix;
}

// Spherical (SO(4)) counterpart of h3_lorentz_boost: rotates in the plane
// spanned by the chosen direction and the w (time) axis.
glm::mat4 h3_spherical_rotation(const glm::vec3& direction, float distance) {
    const float length = glm::length(direction);
    if (length <= 1.0e-6F || std::abs(distance) <= 1.0e-6F) {
        return glm::mat4{1.0F};
    }

    const glm::vec3 n = direction / length;
    const float c = std::cos(distance);
    const float s = std::sin(distance);
    glm::mat4 matrix{1.0F};

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            matrix[col][row] = (row == col ? 1.0F : 0.0F) + (c - 1.0F) * n[row] * n[col];
        }
    }

    for (int i = 0; i < 3; ++i) {
        matrix[3][i] = s * n[i];
        matrix[i][3] = -s * n[i];
    }
    matrix[3][3] = c;
    return matrix;
}

glm::mat4 h3_rotation_xz(float angle) {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    glm::mat4 matrix{1.0F};
    matrix[0][0] = c;
    matrix[2][0] = -s;
    matrix[0][2] = s;
    matrix[2][2] = c;
    return matrix;
}

glm::mat4 h3_rotation_yz(float angle) {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    glm::mat4 matrix{1.0F};
    matrix[1][1] = c;
    matrix[2][1] = -s;
    matrix[1][2] = s;
    matrix[2][2] = c;
    return matrix;
}

glm::mat4 h3_lorentz_inverse(const glm::mat4& matrix) {
    glm::mat4 transposed = glm::transpose(matrix);
    glm::mat4 inverse = transposed;
    for (int i = 0; i < 3; ++i) {
        inverse[3][i] = -transposed[3][i];
        inverse[i][3] = -transposed[i][3];
    }
    return inverse;
}

glm::mat4 h3_frame_from_h2_frame(const math::CameraFrame& frame) {
    glm::mat4 matrix{1.0F};
    matrix[0] = h2_tangent_to_h3(frame.right);
    matrix[1] = glm::vec4{0.0F, 1.0F, 0.0F, 0.0F};
    matrix[2] = h2_tangent_to_h3(frame.forward);
    matrix[3] = h2_point_to_h3(frame.position);
    return matrix;
}

struct LocalPropPoint {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

glm::mat4 h3_vertical_motion(float distance, math::GeometryMode mode) {
    return geo_is_spherical(mode)
               ? h3_spherical_rotation(glm::vec3{0.0F, 1.0F, 0.0F}, distance)
               : h3_lorentz_boost(glm::vec3{0.0F, 1.0F, 0.0F}, distance);
}

glm::vec4 embed_local_prop_point(const LocalPropPoint& p, math::GeometryMode mode) {
    glm::mat4 transform{1.0F};
    transform = transform * (geo_is_spherical(mode)
                                 ? h3_spherical_rotation(glm::vec3{1.0F, 0.0F, 0.0F}, p.x)
                                 : h3_lorentz_boost(glm::vec3{1.0F, 0.0F, 0.0F}, p.x));
    transform = transform * (geo_is_spherical(mode)
                                 ? h3_spherical_rotation(glm::vec3{0.0F, 0.0F, 1.0F}, p.z)
                                 : h3_lorentz_boost(glm::vec3{0.0F, 0.0F, 1.0F}, p.z));
    transform = transform * h3_vertical_motion(p.y, mode);
    return transform * glm::vec4{0.0F, 0.0F, 0.0F, 1.0F};
}

glm::vec3 local_face_normal(const LocalPropPoint& a,
                            const LocalPropPoint& b,
                            const LocalPropPoint& c) {
    const glm::vec3 ab{b.x - a.x, b.y - a.y, b.z - a.z};
    const glm::vec3 ac{c.x - a.x, c.y - a.y, c.z - a.z};
    const glm::vec3 normal = glm::cross(ab, ac);
    const float length = glm::length(normal);
    if (length <= 1.0e-6F) {
        return glm::vec3{0.0F, 1.0F, 0.0F};
    }
    return normal / length;
}

glm::vec3 transformed_normal(const glm::mat4& model, const glm::vec3& normal) {
    const glm::vec3 transformed = glm::mat3(model) * normal;
    const float length = glm::length(transformed);
    if (length <= 1.0e-6F) {
        return glm::vec3{0.0F, 1.0F, 0.0F};
    }
    return transformed / length;
}

void append_prop_triangle(MeshData& mesh,
                          const glm::mat4& model,
                          const LocalPropPoint& a,
                          const LocalPropPoint& b,
                          const LocalPropPoint& c,
                          const glm::vec3& color,
                          math::GeometryMode mode) {
    const unsigned int base = static_cast<unsigned int>(mesh.vertices.size());
    const glm::vec3 normal = transformed_normal(model, local_face_normal(a, b, c));

    append_vertex(mesh, model * embed_local_prop_point(a, mode), normal, color);
    append_vertex(mesh, model * embed_local_prop_point(b, mode), normal, color);
    append_vertex(mesh, model * embed_local_prop_point(c, mode), normal, color);

    mesh.indices.push_back(base + 0U);
    mesh.indices.push_back(base + 1U);
    mesh.indices.push_back(base + 2U);
}

void append_prop_quad(MeshData& mesh,
                      const glm::mat4& model,
                      const LocalPropPoint& a,
                      const LocalPropPoint& b,
                      const LocalPropPoint& c,
                      const LocalPropPoint& d,
                      const glm::vec3& color,
                      math::GeometryMode mode) {
    append_prop_triangle(mesh, model, a, b, c, color, mode);
    append_prop_triangle(mesh, model, a, c, d, color, mode);
}

void append_prop_line(MeshData& mesh,
                      const glm::mat4& model,
                      const LocalPropPoint& a,
                      const LocalPropPoint& b,
                      const glm::vec3& color,
                      math::GeometryMode mode) {
    const unsigned int base = static_cast<unsigned int>(mesh.vertices.size());
    const glm::vec3 normal = transformed_normal(model, glm::vec3{0.0F, 1.0F, 0.0F});

    append_vertex(mesh, model * embed_local_prop_point(a, mode), normal, color);
    append_vertex(mesh, model * embed_local_prop_point(b, mode), normal, color);

    mesh.indices.push_back(base + 0U);
    mesh.indices.push_back(base + 1U);
}

void append_house(MeshData& solid,
                  MeshData& outline,
                  const glm::mat4& model,
                  const glm::vec3& tint,
                  math::GeometryMode mode) {
    const glm::vec3 wall_color = tint * glm::vec3{0.86F, 0.82F, 0.70F};
    const glm::vec3 roof_color = tint * glm::vec3{0.90F, 0.34F, 0.22F};
    const glm::vec3 trim_color = tint * glm::vec3{0.26F, 0.20F, 0.17F};
    const glm::vec3 door_color = tint * glm::vec3{0.16F, 0.27F, 0.42F};
    const glm::vec3 chimney_color = tint * glm::vec3{0.32F, 0.25F, 0.23F};
    const glm::vec3 line_color{0.06F, 0.07F, 0.07F};

    constexpr float w = 0.16F;
    constexpr float d = 0.12F;
    constexpr float h = 0.18F;
    constexpr float peak = 0.32F;
    constexpr float door_left = -0.065F;
    constexpr float door_right = 0.018F;
    constexpr float door_h = 0.11F;
    constexpr float chimney_w0 = -0.070F;
    constexpr float chimney_w1 = -0.025F;
    constexpr float chimney_z0 = -0.012F;
    constexpr float chimney_z1 = 0.034F;
    constexpr float chimney_y0 = 0.22F;
    constexpr float chimney_y1 = 0.34F;

    const LocalPropPoint fbl{-w, 0.0F, d};
    const LocalPropPoint fbr{w, 0.0F, d};
    const LocalPropPoint ftr{w, h, d};
    const LocalPropPoint ftl{-w, h, d};
    const LocalPropPoint frp{0.0F, peak, d};
    const LocalPropPoint dbl{door_left, 0.0F, d};
    const LocalPropPoint dbr{door_right, 0.0F, d};
    const LocalPropPoint dtr{door_right, door_h, d};
    const LocalPropPoint dtl{door_left, door_h, d};

    const LocalPropPoint bbl{-w, 0.0F, -d};
    const LocalPropPoint bbr{w, 0.0F, -d};
    const LocalPropPoint btr{w, h, -d};
    const LocalPropPoint btl{-w, h, -d};
    const LocalPropPoint brp{0.0F, peak, -d};

    append_prop_quad(solid, model, fbl, dbl, dtl, ftl, wall_color, mode);
    append_prop_quad(solid, model, dbr, fbr, ftr, dtr, wall_color, mode);
    append_prop_quad(solid, model, dtl, dtr, ftr, ftl, wall_color, mode);
    append_prop_triangle(solid, model, ftl, ftr, frp, roof_color, mode);
    append_prop_quad(solid, model, bbr, bbl, btl, btr, wall_color, mode);
    append_prop_triangle(solid, model, btr, btl, brp, roof_color, mode);

    append_prop_quad(solid, model, bbl, fbl, ftl, btl, wall_color, mode);
    append_prop_quad(solid, model, fbr, bbr, btr, ftr, wall_color, mode);
    append_prop_quad(solid, model, ftl, frp, brp, btl, roof_color, mode);
    append_prop_quad(solid, model, frp, ftr, btr, brp, roof_color, mode);
    append_prop_quad(solid, model, bbl, bbr, fbr, fbl, trim_color, mode);
    append_prop_quad(solid, model, dbl, dbr, dtr, dtl, door_color, mode);

    const LocalPropPoint chimney_fbl{chimney_w0, chimney_y0, chimney_z1};
    const LocalPropPoint chimney_fbr{chimney_w1, chimney_y0, chimney_z1};
    const LocalPropPoint chimney_ftr{chimney_w1, chimney_y1, chimney_z1};
    const LocalPropPoint chimney_ftl{chimney_w0, chimney_y1, chimney_z1};
    const LocalPropPoint chimney_bbl{chimney_w0, chimney_y0, chimney_z0};
    const LocalPropPoint chimney_bbr{chimney_w1, chimney_y0, chimney_z0};
    const LocalPropPoint chimney_btr{chimney_w1, chimney_y1, chimney_z0};
    const LocalPropPoint chimney_btl{chimney_w0, chimney_y1, chimney_z0};

    append_prop_quad(solid, model, chimney_fbl, chimney_fbr, chimney_ftr, chimney_ftl, chimney_color, mode);
    append_prop_quad(solid, model, chimney_bbr, chimney_bbl, chimney_btl, chimney_btr, chimney_color, mode);
    append_prop_quad(solid, model, chimney_bbl, chimney_fbl, chimney_ftl, chimney_btl, chimney_color, mode);
    append_prop_quad(solid, model, chimney_fbr, chimney_bbr, chimney_btr, chimney_ftr, chimney_color, mode);
    append_prop_quad(solid, model, chimney_ftl, chimney_ftr, chimney_btr, chimney_btl, chimney_color, mode);

    append_prop_line(outline, model, fbl, fbr, line_color, mode);
    append_prop_line(outline, model, fbr, ftr, line_color, mode);
    append_prop_line(outline, model, ftr, frp, line_color, mode);
    append_prop_line(outline, model, frp, ftl, line_color, mode);
    append_prop_line(outline, model, ftl, fbl, line_color, mode);
    append_prop_line(outline, model, bbl, bbr, line_color, mode);
    append_prop_line(outline, model, bbr, btr, line_color, mode);
    append_prop_line(outline, model, btr, brp, line_color, mode);
    append_prop_line(outline, model, brp, btl, line_color, mode);
    append_prop_line(outline, model, btl, bbl, line_color, mode);
    append_prop_line(outline, model, fbl, bbl, line_color, mode);
    append_prop_line(outline, model, fbr, bbr, line_color, mode);
    append_prop_line(outline, model, ftl, btl, line_color, mode);
    append_prop_line(outline, model, ftr, btr, line_color, mode);
    append_prop_line(outline, model, frp, brp, line_color, mode);
    append_prop_line(outline, model, dbl, dbr, line_color, mode);
    append_prop_line(outline, model, dbr, dtr, line_color, mode);
    append_prop_line(outline, model, dtr, dtl, line_color, mode);
    append_prop_line(outline, model, dtl, dbl, line_color, mode);
    append_prop_line(outline, model, chimney_fbl, chimney_fbr, line_color, mode);
    append_prop_line(outline, model, chimney_fbr, chimney_ftr, line_color, mode);
    append_prop_line(outline, model, chimney_ftr, chimney_ftl, line_color, mode);
    append_prop_line(outline, model, chimney_ftl, chimney_fbl, line_color, mode);
    append_prop_line(outline, model, chimney_ftl, chimney_btl, line_color, mode);
    append_prop_line(outline, model, chimney_ftr, chimney_btr, line_color, mode);
    append_prop_line(outline, model, chimney_btr, chimney_btl, line_color, mode);
    append_prop_line(outline, model, chimney_btl, chimney_ftl, line_color, mode);
}

glm::vec3 tint_for_prop(std::size_t index) {
    constexpr std::array<glm::vec3, 6> kTints{{
        glm::vec3{1.00F, 0.96F, 0.92F},
        glm::vec3{0.92F, 1.00F, 0.95F},
        glm::vec3{0.90F, 0.96F, 1.00F},
        glm::vec3{1.00F, 0.93F, 0.98F},
        glm::vec3{0.98F, 0.96F, 0.86F},
        glm::vec3{0.94F, 0.92F, 1.00F},
    }};
    return kTints[index % kTints.size()];
}

float yaw_toward_origin(const tiling::Tile& tile, math::GeometryMode mode) {
    const math::Vec2 projected = geo_project_conformal_disk(tile.center, mode);
    if (!finite_disk_point(projected)) {
        return 0.0F;
    }

    const glm::vec2 to_origin{
        static_cast<float>(-projected.y),
        static_cast<float>(-projected.x),
    };
    if (glm::length(to_origin) <= 1.0e-6F) {
        return 0.0F;
    }

    return std::atan2(-to_origin.x, to_origin.y);
}

glm::mat4 tile_prop_model(const tiling::Tile& tile, math::GeometryMode mode, float yaw) {
    const math::CameraFrame frame = tiling::global_frame_from_tile(math::canonical_frame(), tile, mode);
    return h3_frame_from_h2_frame(frame) * h3_rotation_xz(yaw);
}

HousePropMeshes make_house_prop_meshes(const tiling::TilingPatch& patch) {
    HousePropMeshes meshes;
    const std::vector<int> tile_ids = choose_showcase_tiles(patch, 8);
    const std::vector<int> laser_tile_ids = choose_laser_anchor_tiles(patch);

    for (std::size_t i = 0; i < tile_ids.size(); ++i) {
        const int tile_id = tile_ids[i];
        if (tile_id < 0 || tile_id >= static_cast<int>(patch.tiles.size())) {
            continue;
        }
        if (has_tile_id(laser_tile_ids, tile_id)) {
            continue;
        }

        const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(tile_id)];
        const float yaw = yaw_toward_origin(tile, patch.mode) + 0.16F * static_cast<float>(i % 3);
        append_house(meshes.solid,
                     meshes.outline,
                     tile_prop_model(tile, patch.mode, yaw),
                     tint_for_prop(i),
                     patch.mode);
    }

    return meshes;
}

void append_laser_line_vertex(MeshData& mesh,
                              const math::Vec3& point,
                              const glm::vec3& color,
                              math::GeometryMode) {
    append_vertex(mesh, h2_point_to_h3(point), glm::vec3{0.0F, 1.0F, 0.0F}, color);
}

void append_laser_path(MeshData& mesh, const LaserPath& path, math::GeometryMode mode) {
    const glm::vec3 core_color = glm::mix(path.color, glm::vec3{1.0F, 1.0F, 1.0F}, 0.68F);
    const unsigned int base = static_cast<unsigned int>(mesh.vertices.size());
    append_laser_line_vertex(mesh, path.start, core_color, mode);
    append_laser_line_vertex(mesh, path.end, core_color, mode);
    mesh.indices.push_back(base);
    mesh.indices.push_back(base + 1U);
}

MeshData make_laser_prop_mesh(const tiling::TilingPatch& patch) {
    MeshData mesh;
    const std::vector<LaserPath> paths = make_laser_paths(patch);

    for (const LaserPath& path : paths) {
        append_laser_path(mesh, path, patch.mode);
    }

    return mesh;
}

math::CameraFrame global_camera_frame(const CameraState& camera, const tiling::TilingPatch& patch) {
    const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(camera.current_tile_id)];
    return tiling::global_frame_from_tile(camera.frame, tile, patch.mode);
}

glm::mat4 euclidean_projection(int width, int height, float zoom) {
    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0F;
    const float clamped_zoom = glm::clamp(zoom, kMinViewZoom, kMaxViewZoom);
    const float fov_degrees = glm::clamp(kBaseFovDegrees / clamped_zoom, kMinFovDegrees, kMaxFovDegrees);
    return glm::perspective(glm::radians(fov_degrees), aspect, 0.01F, 8.0F);
}

glm::mat4 lorentz_view(const CameraState& camera, const tiling::TilingPatch& patch) {
    const math::CameraFrame global_frame = global_camera_frame(camera, patch);
    const glm::mat4 floor_frame = h3_frame_from_h2_frame(global_frame);

    // Lifting the eye off the floor is a radial isometry along the embedded up
    // axis: a Lorentz boost in hyperbolic space, an SO(4) rotation on the sphere.
    const glm::mat4 eye_lift = geo_is_spherical(patch.mode)
                                   ? h3_spherical_rotation(glm::vec3{0.0F, 1.0F, 0.0F}, camera.eye_height)
                                   : h3_lorentz_boost(glm::vec3{0.0F, 1.0F, 0.0F}, camera.eye_height);
    const glm::mat4 eye_frame = floor_frame * eye_lift * h3_rotation_yz(camera.pitch);

    // The view is the inverse of the eye frame. Spherical eye frames live in
    // SO(4), so their inverse is the plain transpose.
    return geo_is_spherical(patch.mode) ? glm::transpose(eye_frame) : h3_lorentz_inverse(eye_frame);
}

void update_window_title(GLFWwindow* window, const CameraState& camera, const tiling::TilingPatch& patch) {
    const math::CameraFrame global_frame = global_camera_frame(camera, patch);
    const double origin_distance = geo_distance(math::origin(), global_frame.position, patch.mode);
    const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(camera.current_tile_id)];

    std::ostringstream title;
    title << std::fixed << std::setprecision(2)
          << "Hyperbolica | " << math::geometry_mode_name(patch.mode) << " {" << patch.parameters.p << ","
          << patch.parameters.q << "}"
          << " | tile " << camera.current_tile_id
          << " depth " << tile.depth
          << " | d(origin) " << origin_distance
          << " | speed " << camera.move_speed
          << " | zoom " << camera.zoom
          << " | height " << camera.eye_height
          << " | tiles " << patch.tiles.size();
    glfwSetWindowTitle(window, title.str().c_str());
}

} // namespace

void App::run() {
    GlfwContext glfw;

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#if defined(__APPLE__)
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
#endif

    GLFWwindow* window = glfwCreateWindow(960, 540, "Hyperbolica", nullptr, nullptr);
    if (window == nullptr) {
        throw std::runtime_error("failed to create GLFW window");
    }

    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetKeyCallback(window, key_callback);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glfwSwapInterval(1);

    if (gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress)) == 0) {
        glfwDestroyWindow(window);
        throw std::runtime_error("failed to initialize GLAD");
    }

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    ShaderProgram shader =
        ShaderProgram::from_files(resolve_shader_path("hyperbolic.vert"), resolve_shader_path("hyperbolic.frag"));
    DebugOverlay debug_overlay;
    debug_overlay.init();

    RenderSettings settings;
    DebugInputState debug_input;
    CameraState camera;
    
    settings.tiling_depth = geo_tiling_depth(settings.geometry_mode);
    tiling::TilingPatch patch = tiling::generate_tiling_patch(settings.tiling_parameters, settings.tiling_depth,
                                                              1.0e-6, settings.geometry_mode);
    Mesh center_mesh;
    center_mesh.upload(make_curved_tile_mesh(patch, settings.radial_bands, settings.edge_segments));
    Mesh grid_mesh;
    grid_mesh.upload(make_grid_mesh(patch, settings.edge_segments));
    const HousePropMeshes house_meshes = make_house_prop_meshes(patch);
    Mesh house_solid_mesh;
    upload_optional_mesh(house_solid_mesh, house_meshes.solid);
    Mesh house_outline_mesh;
    upload_optional_mesh(house_outline_mesh, house_meshes.outline);
    Mesh laser_mesh;
    upload_optional_mesh(laser_mesh, make_laser_prop_mesh(patch));

    update_window_title(window, camera, patch);
    float title_update_accumulator = 0.0F;
    auto previous_time = std::chrono::steady_clock::now();

    while (glfwWindowShouldClose(window) == GLFW_FALSE) {
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - previous_time).count();
        previous_time = now;

        glfwPollEvents();
        process_input(window, camera, patch, dt);
        process_debug_input(window,
                            settings,
                            debug_input,
                            patch,
                            center_mesh,
                            grid_mesh,
                            house_solid_mesh,
                            house_outline_mesh,
                            laser_mesh,
                            camera);
        title_update_accumulator += dt;
        if (title_update_accumulator >= 0.25F) {
            update_window_title(window, camera, patch);
            title_update_accumulator = 0.0F;
        }

        glfwGetFramebufferSize(window, &width, &height);

        const glm::vec3 sky_color = active_sky_color(settings);
        glClearColor(sky_color.x, sky_color.y, sky_color.z, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        shader.use();
        shader.set_mat4("uEuclideanProj", euclidean_projection(width, height, camera.zoom));
        
        glm::mat4 lv = lorentz_view(camera, patch);
        glUniformMatrix4fv(glGetUniformLocation(shader.id(), "uLorentzView"), 1, GL_FALSE, &lv[0][0]);
        glUniform1i(glGetUniformLocation(shader.id(), "uProjectionModel"), 1);
        glUniform3fv(glGetUniformLocation(shader.id(), "uAtmosphereColor"), 1, &settings.atmosphere_color[0]);
        const glm::vec3 light_dir = glm::normalize(glm::vec3{0.85F, 1.20F, 0.45F});
        glUniform3fv(glGetUniformLocation(shader.id(), "uLightDir"), 1, &light_dir[0]);
        glUniform1i(glGetUniformLocation(shader.id(), "uGeometryMode"),
                    static_cast<int>(patch.mode));

        // Draws the tile interiors and the grid overlay with the current
        // spherical-pass uniform already bound.
        const GLint depth_bias_loc = glGetUniformLocation(shader.id(), "uDepthBias");
        auto draw_world = [&]() {
            glPolygonMode(GL_FRONT_AND_BACK, settings.show_wireframe ? GL_LINE : GL_FILL);
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(1.0f, 1.0f);
            glUniform1f(depth_bias_loc, 0.0f);
            center_mesh.draw();
            if (settings.show_houses) {
                house_solid_mesh.draw();
            }
            glDisable(GL_POLYGON_OFFSET_FILL);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            if (settings.show_grid) {
                // In spherical mode the fragment shader writes depth from the
                // geodesic distance, so the polygon offset above no longer
                // separates fill from lines; a small depth bias pulls the grid
                // just in front of the tile fill instead.
                glUniform1f(depth_bias_loc, 0.0015f);
                grid_mesh.draw_lines();
                glUniform1f(depth_bias_loc, 0.0f);
            }
            if (settings.show_houses) {
                glUniform1f(depth_bias_loc, 0.0008f);
                glLineWidth(1.5f);
                house_outline_mesh.draw_lines();
                glLineWidth(1.0f);
                glUniform1f(depth_bias_loc, 0.0f);
            }
            if (settings.show_lasers) {
                glUniform1f(depth_bias_loc, 0.0012f);
                glLineWidth(4.0f);
                laser_mesh.draw_lines();
                glLineWidth(1.0f);
                glUniform1f(depth_bias_loc, 0.0f);
            }
        };

        if (geo_is_spherical(patch.mode)) {
            // Stereographic projection blows up at the antipode, so the sphere
            // is drawn in two clipped passes: the near hemisphere projected from
            // one pole and the far hemisphere from the other. gl_ClipDistance
            // discards the wrong hemisphere in each pass. Occlusion is handled
            // entirely in the fragment shader, which writes gl_FragDepth from the
            // true geodesic distance to the camera (acos(w)); that ordering is
            // correct front-to-back by construction, so no depth-range tricks are
            // needed and the two hemispheres compose seamlessly.
            glEnable(GL_CLIP_DISTANCE0);

            glUniform1i(glGetUniformLocation(shader.id(), "uSphericalPass"), 1);
            draw_world();

            glUniform1i(glGetUniformLocation(shader.id(), "uSphericalPass"), 2);
            draw_world();

            glDisable(GL_CLIP_DISTANCE0);
        } else {
            glUniform1i(glGetUniformLocation(shader.id(), "uSphericalPass"), 0);
            draw_world();
        }

        debug_overlay.draw(width, height, settings, camera, patch);

        glfwSwapBuffers(window);
    }

    debug_overlay.shutdown();
    glfwDestroyWindow(window);
}

} // namespace hyper
