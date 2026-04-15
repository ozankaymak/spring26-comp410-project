#include "app.h"

#include "math/hyperbolic.h"
#include "mesh.h"
#include "shader.h"
#include "tiling_core.h"

#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace hyper {
namespace {

constexpr int kProgressDemoTilingDepth = 5;
constexpr float kMinMoveSpeed = 0.25F;
constexpr float kMaxMoveSpeed = 6.0F;

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

struct CameraState {
    math::CameraFrame frame = math::canonical_frame();
    float zoom = 1.0F;
    float move_speed = 2.5F;
    int current_tile_id = 0;
};

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

void process_input(GLFWwindow* window, CameraState& camera, const tiling::TilingPatch& patch, float dt) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }

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
        math::CameraFrame moved_frame = math::move_frame(camera.frame, local_delta);
        int moved_tile_id = camera.current_tile_id;

        if (tiling::rebase_frame_across_edges(patch, moved_tile_id, moved_frame)) {
            camera.frame = moved_frame;
            camera.current_tile_id = moved_tile_id;
        }
    }

    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        camera.zoom = glm::min(camera.zoom + dt, 3.0F);
    }
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        camera.zoom = glm::max(camera.zoom - dt, 0.25F);
    }
}

MeshData make_tile_center_mesh(const tiling::TilingPatch& patch) {
    MeshData mesh;
    const float marker_radius = 0.012F;

    for (const tiling::Tile& tile : patch.tiles) {
        const math::Vec2 projected = math::project_to_poincare_disk(tile.center);
        const glm::vec3 center{static_cast<float>(projected.x), static_cast<float>(projected.y), 0.0F};
        const float depth_factor = static_cast<float>(tile.depth) / 4.0F;
        const glm::vec3 color{
            tile.depth == 0 ? 0.95F : 0.25F + 0.10F * depth_factor,
            tile.depth == 0 ? 0.90F : 0.75F - 0.08F * depth_factor,
            tile.depth == 0 ? 0.25F : 0.95F,
        };

        const unsigned int base = static_cast<unsigned int>(mesh.vertices.size());
        mesh.vertices.push_back(Vertex{center + glm::vec3{-marker_radius, -marker_radius, 0.0F}, color});
        mesh.vertices.push_back(Vertex{center + glm::vec3{marker_radius, -marker_radius, 0.0F}, color});
        mesh.vertices.push_back(Vertex{center + glm::vec3{marker_radius, marker_radius, 0.0F}, color});
        mesh.vertices.push_back(Vertex{center + glm::vec3{-marker_radius, marker_radius, 0.0F}, color});

        mesh.indices.push_back(base + 0U);
        mesh.indices.push_back(base + 1U);
        mesh.indices.push_back(base + 2U);
        mesh.indices.push_back(base + 0U);
        mesh.indices.push_back(base + 2U);
        mesh.indices.push_back(base + 3U);
    }

    return mesh;
}

MeshData make_grid_mesh(const tiling::TilingPatch& patch) {
    MeshData mesh;
    const glm::vec3 grid_color{0.5F, 0.5F, 0.5F};

    for (const tiling::Tile& tile : patch.tiles) {
        // Transform base polygon vertices to this tile's position
        for (size_t i = 0; i < patch.base_polygon_vertices.size(); ++i) {
            const math::Vec3 local_vertex = math::apply_isometry(tile.transform, patch.base_polygon_vertices[i]);
            const math::Vec2 projected = math::project_to_poincare_disk(local_vertex);
            const glm::vec3 pos{static_cast<float>(projected.x), static_cast<float>(projected.y), 0.0F};
            mesh.vertices.push_back(Vertex{pos, grid_color});

            const size_t next_i = (i + 1) % patch.base_polygon_vertices.size();
            const math::Vec3 next_local_vertex = math::apply_isometry(tile.transform, patch.base_polygon_vertices[next_i]);
            const math::Vec2 next_projected = math::project_to_poincare_disk(next_local_vertex);
            const glm::vec3 next_pos{static_cast<float>(next_projected.x), static_cast<float>(next_projected.y), 0.0F};
            mesh.vertices.push_back(Vertex{next_pos, grid_color});

            const unsigned int base = static_cast<unsigned int>(mesh.vertices.size() - 2);
            mesh.indices.push_back(base);
            mesh.indices.push_back(base + 1);
        }
    }

    return mesh;
}

math::CameraFrame global_camera_frame(const CameraState& camera, const tiling::TilingPatch& patch) {
    const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(camera.current_tile_id)];
    return tiling::global_frame_from_tile(camera.frame, tile);
}

glm::mat4 camera_matrix(const CameraState& camera, const tiling::TilingPatch& patch, int width, int height) {
    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0F;
    const math::Vec2 projected_position = math::project_to_poincare_disk(global_camera_frame(camera, patch).position);
    const glm::vec2 center{static_cast<float>(projected_position.x), static_cast<float>(projected_position.y)};
    const glm::mat4 projection =
        glm::ortho(-aspect * camera.zoom, aspect * camera.zoom, -camera.zoom, camera.zoom, -1.0F, 1.0F);
    const glm::mat4 view = glm::translate(glm::mat4{1.0F}, glm::vec3{-center.x, -center.y, 0.0F});
    return projection * view;
}

void update_window_title(GLFWwindow* window, const CameraState& camera, const tiling::TilingPatch& patch) {
    const math::CameraFrame global_frame = global_camera_frame(camera, patch);
    const double origin_distance = math::intrinsic_distance(math::origin(), global_frame.position);
    const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(camera.current_tile_id)];

    std::ostringstream title;
    title << std::fixed << std::setprecision(2)
          << "Hyperbolica | tile " << camera.current_tile_id
          << " depth " << tile.depth
          << " | d(origin) " << origin_distance
          << " | speed " << camera.move_speed
          << " | zoom " << camera.zoom
          << " | tiles " << patch.tiles.size()
          << " | -/= speed, Q/E zoom";
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
        ShaderProgram::from_files(resolve_shader_path("basic.vert"), resolve_shader_path("basic.frag"));
    const tiling::TilingPatch patch =
        tiling::generate_tiling_patch(math::RegularTilingParameters{4, 6}, kProgressDemoTilingDepth);
    Mesh center_mesh;
    center_mesh.upload(make_tile_center_mesh(patch));
    Mesh grid_mesh;
    grid_mesh.upload(make_grid_mesh(patch));

    CameraState camera;
    update_window_title(window, camera, patch);
    float title_update_accumulator = 0.0F;
    auto previous_time = std::chrono::steady_clock::now();

    while (glfwWindowShouldClose(window) == GLFW_FALSE) {
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - previous_time).count();
        previous_time = now;

        glfwPollEvents();
        process_input(window, camera, patch, dt);
        title_update_accumulator += dt;
        if (title_update_accumulator >= 0.25F) {
            update_window_title(window, camera, patch);
            title_update_accumulator = 0.0F;
        }

        glfwGetFramebufferSize(window, &width, &height);

        glClearColor(0.08F, 0.10F, 0.09F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT);

        shader.use();
        shader.set_mat4("u_mvp", camera_matrix(camera, patch, width, height));

        // Draw grid
        grid_mesh.draw_lines();

        // Draw centers
        center_mesh.draw();

        glfwSwapBuffers(window);
    }

    glfwDestroyWindow(window);
}

} // namespace hyper
