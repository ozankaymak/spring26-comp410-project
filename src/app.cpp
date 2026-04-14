#include "app.h"

#include "math/hyperbolic.h"
#include "mesh.h"
#include "shader.h"
#include "tiling_core.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
// #include <imgui.h>
// #include <imgui_impl_glfw.h>
// #include <imgui_impl_opengl3.h>

namespace hyper {
namespace {

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

    const double move_speed = 1.1 * static_cast<double>(dt);
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
        camera.frame = math::move_frame(camera.frame, local_delta);

        // Check if we crossed into a new tile
        const int new_tile_id = tiling::find_current_tile(patch, camera.frame.position);
        if (new_tile_id != camera.current_tile_id && new_tile_id >= 0) {
            const tiling::Tile& new_tile = patch.tiles[static_cast<std::size_t>(new_tile_id)];
            camera.frame = tiling::rebase_frame_to_tile(camera.frame, new_tile);
            camera.current_tile_id = new_tile_id;
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

glm::mat4 camera_matrix(const CameraState& camera, int width, int height) {
    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0F;
    const math::Vec2 projected_position = math::project_to_poincare_disk(camera.frame.position);
    const glm::vec2 center{static_cast<float>(projected_position.x), static_cast<float>(projected_position.y)};
    const glm::mat4 projection =
        glm::ortho(-aspect * camera.zoom, aspect * camera.zoom, -camera.zoom, camera.zoom, -1.0F, 1.0F);
    const glm::mat4 view = glm::translate(glm::mat4{1.0F}, glm::vec3{-center.x, -center.y, 0.0F});
    return projection * view;
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

    // // Initialize ImGui
    // IMGUI_CHECKVERSION();
    // ImGui::CreateContext();
    // ImGuiIO& io = ImGui::GetIO();
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    // ImGui::StyleColorsDark();
    // ImGui_ImplGlfw_InitForOpenGL(window, true);
    // ImGui_ImplOpenGL3_Init("#version 330");

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    ShaderProgram shader =
        ShaderProgram::from_files(resolve_shader_path("basic.vert"), resolve_shader_path("basic.frag"));
    const tiling::TilingPatch patch = tiling::generate_tiling_patch(math::RegularTilingParameters{4, 6}, 3);
    Mesh center_mesh;
    center_mesh.upload(make_tile_center_mesh(patch));
    Mesh grid_mesh;
    grid_mesh.upload(make_grid_mesh(patch));

    CameraState camera;
    auto previous_time = std::chrono::steady_clock::now();

    while (glfwWindowShouldClose(window) == GLFW_FALSE) {
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - previous_time).count();
        previous_time = now;

        glfwPollEvents();
        process_input(window, camera, patch, dt);

        glfwGetFramebufferSize(window, &width, &height);

        glClearColor(0.08F, 0.10F, 0.09F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT);

        shader.use();
        shader.set_mat4("u_mvp", camera_matrix(camera, width, height));

        // Draw grid
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        grid_mesh.draw();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // Draw centers
        center_mesh.draw();

        // // ImGui HUD
        // ImGui_ImplOpenGL3_NewFrame();
        // ImGui_ImplGlfw_NewFrame();
        // ImGui::NewFrame();

        // ImGui::Begin("Hyperbolica Debug");
        // ImGui::Text("Tile ID: %d", camera.current_tile_id);
        // ImGui::Text("Depth: %d", patch.tiles[static_cast<size_t>(camera.current_tile_id)].depth);
        // const math::Vec2 pos_proj = math::project_to_poincare_disk(camera.frame.position);
        // ImGui::Text("Position: (%.3f, %.3f)", pos_proj.x, pos_proj.y);
        // ImGui::Text("Zoom: %.2f", camera.zoom);
        // ImGui::Text("Tiles: %zu", patch.tiles.size());
        // ImGui::End();

        // ImGui::Render();
        // ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    glfwDestroyWindow(window);

    // // Shutdown ImGui
    // ImGui_ImplOpenGL3_Shutdown();
    // ImGui_ImplGlfw_Shutdown();
    // ImGui::DestroyContext();
}

} // namespace hyper
