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
#include <algorithm>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

namespace hyper {
namespace {

constexpr int kProgressDemoTilingDepth = 5;
constexpr float kMinMoveSpeed = 0.25F;
constexpr float kMaxMoveSpeed = 8.0F;

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

struct DebugSettings {
    bool show_grid = true;
    bool show_wireframe = false;
    float fog_density = 0.50F;
    glm::vec3 fog_color{0.08F, 0.10F, 0.09F};
    int edge_segments = 8;
    int radial_bands = 2;
    bool cursor_captured = true;
    bool rebuild_meshes = false;
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

bool g_cursor_captured = true;
bool g_first_mouse = true;
double g_last_mouse_x = 0;
double g_last_mouse_y = 0;

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    (void)scancode;
    (void)mods;
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        g_cursor_captured = !g_cursor_captured;
        glfwSetInputMode(window, GLFW_CURSOR, g_cursor_captured ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
        g_first_mouse = true;
    }
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
            float yaw = -dx * sensitivity;
            
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
            
            camera.frame = math::orthonormalize_frame(camera.frame);
        }
    }
}

MeshData make_curved_tile_mesh(const tiling::TilingPatch& patch, int radial_bands, int edge_segments) {
    MeshData mesh;
    edge_segments = std::max(1, edge_segments);
    radial_bands = std::max(1, radial_bands);

    for (const tiling::Tile& tile : patch.tiles) {
        const float depth_factor = static_cast<float>(tile.depth) / 4.0F;
        const glm::vec3 color{
            tile.depth == 0 ? 0.95F : 0.25F + 0.10F * depth_factor,
            tile.depth == 0 ? 0.90F : 0.75F - 0.08F * depth_factor,
            tile.depth == 0 ? 0.25F : 0.95F,
        };

        const unsigned int center_idx = static_cast<unsigned int>(mesh.vertices.size());
        mesh.vertices.push_back(Vertex{
            glm::vec3(tile.center.t, tile.center.x, tile.center.y), color
        });

        std::vector<math::Vec3> boundary_ring;
        boundary_ring.reserve(patch.base_polygon_vertices.size() * static_cast<size_t>(edge_segments));

        for (size_t i = 0; i < patch.base_polygon_vertices.size(); ++i) {
            const size_t next_i = (i + 1) % patch.base_polygon_vertices.size();
            const math::Vec3 a = math::apply_isometry(tile.transform, patch.base_polygon_vertices[i]);
            const math::Vec3 b = math::apply_isometry(tile.transform, patch.base_polygon_vertices[next_i]);

            for (int s = 0; s < edge_segments; ++s) {
                boundary_ring.push_back(math::geodesic_lerp(a, b, static_cast<double>(s) / edge_segments));
            }
        }

        const int rc = static_cast<int>(boundary_ring.size());
        
        for (int band = 1; band <= radial_bands; ++band) {
            double t = static_cast<double>(band) / radial_bands;
            for (const math::Vec3& boundary_pos : boundary_ring) {
                math::Vec3 hv = math::geodesic_lerp(tile.center, boundary_pos, t);
                mesh.vertices.push_back(Vertex{glm::vec3(hv.t, hv.x, hv.y), color});
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
                math::Vec3 p = math::geodesic_lerp(a, b, static_cast<double>(s) / edge_segments);
                mesh.vertices.push_back(Vertex{glm::vec3(p.t, p.x, p.y), grid_color});
                if (s > 0) {
                    mesh.indices.push_back(base + s - 1);
                    mesh.indices.push_back(base + s);
                }
            }
        }
    }

    return mesh;
}

math::CameraFrame global_camera_frame(const CameraState& camera, const tiling::TilingPatch& patch) {
    const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(camera.current_tile_id)];
    return tiling::global_frame_from_tile(camera.frame, tile);
}

glm::mat4 euclidean_projection(int width, int height, float zoom) {
    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0F;
    // We are viewing a 2D Poincare disk placed at z = 0, so use orthographic projection
    // that bounds [-aspect*zoom, aspect*zoom] horizontally and [-zoom, zoom] vertically.
    return glm::ortho(-aspect * zoom, aspect * zoom, -zoom, zoom, -1.0F, 1.0F);
}

glm::mat3 lorentz_view(const CameraState& camera, const tiling::TilingPatch& patch) {
    math::CameraFrame global_frame = global_camera_frame(camera, patch);
    math::Mat3 cam_isometry = math::frame_to_isometry(global_frame);
    math::Mat3 inv_isometry = math::inverse_isometry(cam_isometry);
    
    glm::mat3 view(1.0f);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            view[j][i] = static_cast<float>(inv_isometry.m[i][j]);
        }
    }
    return view;
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

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 410");

    DebugSettings settings;
    CameraState camera;
    
    const tiling::TilingPatch patch =
        tiling::generate_tiling_patch(math::RegularTilingParameters{4, 6}, kProgressDemoTilingDepth);
    Mesh center_mesh;
    center_mesh.upload(make_curved_tile_mesh(patch, settings.radial_bands, settings.edge_segments));
    Mesh grid_mesh;
    grid_mesh.upload(make_grid_mesh(patch, settings.edge_segments));

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

        if (settings.rebuild_meshes) {
            center_mesh.upload(make_curved_tile_mesh(patch, settings.radial_bands, settings.edge_segments));
            grid_mesh.upload(make_grid_mesh(patch, settings.edge_segments));
            settings.rebuild_meshes = false;
        }

        glClearColor(settings.fog_color.x, settings.fog_color.y, settings.fog_color.z, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        shader.use();
        shader.set_mat4("uEuclideanProj", euclidean_projection(width, height, camera.zoom));
        
        glm::mat3 lv = lorentz_view(camera, patch);
        glUniformMatrix3fv(glGetUniformLocation(shader.id(), "uLorentzView"), 1, GL_FALSE, &lv[0][0]);
        glUniform1f(glGetUniformLocation(shader.id(), "uFogDensity"), settings.fog_density);
        glUniform3fv(glGetUniformLocation(shader.id(), "uFogColor"), 1, &settings.fog_color[0]);

        glPolygonMode(GL_FRONT_AND_BACK, settings.show_wireframe ? GL_LINE : GL_FILL);
        
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 1.0f);
        center_mesh.draw();
        glDisable(GL_POLYGON_OFFSET_FILL);
        
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        if (settings.show_grid) {
            grid_mesh.draw_lines();
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(280, 0), ImGuiCond_Once);
        ImGui::SetNextWindowBgAlpha(0.75f);
        ImGui::Begin("Settings");

        ImGui::SliderFloat("Speed", &camera.move_speed, 0.5f, 8.0f);
        ImGui::Separator();
        ImGui::SliderFloat("Fog", &settings.fog_density, 0.0f, 0.8f);
        ImGui::ColorEdit3("Sky color", &settings.fog_color.x);
        ImGui::Separator();
        ImGui::Checkbox("Floor grid", &settings.show_grid);
        ImGui::Checkbox("Wireframe", &settings.show_wireframe);
        ImGui::Separator();
        ImGui::Text("Tiling Tessellation");
        if (ImGui::SliderInt("Edge Segments", &settings.edge_segments, 1, 24)) settings.rebuild_meshes = true;
        if (ImGui::SliderInt("Radial Bands", &settings.radial_bands, 1, 8)) settings.rebuild_meshes = true;
        ImGui::Separator();
        ImGui::TextDisabled("Tile ID: %d | Depth: %d", camera.current_tile_id, patch.tiles[camera.current_tile_id].depth);
        ImGui::TextDisabled("Press Esc to toggle mouse out from camera");
        ImGui::End();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
}

} // namespace hyper
