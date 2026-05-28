#include "app.h"

#include "math/hyperbolic.h"
#include "mesh.h"
#include "shader.h"
#include "tiling_core.h"

#include <chrono>
#include <array>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
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
    math::RegularTilingParameters tiling_parameters{4, 6};
    int tiling_depth = kProgressDemoTilingDepth;
    bool show_grid = true;
    bool show_wireframe = false;
    bool show_debug_ui = true;
    float fog_density = 0.50F;
    glm::vec3 fog_color{0.07F, 0.085F, 0.095F};
    int edge_segments = 8;
    int radial_bands = 2;
};

math::CameraFrame display_aligned_frame() {
    return math::orthonormalize_frame(math::CameraFrame{
        math::origin(),
        math::Vec3{0.0, 0.0, 1.0},
        math::Vec3{0.0, 1.0, 0.0},
    });
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
    bool g_down = false;
    bool f_down = false;
    bool z_down = false;
    bool x_down = false;
    bool c_down = false;
    bool v_down = false;
    bool b_down = false;
    bool n_down = false;
};

struct OverlayVertex {
    glm::vec2 position{};
    glm::vec4 color{};
};

struct TilingPreset {
    int key = 0;
    math::RegularTilingParameters parameters{};
    const char* label = "";
};

math::CameraFrame global_camera_frame(const CameraState& camera, const tiling::TilingPatch& patch);
glm::vec4 h2_point_to_h3(const math::Vec3& point);
void append_vertex(MeshData& mesh,
                   const glm::vec4& position,
                   const glm::vec3& normal,
                   const glm::vec3& color);
MeshData make_curved_tile_mesh(const tiling::TilingPatch& patch, int radial_bands, int edge_segments);
MeshData make_grid_mesh(const tiling::TilingPatch& patch, int edge_segments);

constexpr std::array<TilingPreset, 4> kTilingPresets{{
    TilingPreset{GLFW_KEY_1, math::RegularTilingParameters{4, 6}, "1 {4,6} 6 SQUARES"},
    TilingPreset{GLFW_KEY_2, math::RegularTilingParameters{3, 7}, "2 {3,7} 7 TRIANGLES"},
    TilingPreset{GLFW_KEY_3, math::RegularTilingParameters{5, 4}, "3 {5,4} 4 PENTAGONS"},
    TilingPreset{GLFW_KEY_4, math::RegularTilingParameters{7, 3}, "4 {7,3} 3 HEPTAGONS"},
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

    void add_panel(const RenderSettings& settings,
                   const CameraState& camera,
                   const tiling::TilingPatch& patch) {
        const math::CameraFrame global_frame = global_camera_frame(camera, patch);
        const double origin_distance = math::intrinsic_distance(math::origin(), global_frame.position);
        const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(camera.current_tile_id)];

        add_rect(10.0F, 10.0F, 392.0F, 360.0F, glm::vec4{0.02F, 0.03F, 0.04F, 0.78F});
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
            text << "TILING {" << patch.parameters.p << "," << patch.parameters.q << "}";
            line(text.str(), value);
        }
        {
            std::ostringstream text;
            text << patch.parameters.p << " SIDES | " << patch.parameters.q << " AT CORNER";
            line(text.str(), label);
        }
        line(kTilingPresets[0].label, label);
        line(kTilingPresets[1].label, label);
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
            text << "Z/X FOG " << std::fixed << std::setprecision(2) << settings.fog_density;
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

        add_text(22.0F, 342.0F, "F1 HIDE PANEL", 2.0F, glm::vec4{0.58F, 0.70F, 0.74F, 0.95F});
    }

    GLuint program_ = 0;
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
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

void rebuild_tiling(const RenderSettings& settings,
                    tiling::TilingPatch& patch,
                    Mesh& center_mesh,
                    Mesh& grid_mesh,
                    CameraState& camera) {
    patch = tiling::generate_tiling_patch(settings.tiling_parameters, settings.tiling_depth);
    center_mesh.upload(make_curved_tile_mesh(patch, settings.radial_bands, settings.edge_segments));
    grid_mesh.upload(make_grid_mesh(patch, settings.edge_segments));
    camera.frame = display_aligned_frame();
    camera.pitch = -0.28F;
    camera.eye_height = 0.32F;
    camera.current_tile_id = 0;
}

bool apply_tiling_preset(RenderSettings& settings, math::RegularTilingParameters parameters) {
    if (!math::is_hyperbolic_tiling(parameters.p, parameters.q)) {
        return false;
    }
    if (settings.tiling_parameters.p == parameters.p && settings.tiling_parameters.q == parameters.q) {
        return false;
    }

    settings.tiling_parameters = parameters;
    return true;
}

bool process_debug_input(GLFWwindow* window,
                         RenderSettings& settings,
                         DebugInputState& input,
                         tiling::TilingPatch& patch,
                         Mesh& center_mesh,
                         Mesh& grid_mesh,
                         CameraState& camera) {
    bool rebuild_meshes = false;
    bool rebuild_patch = false;

    if (consume_key_press(window, GLFW_KEY_F1, input.f1_down)) {
        settings.show_debug_ui = !settings.show_debug_ui;
    }
    if (consume_key_press(window, GLFW_KEY_1, input.preset1_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[0].parameters);
    }
    if (consume_key_press(window, GLFW_KEY_2, input.preset2_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[1].parameters);
    }
    if (consume_key_press(window, GLFW_KEY_3, input.preset3_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[2].parameters);
    }
    if (consume_key_press(window, GLFW_KEY_4, input.preset4_down)) {
        rebuild_patch |= apply_tiling_preset(settings, kTilingPresets[3].parameters);
    }
    if (consume_key_press(window, GLFW_KEY_G, input.g_down)) {
        settings.show_grid = !settings.show_grid;
    }
    if (consume_key_press(window, GLFW_KEY_F, input.f_down)) {
        settings.show_wireframe = !settings.show_wireframe;
    }
    if (consume_key_press(window, GLFW_KEY_Z, input.z_down)) {
        settings.fog_density = glm::max(0.0F, settings.fog_density - 0.05F);
    }
    if (consume_key_press(window, GLFW_KEY_X, input.x_down)) {
        settings.fog_density = glm::min(0.95F, settings.fog_density + 0.05F);
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
        rebuild_tiling(settings, patch, center_mesh, grid_mesh, camera);
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
            
            camera.frame = math::orthonormalize_frame(camera.frame);
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
                boundary_ring.push_back(math::geodesic_lerp(a, b, static_cast<double>(s) / edge_segments));
            }
        }

        const int rc = static_cast<int>(boundary_ring.size());
        
        for (int band = 1; band <= radial_bands; ++band) {
            double t = static_cast<double>(band) / radial_bands;
            for (const math::Vec3& boundary_pos : boundary_ring) {
                math::Vec3 hv = math::geodesic_lerp(tile.center, boundary_pos, t);
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
                math::Vec3 p = math::geodesic_lerp(a, b, static_cast<double>(s) / edge_segments);
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

math::CameraFrame global_camera_frame(const CameraState& camera, const tiling::TilingPatch& patch) {
    const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(camera.current_tile_id)];
    return tiling::global_frame_from_tile(camera.frame, tile);
}

glm::mat4 euclidean_projection(int width, int height, float zoom) {
    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0F;
    const float clamped_zoom = glm::clamp(zoom, 0.25F, 3.0F);
    const float fov_degrees = glm::clamp(74.0F / clamped_zoom, 34.0F, 100.0F);
    return glm::perspective(glm::radians(fov_degrees), aspect, 0.01F, 8.0F);
}

glm::mat4 lorentz_view(const CameraState& camera, const tiling::TilingPatch& patch) {
    const math::CameraFrame global_frame = global_camera_frame(camera, patch);
    const glm::mat4 floor_frame = h3_frame_from_h2_frame(global_frame);
    const glm::mat4 eye_frame =
        floor_frame * h3_lorentz_boost(glm::vec3{0.0F, 1.0F, 0.0F}, camera.eye_height) *
        h3_rotation_yz(camera.pitch);
    return h3_lorentz_inverse(eye_frame);
}

void update_window_title(GLFWwindow* window, const CameraState& camera, const tiling::TilingPatch& patch) {
    const math::CameraFrame global_frame = global_camera_frame(camera, patch);
    const double origin_distance = math::intrinsic_distance(math::origin(), global_frame.position);
    const tiling::Tile& tile = patch.tiles[static_cast<std::size_t>(camera.current_tile_id)];

    std::ostringstream title;
    title << std::fixed << std::setprecision(2)
          << "Hyperbolica | {" << patch.parameters.p << "," << patch.parameters.q << "}"
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
    
    tiling::TilingPatch patch = tiling::generate_tiling_patch(settings.tiling_parameters, settings.tiling_depth);
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
        process_debug_input(window, settings, debug_input, patch, center_mesh, grid_mesh, camera);
        title_update_accumulator += dt;
        if (title_update_accumulator >= 0.25F) {
            update_window_title(window, camera, patch);
            title_update_accumulator = 0.0F;
        }

        glfwGetFramebufferSize(window, &width, &height);

        glClearColor(settings.fog_color.x, settings.fog_color.y, settings.fog_color.z, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        shader.use();
        shader.set_mat4("uEuclideanProj", euclidean_projection(width, height, camera.zoom));
        
        glm::mat4 lv = lorentz_view(camera, patch);
        glUniformMatrix4fv(glGetUniformLocation(shader.id(), "uLorentzView"), 1, GL_FALSE, &lv[0][0]);
        glUniform1i(glGetUniformLocation(shader.id(), "uProjectionModel"), 1);
        glUniform1f(glGetUniformLocation(shader.id(), "uFogDensity"), settings.fog_density);
        glUniform3fv(glGetUniformLocation(shader.id(), "uFogColor"), 1, &settings.fog_color[0]);
        const glm::vec3 light_dir = glm::normalize(glm::vec3{0.85F, 1.20F, 0.45F});
        glUniform3fv(glGetUniformLocation(shader.id(), "uLightDir"), 1, &light_dir[0]);

        glPolygonMode(GL_FRONT_AND_BACK, settings.show_wireframe ? GL_LINE : GL_FILL);
        
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonOffset(1.0f, 1.0f);
        center_mesh.draw();
        glDisable(GL_POLYGON_OFFSET_FILL);
        
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        if (settings.show_grid) {
            grid_mesh.draw_lines();
        }

        debug_overlay.draw(width, height, settings, camera, patch);

        glfwSwapBuffers(window);
    }

    debug_overlay.shutdown();
    glfwDestroyWindow(window);
}

} // namespace hyper
