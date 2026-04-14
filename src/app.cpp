#include "app.h"

#include "mesh.h"
#include "shader.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

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
    glm::vec2 center{0.0F, 0.0F};
    float zoom = 1.0F;
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

void process_input(GLFWwindow* window, CameraState& camera, float dt) {
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }

    const float pan_speed = 1.1F * camera.zoom * dt;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        camera.center.x -= pan_speed;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        camera.center.x += pan_speed;
    }
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        camera.center.y += pan_speed;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        camera.center.y -= pan_speed;
    }
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        camera.zoom = glm::min(camera.zoom + dt, 3.0F);
    }
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        camera.zoom = glm::max(camera.zoom - dt, 0.25F);
    }
}

glm::mat4 camera_matrix(const CameraState& camera, int width, int height) {
    const float aspect = height > 0 ? static_cast<float>(width) / static_cast<float>(height) : 1.0F;
    const glm::mat4 projection =
        glm::ortho(-aspect * camera.zoom, aspect * camera.zoom, -camera.zoom, camera.zoom, -1.0F, 1.0F);
    const glm::mat4 view = glm::translate(glm::mat4{1.0F}, glm::vec3{-camera.center.x, -camera.center.y, 0.0F});
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

    int width = 0;
    int height = 0;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);

    ShaderProgram shader = ShaderProgram::from_files(resolve_shader_path("basic.vert"), resolve_shader_path("basic.frag"));
    Mesh mesh;
    mesh.upload(make_test_triangle_mesh());

    CameraState camera;
    auto previous_time = std::chrono::steady_clock::now();

    while (glfwWindowShouldClose(window) == GLFW_FALSE) {
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - previous_time).count();
        previous_time = now;

        glfwPollEvents();
        process_input(window, camera, dt);

        glfwGetFramebufferSize(window, &width, &height);

        glClearColor(0.08F, 0.10F, 0.09F, 1.0F);
        glClear(GL_COLOR_BUFFER_BIT);

        shader.use();
        shader.set_mat4("u_mvp", camera_matrix(camera, width, height));
        mesh.draw();

        glfwSwapBuffers(window);
    }

    glfwDestroyWindow(window);
}

} // namespace hyper
