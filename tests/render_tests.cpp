#include "mesh.h"
#include "shader.h"

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

template <typename Exception, typename Fn>
void require_throws(Fn&& fn, const std::string& message) {
    bool threw = false;
    try {
        fn();
    } catch (const Exception&) {
        threw = true;
    }

    require(threw, message);
}

std::filesystem::path source_path(const std::filesystem::path& relative) {
    return std::filesystem::path{HYPER_SOURCE_DIR} / relative;
}

void test_shader_sources_are_available() {
    const std::string vertex_source = hyper::read_text_file(source_path("shaders/basic.vert"));
    const std::string fragment_source = hyper::read_text_file(source_path("shaders/basic.frag"));

    require(vertex_source.find("#version 330 core") != std::string::npos, "vertex shader declares GLSL version");
    require(vertex_source.find("u_mvp") != std::string::npos, "vertex shader exposes the transform uniform");
    require(fragment_source.find("#version 330 core") != std::string::npos, "fragment shader declares GLSL version");
    require(fragment_source.find("frag_color") != std::string::npos, "fragment shader writes a color output");

    require_throws<std::runtime_error>(
        [] { (void)hyper::read_text_file(source_path("shaders/missing.vert")); },
        "missing shader file is reported");
}

void test_triangle_mesh_data() {
    const hyper::MeshData mesh = hyper::make_test_triangle_mesh();

    require(mesh.vertices.size() == 3, "test triangle has three vertices");
    require(mesh.indices.size() == 3, "test triangle has three indices");
    require(std::all_of(mesh.indices.begin(), mesh.indices.end(),
                        [&mesh](unsigned int index) { return index < mesh.vertices.size(); }),
            "test triangle indices reference valid vertices");

    require(mesh.vertices[0].position.x < mesh.vertices[1].position.x, "triangle base has left and right vertices");
    require(mesh.vertices[2].position.y > mesh.vertices[0].position.y, "triangle apex is above the base");
}

} // namespace

int main() {
    try {
        test_shader_sources_are_available();
        test_triangle_mesh_data();
    } catch (const std::exception& error) {
        std::cerr << "render_tests failed: " << error.what() << '\n';
        return 1;
    }

    return 0;
}

