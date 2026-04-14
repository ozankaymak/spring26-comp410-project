#include "mesh.h"
#include "shader.h"
#include "test_support.h"

#include <algorithm>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace {

using hyper::test::require;
using hyper::test::require_throws;

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
    return hyper::test::run("render_tests", [] {
        test_shader_sources_are_available();
        test_triangle_mesh_data();
    });
}
