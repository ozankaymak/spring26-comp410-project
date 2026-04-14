#pragma once

#include <vector>

#include <glad/glad.h>
#include <glm/glm.hpp>

namespace hyper {

struct Vertex {
    glm::vec3 position{};
    glm::vec3 color{};
};

struct MeshData {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
};

MeshData make_test_triangle_mesh();

class Mesh {
public:
    Mesh() = default;
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    void upload(const MeshData& data);
    void draw() const;
    void reset();

    bool uploaded() const;
    GLsizei index_count() const;

private:
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLuint ebo_ = 0;
    GLsizei index_count_ = 0;
};

} // namespace hyper

