#include "mesh.h"

#include <cstddef>
#include <stdexcept>
#include <utility>

namespace hyper {

MeshData make_test_triangle_mesh() {
    return MeshData{
        {
            Vertex{glm::vec4{-0.60F, -0.45F, 0.0F, 1.0F}, glm::vec3{0.0F, 0.0F, 1.0F},
                   glm::vec3{0.90F, 0.25F, 0.18F}},
            Vertex{glm::vec4{0.60F, -0.45F, 0.0F, 1.0F}, glm::vec3{0.0F, 0.0F, 1.0F},
                   glm::vec3{0.15F, 0.70F, 0.45F}},
            Vertex{glm::vec4{0.00F, 0.60F, 0.0F, 1.0F}, glm::vec3{0.0F, 0.0F, 1.0F},
                   glm::vec3{0.20F, 0.45F, 0.95F}},
        },
        {0U, 1U, 2U},
    };
}

Mesh::~Mesh() {
    reset();
}

Mesh::Mesh(Mesh&& other) noexcept
    : vao_(std::exchange(other.vao_, 0)),
      vbo_(std::exchange(other.vbo_, 0)),
      ebo_(std::exchange(other.ebo_, 0)),
      index_count_(std::exchange(other.index_count_, 0)) {}

Mesh& Mesh::operator=(Mesh&& other) noexcept {
    if (this != &other) {
        reset();
        vao_ = std::exchange(other.vao_, 0);
        vbo_ = std::exchange(other.vbo_, 0);
        ebo_ = std::exchange(other.ebo_, 0);
        index_count_ = std::exchange(other.index_count_, 0);
    }

    return *this;
}

void Mesh::upload(const MeshData& data) {
    if (data.vertices.empty() || data.indices.empty()) {
        throw std::invalid_argument("mesh upload requires vertices and indices");
    }

    reset();
    index_count_ = static_cast<GLsizei>(data.indices.size());

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
    glGenBuffers(1, &ebo_);

    glBindVertexArray(vao_);

    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(data.vertices.size() * sizeof(Vertex)),
                 data.vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(data.indices.size() * sizeof(unsigned int)),
                 data.indices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, position)));

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, normal)));

    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void*>(offsetof(Vertex, color)));

    glBindVertexArray(0);
}

void Mesh::draw() const {
    if (!uploaded()) {
        return;
    }

    glBindVertexArray(vao_);
    glDrawElements(GL_TRIANGLES, index_count_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Mesh::draw_lines() const {
    if (!uploaded()) {
        return;
    }

    glBindVertexArray(vao_);
    glDrawElements(GL_LINES, index_count_, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
}

void Mesh::reset() {
    if (ebo_ != 0) {
        glDeleteBuffers(1, &ebo_);
        ebo_ = 0;
    }

    if (vbo_ != 0) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }

    if (vao_ != 0) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }

    index_count_ = 0;
}

bool Mesh::uploaded() const {
    return vao_ != 0 && vbo_ != 0 && ebo_ != 0 && index_count_ > 0;
}

GLsizei Mesh::index_count() const {
    return index_count_;
}

} // namespace hyper
