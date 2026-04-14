#pragma once
#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>

namespace hyper {

struct HyperVertex {
    glm::vec3 position;
    glm::vec3 normal;
};

class Mesh {
public:
    Mesh() = default;
    ~Mesh();

    void upload(const std::vector<HyperVertex>& vertices,
                const std::vector<GLuint>& indices);
    void draw() const;

private:
    void cleanup();

    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    GLuint ebo_ = 0;
    GLsizei index_count_ = 0;
};

} // namespace hyper#pragma once
#include <glm/glm.hpp>
#include <cstddef>
#include <vector>

namespace hyper {

struct HyperVertex {
    glm::dvec4 position; // point on the embedded manifold
    glm::vec3 normal;
    glm::vec2 texcoord;
    glm::vec3 color;
};

struct MeshInstance {
    glm::dmat4 model;
    glm::vec3 color;
    float _pad = 0.0f;
};

class Mesh {
public:
    Mesh() = default;
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    void upload(const std::vector<HyperVertex>& vertices,
                const std::vector<GLuint>& indices);

    void draw() const;
    void draw_lines() const;
    void set_instances(const std::vector<MeshInstance>& instances);
    void clear_instances();
    void draw_instanced() const;
    void draw_lines_instanced() const;

private:
    GLuint vao_ = 0, vbo_ = 0, ebo_ = 0;
    GLsizei index_count_ = 0;
    GLuint instance_vbo_ = 0;
    GLsizei instance_count_ = 0;

    void cleanup();
};