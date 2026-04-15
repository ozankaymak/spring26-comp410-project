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

} // namespace hyper
