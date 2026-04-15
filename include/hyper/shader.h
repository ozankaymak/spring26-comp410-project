#pragma once

#include <string>

#include <glad/glad.h>
#include <glm/glm.hpp>

namespace hyper {

class Shader {
public:
    Shader() = default;
    ~Shader();

    bool load(const std::string& vert_path, const std::string& frag_path);
    void use() const;

    void set_int(const std::string& name, int value) const;
    void set_float(const std::string& name, float value) const;
    void set_vec3(const std::string& name, const glm::vec3& v) const;
    void set_mat4(const std::string& name, const glm::mat4& m) const;

private:
    GLuint compile(GLenum type, const char* source);
    GLint uniform_location(const std::string& name) const;

    GLuint id = 0;
};

} // namespace hyper
