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

} // namespace hyper#pragma once
#include <glad/glad.h>
#include <glm/glm.hpp>
#include <string>
#include <unordered_map>

namespace hyper {

class Shader {
public:
    Shader() = default;
    ~Shader();

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    // Load vertex + fragment shader from files
    bool load(const std::string& vert_path, const std::string& frag_path);

    void use() const;

    // Uniform setters
    void set_int(const std::string& name, int value) const;
    void set_float(const std::string& name, float value) const;
    void set_vec3(const std::string& name, const glm::vec3& v) const;
    void set_vec4(const std::string& name, const glm::vec4& v) const;
    void set_mat4(const std::string& name, const glm::mat4& m) const;
    void set_dmat4(const std::string& name, const glm::dmat4& m) const;

private:
    void release();
    GLuint compile(GLenum type, const char* source);
    GLint uniform_location(const std::string& name) const;

    GLuint id = 0;
    mutable std::unordered_map<std::string, GLint> uniform_locations_;
};

} // namespace hyper