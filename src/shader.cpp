#include "shader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

#include <glm/gtc/type_ptr.hpp>

namespace hyper {
namespace {

std::string shader_log(GLuint shader) {
    GLint length = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);

    if (length <= 1) {
        return {};
    }

    std::vector<char> buffer(static_cast<std::size_t>(length));
    glGetShaderInfoLog(shader, length, nullptr, buffer.data());
    return std::string(buffer.data());
}

std::string program_log(GLuint program) {
    GLint length = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);

    if (length <= 1) {
        return {};
    }

    std::vector<char> buffer(static_cast<std::size_t>(length));
    glGetProgramInfoLog(program, length, nullptr, buffer.data());
    return std::string(buffer.data());
}

GLuint compile_shader(GLenum type, const std::string& source, const std::filesystem::path& path) {
    const GLuint shader = glCreateShader(type);
    const char* source_text = source.c_str();
    glShaderSource(shader, 1, &source_text, nullptr);
    glCompileShader(shader);

    GLint ok = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);

    if (ok != GL_TRUE) {
        const std::string log = shader_log(shader);
        glDeleteShader(shader);
        throw std::runtime_error("failed to compile shader " + path.string() + ": " + log);
    }

    return shader;
}

} // namespace

std::string read_text_file(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("failed to open file: " + path.string());
    }

    std::ostringstream contents;
    contents << file.rdbuf();
    return contents.str();
}

ShaderProgram::ShaderProgram(GLuint id) : id_(id) {}

ShaderProgram::~ShaderProgram() {
    if (id_ != 0) {
        glDeleteProgram(id_);
    }
}

ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept : id_(std::exchange(other.id_, 0)) {}

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept {
    if (this != &other) {
        if (id_ != 0) {
            glDeleteProgram(id_);
        }
        id_ = std::exchange(other.id_, 0);
    }

    return *this;
}

ShaderProgram ShaderProgram::from_files(const std::filesystem::path& vertex_path,
                                        const std::filesystem::path& fragment_path) {
    const std::string vertex_source = read_text_file(vertex_path);
    const std::string fragment_source = read_text_file(fragment_path);

    const GLuint vertex_shader = compile_shader(GL_VERTEX_SHADER, vertex_source, vertex_path);
    const GLuint fragment_shader = compile_shader(GL_FRAGMENT_SHADER, fragment_source, fragment_path);

    const GLuint program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    GLint ok = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &ok);

    if (ok != GL_TRUE) {
        const std::string log = program_log(program);
        glDeleteProgram(program);
        throw std::runtime_error("failed to link shader program: " + log);
    }

    return ShaderProgram(program);
}

void ShaderProgram::use() const {
    glUseProgram(id_);
}

void ShaderProgram::set_mat4(const std::string& name, const glm::mat4& value) const {
    const GLint location = glGetUniformLocation(id_, name.c_str());
    if (location >= 0) {
        glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(value));
    }
}

GLuint ShaderProgram::id() const {
    return id_;
}

bool ShaderProgram::valid() const {
    return id_ != 0;
}

} // namespace hyper

