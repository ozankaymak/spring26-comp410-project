#pragma once

#include <filesystem>
#include <string>

#include <glad/glad.h>
#include <glm/glm.hpp>

namespace hyper {

std::string read_text_file(const std::filesystem::path& path);

class ShaderProgram {
public:
    ShaderProgram() = default;
    ~ShaderProgram();

    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;

    ShaderProgram(ShaderProgram&& other) noexcept;
    ShaderProgram& operator=(ShaderProgram&& other) noexcept;

    static ShaderProgram from_files(const std::filesystem::path& vertex_path,
                                    const std::filesystem::path& fragment_path);

    void use() const;
    void set_mat4(const std::string& name, const glm::mat4& value) const;

    GLuint id() const;
    bool valid() const;

private:
    explicit ShaderProgram(GLuint id);

    GLuint id_ = 0;
};

} // namespace hyper

