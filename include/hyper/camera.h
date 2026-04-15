#pragma once

#include <glm/glm.hpp>

namespace hyper {

class Camera {
public:
    Camera();

    void set_position(const glm::vec3& pos);
    glm::vec3 get_position() const;

    void rotate_yaw(float delta);
    void rotate_pitch(float delta);

    glm::mat4 get_view_matrix() const;

private:
    glm::vec3 position_;
    float yaw_;
    float pitch_;
};

} // namespace hyper
