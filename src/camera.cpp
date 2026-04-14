#include "hyper/camera.h"
#include <glm/gtc/matrix_transform.hpp>

namespace hyper {

Camera::Camera() : position_(0.0f, 0.0f, 3.0f), yaw_(0.0f), pitch_(0.0f) {}

void Camera::set_position(const glm::vec3& pos) {
    position_ = pos;
}

glm::vec3 Camera::get_position() const {
    return position_;
}

void Camera::rotate_yaw(float delta) {
    yaw_ += delta;
}

void Camera::rotate_pitch(float delta) {
    pitch_ += delta;
    pitch_ = glm::clamp(pitch_, -89.0f, 89.0f);
}

glm::mat4 Camera::get_view_matrix() const {
    glm::mat4 view = glm::mat4(1.0f);
    view = glm::rotate(view, glm::radians(pitch_), glm::vec3(1.0f, 0.0f, 0.0f));
    view = glm::rotate(view, glm::radians(yaw_), glm::vec3(0.0f, 1.0f, 0.0f));
    view = glm::translate(view, -position_);
    return view;
}

} // namespace hyper