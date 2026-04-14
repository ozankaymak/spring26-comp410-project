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

} // namespace hyper#pragma once
#include "hyper/math/geometry.h"
#include <glm/glm.hpp>

namespace hyper {

// First-person camera constrained to walk on the xz-hyperboloid.
//
// The camera is split into two independent parts:
//   - floor_frame_: a 4×4 Lorentz frame whose position (col 3) is always
//     on the xz-hyperboloid (y ≡ 0), expressed in the coordinates of the
//     current tile. Columns 0 and 2 are the right/forward tangent directions
//     in the floor plane. Column 1 is always (0,1,0,0). Movement (WASD)
//     updates this frame via Lorentz boosts in the xz-plane.
//   - pitch_: a scalar angle, applied only when building the view matrix.
//     It tilts the view up/down without moving the feet.
//
// Eye position = floor_pos boosted upward by eye_height along local +y.
class Camera {
public:
    explicit Camera(GeometryMode geometry_mode = GeometryMode::Hyperbolic);

    // Move along floor geodesics (pitch has no effect on direction)
    void move_forward(float dist);
    void move_right  (float dist);

    // Horizontal (yaw) and vertical (pitch) look — only yaw moves the frame
    void rotate_yaw  (float angle);
    void rotate_pitch(float angle);

    // View matrix for the shader: Minkowski inverse of the full eye frame
    glm::dmat4 get_view_matrix() const;

    // Eye position on the hyperboloid (used for debug display)
    glm::dvec4 get_position() const;

    // "Feet" position on the xz-hyperbolic plane (y ≡ 0)
    glm::dvec4 get_floor_position() const;

    // Current tile-local floor Lorentz frame.
    glm::dmat4 get_floor_frame() const { return floor_frame_; }

    void set_floor_frame(const glm::dmat4& frame) { floor_frame_ = frame; orthogonalize_floor_frame(); }
    void set_geometry_mode(GeometryMode mode);
    GeometryMode geometry_mode() const { return geometry_mode_; }

    // Apply a tile-local hyperbolic isometry (Lorentz transform) to the camera frame.
    void apply_isometry(const glm::dmat4& M);

    double eye_height       = 0.30;   // hyperbolic distance above floor
    float mouse_sensitivity = 0.002f;
    float move_speed        = 2.5f;

private:
    GeometryMode geometry_mode_ = GeometryMode::Hyperbolic;
    glm::dmat4 floor_frame_; // Lorentz frame locked to xz-hyperboloid
    double     pitch_        = -0.20; // radians; negative = looking slightly down

    // Re-orthogonalize floor_frame_ after drift (Minkowski Gram-Schmidt,
    // keeping the xz-hyperboloid constraint hard-enforced)
    void orthogonalize_floor_frame();
};

} // namespace hyper