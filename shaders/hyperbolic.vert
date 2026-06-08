#version 330 core

layout(location = 0) in vec4 aPosition; // embedded curved-space point (x, y, z, w)
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;

uniform mat4 uLorentzView;
uniform mat4 uEuclideanProj;
uniform int uProjectionModel; // 0 = Klein/gnomonic, 1 = Poincare/stereographic
uniform int uGeometryMode;    // 0 = hyperbolic, 1 = spherical
uniform int uSphericalPass;   // 0 = full, 1 = near hemisphere, 2 = far hemisphere

out vec3 vColor;
out vec3 vNormal;
out float vHypDist;

float safe_acosh(float value) {
    float x = max(value, 1.0);
    return log(x + sqrt(max(x * x - 1.0, 0.0)));
}

void main() {
    vec4 viewPos = uLorentzView * aPosition;

    vec3 projected;
    if (uGeometryMode == 1) {
        // Spherical: distance is the angle to the camera, projection is the
        // stereographic map taken from the near or far pole.
        vHypDist = acos(clamp(viewPos.w, -1.0, 1.0));

        if (uSphericalPass == 1) {
            gl_ClipDistance[0] = viewPos.w;       // keep the near hemisphere
        } else if (uSphericalPass == 2) {
            gl_ClipDistance[0] = -viewPos.w;      // keep the far hemisphere
        } else {
            gl_ClipDistance[0] = 1.0;
        }

        if (uSphericalPass == 2) {
            projected = viewPos.xyz / max(1.0 - viewPos.w, 0.0001);
        } else {
            projected = viewPos.xyz / max(viewPos.w + 1.0, 0.0001);
        }
    } else {
        // Hyperbolic: Klein or Poincare projection of the hyperboloid point.
        vHypDist = safe_acosh(viewPos.w);
        gl_ClipDistance[0] = 1.0;
        projected = (uProjectionModel == 0)
            ? viewPos.xyz / max(viewPos.w, 0.0001)
            : viewPos.xyz / max(viewPos.w + 1.0, 0.0001);
    }

    vColor = aColor;
    vNormal = aNormal;

    // OpenGL's perspective matrix looks down -Z; the camera's local forward
    // axis is +Z.
    gl_Position = uEuclideanProj * vec4(projected.x, projected.y, -projected.z, 1.0);
}
