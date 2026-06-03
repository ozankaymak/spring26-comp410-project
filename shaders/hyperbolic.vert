#version 330 core

layout(location = 0) in vec4 aPosition; // H3 hyperboloid point (x, y, z, w)
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;

uniform mat4 uLorentzView;
uniform mat4 uEuclideanProj;
uniform int uProjectionModel; // 0 = Klein, 1 = Poincare

out vec3 vColor;
out vec3 vNormal;
out float vHypDist;

float safe_acosh(float value) {
    float x = max(value, 1.0);
    return log(x + sqrt(max(x * x - 1.0, 0.0)));
}

void main() {
    vec4 viewPos = uLorentzView * aPosition;
    vHypDist = safe_acosh(viewPos.w);

    vec3 projected = (uProjectionModel == 0)
        ? viewPos.xyz / max(viewPos.w, 0.0001)
        : viewPos.xyz / max(viewPos.w + 1.0, 0.0001);

    vColor = aColor;
    vNormal = aNormal;

    // OpenGL's perspective matrix looks down -Z; the hyperbolic camera's
    // local forward axis is +Z.
    gl_Position = uEuclideanProj * vec4(projected.x, projected.y, -projected.z, 1.0);
}
