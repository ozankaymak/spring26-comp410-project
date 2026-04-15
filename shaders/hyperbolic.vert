#version 410 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;

uniform mat3 uLorentzView;
uniform mat4 uEuclideanProj;

out vec3 vColor;
out float vHypDist;

void main() {
    // aPosition is (t, x, y) in the hyperboloid model.
    // Apply camera Lorentz view (inverse of camera frame)
    vec3 viewPos = uLorentzView * aPosition;

    // Hyperbolic distance from the camera (which is now at origin (1,0,0) of viewPos).
    vHypDist = acosh(max(viewPos.x, 1.0)); // wait, x is t because we use (t,x,y) -> so viewPos.x is actually t. 
    // In glsl, vec3 properties are x,y,z. so viewPos.x corresponds to aPosition.x? No!
    // Since aPosition is (t, x, y), viewPos.x is t, viewPos.y is x, viewPos.z is y.
    
    // So HypDist = acosh(t)
    vHypDist = acosh(max(viewPos.x, 1.0));

    // Poincare projection
    vec2 projected = vec2(viewPos.y, viewPos.z) / (viewPos.x + 1.0);

    vColor = aColor;
    
    gl_Position = uEuclideanProj * vec4(projected.x, projected.y, 0.0, 1.0);
}