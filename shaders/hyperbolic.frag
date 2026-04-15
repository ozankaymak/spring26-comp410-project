#version 410 core

in vec3 vColor;
in float vHypDist;

uniform float uFogDensity;
uniform vec3  uFogColor;

out vec4 FragColor;

void main() {
    // Exponential fog based on intrinsic hyperbolic distance squared.
    float fog = exp(-uFogDensity * vHypDist * vHypDist);
    fog = clamp(fog, 0.0, 1.0);

    vec3 finalColor = mix(uFogColor, vColor, fog);
    FragColor = vec4(finalColor, 1.0);
}