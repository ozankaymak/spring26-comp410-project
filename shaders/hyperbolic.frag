#version 330 core

in vec3 vColor;
in vec3 vNormal;
in float vHypDist;

uniform float uFogDensity;
uniform vec3 uFogColor;
uniform vec3 uLightDir;

out vec4 FragColor;

void main() {
    vec3 normal = normalize(vNormal);
    vec3 light_dir = normalize(uLightDir);
    float diffuse = max(dot(normal, light_dir), 0.0);
    float lighting = 0.34 + 0.66 * diffuse;

    float fog = exp(-uFogDensity * vHypDist * vHypDist);
    fog = clamp(fog, 0.0, 1.0);

    vec3 lit_color = vColor * lighting;
    FragColor = vec4(mix(uFogColor, lit_color, fog), 1.0);
}
