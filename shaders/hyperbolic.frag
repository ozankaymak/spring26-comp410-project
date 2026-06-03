#version 330 core

in vec3 vColor;
in vec3 vNormal;
in float vHypDist;

uniform float uFogDensity;
uniform vec3 uFogColor;
uniform float uAtmosphereStrength;
uniform vec3 uAtmosphereColor;
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
    vec3 fogged_color = mix(uFogColor, lit_color, fog);

    float atmosphere = 1.0 - exp(-0.18 * vHypDist);
    atmosphere = clamp(atmosphere * uAtmosphereStrength, 0.0, 1.0);

    FragColor = vec4(mix(fogged_color, uAtmosphereColor, atmosphere), 1.0);
}
