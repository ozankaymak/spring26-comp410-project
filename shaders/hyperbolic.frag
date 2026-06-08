#version 330 core

in vec3 vColor;
in vec3 vNormal;
in float vHypDist;

uniform vec3 uAtmosphereColor;
uniform vec3 uLightDir;
uniform int uGeometryMode; // 0 = hyperbolic, 1 = spherical
uniform float uDepthBias;  // pulls grid lines slightly toward the camera

out vec4 FragColor;

const float kPi = 3.14159265358979323846;

void main() {
    vec3 normal = normalize(vNormal);
    vec3 light_dir = normalize(uLightDir);
    float diffuse = max(dot(normal, light_dir), 0.0);
    float lighting = 0.34 + 0.66 * diffuse;

    vec3 lit_color = vColor * lighting;

    float atmosphere = 1.0 - exp(-0.18 * vHypDist);
    atmosphere = clamp(atmosphere, 0.0, 1.0);

    FragColor = vec4(mix(lit_color, uAtmosphereColor, atmosphere), 1.0);

    if (uGeometryMode == 1) {
        // On the closed sphere the tiled floor folds over itself in screen
        // space, and the stereographic perspective depth is not a reliable
        // front-to-back order. Drive the depth buffer straight from the
        // geodesic distance to the camera (vHypDist in [0, pi]) instead, which
        // is monotonic by construction: nearer geometry always wins, the two
        // hemispheres meet seamlessly at the equator, and the see-through
        // overlap disappears.
        gl_FragDepth = clamp(vHypDist / kPi - uDepthBias, 0.0, 1.0);
    } else {
        gl_FragDepth = gl_FragCoord.z;
    }
}
