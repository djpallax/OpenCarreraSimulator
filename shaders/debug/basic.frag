#version 450

layout(location = 0) in vec3 fragNormal;
layout(location = 1) in vec2 fragTexcoord;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 normal = normalize(fragNormal);
    vec3 lightDirection = normalize(vec3(-0.55, -0.35, 0.75));
    float diffuse = max(dot(normal, lightDirection), 0.0);
    float hemi = normal.z * 0.5 + 0.5;

    vec3 base = mix(vec3(0.16, 0.20, 0.26), vec3(0.48, 0.62, 0.82), hemi);
    vec3 color = base * (0.22 + 0.78 * diffuse);

    // Keep UV alive in the interface for Step 5 material work.
    color += vec3(fragTexcoord.x * 0.015, fragTexcoord.y * 0.015, 0.0);
    outColor = vec4(color, 1.0);
}
