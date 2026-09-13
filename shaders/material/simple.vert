#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inTexcoord;

layout(location = 0) out vec3 fragWorldPosition;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) out vec2 fragTexcoord;

layout(set = 0, binding = 0, std140) uniform SceneUniforms {
    mat4 viewProjection;
    vec4 cameraPosition;
} scene;

layout(push_constant) uniform PushConstants {
    mat4 model;
    vec4 baseColorFactor;
    float metallicFactor;
    float roughnessFactor;
    uint materialFlags;
    float alphaCutoff;
} pc;

void main() {
    vec4 worldPosition = pc.model * vec4(inPosition, 1.0);
    gl_Position = scene.viewProjection * worldPosition;
    fragWorldPosition = worldPosition.xyz;
    fragNormal = normalize(mat3(pc.model) * inNormal);
    fragTexcoord = inTexcoord;
}
