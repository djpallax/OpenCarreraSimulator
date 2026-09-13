#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inTexcoord;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec2 fragTexcoord;

layout(push_constant) uniform PushConstants {
    mat4 mvp;
    mat4 model;
} pc;

void main() {
    gl_Position = pc.mvp * vec4(inPosition, 1.0);
    fragNormal = normalize(mat3(pc.model) * inNormal);
    fragTexcoord = inTexcoord;
}
