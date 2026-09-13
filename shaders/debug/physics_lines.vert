#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inColor;

layout(location = 0) out vec4 fragColor;

layout(set = 0, binding = 0, std140) uniform SceneUniforms {
    mat4 viewProjection;
    vec4 cameraPosition;
} scene;

void main() {
    gl_Position = scene.viewProjection * vec4(inPosition, 1.0);
    fragColor = inColor;
}
