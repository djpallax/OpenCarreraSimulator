#version 450

layout(location = 0) in vec3 fragWorldPosition;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in vec2 fragTexcoord;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0, std140) uniform SceneUniforms {
    mat4 viewProjection;
    vec4 cameraPosition;
} scene;

layout(set = 1, binding = 0) uniform sampler2D baseColorTexture;

layout(push_constant) uniform PushConstants {
    mat4 model;
    vec4 baseColorFactor;
    float metallicFactor;
    float roughnessFactor;
    uint materialFlags;
    float alphaCutoff;
} pc;

const uint MATERIAL_UNLIT = 1u;
const uint MATERIAL_ALPHA_MASK = 2u;
const uint MATERIAL_DOUBLE_SIDED = 4u;

void main() {
    // Materials without a source texture receive the engine's 1x1 white fallback,
    // so every draw uses the same descriptor layout and shader path.
    vec4 base = pc.baseColorFactor * texture(baseColorTexture, fragTexcoord);
    if ((pc.materialFlags & MATERIAL_ALPHA_MASK) != 0u && base.a < pc.alphaCutoff) {
        discard;
    }

    if ((pc.materialFlags & MATERIAL_UNLIT) != 0u) {
        outColor = base;
        return;
    }

    vec3 normal = normalize(fragNormal);
    if ((pc.materialFlags & MATERIAL_DOUBLE_SIDED) != 0u && !gl_FrontFacing) {
        normal = -normal;
    }
    vec3 lightDirection = normalize(vec3(-0.55, -0.35, 0.75));
    vec3 viewDirection = normalize(scene.cameraPosition.xyz - fragWorldPosition);
    vec3 halfVector = normalize(lightDirection + viewDirection);

    float diffuseTerm = max(dot(normal, lightDirection), 0.0);
    float roughness = clamp(pc.roughnessFactor, 0.04, 1.0);
    float metallic = clamp(pc.metallicFactor, 0.0, 1.0);
    float shininess = mix(128.0, 4.0, roughness);
    float specularTerm = pow(max(dot(normal, halfVector), 0.0), shininess);

    vec3 dielectricF0 = vec3(0.04);
    vec3 specularColor = mix(dielectricF0, base.rgb, metallic);
    vec3 diffuseColor = base.rgb * (1.0 - metallic);
    vec3 ambient = diffuseColor * 0.16;
    vec3 diffuse = diffuseColor * diffuseTerm * 0.84;
    vec3 specular = specularColor * specularTerm * mix(0.35, 1.0, 1.0 - roughness);

    outColor = vec4(ambient + diffuse + specular, base.a);
}
