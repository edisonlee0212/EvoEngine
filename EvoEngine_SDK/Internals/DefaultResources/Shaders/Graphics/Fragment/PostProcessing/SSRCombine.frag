#extension GL_ARB_shading_language_include : enable

#include "PerFrame.glsl"
#include "GltfRasterMaterial.glsl"
#include "SSRConstants.glsl"

layout (location = 0) out vec4 FragColor;
layout (location = 0) in VS_OUT {
    vec2 TexCoord;
} fs_in;

layout(set = 1, binding = 0) uniform sampler2D originalColor;
layout(set = 1, binding = 1) uniform sampler2D reflectedColorVisibility;

layout(set = 2, binding = 17) uniform sampler2D inDepth;
layout(set = 2, binding = 18) uniform sampler2D inNormal;
layout(set = 2, binding = 19) uniform sampler2D inMaterial;

void main()
{
    vec2 texCoord = fs_in.TexCoord;

    vec4 material_sample = texture(inMaterial, texCoord);
    int material_index = int(round(material_sample.z));
    GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(uint(material_index), material_sample.xy, material_sample.xy);
	float roughness = surface.roughness;
	float metallic = surface.metallic;

    vec4 color = texture(originalColor, texCoord);
    vec4 reflected = texture(reflectedColorVisibility, texCoord);

    float factor = clamp(reflected.a * metallic * (1.f - roughness), 0.0f, 1.0f);

    vec3 result = color.xyz * (1.0f - factor) + reflected.rgb * factor;
    FragColor = vec4(result.xyz, color.w);
}
