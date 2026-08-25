#extension GL_ARB_shading_language_include : enable

#define EE_MATERIALS_BLOCK_SET 0
#define EE_MATERIALS_BLOCK_BINDING 3
#include "Materials.glsl"
#include "SSRConstants.glsl"

layout(location = 0) out vec4 FragColor;
layout(location = 0) in VS_OUT {
  vec2 TexCoord;
}
fs_in;

layout(set = 1, binding = 0) uniform sampler2D originalColor;
layout(set = 1, binding = 1) uniform sampler2D reflectedColorVisibility;

layout(set = 2, binding = 17) uniform sampler2D inDepth;
layout(set = 2, binding = 18) uniform sampler2D inNormal;
layout(set = 2, binding = 19) uniform sampler2D inMaterial;

void main() {
  vec2 texCoord = fs_in.TexCoord;

  int material_index = int(round(texture(inMaterial, texCoord).w));
  vec2 materialTexCoord = texture(inMaterial, texCoord).xy;
  MaterialProperties materialProperties = EE_MATERIAL_PROPERTIES[material_index];

  float roughness = materialProperties.roughness;
  float metallic = materialProperties.metallic;

  vec4 color = texture(originalColor, texCoord);
  vec4 reflected = texture(reflectedColorVisibility, texCoord);

  float factor = clamp(reflected.a * metallic * (1.f - roughness), 0.0f, 1.0f);

  vec3 result = color.xyz * (1.0f - factor) + reflected.rgb * factor;
  FragColor = vec4(result.xyz, color.w);
}