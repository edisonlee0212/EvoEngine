#extension GL_ARB_shading_language_include : enable

#include "PerFrame.slangh"
#include "SSRConstants.slangh"

layout(local_size_x = 16, local_size_y = 16, local_size_z = 1) in;

layout(set = 1, binding = 0) uniform sampler2D originalColor;
layout(set = 1, binding = 1) uniform sampler2D reflectedColorVisibility;
layout(set = 2, binding = 21) uniform sampler2D inNormalRoughness;
layout(set = 2, binding = 22) uniform sampler2D inPbrFlags;
layout(set = 3, binding = 0, rgba32f) uniform writeonly image2D outColor;

void main() {
  uvec2 pixel = gl_GlobalInvocationID.xy;
  uvec2 resolution = uvec2(imageSize(outColor));
  if (pixel.x >= resolution.x || pixel.y >= resolution.y) {
    return;
  }

  vec2 texCoord = (vec2(pixel) + vec2(0.5f)) / vec2(resolution);
  float roughness = textureLod(inNormalRoughness, texCoord, 0.0f).a;
  float metallic = textureLod(inPbrFlags, texCoord, 0.0f).x;

  vec4 color = textureLod(originalColor, texCoord, 0.0f);
  vec4 reflected = textureLod(reflectedColorVisibility, texCoord, 0.0f);

  float factor = clamp(reflected.a * metallic * (1.f - roughness), 0.0f, 1.0f);
  vec3 result = color.xyz * (1.0f - factor) + reflected.rgb * factor;
  imageStore(outColor, ivec2(pixel), vec4(result.xyz, color.w));
}
