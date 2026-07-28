#extension GL_ARB_shading_language_include : enable

layout(set = 0, binding = 0) uniform sampler2D inputColor;
layout(set = 0, binding = 1, rgba32f) uniform writeonly image2D blendColor;
layout(set = 0, binding = 2, rgba32f) uniform writeonly image2D edgeColor;

layout(push_constant) uniform EE_SMAA_CONSTANTS {
  vec4 metrics;
  vec4 subsampleIndices;
  int toneMapped;
  int debugMode;
  int padding0;
  int padding1;
} pc;

vec3 LinearToSrgb(vec3 color) {
  vec3 low = color * 12.92f;
  vec3 high = 1.055f * pow(max(color, vec3(0.0f)), vec3(1.0f / 2.4f)) - 0.055f;
  return mix(low, high, greaterThan(color, vec3(0.0031308f)));
}

vec3 SrgbToLinear(vec3 color) {
  vec3 low = color / 12.92f;
  vec3 high = pow((max(color, vec3(0.0f)) + 0.055f) / 1.055f, vec3(2.4f));
  return mix(low, high, greaterThan(color, vec3(0.04045f)));
}

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatchThreadId : SV_DispatchThreadID) {
  ivec2 pixel = ivec2(dispatchThreadId.xy);
  ivec2 size = textureSize(inputColor, 0);
  if (any(greaterThanEqual(pixel, size))) {
    return;
  }

  vec4 inputValue = texelFetch(inputColor, pixel, 0);
  vec4 blendValue = inputValue;
  vec4 edgeValue = inputValue;
  if (pc.toneMapped != 0) {
    blendValue.rgb = SrgbToLinear(clamp(inputValue.rgb, vec3(0.0f), vec3(1.0f)));
  } else {
    blendValue.rgb = max(inputValue.rgb, vec3(0.0f));
    float luminance = dot(blendValue.rgb, vec3(0.2126f, 0.7152f, 0.0722f));
    edgeValue.rgb = LinearToSrgb(clamp(blendValue.rgb / (1.0f + luminance), vec3(0.0f), vec3(1.0f)));
  }
  imageStore(blendColor, pixel, blendValue);
  imageStore(edgeColor, pixel, edgeValue);
}
