import EvoEngine.LightingFixedSet3;

[[vk::binding(0, 4)]]
Sampler2D originalColor;

[[vk::binding(1, 4)]]
Sampler2D reflectedColorVisibility;

[[vk::binding(17, 1)]]
Sampler2D inDepth;

[[vk::binding(23, 1)]] Sampler2D inEmissive;

[[vk::binding(0, 5)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outColor;

float3 ComposeReflection(float2 tex_coord, float3 original, float4 reflected) {
  if (EE_BASIC_CONSTANTS.meshlet_culling_flags != 0) return reflected.rgb;
  float ndc_depth = inDepth.SampleLevel(tex_coord, 0.0f).x;
  if (!isfinite(ndc_depth) || ndc_depth >= 1.0f || reflected.a <= 0.0f || EE_INDIRECT_LIGHTING_DEBUG_VIEW() != 0) {
    return original;
  }

  if (inEmissive.SampleLevel(tex_coord, 0.0f).a < 0.0f) return original;
  float3 result = original + reflected.rgb;
  return max(all(isfinite(result)) ? result : original, float3(0.0f));
}

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  uint width = 0;
  uint height = 0;
  outColor.GetDimensions(width, height);
  uint2 resolution = uint2(width, height);
  if (any(pixel >= resolution)) return;

  float2 tex_coord = (float2(pixel) + float2(0.5f)) / float2(resolution);
  float4 original = originalColor.SampleLevel(tex_coord, 0.0f);
  float4 reflected = reflectedColorVisibility.SampleLevel(tex_coord, 0.0f);
  outColor[int2(pixel)] = float4(ComposeReflection(tex_coord, original.rgb, reflected), original.a);
}
