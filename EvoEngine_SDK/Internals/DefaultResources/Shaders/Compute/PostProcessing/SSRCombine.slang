[[vk::binding(0, 1)]]
Sampler2D originalColor;

[[vk::binding(1, 1)]]
Sampler2D reflectedColorVisibility;

[[vk::binding(21, 2)]]
Sampler2D inNormalRoughness;

[[vk::binding(22, 2)]]
Sampler2D inPbrFlags;

[[vk::binding(0, 3)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outColor;

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
  float roughness = inNormalRoughness.SampleLevel(tex_coord, 0.0f).a;
  float metallic = inPbrFlags.SampleLevel(tex_coord, 0.0f).x;
  float4 color = originalColor.SampleLevel(tex_coord, 0.0f);
  float4 reflected = reflectedColorVisibility.SampleLevel(tex_coord, 0.0f);
  float factor = clamp(reflected.a * metallic * (1.0f - roughness), 0.0f, 1.0f);
  outColor[int2(pixel)] = float4(color.xyz * (1.0f - factor) + reflected.rgb * factor, color.w);
}
