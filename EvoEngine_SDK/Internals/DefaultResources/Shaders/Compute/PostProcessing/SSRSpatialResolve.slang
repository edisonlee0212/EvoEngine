import EvoEngine.SSRConstants;

[[vk::binding(0, 0)]]
Sampler2D inReflectionConfidence;

[[vk::binding(1, 0)]]
[vk::image_format("rgba32f")]
RWTexture2D<float4> outReflectionConfidence;

[[vk::binding(17, 1)]]
Sampler2D inDepth;

[[vk::binding(21, 1)]]
Sampler2D inNormalRoughness;

[shader("compute")]
[numthreads(16, 16, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint2 pixel = dispatch_thread_id.xy;
  uint width = 0;
  uint height = 0;
  outReflectionConfidence.GetDimensions(width, height);
  uint2 resolution = uint2(width, height);
  if (any(pixel >= resolution)) return;

  float2 texel_size = 1.0f / float2(resolution);
  float2 tex_coord = (float2(pixel) + float2(0.5f)) * texel_size;
  float4 center = inReflectionConfidence.SampleLevel(tex_coord, 0.0f);
  if (EE_SSR_CONSTANTS.debug_mode != 0 || center.a <= 0.0001f || center.a >= 0.85f) {
    outReflectionConfidence[int2(pixel)] = center;
    return;
  }

  float center_depth = inDepth.SampleLevel(tex_coord, 0.0f).x;
  float4 center_normal_roughness = inNormalRoughness.SampleLevel(tex_coord, 0.0f);
  float center_normal_length = length(center_normal_roughness.xyz);
  if (!isfinite(center_depth) || center_depth >= 1.0f || !isfinite(center_normal_length) ||
      center_normal_length <= 0.00001f) {
    outReflectionConfidence[int2(pixel)] = center;
    return;
  }

  float3 center_normal = center_normal_roughness.xyz / center_normal_length;
  float3 radiance_sum = center.rgb;
  float confidence_sum = center.a;
  float weight_sum = 1.0f;
  for (int y = -2; y <= 2; ++y) {
    for (int x = -2; x <= 2; ++x) {
      if (x == 0 && y == 0) continue;
      float2 sample_uv = tex_coord + float2(x, y) * texel_size;
      if (any(sample_uv <= float2(0.0f)) || any(sample_uv >= float2(1.0f))) continue;
      float4 sample_reflection = inReflectionConfidence.SampleLevel(sample_uv, 0.0f);
      if (sample_reflection.a <= 0.0001f || !all(isfinite(sample_reflection))) continue;
      float sample_depth = inDepth.SampleLevel(sample_uv, 0.0f).x;
      float4 sample_normal_roughness = inNormalRoughness.SampleLevel(sample_uv, 0.0f);
      float sample_normal_length = length(sample_normal_roughness.xyz);
      if (!isfinite(sample_depth) || sample_depth >= 1.0f || !isfinite(sample_normal_length) ||
          sample_normal_length <= 0.00001f) continue;
      float3 sample_normal = sample_normal_roughness.xyz / sample_normal_length;
      float spatial_weight = exp(-0.5f * float(x * x + y * y));
      float depth_weight = exp(-abs(sample_depth - center_depth) * 512.0f);
      float normal_weight = pow(max(dot(center_normal, sample_normal), 0.0f), 32.0f);
      float roughness_weight = exp(-abs(sample_normal_roughness.a - center_normal_roughness.a) * 16.0f);
      float weight = spatial_weight * depth_weight * normal_weight * roughness_weight * sample_reflection.a;
      radiance_sum += sample_reflection.rgb * weight;
      confidence_sum += sample_reflection.a * weight;
      weight_sum += weight;
    }
  }

  if (weight_sum <= 0.0001f) {
    outReflectionConfidence[int2(pixel)] = center;
    return;
  }
  float3 resolved_radiance = radiance_sum / weight_sum;
  float resolved_confidence = max(center.a, confidence_sum / weight_sum * 0.9f);
  outReflectionConfidence[int2(pixel)] = float4(resolved_radiance, clamp(resolved_confidence, 0.0f, 1.0f));
}
