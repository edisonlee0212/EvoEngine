import EvoEngine.PerFrame;
import EvoEngine.PointCloudRayTracingPayload;
import EvoEngine.DDGI;

struct DdgiProbeRaySample {
  float3 direction;
  float padding0;
  float3 start;
  uint hit_count;
  uint64_t handle;
  uint64_t padding1;
  HitInfo hit_info;
};

struct DdgiProbeRayVisualizationConstants {
  uint2 camera_ray_count;
  float2 miss_distance_alpha;
};

struct DdgiProbeRayVertexOutput {
  float4 position : SV_Position;
  [[vk::location(0)]]
  float4 color : COLOR0;
};

[[vk::binding(0, 1)]]
StructuredBuffer<DdgiProbeRaySample> EE_DDGI_PROBE_RAY_SAMPLES;

[[vk::push_constant]]
ConstantBuffer<DdgiProbeRayVisualizationConstants> constants;

float3 EeDdgiDebugRayEnd(DdgiProbeRaySample ray_sample) {
  float3 direction = EE_DDGI_SAFE_NORMALIZE(ray_sample.direction, float3(0.0f, 1.0f, 0.0f));
  if (EE_DDGI_IS_INACTIVE_PROBE_RAY(ray_sample.hit_count, ray_sample.hit_info.color.a))
    return ray_sample.start + direction * constants.miss_distance_alpha.x * 0.15f;
  if (ray_sample.hit_count == 0u) return ray_sample.start + direction * constants.miss_distance_alpha.x;
  return ray_sample.hit_info.position;
}

float4 EeDdgiDebugRayColor(DdgiProbeRaySample ray_sample) {
  float alpha = clamp(constants.miss_distance_alpha.y, 0.0f, 1.0f);
  float4 color = float4(0.1f, 0.55f, 1.0f, 0.35f * alpha);
  if (EE_DDGI_IS_INACTIVE_PROBE_RAY(ray_sample.hit_count, ray_sample.hit_info.color.a))
    color = float4(0.45f, 0.45f, 0.48f, 0.2f * alpha);
  else if (ray_sample.hit_count != 0u && ray_sample.hit_info.color.a < 0.0f)
    color = float4(1.0f, 0.35f, 0.08f, 0.8f * alpha);
  else if (ray_sample.hit_count != 0u)
    color = float4(0.15f, 1.0f, 0.35f, alpha);
  if (ray_sample.padding0 > 0.5f) {
    color.rgb = lerp(color.rgb, float3(1.0f, 0.95f, 0.15f), 0.45f);
    color.a = max(color.a, 0.6f * alpha);
  }
  return color;
}

[shader("vertex")]
DdgiProbeRayVertexOutput main(uint vertex_index : SV_VertexID) {
  DdgiProbeRayVertexOutput output;
  uint ray_index = vertex_index / 2u;
  DdgiProbeRaySample ray_sample = EE_DDGI_PROBE_RAY_SAMPLES[ray_index];
  float3 world_position = (vertex_index & 1u) == 0u ? ray_sample.start : EeDdgiDebugRayEnd(ray_sample);
  output.color = EeDdgiDebugRayColor(ray_sample);
  output.position = mul(float4(world_position, 1.0f), EE_CAMERAS[constants.camera_ray_count.x].projection_view);
  return output;
}
