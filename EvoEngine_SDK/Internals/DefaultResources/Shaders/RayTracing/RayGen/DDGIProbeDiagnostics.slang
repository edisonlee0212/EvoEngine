// @evoengine-dialect native
import EvoEngine.DDGI;
import EvoEngine.DDGIProbeRayConstants;
import EvoEngine.DDGIProbeRayData;
import EvoEngine.PointCloudRayTracingPayload;
import EvoEngine.Random;
import EvoEngine.RayTracingBasic;

struct DdgiProbeRaySample {
  float3 direction;
  float padding0;
  float3 start;
  uint hit_count;
  uint64_t handle;
  uint64_t padding1;
  HitInfo hit_info;
};

[[vk::binding(0, 2)]] RWStructuredBuffer<DdgiProbeRayData> EE_DDGI_PROBE_RAY_DATA;
[[vk::binding(1, 2)]] StructuredBuffer<float4> EE_DDGI_PROBE_STATE;
[[vk::binding(2, 2)]] RWStructuredBuffer<DdgiProbeRaySample> EE_DDGI_SELECTED_RAY_DIAGNOSTICS;

float4 EE_DDGI_PROBE_RAY_ROTATION() {
  return normalize(float4(EE_DDGI_PROBE_RAY_CONSTANTS.first_probe.w,
                          EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_x.w,
                          EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_y.w,
                          EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_z.w));
}

float3 EE_DDGI_RAY_DIRECTION(const uint ray_index, const uint ray_count, const uint fixed_ray_count) {
  return EE_DDGI_ROTATED_PROBE_RAY_DIRECTION(ray_index, ray_count, fixed_ray_count, EE_DDGI_PROBE_RAY_ROTATION());
}

[shader("raygeneration")]
void main() {
  const uint ray_count = max(EE_DDGI_PROBE_RAY_CONSTANTS.probe_counts_and_ray_count.w, 1u);
  const uint sample_index = DispatchRaysIndex().x;
  const uint local_probe_index = sample_index / ray_count;
  const uint probe_index = local_probe_index;
  const uint ray_index = sample_index - local_probe_index * ray_count;
  const uint3 probe_counts = max(EE_DDGI_PROBE_RAY_CONSTANTS.probe_counts_and_ray_count.xyz, uint3(1u));
  if (probe_index >= probe_counts.x * probe_counts.y * probe_counts.z) {
    return;
  }
  const bool skip_inactive_probe_trace =
      (EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.z & (1u << 0u)) != 0u;
  const uint fixed_ray_count =
      min(uint(max(EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.z, 0.0f)), ray_count);
  const bool fixed_ray = ray_index < fixed_ray_count;
  const uint probes_per_layer = probe_counts.x * probe_counts.y;
  const uint3 probe_grid = uint3(probe_index % probe_counts.x, (probe_index / probe_counts.x) % probe_counts.y,
                                 probe_index / probes_per_layer);
  const uint physical_probe_index = EE_DDGI_SCROLL_PROBE_INDEX(
      probe_grid, EE_DDGI_PROBE_RAY_CONSTANTS.probe_scroll_offset.xyz, probe_counts);
  const float4 probe_state = EE_DDGI_PROBE_STATE[physical_probe_index];
  const float3 origin = EE_DDGI_PROBE_RAY_CONSTANTS.first_probe.xyz +
                        EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_x.xyz * probe_grid.x +
                        EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_y.xyz * probe_grid.y +
                        EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_z.xyz * probe_grid.z + probe_state.xyz;
  const float3 direction = EE_DDGI_RAY_DIRECTION(ray_index, ray_count, fixed_ray_count);
  const bool inactive_probe = skip_inactive_probe_trace && probe_state.w > 0.5f;
  const bool capture_diagnostics =
      local_probe_index == EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.x;
  uint primary_ray_seed = EE_XXHASH32(
      uint3(uint(EE_DDGI_PROBE_RAY_CONSTANTS.probe_scroll_offset.w), probe_index, ray_index));
  if (primary_ray_seed == EE_DDGI_FIXED_RAY_PAYLOAD_FLAG) {
    primary_ray_seed ^= 0xa511e9b3u;
  }

  PointCloudRayTracingPayload hit_value = {};
  hit_value.seed = fixed_ray ? EE_DDGI_FIXED_RAY_PAYLOAD_FLAG : primary_ray_seed;
  hit_value.hit_info.position = origin;
  if (inactive_probe && !fixed_ray) {
    hit_value.hit_info.color.a = EE_DDGI_INACTIVE_RAY_ALPHA;
  } else {
    RayDesc ray = {origin, 0.0f, direction, EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.x};
    TraceRay(EE_TLAS, RAY_FLAG_NONE, EE_DDGI_RAY_MASK_GEOMETRY, 0, 0, 0, ray, hit_value);
  }

  float signed_hit_distance = EE_DDGI_PROBE_RAY_MISS_DISTANCE;
  if (inactive_probe && !fixed_ray) {
    signed_hit_distance = EE_DDGI_PROBE_RAY_INACTIVE_DISTANCE;
  } else if (hit_value.hit_count != 0u) {
    const float hit_distance = max(hit_value.hit_info.vertex_info1, 0.0f);
    signed_hit_distance = hit_value.hit_info.color.a < 0.0f
                              ? hit_distance * EE_DDGI_PROBE_RAY_BACKFACE_DISTANCE_SCALE
                              : hit_distance;
  }
  DdgiProbeRayData ray_data;
  ray_data.radiance_and_signed_distance =
      float4(max(hit_value.hit_info.color.rgb, float3(0.0f)), signed_hit_distance);
  EE_DDGI_PROBE_RAY_DATA[sample_index] = ray_data;

  if (capture_diagnostics) {
    DdgiProbeRaySample diagnostic_sample;
    diagnostic_sample.direction = direction;
    diagnostic_sample.padding0 = fixed_ray ? 1.0f : 0.0f;
    diagnostic_sample.start = origin;
    diagnostic_sample.hit_count = hit_value.hit_count;
    diagnostic_sample.handle = hit_value.handle;
    diagnostic_sample.padding1 = uint64_t(0);
    diagnostic_sample.hit_info = hit_value.hit_info;
    EE_DDGI_SELECTED_RAY_DIAGNOSTICS[ray_index] = diagnostic_sample;
  }
}
