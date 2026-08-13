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
[[vk::binding(20, 2)]] StructuredBuffer<DdgiEmissiveGuideInfo> EE_DDGI_EMISSIVE_GUIDES;
[[vk::binding(21, 2)]] RWStructuredBuffer<DdgiProbeRaySampleInfo> EE_DDGI_PROBE_RAY_SAMPLE_INFO;

float4 EE_DDGI_PROBE_RAY_ROTATION() {
  return normalize(float4(EE_DDGI_PROBE_RAY_CONSTANTS.first_probe.w, EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_x.w,
                          EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_y.w, EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_z.w));
}

float3 EE_DDGI_RAY_DIRECTION(const uint ray_index, const uint ray_count, const uint fixed_ray_count) {
  return EE_DDGI_ROTATED_PROBE_RAY_DIRECTION(ray_index, ray_count, fixed_ray_count, EE_DDGI_PROBE_RAY_ROTATION());
}

float EE_DDGI_GUIDE_RAW_WEIGHT(const uint guide_index, const float3 origin) {
  DdgiEmissiveGuideInfo guide = EE_DDGI_EMISSIVE_GUIDES[guide_index];
  float3 delta = guide.center_and_radius.xyz - origin;
  float radius = guide.center_and_radius.w;
  float power = guide.power_and_reserved.x;
  float distance2 = dot(delta, delta);
  if (!(radius > 0.0f) || !(power > 0.0f) || !isfinite(radius) || !isfinite(power) || !isfinite(distance2)) {
    return 0.0f;
  }
  return min(power / max(distance2, max(radius * radius, 1e-6f)), 1e30f);
}

void EE_DDGI_GUIDE_CAP(const uint guide_index, const float3 origin, out float3 axis, out float cos_theta_max,
                       out float inverse_solid_angle) {
  DdgiEmissiveGuideInfo guide = EE_DDGI_EMISSIVE_GUIDES[guide_index];
  float3 delta = guide.center_and_radius.xyz - origin;
  float radius = guide.center_and_radius.w;
  float distance2 = dot(delta, delta);
  float distance = sqrt(max(distance2, 0.0f));
  if (distance <= radius) {
    axis = float3(0.0f, 0.0f, 1.0f);
    cos_theta_max = -1.0f;
    inverse_solid_angle = 1.0f / (4.0f * EE_DDGI_PI);
  } else {
    axis = delta / distance;
    float sin_theta_max = clamp(radius / distance, 0.0f, 1.0f);
    cos_theta_max = sqrt(max(1.0f - sin_theta_max * sin_theta_max, 0.0f));
    inverse_solid_angle = 1.0f / max(2.0f * EE_DDGI_PI * (1.0f - cos_theta_max), 1e-8f);
  }
}

float EE_DDGI_GUIDE_WEIGHT_SUM(const float3 origin, const uint guide_count) {
  float weight_sum = 0.0f;
  for (uint guide_index = 0u; guide_index < guide_count; ++guide_index) {
    weight_sum += EE_DDGI_GUIDE_RAW_WEIGHT(guide_index, origin);
  }
  return weight_sum;
}

float3 EE_DDGI_SAMPLE_CAP(const float3 axis, const float cos_theta_max, const float2 sample) {
  float cos_theta = lerp(1.0f, cos_theta_max, sample.x);
  float sin_theta = sqrt(max(1.0f - cos_theta * cos_theta, 0.0f));
  float phi = 2.0f * EE_DDGI_PI * sample.y;
  float3 helper = abs(axis.z) < 0.999f ? float3(0.0f, 0.0f, 1.0f) : float3(0.0f, 1.0f, 0.0f);
  float3 tangent = EE_DDGI_SAFE_NORMALIZE(cross(helper, axis), float3(1.0f, 0.0f, 0.0f));
  float3 bitangent = cross(axis, tangent);
  return EE_DDGI_SAFE_NORMALIZE(
      axis * cos_theta + tangent * (cos(phi) * sin_theta) + bitangent * (sin(phi) * sin_theta), axis);
}

float EE_DDGI_GUIDED_DIRECTION_PDF(const float3 origin, const float3 direction, const uint guide_count,
                                   const float weight_sum) {
  if (!(weight_sum > 0.0f)) {
    return 1.0f / (4.0f * EE_DDGI_PI);
  }
  float pdf = 0.0f;
  for (uint guide_index = 0u; guide_index < guide_count; ++guide_index) {
    float raw_weight = EE_DDGI_GUIDE_RAW_WEIGHT(guide_index, origin);
    if (raw_weight > 0.0f) {
      float3 axis;
      float cos_theta_max;
      float inverse_solid_angle;
      EE_DDGI_GUIDE_CAP(guide_index, origin, axis, cos_theta_max, inverse_solid_angle);
      if (dot(axis, direction) >= cos_theta_max) {
        pdf += raw_weight * inverse_solid_angle / weight_sum;
      }
    }
  }
  return pdf;
}

float3 EE_DDGI_SAMPLE_GUIDED_DIRECTION(const float3 origin, const uint guide_count, inout uint seed,
                                       out float weight_sum) {
  weight_sum = EE_DDGI_GUIDE_WEIGHT_SUM(origin, guide_count);
  float3 sample = EE_PCG_RANDOM_3(seed);
  if (!(weight_sum > 0.0f)) {
    return EE_DDGI_SAMPLE_CAP(float3(0.0f, 0.0f, 1.0f), -1.0f, sample.yz);
  }
  float selector = sample.x * weight_sum;
  uint selected_guide = 0u;
  float cumulative_weight = 0.0f;
  for (uint guide_index = 0u; guide_index < guide_count; ++guide_index) {
    float weight = EE_DDGI_GUIDE_RAW_WEIGHT(guide_index, origin);
    if (weight > 0.0f) {
      selected_guide = guide_index;
      cumulative_weight += weight;
      if (selector <= cumulative_weight) {
        break;
      }
    }
  }
  float3 axis;
  float cos_theta_max;
  float inverse_solid_angle;
  EE_DDGI_GUIDE_CAP(selected_guide, origin, axis, cos_theta_max, inverse_solid_angle);
  return EE_DDGI_SAMPLE_CAP(axis, cos_theta_max, sample.yz);
}

[shader("raygeneration")] void main() {
  const uint ray_count = EE_DDGI_TOTAL_RAY_COUNT();
  const uint uniform_ray_count = EE_DDGI_UNIFORM_RAY_COUNT();
  const uint guided_ray_count = EE_DDGI_GUIDED_RAY_COUNT();
  const uint guide_count = EE_DDGI_EMISSIVE_GUIDE_COUNT();
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
  const uint fixed_ray_count = min(EE_DDGI_FIXED_RAY_COUNT(), uniform_ray_count);
  const bool fixed_ray = ray_index < fixed_ray_count;
  const bool guided_ray = ray_index >= uniform_ray_count;
  const uint probes_per_layer = probe_counts.x * probe_counts.y;
  const uint3 probe_grid = uint3(probe_index % probe_counts.x, (probe_index / probe_counts.x) % probe_counts.y,
                                 probe_index / probes_per_layer);
  const uint physical_probe_index =
      EE_DDGI_SCROLL_PROBE_INDEX(probe_grid, EE_DDGI_PROBE_RAY_CONSTANTS.probe_scroll_offset.xyz, probe_counts);
  const float4 probe_state = EE_DDGI_PROBE_STATE[physical_probe_index];
  const float3 origin = EE_DDGI_PROBE_RAY_CONSTANTS.first_probe.xyz +
                        EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_x.xyz * probe_grid.x +
                        EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_y.xyz * probe_grid.y +
                        EE_DDGI_PROBE_RAY_CONSTANTS.probe_step_z.xyz * probe_grid.z + probe_state.xyz;
  const bool inactive_probe = skip_inactive_probe_trace && probe_state.w > 0.5f;
  const bool capture_diagnostics =
      local_probe_index == EE_DDGI_PROBE_RAY_CONSTANTS.selected_probe_volume_flags_environment.x;
  uint primary_ray_seed =
      EE_XXHASH32(uint3(uint(EE_DDGI_PROBE_RAY_CONSTANTS.probe_scroll_offset.w), probe_index, ray_index));
  if (primary_ray_seed == EE_DDGI_FIXED_RAY_PAYLOAD_FLAG) {
    primary_ray_seed ^= 0xa511e9b3u;
  }
  float guide_weight_sum = 0.0f;
  float3 direction;
  if (guided_ray) {
    uint guide_seed = primary_ray_seed ^ 0x68bc21ebu;
    direction = EE_DDGI_SAMPLE_GUIDED_DIRECTION(origin, guide_count, guide_seed, guide_weight_sum);
  } else {
    direction = EE_DDGI_RAY_DIRECTION(ray_index, uniform_ray_count, fixed_ray_count);
    if (guided_ray_count > 0u) {
      guide_weight_sum = EE_DDGI_GUIDE_WEIGHT_SUM(origin, guide_count);
    }
  }
  if (guided_ray_count > 0u) {
    float guided_pdf = EE_DDGI_GUIDED_DIRECTION_PDF(origin, direction, guide_count, guide_weight_sum);
    uint uniform_blend_ray_count = uniform_ray_count - fixed_ray_count;
    uint blend_ray_count = uniform_blend_ray_count + guided_ray_count;
    float mixture_pdf =
        (float(uniform_blend_ray_count) * (1.0f / (4.0f * EE_DDGI_PI)) + float(guided_ray_count) * guided_pdf) /
        max(float(blend_ray_count), 1.0f);
    DdgiProbeRaySampleInfo sample_info = EE_DDGI_PACK_PROBE_RAY_SAMPLE_INFO(
        direction, isfinite(mixture_pdf) && mixture_pdf > 0.0f ? rcp(mixture_pdf) : 0.0f);
    EE_DDGI_PROBE_RAY_SAMPLE_INFO[sample_index] = sample_info;
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
    signed_hit_distance =
        hit_value.hit_info.color.a < 0.0f ? hit_distance * EE_DDGI_PROBE_RAY_BACKFACE_DISTANCE_SCALE : hit_distance;
  }
  DdgiProbeRayData ray_data;
  ray_data.radiance_and_signed_distance = float4(max(hit_value.hit_info.color.rgb, float3(0.0f)), signed_hit_distance);
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
