#include "DDGI.slangh"
#include "DDGIProbeRayData.slangh"

struct EeDdgiProbeRelocationConstants {
  uint4 probe_count_ray_count_and_flags;
  uint4 probe_counts;
  float4 relocation_parameters;
  int4 probe_scroll_offset;
  float4 probe_step_x;
  float4 probe_step_y;
  float4 probe_step_z;
};

[[vk::push_constant]]
ConstantBuffer<EeDdgiProbeRelocationConstants> constants;

[[vk::binding(0, 0)]]
StructuredBuffer<DdgiProbeRayData> EE_DDGI_PROBE_RAY_DATA;

[[vk::binding(1, 0)]]
RWStructuredBuffer<float4> EE_DDGI_PROBE_STATE;

[shader("compute")]
[numthreads(32, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint probe_index = dispatch_thread_id.x;
  uint dispatch_probe_count = max(constants.probe_count_ray_count_and_flags.x, 1u);
  uint ray_count = max(constants.probe_count_ray_count_and_flags.y, 1u);
  bool reset_offsets = constants.probe_count_ray_count_and_flags.w != 0u;
  if (probe_index >= dispatch_probe_count) {
    return;
  }

  if (reset_offsets) {
    float4 state = EE_DDGI_PROBE_STATE[probe_index];
    state.xyz = float3(0.0f);
    EE_DDGI_PROBE_STATE[probe_index] = state;
    return;
  }

  uint3 total_probe_counts = max(constants.probe_counts.xyz, uint3(1u));
  uint probes_per_layer = total_probe_counts.x * total_probe_counts.y;
  uint3 logical_probe_grid =
      uint3(probe_index % total_probe_counts.x, (probe_index / total_probe_counts.x) % total_probe_counts.y,
            probe_index / probes_per_layer);
  uint physical_probe_index =
      EE_DDGI_SCROLL_PROBE_INDEX(logical_probe_grid, constants.probe_scroll_offset.xyz, total_probe_counts);
  float relocation_distance = max(constants.relocation_parameters.x, 0.0f);
  float fixed_ray_backface_threshold = clamp(constants.relocation_parameters.y, 0.0f, 1.0f);
  if (relocation_distance <= 0.0f) {
    float4 state = EE_DDGI_PROBE_STATE[physical_probe_index];
    state.xyz = float3(0.0f);
    EE_DDGI_PROBE_STATE[physical_probe_index] = state;
    return;
  }

  float3 relocation_offset = EE_DDGI_PROBE_STATE[physical_probe_index].xyz;
  uint fixed_ray_count = min(constants.probe_counts.w, ray_count);
  if (fixed_ray_count == 0u) {
    return;
  }

  uint invalid_ray_index = 0xffffffffu;
  uint closest_backface_ray_index = invalid_ray_index;
  uint closest_frontface_ray_index = invalid_ray_index;
  uint farthest_frontface_ray_index = invalid_ray_index;
  float closest_backface_distance = 1e27f;
  float closest_frontface_distance = 1e27f;
  float farthest_frontface_distance = 0.0f;
  float backface_count = 0.0f;
  uint ray_offset = probe_index * ray_count;
  for (uint ray_index = 0u; ray_index < fixed_ray_count; ++ray_index) {
    DdgiProbeRayData ray_data = EE_DDGI_PROBE_RAY_DATA[ray_offset + ray_index];
    if (!EE_DDGI_PROBE_RAY_HIT(ray_data)) {
      continue;
    }

    float signed_hit_distance = EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data);
    float hit_distance = abs(signed_hit_distance);
    if (EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data)) {
      backface_count += 1.0f;
      hit_distance *= 5.0f;
      if (hit_distance < closest_backface_distance) {
        closest_backface_distance = hit_distance;
        closest_backface_ray_index = ray_index;
      }
    } else {
      if (hit_distance < closest_frontface_distance) {
        closest_frontface_distance = hit_distance;
        closest_frontface_ray_index = ray_index;
      } else if (hit_distance > farthest_frontface_distance) {
        farthest_frontface_distance = hit_distance;
        farthest_frontface_ray_index = ray_index;
      }
    }
  }

  float3 candidate_offset = float3(1e27f);
  if (closest_backface_ray_index != invalid_ray_index &&
      backface_count / float(fixed_ray_count) > fixed_ray_backface_threshold) {
    float3 closest_backface_direction =
        EE_DDGI_PROBE_RAY_DIRECTION(closest_backface_ray_index, ray_count, fixed_ray_count);
    candidate_offset =
        relocation_offset + closest_backface_direction * (closest_backface_distance + relocation_distance * 0.5f);
  } else if (closest_frontface_ray_index != invalid_ray_index && closest_frontface_distance < relocation_distance &&
             farthest_frontface_ray_index != invalid_ray_index) {
    float3 closest_frontface_direction =
        EE_DDGI_PROBE_RAY_DIRECTION(closest_frontface_ray_index, ray_count, fixed_ray_count);
    float3 farthest_frontface_direction =
        EE_DDGI_PROBE_RAY_DIRECTION(farthest_frontface_ray_index, ray_count, fixed_ray_count);
    if (dot(closest_frontface_direction, farthest_frontface_direction) <= 0.0f) {
      candidate_offset = relocation_offset + farthest_frontface_direction * min(farthest_frontface_distance, 1.0f);
    }
  } else if (closest_frontface_distance > relocation_distance) {
    float relocation_offset_length = length(relocation_offset);
    if (relocation_offset_length > 0.0001f) {
      float move_back_margin = min(closest_frontface_distance - relocation_distance, relocation_offset_length);
      candidate_offset = relocation_offset - relocation_offset / relocation_offset_length * move_back_margin;
    } else {
      candidate_offset = float3(0.0f);
    }
  }

  if (EE_DDGI_RELOCATION_OFFSET_INSIDE_VOXEL(candidate_offset, constants.probe_step_x.xyz,
                                            constants.probe_step_y.xyz, constants.probe_step_z.xyz)) {
    relocation_offset = candidate_offset;
  }
  float4 output_state = EE_DDGI_PROBE_STATE[physical_probe_index];
  output_state.xyz = relocation_offset;
  EE_DDGI_PROBE_STATE[physical_probe_index] = output_state;
}
