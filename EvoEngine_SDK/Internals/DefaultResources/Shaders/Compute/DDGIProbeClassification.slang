// @evoengine-dialect native
import EvoEngine.DDGI;
import EvoEngine.DDGIProbeRayData;

struct EeDdgiProbeClassificationConstants {
  uint4 probe_count_ray_count_and_flags;
  uint4 probe_counts;
  float4 classification_parameters;
  int4 probe_scroll_offset;
  float4 probe_step_x;
  float4 probe_step_y;
  float4 probe_step_z;
};

[[vk::push_constant]]
ConstantBuffer<EeDdgiProbeClassificationConstants> constants;

[[vk::binding(0, 0)]]
StructuredBuffer<DdgiProbeRayData> EE_DDGI_PROBE_RAY_DATA;

[[vk::binding(1, 0)]]
RWStructuredBuffer<float4> EE_DDGI_PROBE_STATE;

void EE_DDGI_WRITE_PROBE_CLASSIFICATION(uint probe_index, float inactive) {
  float4 state = EE_DDGI_PROBE_STATE[probe_index];
  state.w = inactive;
  EE_DDGI_PROBE_STATE[probe_index] = state;
}

[shader("compute")]
[numthreads(32, 1, 1)]
void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint probe_index = dispatch_thread_id.x;
  uint dispatch_probe_count = max(constants.probe_count_ray_count_and_flags.x, 1u);
  uint ray_count = max(constants.probe_count_ray_count_and_flags.y, 1u);
  bool reset_classification = constants.probe_count_ray_count_and_flags.w != 0u;
  if (probe_index >= dispatch_probe_count) {
    return;
  }

  if (reset_classification) {
    EE_DDGI_WRITE_PROBE_CLASSIFICATION(probe_index, 0.0f);
    return;
  }

  uint3 total_probe_counts = max(constants.probe_counts.xyz, uint3(1u));
  uint probes_per_layer = total_probe_counts.x * total_probe_counts.y;
  uint3 logical_probe_grid =
      uint3(probe_index % total_probe_counts.x, (probe_index / total_probe_counts.x) % total_probe_counts.y,
            probe_index / probes_per_layer);
  uint physical_probe_index =
      EE_DDGI_SCROLL_PROBE_INDEX(logical_probe_grid, constants.probe_scroll_offset.xyz, total_probe_counts);
  uint fixed_ray_count = min(constants.probe_counts.w, ray_count);
  if (fixed_ray_count == 0u) {
    EE_DDGI_WRITE_PROBE_CLASSIFICATION(physical_probe_index, 0.0f);
    return;
  }

  uint ray_offset = probe_index * ray_count;
  float backface_count = 0.0f;
  for (uint ray_index = 0u; ray_index < fixed_ray_count; ++ray_index) {
    DdgiProbeRayData ray_data = EE_DDGI_PROBE_RAY_DATA[ray_offset + ray_index];
    if (EE_DDGI_PROBE_RAY_BACKFACE_HIT(ray_data)) {
      backface_count += 1.0f;
    }
  }

  float fixed_ray_backface_threshold = clamp(constants.classification_parameters.x, 0.0f, 1.0f);
  if (backface_count / float(fixed_ray_count) > fixed_ray_backface_threshold) {
    EE_DDGI_WRITE_PROBE_CLASSIFICATION(physical_probe_index, 1.0f);
    return;
  }

  for (uint ray_index = 0u; ray_index < fixed_ray_count; ++ray_index) {
    DdgiProbeRayData ray_data = EE_DDGI_PROBE_RAY_DATA[ray_offset + ray_index];
    if (!EE_DDGI_PROBE_RAY_FRONTFACE_HIT(ray_data)) {
      continue;
    }
    float hit_distance = EE_DDGI_PROBE_RAY_SIGNED_DISTANCE(ray_data);
    float3 ray_direction = EE_DDGI_PROBE_RAY_DIRECTION(ray_index, ray_count, fixed_ray_count);
    float voxel_plane_distance = EE_DDGI_VOXEL_PLANE_DISTANCE(ray_direction, constants.probe_step_x.xyz,
                                                              constants.probe_step_y.xyz, constants.probe_step_z.xyz);
    if (hit_distance <= voxel_plane_distance) {
      EE_DDGI_WRITE_PROBE_CLASSIFICATION(physical_probe_index, 0.0f);
      return;
    }
  }
  EE_DDGI_WRITE_PROBE_CLASSIFICATION(physical_probe_index, 1.0f);
}
