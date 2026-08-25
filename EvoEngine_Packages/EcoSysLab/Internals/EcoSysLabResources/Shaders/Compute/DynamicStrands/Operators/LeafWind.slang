import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;
import EcoSysLab.Noise;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const uint ITERATIONS_PER_TASK = ((COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
[[vk::push_constant]] cbuffer PUSH_CONSTANTS {
  float3 main_force;
  uint leaf_size;

  float turbulence_direction_frequency;
  float turbulence_speed_frequency;
  float turbulence_strength;
  float simulated_time;
};

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex, uint3 dispatch_thread_id : SV_DispatchThreadID) {
  const uint baseID = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  const uint laneID = lane_id;
  [[unroll]]
  for (uint i = 0; i < ITERATIONS_PER_TASK; i++) {
    uint local_index = laneID + i * WORKGROUP_SIZE;
    uint leaf_index = baseID + local_index;

    if (leaf_index >= leaf_size)
      break;

    float3 position = foliage[leaf_index].x.xyz;

    float3 up = rotate_vec3(foliage[leaf_index].q, float3(0, 1, 0));

    float3 turbulence_direction;
    turbulence_direction.x =
        EE_SIMPLEX_NOISE(float4((position + float3(100, 100, 100)) * turbulence_direction_frequency, simulated_time)) *
            2.f -
        1.f;
    turbulence_direction.y =
        EE_SIMPLEX_NOISE(float4((position + float3(200, 200, 200)) * turbulence_direction_frequency, simulated_time)) *
            2.f -
        1.f;
    turbulence_direction.z =
        EE_SIMPLEX_NOISE(float4((position + float3(300, 300, 300)) * turbulence_direction_frequency, simulated_time)) *
            2.f -
        1.f;

    float turbulence_base_speed =
        EE_SIMPLEX_NOISE(float4((position + float3(400, 400, 400)) * turbulence_speed_frequency, simulated_time));

    float max_turbulence_speed = length(main_force) * turbulence_strength;
    float3 turbulance = length(turbulence_direction) == 0.f
                          ? float3(0.0f)
                          : normalize(turbulence_direction) * max_turbulence_speed * turbulence_base_speed;
    float3 wind_force = main_force + turbulance;

    float directional_scale = max(0.05f, 1.f - abs(dot(up, normalize(wind_force))));

    foliage[leaf_index].acceleration += wind_force * directional_scale * foliage[leaf_index].inv_mass;
  }
}