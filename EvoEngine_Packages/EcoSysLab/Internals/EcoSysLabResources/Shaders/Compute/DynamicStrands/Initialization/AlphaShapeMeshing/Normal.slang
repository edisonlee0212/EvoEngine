import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet0;
import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = COMPUTE_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint ITERATIONS_PER_TASK =
    (COMPUTE_WORK_GROUP_INVOCATIONS + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE;

struct PushConstants {
  uint uniform_particle_size;
};

[[vk::push_constant]] ConstantBuffer<PushConstants> constants;

[shader("compute")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
void main(uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex) {
  const uint base_id = group_id.x * COMPUTE_WORK_GROUP_INVOCATIONS;
  [[unroll]] for (uint i = 0; i < ITERATIONS_PER_TASK; ++i) {
    const uint particle_index = base_id + lane_id + i * WORKGROUP_SIZE;
    if (particle_index >= constants.uniform_particle_size)
      break;

    UniformParticle particle = uniform_particles[particle_index];
    if (particle.is_on_surface == 2) {
      uniform_particles[particle_index].is_on_surface = 0;
      continue;
    }
    if (all(particle.normal_deg == float4(0.0f))) {
      uniform_particles[particle_index].normal_q = float4(0.0f);
      continue;
    }

    particle.normal_deg.xyz = normalize(ProjectOntoPlane(particle.normal_deg.xyz, particle.tangent.xyz));
    Segment segment = segments[particle.segment_handle];
    const float3 right = rotate_vec3(segment.q0, float3(1, 0, 0));
    const float3 up = rotate_vec3(segment.q0, float3(0, 1, 0));
    const float3 forward = rotate_vec3(segment.q0, float3(0, 0, 1));
    particle.normal_deg.xyz = float3(dot(particle.normal_deg.xyz, right), dot(particle.normal_deg.xyz, up),
                                     dot(particle.normal_deg.xyz, forward));
    particle.is_on_surface = 1;
    uniform_particles[particle_index] = particle;
  }
}
