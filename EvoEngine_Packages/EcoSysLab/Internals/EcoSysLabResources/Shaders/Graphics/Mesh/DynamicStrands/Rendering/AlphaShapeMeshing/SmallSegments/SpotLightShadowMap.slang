import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet1;
import EvoEngine.PerFrame;
import EcoSysLab.Math;
import EcoSysLab.DynamicStrandsSmallSegmentsRenderingConstants;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint SMALL_SEGMENT_VERTEX_ITERATIONS =
    ((SMALL_SEGMENT_SHADOW_MAP_VERTICES_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
static const uint SMALL_SEGMENT_PRIMITIVE_ITERATIONS =
    ((SMALL_SEGMENT_SHADOW_MAP_TRIANGLE_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

struct EcoSysLabMeshVertex {
  float4 position : SV_Position;
};

struct Task {
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};



// group_id.x runs from [0 .. parentTask.groupCountX - 1]
[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload Task ts_in, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<EcoSysLabMeshVertex, SMALL_SEGMENT_SHADOW_MAP_VERTICES_SIZE> vertices,
          OutputIndices<uint3, SMALL_SEGMENT_SHADOW_MAP_TRIANGLE_SIZE> output_indices) {
  const uint laneID = lane_id;
  uint uniform_particle_id = ts_in.baseID + ts_in.deltaIDs[group_id.x];

  if (uniform_particle_id >= uniform_particles_size)
    return;

  SetMeshOutputCounts(SMALL_SEGMENT_SHADOW_MAP_VERTICES_SIZE, SMALL_SEGMENT_SHADOW_MAP_TRIANGLE_SIZE);

  UniformParticle uniform_particle = uniform_particles[uniform_particle_id];
  UniformParticle prev_uniform_particle = uniform_particles[uniform_particle.prev_particle_handle];

  Segment segment = segments[uniform_particle.segment_handle];
  Segment prev_segment = segments[prev_uniform_particle.segment_handle];

  float3 start_position = prev_uniform_particle.position_t.xyz;
  float3 end_position = uniform_particle.position_t.xyz;

  if (segment.group_index != prev_segment.group_index) {
    start_position = end_position - rotate_vec3(segment.q, float3(0, 0, -1)) * segment.rest_length * 0.2f;
  }
  start_position *= position_scale.xyz;
  end_position *= position_scale.xyz;
  float3 center_position = (start_position + end_position) * 0.5f;

  float3 front = normalize(end_position - start_position);
  float4 uniform_particle_rotation = quatLookAt(front, float3(front.y, front.z, front.x));

  float radius = segment.radius * 2.f * thickness_multiplier;
  float3 uniform_particle_scale = float3(radius, radius, distance(start_position, end_position));

  float4x4 instance_matrix =
      mul(mul(scale(uniform_particle_scale), mat4_cast(uniform_particle_rotation)), translate(center_position));

  float4x4 transform = mul(instance_matrix, EE_SPOT_LIGHTS[EE_CAMERA_INDEX].light_space_matrix);

  [[unroll]]
  for (uint i = 0; i < uint(SMALL_SEGMENT_VERTEX_ITERATIONS); ++i) {
    uint vertex_index = laneID + i * WORKGROUP_SIZE;
    if (vertex_index >= SMALL_SEGMENT_SHADOW_MAP_VERTICES_SIZE)
      break;
    float3 vertex_position = shadow_map_positions[vertex_index];

    vertices[vertex_index].position = mul(float4(vertex_position, 1.0f), transform);
  }

  [[unroll]]
  for (uint i = 0; i < uint(SMALL_SEGMENT_PRIMITIVE_ITERATIONS); ++i) {
    uint triangle_index = laneID + i * WORKGROUP_SIZE;
    if (triangle_index < SMALL_SEGMENT_SHADOW_MAP_TRIANGLE_SIZE) {
      output_indices[triangle_index] = shadow_map_triangles[triangle_index];
    }
  }
}
