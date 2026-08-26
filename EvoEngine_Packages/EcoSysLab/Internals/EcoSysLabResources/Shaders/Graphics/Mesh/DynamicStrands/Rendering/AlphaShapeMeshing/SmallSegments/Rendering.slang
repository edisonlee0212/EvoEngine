import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet1;
import EvoEngine.PerFrame;
import EcoSysLab.Math;
import EcoSysLab.DynamicStrandsSmallSegmentsRenderingConstants;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
static const uint SMALL_SEGMENT_VERTEX_ITERATIONS = ((SMALL_SEGMENT_VERTICES_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
static const uint SMALL_SEGMENT_PRIMITIVE_ITERATIONS = ((SMALL_SEGMENT_TRIANGLE_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

struct EcoSysLabMeshVertex {
  float4 position : SV_Position;
  [[vk::location(0)]] float3 FragPos : TEXCOORD0;
  [[vk::location(1)]] float3 Normal : TEXCOORD1;
  [[vk::location(2)]] float3 Tangent : TEXCOORD2;
  [[vk::location(3)]] float2 TexCoord : TEXCOORD3;
  [[vk::location(4)]] float4 Color : TEXCOORD4;
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
          OutputVertices<EcoSysLabMeshVertex, SMALL_SEGMENT_VERTICES_SIZE> vertices,
          OutputIndices<uint3, SMALL_SEGMENT_TRIANGLE_SIZE> output_indices) {
  const uint laneID = lane_id;
  uint uniform_particle_id = ts_in.baseID + ts_in.deltaIDs[group_id.x];

  if (uniform_particle_id >= uniform_particles_size)
    return;

  SetMeshOutputCounts(SMALL_SEGMENT_VERTICES_SIZE, SMALL_SEGMENT_TRIANGLE_SIZE);

  UniformParticle uniform_particle = uniform_particles[uniform_particle_id];
  UniformParticle prev_uniform_particle = uniform_particles[uniform_particle.prev_particle_handle];

  int group_index = segments[uniform_particle.segment_handle].group_index;
  int prev_group_index = segments[prev_uniform_particle.segment_handle].group_index;

  float3 start_position = prev_uniform_particle.position_t.xyz;
  float3 end_position = uniform_particle.position_t.xyz;

  bool start_is_tip = false;
  bool end_is_tip = false;

  if (group_index != prev_group_index) {
    start_position = end_position - rotate_vec3(segments[uniform_particle.segment_handle].q, float3(0, 0, -1)) *
                                        segments[uniform_particle.segment_handle].rest_length * 0.2f;
    start_is_tip = true;
  }

  if (uniform_particle.next_particle_handle != -1) {
    if (group_index != segments[uniform_particles[uniform_particle.next_particle_handle].segment_handle].group_index) {
      end_is_tip = true;
    }
  }

  float4 color = float4(0, 0, 0, 0);

  if (uniform_particle.is_bark == 0) {
    color.a = 0.8f;
  }

  bool highlight1 = false;
  bool highlight2 = false;
  bool selected = false;

  if (segment_particle0s[uniform_particle.segment_handle].highlighted == 1) {
    highlight1 = true;
  }
  if (segment_particle0s[uniform_particle.segment_handle].highlighted == 2) {
    highlight2 = true;
  }
  if (segment_particle0s[uniform_particle.segment_handle].selected == 1) {
    selected = true;
  }
  if (segment_particle1s[uniform_particle.segment_handle].highlighted == 1) {
    highlight1 = true;
  }
  if (segment_particle1s[uniform_particle.segment_handle].highlighted == 2) {
    highlight2 = true;
  }
  if (segment_particle1s[uniform_particle.segment_handle].selected == 1) {
    selected = true;
  }

  if (highlight2) {
    color = float4(1, 0, 1, 1);
  } else if (selected) {
    color = float4(1, 0, 0, 1);
  } else if (highlight1) {
    color = float4(0, 0, 1, 1);
  }

  start_position *= position_scale.xyz;
  end_position *= position_scale.xyz;

  float3 center_position = (start_position + end_position) * 0.5f;

  float3 front = normalize(end_position - start_position);
  float4 uniform_particle_rotation = quatLookAt(front, float3(front.y, front.z, front.x));

  float radius = segments[uniform_particle.segment_handle].radius * 2.f * thickness_multiplier;
  float3 uniform_particle_scale = float3(radius, radius, distance(start_position, end_position));

  float4x4 instance_matrix =
      mul(mul(scale(uniform_particle_scale), mat4_cast(uniform_particle_rotation)), translate(center_position));

  float4x4 transform = mul(instance_matrix, EE_CAMERAS[EE_CAMERA_INDEX].projection_view);

  [[unroll]]
  for (uint i = 0; i < uint(SMALL_SEGMENT_VERTEX_ITERATIONS); ++i) {
    uint vertex_index = laneID + i * WORKGROUP_SIZE;
    if (vertex_index > SMALL_SEGMENT_VERTICES_SIZE)
      break;

    float3 vertex_position = positions0[vertex_index];

    if (start_is_tip && end_is_tip) {
      vertex_position = positions3[vertex_index];
    } else if (start_is_tip && !end_is_tip) {
      vertex_position = positions2[vertex_index];
    } else if (!start_is_tip && end_is_tip) {
      vertex_position = positions1[vertex_index];
    }

    vertices[vertex_index].FragPos = mul(float4(vertex_position, 1.0f), instance_matrix).xyz;
    vertices[vertex_index].TexCoord = tex_coords[vertex_index];
    vertices[vertex_index].Normal = normalize(mul(float4(normals[vertex_index], 0.0f), instance_matrix).xyz);
    vertices[vertex_index].Tangent = normalize(mul(float4(tangents[vertex_index], 0.0f), instance_matrix).xyz);
    vertices[vertex_index].Color = color;
    vertices[vertex_index].position = mul(float4(vertex_position, 1.0f), transform);
  }

  [[unroll]]
  for (uint i = 0; i < uint(SMALL_SEGMENT_PRIMITIVE_ITERATIONS); ++i) {
    uint triangle_index = laneID + i * WORKGROUP_SIZE;
    if (triangle_index <= SMALL_SEGMENT_TRIANGLE_SIZE) {
      output_indices[triangle_index] = triangles[triangle_index];
    }
  }
}
