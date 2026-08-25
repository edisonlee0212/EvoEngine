import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet1;
import EvoEngine.PerFrame;
import EcoSysLab.Math;
import EcoSysLab.DynamicStrandsSmallSegmentsRenderingVisualizationConstants;

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

  if (group_index != prev_group_index) {
    start_position = end_position - rotate_vec3(segments[uniform_particle.segment_handle].q, float3(0, 0, -1)) *
                                        segments[uniform_particle.segment_handle].rest_length * 0.2f;
  }
  start_position *= position_scale.xyz;
  end_position *= position_scale.xyz;
  float3 center_position = (start_position + end_position) * 0.5f;
  float3 front = normalize(end_position - start_position);
  float4 uniform_particle_rotation = quatLookAt(front, float3(front.y, front.z, front.x));
  float radius = segments[uniform_particle.segment_handle].radius * 4.f * thickness_multiplier;
  float3 uniform_particle_scale = float3(radius, radius, distance(start_position, end_position));
  float4x4 instance_matrix =
      mul(mul(scale(uniform_particle_scale), mat4_cast(uniform_particle_rotation)), translate(center_position));
  float4x4 transform = mul(instance_matrix, EE_CAMERAS[EE_CAMERA_INDEX].projection_view);

  float4 color;
  if (color_mode == 0) {
    color = color0;
  } else if (color_mode == 1) {
    color = segments[segments[uniform_particle.segment_handle].node_handle].color;
  } else if (color_mode == 2) {
    color = segments[uniform_particle.segment_handle].group_index == 0
                ? float4(0.6, 0.3, 0.0, 1.0)
                : segments[segments[uniform_particle.segment_handle].group_index].color;
  } else if (color_mode == 3) {
    color = lerp(color0, color1,
                clamp(fmod(segments[uniform_particle.segment_handle].boundary_distance, factor) / factor, 0.0, 1.0));
  } else if (color_mode == 4) {
    color = lerp(color0, color1, clamp(segments[uniform_particle.segment_handle].strength, 0.0f, 1.0f));
  } else if (color_mode == 5) {
    color = lerp(color0, color1,
                clamp(abs(segments[uniform_particle.segment_handle].shear_stretch_strain) /
                          segments[uniform_particle.segment_handle].shear_stretch_strain_limit,
                      0.0, 1.0));
  } else if (color_mode == 6) {
    color = lerp(color0, color1,
                clamp(segments[uniform_particle.segment_handle].shear_stretch_strain_limit / factor, 0.0, 1.0));
  } else if (color_mode == 7) {
    color = segments[uniform_particle.segment_handle].color;
  } else if (color_mode == 8) {
    color = segments[segments[uniform_particle.segment_handle].strand_handle].color;
  }

  bool highlight1 = false;
  bool highlight2 = false;
  bool selected = false;

  if (segments[uniform_particle.segment_handle].particle0.highlighted == 1) {
    highlight1 = true;
  }
  if (segments[uniform_particle.segment_handle].particle0.highlighted == 2) {
    highlight2 = true;
  }
  if (segments[uniform_particle.segment_handle].particle0.selected == 1) {
    selected = true;
  }
  if (segments[uniform_particle.segment_handle].particle1.highlighted == 1) {
    highlight1 = true;
  }
  if (segments[uniform_particle.segment_handle].particle1.highlighted == 2) {
    highlight2 = true;
  }
  if (segments[uniform_particle.segment_handle].particle1.selected == 1) {
    selected = true;
  }

  if (highlight2) {
    color = float4(1, 0, 1, 1);
  } else if (selected) {
    color = float4(1, 0, 0, 1);
  } else if (highlight1) {
    color = float4(0, 0, 1, 1);
  }

  [[unroll]]
  for (uint i = 0; i < uint(SMALL_SEGMENT_VERTEX_ITERATIONS); ++i) {
    uint vertex_index = laneID + i * WORKGROUP_SIZE;
    if (vertex_index > SMALL_SEGMENT_VERTICES_SIZE)
      break;

    float3 vertex_position = positions[vertex_index];

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
