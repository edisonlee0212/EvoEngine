import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EvoEngine.PerFrame;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
#define segment_pair_VERTICES_SIZE 8
#define segment_pair_TRIANGLE_SIZE 12



static const uint segment_pair_VERTEX_ITERATIONS = ((segment_pair_VERTICES_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
static const uint segment_pair_PRIMITIVE_ITERATIONS = ((segment_pair_TRIANGLE_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

struct EcoSysLabMeshVertex {
  float4 position : SV_Position;
  [[vk::location(0)]] float3 frag_pos : TEXCOORD0;
  [[vk::location(1)]] float3 normal : TEXCOORD1;
  [[vk::location(2)]] float3 tangent : TEXCOORD2;
  [[vk::location(3)]] float4 color : TEXCOORD3;
};

struct Task {
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};

static const float3 positions[] = {float3(-0.5, -0.5, -0.5), float3(0.5, 0.5, -0.5), float3(0.5, -0.5, -0.5), float3(-0.5, 0.5, -0.5),

                    float3(-0.5, -0.5, 0.5),  float3(0.5, -0.5, 0.5), float3(0.5, 0.5, 0.5),   float3(-0.5, 0.5, 0.5)};

static const uint3 triangles[] = {uint3(0, 1, 2), uint3(1, 0, 3), uint3(4, 5, 6),

                     uint3(6, 7, 4), uint3(7, 3, 0), uint3(0, 4, 7),

                     uint3(6, 2, 1), uint3(2, 6, 5), uint3(0, 2, 5),

                     uint3(5, 4, 0), uint3(3, 6, 1), uint3(6, 3, 7)};

[[vk::push_constant]] cbuffer STRANDS_RENDER_CONSTANTS {
  float4 color0;
  float4 color1;

  uint camera_index;
  uint segment_pairs_size;
  uint color_mode;
  float multiplier;
  float factor;
};



// group_id.x runs from [0 .. parentTask.groupCountX - 1]
[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload Task ts_in, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<EcoSysLabMeshVertex, segment_pair_VERTICES_SIZE> vertices,
          OutputIndices<uint3, segment_pair_TRIANGLE_SIZE> output_indices) {
  const uint laneID = lane_id;
  uint segment_pair_id = ts_in.baseID + ts_in.deltaIDs[group_id.x];

  if (segment_pair_id >= segment_pairs_size)
    return;

  SetMeshOutputCounts(segment_pair_VERTICES_SIZE, segment_pair_TRIANGLE_SIZE);

  SegmentPair segment_pair = segment_pairs[segment_pair_id];
  Segment segment0 = segments[segment_pair.segment0_handle];
  Particle segment0_particle0 = segment_particle0s[segment_pair.segment0_handle];
  Particle segment0_particle1 = segment_particle1s[segment_pair.segment0_handle];
  Segment segment1 = segments[segment_pair.segment1_handle];
  Particle segment1_particle0 = segment_particle0s[segment_pair.segment1_handle];
  Particle segment1_particle1 = segment_particle1s[segment_pair.segment1_handle];

  float3 start_position = (segment0_particle0.x + segment0_particle1.x) * 0.5f;

  float3 end_position = (segment1_particle0.x + segment1_particle1.x) * 0.5f;

  float3 center_position = (start_position + end_position) * 0.5f;
  float center_thickness = (segment0.radius + segment1.radius) * 0.1f * multiplier;

  float3 front = normalize(end_position - start_position);
  float4 rotation = quatLookAt(front, float3(front.y, front.z, front.x));
  float4x4 instance_matrix =
      scale(float3(center_thickness, center_thickness, distance(start_position, end_position))) *
      mat4_cast(rotation) * translate(center_position);
  float4x4 transform = mul(instance_matrix, EE_CAMERAS[camera_index].projection_view);

  float4 color = color0;
  if (color_mode == 0) {
    color = color0;
  } else if (color_mode == 1) {
    color =
        lerp(color0, color1,
            clamp(abs(segment_pair.bending_twist_bundle_strain.x) / segment_pair.bending_twist_bundle_strain_limit.x,
                  0.0, 1.0));
  } else if (color_mode == 2) {
    color =
        lerp(color0, color1,
            clamp(abs(segment_pair.bending_twist_bundle_strain.y) / segment_pair.bending_twist_bundle_strain_limit.y,
                  0.0, 1.0));
  } else if (color_mode == 3) {
    color =
        lerp(color0, color1,
            clamp(abs(segment_pair.bending_twist_bundle_strain.z) / segment_pair.bending_twist_bundle_strain_limit.z,
                  0.0, 1.0));
  } else if (color_mode == 4) {
    float bending = clamp(
        abs(segment_pair.bending_twist_bundle_strain.x) / segment_pair.bending_twist_bundle_strain_limit.x, 0.0, 1.0);
    float twisting = clamp(
        abs(segment_pair.bending_twist_bundle_strain.y) / segment_pair.bending_twist_bundle_strain_limit.y, 0.0, 1.0);
    float bundle = clamp(
        abs(segment_pair.bending_twist_bundle_strain.z) / segment_pair.bending_twist_bundle_strain_limit.z, 0.0, 1.0);
    color = lerp(color0, color1, max(max(bending, twisting), bundle));
  } else if (color_mode == 5) {
    float connectivity = segment_pair.connectivity_strain / segment_pair.connectivity_strain_limit;
    color = lerp(color0, color1, clamp(connectivity, 0.0, 1.0));
  } else if (color_mode == 6) {
    /*color = lerp(color0, color1, clamp(
                            abs(segment_pair.bending_twist_bundle_strain_limit.x) / factor, 0.0, 1.0));*/
    float bending = clamp(
        abs(segment_pair.bending_twist_bundle_strain.x) * 200.f / segment_pair.bending_twist_bundle_strain_limit.x, 0.0,
        1.0);
    color = lerp(color0, color1, bending);
  } else if (color_mode == 7) {
    /*color = lerp(color0, color1, clamp(
                            abs(segment_pair.bending_twist_bundle_strain_limit.y) / factor, 0.0, 1.0));*/
    float twisting = clamp(
        abs(segment_pair.bending_twist_bundle_strain.y) * 200.f / segment_pair.bending_twist_bundle_strain_limit.y, 0.0,
        1.0);
    color = lerp(color0, color1, twisting);
  } else if (color_mode == 8) {
    color = lerp(color0, color1, clamp(abs(segment_pair.bending_twist_bundle_strain_limit.z) / factor, 0.0, 1.0));
  } else if (color_mode == 9) {
    color = lerp(color0, color1, clamp(abs(segment_pair.connectivity_strain_limit) / factor, 0.0, 1.0));
  }

  if (segment0_particle0.highlighted == 1 || segment0_particle1.highlighted == 1 ||
      segment1_particle0.highlighted == 1 || segment1_particle1.highlighted == 1) {
    color = lerp(color, float4(0, 1, 0, 1), 0.8f);
  } else if (segment0_particle0.highlighted == 2 || segment0_particle1.highlighted == 2 ||
             segment1_particle0.highlighted == 2 || segment1_particle1.highlighted == 2) {
    color = lerp(color, float4(1, 0, 0, 1), 0.8f);
  } else if (segment0_particle0.selected == 1 || segment0_particle1.selected == 1 || segment1_particle0.selected == 1 ||
             segment1_particle1.selected == 1) {
    color = lerp(color, float4(1, 1, 1, 1), 0.8f);
  }

  [[unroll]]
  for (uint i = 0; i < uint(segment_pair_VERTEX_ITERATIONS); ++i) {
    uint vertex_index = laneID + i * WORKGROUP_SIZE;
    if (vertex_index >= segment_pair_VERTICES_SIZE)
      break;
    float3 vertex_position = positions[vertex_index];

    vertices[vertex_index].frag_pos = mul(float4(vertex_position, 1.0), instance_matrix).xyz;
    vertices[vertex_index].normal = float3(0, 1, 0);
    vertices[vertex_index].tangent = float3(0, 0, 1);
    vertices[vertex_index].color = color;
    vertices[vertex_index].position = mul(float4(vertex_position, 1.0), transform);
  }

  [[unroll]]
  for (uint i = 0; i < uint(segment_pair_PRIMITIVE_ITERATIONS); ++i) {
    uint triangle_index = laneID + i * WORKGROUP_SIZE;
    if (triangle_index < segment_pair_TRIANGLE_SIZE) {
      output_indices[triangle_index] = triangles[triangle_index];
    }
  }
}
