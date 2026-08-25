import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EvoEngine.PerFrame;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
#define LEAF_VERTICES_SIZE 4
#define LEAF_TRIANGLE_SIZE 2



static const uint LEAF_VERTEX_ITERATIONS = ((LEAF_VERTICES_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
static const uint LEAF_PRIMITIVE_ITERATIONS = ((LEAF_TRIANGLE_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

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

static const float3 positions[] = {float3(-0.5, 0, -0.5), float3(-0.5, 0, 0.5), float3(0.5, 0, 0.5), float3(0.5, 0, -0.5)};

static const uint3 triangles[] = {
    uint3(0, 1, 2),
    uint3(2, 3, 0),
};

[[vk::push_constant]] cbuffer STRANDS_RENDER_CONSTANTS {
  float4 color0;
  float4 color1;

  uint camera_index;
  uint foliage_size;
  uint color_mode;
};



// group_id.x runs from [0 .. parentTask.groupCountX - 1]
[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload Task ts_in, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<EcoSysLabMeshVertex, LEAF_VERTICES_SIZE> vertices,
          OutputIndices<uint3, LEAF_TRIANGLE_SIZE> output_indices) {
  const uint laneID = lane_id;
  uint leaf_id = ts_in.baseID + ts_in.deltaIDs[group_id.x];

  if (leaf_id >= foliage_size)
    return;

  SetMeshOutputCounts(LEAF_VERTICES_SIZE, LEAF_TRIANGLE_SIZE);

  Leaf leaf = foliage[leaf_id];

  float3 leaf_position = leaf.x;
  float4 leaf_rotation = leaf.q;
  float3 leaf_scale = leaf.scale;
  float4x4 instance_matrix = mul(mul(scale(leaf_scale), mat4_cast(leaf_rotation)), translate(leaf_position));

  float4x4 transform = mul(instance_matrix, EE_CAMERAS[camera_index].projection_view);

  float4 color;
  if (color_mode == 0) {
    color = color0;
  } else if (color_mode == 1) {
    color = lerp(color0, color1, clamp(leaf.attachment_integrity, 0.0f, 1.f));
  } else if (color_mode == 2) {
    color = lerp(color0, color1, clamp(leaf.rotation_integrity, 0.0f, 1.f));
  }

  float highlight1_ratio = 0.0f;
  float highlight2_ratio = 0.0f;
  float selected_ratio = 0.0f;

  if (leaf.highlighted == 1) {
    highlight1_ratio += 0.8f;
  } else if (leaf.highlighted == 2) {
    highlight2_ratio += 0.8f;
  } else if (leaf.selected == 1) {
    selected_ratio += 0.8f;
  }

  if (highlight1_ratio != 0.f) {
    color = lerp(color, float4(0, 0, 1, 1), highlight1_ratio);
  } else if (highlight2_ratio != 0.f) {
    color = lerp(color, float4(1, 0, 1, 1), highlight2_ratio);
  } else if (selected_ratio != 0.f) {
    color = lerp(color, float4(1, 0, 0, 1), selected_ratio);
  }

  [[unroll]]
  for (uint i = 0; i < uint(LEAF_VERTEX_ITERATIONS); ++i) {
    uint vertex_index = laneID + i * WORKGROUP_SIZE;
    if (vertex_index >= LEAF_VERTICES_SIZE)
      break;
    float3 vertex_position = positions[vertex_index];

    vertices[vertex_index].frag_pos = mul(float4(vertex_position, 1.0), instance_matrix).xyz;
    vertices[vertex_index].normal = float3(0, 1, 0);
    vertices[vertex_index].tangent = float3(0, 0, 1);
    vertices[vertex_index].color = color;
    vertices[vertex_index].position = mul(float4(vertex_position, 1.0), transform);
  }

  [[unroll]]
  for (uint i = 0; i < uint(LEAF_PRIMITIVE_ITERATIONS); ++i) {
    uint triangle_index = laneID + i * WORKGROUP_SIZE;
    if (triangle_index < LEAF_TRIANGLE_SIZE) {
      output_indices[triangle_index] = triangles[triangle_index];
    }
  }
}
