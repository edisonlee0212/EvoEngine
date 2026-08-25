import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.KineticVoronoiMeshingSet1;
import EvoEngine.PerFrame;
import EcoSysLab.Math;
import EcoSysLab.KineticVoronoiMeshing;
import EcoSysLab.KineticVoronoiRenderingConstants;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const float PI = 3.1415926535897932384626433832795;
#define MAX_VERTEX_COUNT 3  // individual vertices for each triangle
#define MAX_PRIMITIVE_COUNT 1



static const uint VERTEX_ITERATIONS = ((MAX_VERTEX_COUNT + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
static const uint PRIMITIVE_ITERATIONS = ((MAX_PRIMITIVE_COUNT + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

struct EcoSysLabMeshVertex {
  float4 position : SV_Position;
  [[vk::location(0)]] float3 FragPos : TEXCOORD0;
  [[vk::location(1)]] float3 Normal : TEXCOORD1;
  [[vk::location(2)]] float3 Tangent : TEXCOORD2;
  [[vk::location(3)]] float2 TexCoord : TEXCOORD3;
  [[vk::location(4)]] float4 Color : TEXCOORD4;
  [[vk::location(5)]] nointerpolation int MaterialIndex;
};

struct Task {
  uint baseID;
  uint8_t deltaIDs[EXT_TASK_WORK_GROUP_INVOCATIONS];
};
// group_id.x runs from [0 .. parentTask.groupCountX - 1]
float3 SetUpTriangleVertex(OutputVertices<EcoSysLabMeshVertex, MAX_VERTEX_COUNT> vertices, uint vertex_index,
                           in float3 vertex_normal, in SegmentMeshletVertex segment_meshlet_vertex, in float4 color,
                           in float2 uv_coords, in bool is_bark, in bool has_pair_handle) {
  float4x4 transform = EE_CAMERAS[EE_CAMERA_INDEX].projection_view;

  float3 pos = segment_meshlet_vertex.x - fracture_distance * segment_meshlet_vertex.shift;
  vertices[vertex_index].FragPos = pos;
  // this is certainly incorrect, but somehow the lighting computation expects the x-coordinate to be inverted
  vertices[vertex_index].Normal = float3(-vertex_normal.x, vertex_normal.y, vertex_normal.z);
  vertices[vertex_index].Tangent = float3(0.0, 1.0, 0.0);  // TODO

  if (is_bark) {
    uv_coords.x *= uv_circum_factor;
    uv_coords.y *= uv_height_factor;
  }

  vertices[vertex_index].TexCoord = uv_coords;
  vertices[vertex_index].MaterialIndex = is_bark ? bark_material_index : inner_wood_material_index;

  switch (color_mode) {
    case COLOR_STANDARD:
      vertices[vertex_index].Color = color;
      break;
    case COLOR_NORMALS:
      vertices[vertex_index].Color = float4(vertex_normal, 1.0);
      break;
    case COLOR_UVS:
      vertices[vertex_index].Color = float4(uv_coords, 0.0, 1.0);
      break;
    case COLOR_NEIGHBORS:
      if (has_pair_handle) {
        vertices[vertex_index].Color = float4(0.0, 1.0, 0.0, 1.0);
      } else {
        vertices[vertex_index].Color = float4(1.0, 0.0, 0.0, 1.0);
      }

      break;

  }

  vertices[vertex_index].position = mul(float4(pos, 1.0), transform);

  return segment_meshlet_vertex.x;
}

[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload Task ts_in, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<EcoSysLabMeshVertex, MAX_VERTEX_COUNT> vertices,
          OutputIndices<uint3, MAX_PRIMITIVE_COUNT> output_indices) {
  const uint laneID = lane_id;
  uint tri_id = ts_in.baseID + ts_in.deltaIDs[group_id.x];

  // TODO: the loop structure here does not make sense
  SegmentMeshletTriangle tri = segment_meshlet_triangles[tri_id];
  // report amount of accepted triangles
  SetMeshOutputCounts(MAX_VERTEX_COUNT, 1);

  // assign all accepted triangles
  [[unroll]]
  for (uint i = 0; i < uint(PRIMITIVE_ITERATIONS); ++i) {
    uint triangle_index = laneID + i * WORKGROUP_SIZE;  // should always be 0 in our setup
    if (triangle_index < MAX_PRIMITIVE_COUNT) {
      float4 color = float4(0, 0, 0, 0);

      [[unroll]]
      for (int j = 0; j < 3; j++) {
        uint global_vertex_index = tri.vertex_indices[j];
        SegmentMeshletVertex segment_meshlet_vertex = segment_meshlet_vertices[global_vertex_index];
        SetUpTriangleVertex(vertices, j, tri.normal[j].xyz, segment_meshlet_vertex, color, tri.uv[j].xy,
                            tri.neighbor_segment_index == -2, tri.segment_pair_index != -1);
      }

      output_indices[0] = uint3(0, 1, 2);
    }
  }
}
