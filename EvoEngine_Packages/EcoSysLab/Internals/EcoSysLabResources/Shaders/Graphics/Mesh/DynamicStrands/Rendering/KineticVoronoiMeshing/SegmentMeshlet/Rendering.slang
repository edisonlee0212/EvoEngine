import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.KineticVoronoiMeshingSet1;
import EvoEngine.PerFrame;
import EcoSysLab.Math;
import EcoSysLab.KineticVoronoiMeshing;
import EcoSysLab.KineticVoronoiRenderingConstants;
import EcoSysLab.VolumeMeasure;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const float PI = 3.1415926535897932384626433832795;
#define MAX_VERTEX_COUNT 3  // individual vertices for each triangle
#define MAX_PRIMITIVE_COUNT 1

static const uint VERTEX_ITERATIONS = ((MAX_VERTEX_COUNT + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
static const uint PRIMITIVE_ITERATIONS = ((MAX_PRIMITIVE_COUNT + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

// Volume scratch on strands descriptor set (graphics set 1).
[[vk::binding(14, 1)]] StructuredBuffer<float, Std430DataLayout> kinetic_segment_volumes;
[[vk::binding(18, 1)]] StructuredBuffer<float, Std430DataLayout> kinetic_segment_initial_volumes;

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

// Soft selection/highlight tint (rgb = tint, a = mix ratio).
float4 SelectionOverlayColor(int segment_index) {
  if (segment_index < 0) {
    return float4(0.0, 0.0, 0.0, 0.0);
  }
  Particle particle0 = segment_particle0s[segment_index];
  Particle particle1 = segment_particle1s[segment_index];

  float highlight1_ratio = 0.0;
  float highlight2_ratio = 0.0;
  float selected_ratio = 0.0;

  if (particle0.highlighted == 1) {
    highlight1_ratio += 0.4;
  }
  if (particle0.highlighted == 2) {
    highlight2_ratio += 0.4;
  }
  if (particle0.selected == 1) {
    selected_ratio += 0.4;
  }
  if (particle1.highlighted == 1) {
    highlight1_ratio += 0.4;
  }
  if (particle1.highlighted == 2) {
    highlight2_ratio += 0.4;
  }
  if (particle1.selected == 1) {
    selected_ratio += 0.4;
  }

  if (highlight2_ratio != 0.0) {
    return float4(1.0, 0.0, 1.0, highlight2_ratio);
  }
  if (selected_ratio != 0.0) {
    return float4(1.0, 0.0, 0.0, selected_ratio);
  }
  if (highlight1_ratio != 0.0) {
    return float4(0.0, 0.0, 1.0, highlight1_ratio);
  }
  return float4(0.0, 0.0, 0.0, 0.0);
}

// neighbor_segment_index: -2 = bark, -1/-3 = grey (never had / removed neighbor), >= 0 = lateral neighbor.
float4 NeighborConnectivityColor(SegmentMeshletTriangle tri) {
  if (tri.neighbor_segment_index == -2) {
    return float4(0.45, 0.28, 0.12, 1.0);
  }
  if (tri.neighbor_segment_index == -1 || tri.neighbor_segment_index == -3) {
    return float4(0.55, 0.55, 0.55, 1.0);
  }
  if (tri.segment_pair_index < 0) {
    return float4(1.0, 0.0, 0.0, 1.0);
  }
  SegmentPair pair = segment_pairs[tri.segment_pair_index];
  if (pair.connectivity_integrity <= 0.0 && pair.bend_twist_bundle_integrity <= 0.0) {
    return float4(1.0, 0.0, 0.0, 1.0);
  }
  return float4(0.0, 1.0, 0.0, 1.0);
}

// Raw neighbor-tag visualization: -2 bark, -1 open/interior, >=0 lateral, <-2 out-of-range.
float4 NeighborTagColor(SegmentMeshletTriangle tri) {
  if (tri.neighbor_segment_index == -2) {
    return float4(0.45, 0.28, 0.12, 1.0);
  }
  if (tri.neighbor_segment_index == -1) {
    return float4(0.15, 0.55, 1.0, 1.0);
  }
  if (tri.neighbor_segment_index >= 0) {
    return float4(0.15, 0.75, 0.25, 1.0);
  }
  return float4(0.95, 0.15, 0.85, 1.0);
}

float3 SetUpTriangleVertex(OutputVertices<EcoSysLabMeshVertex, MAX_VERTEX_COUNT> vertices, uint vertex_index,
                           in float3 vertex_normal, in SegmentMeshletVertex segment_meshlet_vertex, in float2 uv_coords,
                           in bool is_bark, in float4 debug_color) {
  float4x4 transform = EE_CAMERAS[EE_CAMERA_INDEX].projection_view;

  float3 pos = segment_meshlet_vertex.x - fracture_distance * segment_meshlet_vertex.shift;
  vertices[vertex_index].FragPos = pos;
  // Keep shading aligned with stored / exported normals (no legacy X flip).
  vertices[vertex_index].Normal = vertex_normal;
  vertices[vertex_index].Tangent = float3(0.0, 1.0, 0.0);  // TODO

  if (is_bark) {
    uv_coords.x *= uv_circum_factor;
    uv_coords.y *= uv_height_factor;
  }

  vertices[vertex_index].TexCoord = uv_coords;
  vertices[vertex_index].MaterialIndex = is_bark ? bark_material_index : inner_wood_material_index;

  float4 selection_overlay = SelectionOverlayColor(segment_meshlet_vertex.segment_index);

  switch (color_mode) {
    case COLOR_STANDARD:
      vertices[vertex_index].Color = selection_overlay;
      break;
    case COLOR_NORMALS: {
      float4 color = float4(vertex_normal, 1.0);
      if (selection_overlay.a > 0.0) {
        color.rgb = lerp(color.rgb, selection_overlay.rgb, selection_overlay.a);
      }
      vertices[vertex_index].Color = color;
      break;
    }
    case COLOR_UVS: {
      float4 color = float4(uv_coords, 0.0, 1.0);
      if (selection_overlay.a > 0.0) {
        color.rgb = lerp(color.rgb, selection_overlay.rgb, selection_overlay.a);
      }
      vertices[vertex_index].Color = color;
      break;
    }
    case COLOR_VOLUME_CHANGE_HEATMAP: {
      float initial_volume = kinetic_segment_initial_volumes[segment_meshlet_vertex.segment_index];
      float current_volume = kinetic_segment_volumes[segment_meshlet_vertex.segment_index];
      float4 color = float4(VolumeChangeHeatmapColor(initial_volume, current_volume), 1.0);
      if (selection_overlay.a > 0.0) {
        color.rgb = lerp(color.rgb, selection_overlay.rgb, selection_overlay.a);
      }
      vertices[vertex_index].Color = color;
      break;
    }
    case COLOR_NEIGHBORS:
    case COLOR_NEIGHBOR_CONNECTIVITY:
    case COLOR_NEIGHBOR_TAGS: {
      float4 color = debug_color;
      if (selection_overlay.a > 0.0) {
        color.rgb = lerp(color.rgb, selection_overlay.rgb, selection_overlay.a);
      }
      vertices[vertex_index].Color = color;
      break;
    }
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

  SegmentMeshletTriangle tri = segment_meshlet_triangles[tri_id];
  SetMeshOutputCounts(MAX_VERTEX_COUNT, 1);

  [[unroll]]
  for (uint i = 0; i < uint(PRIMITIVE_ITERATIONS); ++i) {
    uint triangle_index = laneID + i * WORKGROUP_SIZE;
    if (triangle_index < MAX_PRIMITIVE_COUNT) {
      float4 debug_color = float4(0.0, 0.0, 0.0, 0.0);
      if (color_mode == COLOR_NEIGHBOR_CONNECTIVITY) {
        debug_color = NeighborConnectivityColor(tri);
      } else if (color_mode == COLOR_NEIGHBOR_TAGS) {
        debug_color = NeighborTagColor(tri);
      } else if (color_mode == COLOR_NEIGHBORS) {
        debug_color =
            tri.segment_pair_index != -1 ? float4(0.0, 1.0, 0.0, 1.0) : float4(1.0, 0.0, 0.0, 1.0);
      }

      [[unroll]]
      for (int j = 0; j < 3; j++) {
        uint global_vertex_index = tri.vertex_indices[j];
        SegmentMeshletVertex segment_meshlet_vertex = segment_meshlet_vertices[global_vertex_index];
        SetUpTriangleVertex(vertices, j, tri.normal[j].xyz, segment_meshlet_vertex, tri.uv[j].xy,
                            tri.neighbor_segment_index == -2, debug_color);
      }

      output_indices[0] = uint3(0, 1, 2);
    }
  }
}
