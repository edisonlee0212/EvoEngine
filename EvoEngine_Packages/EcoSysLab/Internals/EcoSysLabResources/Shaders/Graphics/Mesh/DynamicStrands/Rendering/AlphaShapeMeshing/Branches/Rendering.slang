import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet1;
import EvoEngine.PerFrame;
import EcoSysLab.AlphaShapeRenderingConstants;
import EcoSysLab.AlphaShapeTopology;
import EcoSysLab.Math;
import EcoSysLab.VolumeMeasure;

// Volume scratch on strands descriptor set (graphics set 1).
[[vk::binding(16, 1)]] StructuredBuffer<float, Std430DataLayout> alpha_tet_current_volumes;
[[vk::binding(17, 1)]] StructuredBuffer<float, Std430DataLayout> alpha_tet_initial_volumes;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;

static const float PI = 3.1415926535897932384626433832795;
#define TETRAHEDRON_VERTICES_SIZE 12  // individual vertices for each triangle
#define TETRAHEDRON_TRIANGLE_SIZE 4



static const uint TETRAHEDRON_VERTEX_ITERATIONS = ((TETRAHEDRON_VERTICES_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
static const uint TETRAHEDRON_PRIMITIVE_ITERATIONS = ((TETRAHEDRON_TRIANGLE_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

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
float3 PseudoRandomColor(int seed) {
  // Constants for the pseudo-random number generator
  const int prime1 = 15731;
  const int prime2 = 789221;
  const int prime3 = 1376312589;

  // Generate pseudo-random numbers
  int n = (seed * prime1) ^ (seed * prime2) ^ (seed * prime3);
  n = (n >> 13) ^ n;

  // Normalize the values to be in the range [0, 1]
  float r = frac(sin(float(n) * 12.9898) * 43758.5453);
  float g = frac(sin(float(n + 1) * 12.9898) * 43758.5453);
  float b = frac(sin(float(n + 2) * 12.9898) * 43758.5453);

  // Return the color as a float3
  return float3(r, g, b);
}

void SetUpTriangleVertex(OutputVertices<EcoSysLabMeshVertex, TETRAHEDRON_VERTICES_SIZE> vertices, uint vertex_index,
                         in float3 vertex_normal, in UniformParticle uniform_particle, int is_bark, uint tet_id) {
  float3 vertex_position = uniform_particle.position_t.xyz +
                         vertex_normal * (global_extrusion_distance + uniform_particle.local_extrusion_distance);
  float4x4 transform = EE_CAMERAS[EE_CAMERA_INDEX].projection_view;

  vertices[vertex_index].FragPos = vertex_position;
  vertices[vertex_index].Normal = vertex_normal;
  vertices[vertex_index].Tangent = uniform_particle.tangent.xyz;

  if (is_bark != 0) {
    vertices[vertex_index].TexCoord =
        AlphaBarkPolarUv(uniform_particle, u_multiplier, v_multiplier, use_polar_coordinates_for_uv);
  } else {
    vertices[vertex_index].TexCoord = AlphaInteriorDiskUv(uniform_particle, texture_diameter);
  }

  // Deferred lighting multiplies material albedo by vertex color; white keeps bark/wood visible in Default.
  float4 color = float4(1, 1, 1, 1);

  int segment_handle = uniform_particle.segment_handle;

  bool highlight1 = false;
  bool highlight2 = false;
  bool selected = false;

  if (segment_particle0s[segment_handle].highlighted == 1) {
    highlight1 = true;
  }
  if (segment_particle0s[segment_handle].highlighted == 2) {
    highlight2 = true;
  }
  if (segment_particle0s[segment_handle].selected == 1) {
    selected = true;
  }
  if (segment_particle1s[segment_handle].highlighted == 1) {
    highlight1 = true;
  }
  if (segment_particle1s[segment_handle].highlighted == 2) {
    highlight2 = true;
  }
  if (segment_particle1s[segment_handle].selected == 1) {
    selected = true;
  }

  if (highlight2) {
    color = float4(1, 0, 1, 1.0);
  } else if (selected) {
    color = float4(1, 0, 0, 1.0);
  } else if (highlight1) {
    color = float4(0, 0, 1, 1.0);
  }

  vertices[vertex_index].Color = color;
  vertices[vertex_index].MaterialIndex = is_bark == 0 ? inner_wood_material_index : bark_material_index;

  if (vertex_colors == COLOR_DEFAULT) {
  } else if (vertex_colors == COLOR_NORMALS) {
    vertices[vertex_index].Color = float4(vertex_normal, 0.0);
  } else if (vertex_colors == COLOR_TANGENTS) {
    vertices[vertex_index].Color = uniform_particle.tangent;
  } else if (vertex_colors == COLOR_GROUPS) {
    vertices[vertex_index].Color = float4(PseudoRandomColor(segments[uniform_particle.segment_handle].group_index), 1.0);
  } else if (vertex_colors == COLOR_DEGREE) {
    vertices[vertex_index].Color =
        float4(fmod(floor(uniform_particle.normal_deg.w / 4.0f), 2.0f),
             fmod(floor(uniform_particle.normal_deg.w / 2.0f), 2.0f), fmod(uniform_particle.normal_deg.w, 2.0f), 1.0);
  } else if (vertex_colors == COLOR_BARK) {
    vertices[vertex_index].Color = float4(1.0 - is_bark, 0.0, is_bark, 1.0);
  } else if (vertex_colors == COLOR_NORMAL_QUATERNION) {
    vertices[vertex_index].Color = 0.5 * (float4(1, 1, 1, 1) + uniform_particle.normal_q);  // normal quaternion
  } else if (vertex_colors == COLOR_UP) {
    Segment segment = segments[uniform_particle.segment_handle];
    vertices[vertex_index].Color = float4(abs(rotate_vec3(segment.q, float3(0, 1, 0))), 1.0);  // up
  } else if (vertex_colors == COLOR_INIT_UP) {
    Segment segment = segments[uniform_particle.segment_handle];
    vertices[vertex_index].Color = float4(abs(rotate_vec3(segment.q0, float3(0, 1, 0))), 1.0);  // initial up
  } else if (vertex_colors == COLOR_AXIS) {
    Segment segment = segments[uniform_particle.segment_handle];
    vertices[vertex_index].Color = float4(abs(rotate_vec3(segment.q, float3(0, 0, 1))), 1.0);  // axis
  } else if (vertex_colors == COLOR_INIT_AXIS) {
    Segment segment = segments[uniform_particle.segment_handle];
    vertices[vertex_index].Color = float4(abs(rotate_vec3(segment.q0, float3(0, 0, 1))), 1.0);  // initial axis
  } else if (vertex_colors == COLOR_INIT_ANGLE) {
    float angle = uniform_particle.normal_deg.w;
    vertices[vertex_index].Color = float4(angle / PI, -angle / PI, 0, 1.0);  // init angle
  } else if (vertex_colors == COLOR_VOLUME_CHANGE_HEATMAP) {
    float initial_volume = alpha_tet_initial_volumes[tet_id];
    float current_volume = alpha_tet_current_volumes[tet_id];
    vertices[vertex_index].Color = float4(VolumeChangeHeatmapColor(initial_volume, current_volume), 1.0);
  }

  vertices[vertex_index].position = mul(float4(vertex_position, 1.0), transform);
}

[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload Task ts_in, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<EcoSysLabMeshVertex, TETRAHEDRON_VERTICES_SIZE> vertices,
          OutputIndices<uint3, TETRAHEDRON_TRIANGLE_SIZE> output_indices) {
  const uint laneID = lane_id;
  uint tet_id = ts_in.baseID + ts_in.deltaIDs[group_id.x];

  DelaunayTetrahedron tet = delaunay_tetrahedrons[tet_id];
  // report amount of accepted triangles
  SetMeshOutputCounts(TETRAHEDRON_VERTICES_SIZE, tet.triangles_accepted);

  // assign all accepted triangles
  [[unroll]]
  for (uint i = 0; i < uint(TETRAHEDRON_PRIMITIVE_ITERATIONS); ++i) {
    uint triangle_index = laneID + i * WORKGROUP_SIZE;
    if (triangle_index < TETRAHEDRON_TRIANGLE_SIZE) {
      if (tet.render_neighbor[triangle_index] == 1) {
        // count triangles downwards
        int index;
        InterlockedAdd(delaunay_tetrahedrons[tet_id].triangles_accepted, -1, index);
        output_indices[index - 1] = triangles[triangle_index];

        [[unroll]]
        for (int j = 0; j < 3; j++) {
          uint vertex_index = triangles[triangle_index][j];
          uint particle_index = tet.indices[lookup[vertex_index]];
          UniformParticle uniform_particle = uniform_particles[particle_index];

          if (uniform_particle.is_on_surface == 1) {
            Segment segment = segments[uniform_particle.segment_handle];
            Particle segment_particle0 = segment_particle0s[uniform_particle.segment_handle];
            Particle segment_particle1 = segment_particle1s[uniform_particle.segment_handle];
            // basis vectors of segment space
            float3 right = rotate_vec3(segment.q, float3(1, 0, 0));
            float3 up = rotate_vec3(segment.q, float3(0, 1, 0));
            float3 forward = rotate_vec3(segment.q, float3(0, 0, 1));

            // now do a basis transform from segment to world space
            float3x3 seg_to_world = float3x3(right, up, forward);
            float3 normal = normalize(mul(uniform_particle.normal_deg.xyz, seg_to_world));

            SetUpTriangleVertex(vertices, vertex_index, normal, uniform_particle, tet.is_bark[triangle_index], tet_id);
          } else {
            SetUpTriangleVertex(vertices, vertex_index, normalize(uniform_particle.normal_deg.xyz), uniform_particle,
                                tet.is_bark[triangle_index], tet_id);
          }
        }
      }
    }
  }
}
