import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet1;
import EvoEngine.PerFrame;
import EcoSysLab.AlphaShapeRenderingConstants;
import EcoSysLab.AlphaShapeTopology;
import EcoSysLab.Math;

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

void SetUpTriangleVertex(OutputVertices<EcoSysLabMeshVertex, TETRAHEDRON_VERTICES_SIZE> vertices, uint vertex_index, in float3 vertex_normal, in UniformParticle uniform_particle, int is_bark) {
  float3 vertex_position = uniform_particle.position_t.xyz +
                         vertex_normal * (global_extrusion_distance + uniform_particle.local_extrusion_distance);
  float4x4 transform = EE_CAMERAS[EE_CAMERA_INDEX].projection_view;

  vertices[vertex_index].FragPos = vertex_position;
  vertices[vertex_index].Normal = vertex_normal;
  vertices[vertex_index].Tangent = uniform_particle.tangent.xyz;

  if (use_polar_coordinates_for_uv == 1) {
    float tex_x = fmod(uniform_particle.profile_polar_coordinate.y * u_multiplier / 3.1415926f, 2.f);
    if (tex_x > 1.f) {
      tex_x = 2.f - tex_x;
    }
    float tex_y = float(uniform_particle.segment_index) * v_multiplier;
    vertices[vertex_index].TexCoord = float2(tex_x, tex_y);
  } else {
    float tex_x = (uniform_particle.initial_position.y + uniform_particle.initial_position.z) * u_multiplier;
    float tex_y = uniform_particle.initial_position.x * v_multiplier;
    vertices[vertex_index].TexCoord = float2(tex_x, tex_y);
  }

  float4 color = float4(0, 0, 0, 0);

  int segment_handle = uniform_particle.segment_handle;

  bool highlight1 = false;
  bool highlight2 = false;
  bool selected = false;

  if (segments[segment_handle].particle0.highlighted == 1) {
    highlight1 = true;
  }
  if (segments[segment_handle].particle0.highlighted == 2) {
    highlight2 = true;
  }
  if (segments[segment_handle].particle0.selected == 1) {
    selected = true;
  }
  if (segments[segment_handle].particle1.highlighted == 1) {
    highlight1 = true;
  }
  if (segments[segment_handle].particle1.highlighted == 2) {
    highlight2 = true;
  }
  if (segments[segment_handle].particle1.selected == 1) {
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

  if (vertex_colors == 0) {
  } else if (vertex_colors == 1) {
    vertices[vertex_index].Color = float4(vertex_normal, 0.0);
  } else if (vertex_colors == 2) {
    vertices[vertex_index].Color = uniform_particle.tangent;
  } else if (vertex_colors == 3) {
    vertices[vertex_index].Color = float4(PseudoRandomColor(segments[uniform_particle.segment_handle].group_index), 1.0);
  } else if (vertex_colors == 4) {
    vertices[vertex_index].Color =
        float4(fmod(floor(uniform_particle.normal_deg.w / 4.0f), 2.0f),
             fmod(floor(uniform_particle.normal_deg.w / 2.0f), 2.0f), fmod(uniform_particle.normal_deg.w, 2.0f), 1.0);
  } else if (vertex_colors == 5) {
    vertices[vertex_index].Color = float4(1.0 - is_bark, 0.0, is_bark, 1.0);
  } else if (vertex_colors == 6) {
    vertices[vertex_index].Color = 0.5 * (float4(1, 1, 1, 1) + uniform_particle.normal_q);  // normal quaternion
  } else if (vertex_colors == 7) {
    Segment segment = segments[uniform_particle.segment_handle];
    vertices[vertex_index].Color = float4(abs(rotate_vec3(segment.q, float3(0, 1, 0))), 1.0);  // up
  } else if (vertex_colors == 8) {
    Segment segment = segments[uniform_particle.segment_handle];
    vertices[vertex_index].Color = float4(abs(rotate_vec3(segment.q0, float3(0, 1, 0))), 1.0);  // initial up
  } else if (vertex_colors == 9) {
    Segment segment = segments[uniform_particle.segment_handle];
    float3 up = rotate_vec3(segment.q, float3(0, 1, 0));
    vertices[vertex_index].Color = float4(abs(rotate_vec3(segment.q, float3(0, 0, 1))), 1.0);  // axis
  } else if (vertex_colors == 10) {
    Segment segment = segments[uniform_particle.segment_handle];
    float3 up = rotate_vec3(segment.q0, float3(0, 1, 0));
    vertices[vertex_index].Color = float4(abs(rotate_vec3(segment.q0, float3(0, 0, 1))), 1.0);  // initial axis
  } else if (vertex_colors == 11) {
    Segment segment = segments[uniform_particle.segment_handle];
    float3 up = rotate_vec3(segment.q0, float3(0, 1, 0));
    // Compute the angle between the two vectors
    float angle = uniform_particle.normal_deg.w;
    vertices[vertex_index].Color = float4(angle / PI, -angle / PI, 0, 1.0);  // init angle
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
            // basis vectors of segment space
            float3 right = rotate_vec3(segment.q, float3(1, 0, 0));
            float3 up = rotate_vec3(segment.q, float3(0, 1, 0));
            float3 forward = rotate_vec3(segment.q, float3(0, 0, 1));

            // now do a basis transform from segment to world space
            float3x3 seg_to_world = float3x3(right, up, forward);
            float3 normal = normalize(mul(uniform_particle.normal_deg.xyz, seg_to_world));

            SetUpTriangleVertex(vertices, vertex_index, normal, uniform_particle, tet.is_bark[triangle_index]);
          } else {
            SetUpTriangleVertex(vertices, vertex_index, normalize(uniform_particle.normal_deg.xyz), uniform_particle,
                                tet.is_bark[triangle_index]);
          }
        }
      }
    }
  }
}
