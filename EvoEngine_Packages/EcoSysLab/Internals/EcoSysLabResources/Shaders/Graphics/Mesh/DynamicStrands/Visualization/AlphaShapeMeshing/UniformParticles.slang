import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EcoSysLab.AlphaShapeMeshing;
import EcoSysLab.AlphaShapeMeshingSet1;
import EvoEngine.PerFrame;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
#define PARTICLE_VERTICES_SIZE 8
#define PARTICLE_TRIANGLE_SIZE 12



static const uint PARTICLE_VERTEX_ITERATIONS = ((PARTICLE_VERTICES_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
static const uint PARTICLE_PRIMITIVE_ITERATIONS = ((PARTICLE_TRIANGLE_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

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
  uint uniform_particles_size;
  uint color_mode;
  float multiplier;
};



// group_id.x runs from [0 .. parentTask.groupCountX - 1]
[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload Task ts_in, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<EcoSysLabMeshVertex, PARTICLE_VERTICES_SIZE> vertices,
          OutputIndices<uint3, PARTICLE_TRIANGLE_SIZE> output_indices) {
  const uint laneID = lane_id;
  uint uniform_particle_id = ts_in.baseID + ts_in.deltaIDs[group_id.x];

  if (uniform_particle_id >= uniform_particles_size)
    return;

  SetMeshOutputCounts(PARTICLE_VERTICES_SIZE, PARTICLE_TRIANGLE_SIZE);

  UniformParticle uniform_particle = uniform_particles[uniform_particle_id];
  Segment segment = segments[uniform_particle.segment_handle];
  float3 center_position = uniform_particle.position_t.xyz;
  float center_thickness = segment.radius * 3.0;
  float3 uniform_particle_size = float3(center_thickness * multiplier, center_thickness * multiplier, segment.radius);
  float4 color = color0;

  if (color_mode == 1) {
    color = segment.color;
  } else if (color_mode == 2) {
    if (uniform_particle.is_single_strand_particle == 0) {
      color = float4(0.0f, 1.0f, 1.0f, 1.0f);
    } else {
      color = float4(1.0f, 0.0f, 0.0f, 1.0f);
    }
  }

  float3 front = normalize(rotate_vec3(segment.q, float3(0, 0, -1)));
  float4 rotation = quatLookAt(front, float3(front.y, front.z, front.x));

  float4x4 instance_matrix = mul(mul(scale(uniform_particle_size), mat4_cast(rotation)), translate(center_position));
  float4x4 transform = mul(instance_matrix, EE_CAMERAS[camera_index].projection_view);

  Particle particle0 = segment.particle0;
  Particle particle1 = segment.particle1;

  float highlight1_ratio = 0.0f;
  float highlight2_ratio = 0.0f;
  float selected_ratio = 0.0f;

  if (particle0.highlighted == 1) {
    highlight1_ratio += 0.4f;
  } else if (particle0.highlighted == 2) {
    highlight2_ratio += 0.4f;
  } else if (particle0.selected == 1) {
    selected_ratio += 0.4f;
  }
  if (particle1.highlighted == 1) {
    highlight1_ratio += 0.4f;
  } else if (particle1.highlighted == 2) {
    highlight2_ratio += 0.4f;
  } else if (particle1.selected == 1) {
    selected_ratio += 0.4f;
  }

  if (highlight1_ratio != 0.f) {
    color = lerp(color, float4(0, 1, 0, 1), highlight1_ratio);
  } else if (highlight2_ratio != 0.f) {
    color = lerp(color, float4(1, 0, 0, 1), highlight2_ratio);
  } else if (selected_ratio != 0.f) {
    color = lerp(color, float4(1, 1, 1, 1), selected_ratio);
  }

  [[unroll]]
  for (uint i = 0; i < uint(PARTICLE_VERTEX_ITERATIONS); ++i) {
    uint vertex_index = laneID + i * WORKGROUP_SIZE;
    if (vertex_index > PARTICLE_VERTICES_SIZE)
      break;
    float3 vertex_position = positions[vertex_index];

    vertices[vertex_index].frag_pos = mul(float4(vertex_position, 1.0), instance_matrix).xyz;
    vertices[vertex_index].normal = float3(0, 1, 0);
    vertices[vertex_index].tangent = float3(0, 0, 1);
    vertices[vertex_index].color = color;
    vertices[vertex_index].position = mul(float4(vertex_position, 1.0), transform);
  }

  [[unroll]]
  for (uint i = 0; i < uint(PARTICLE_PRIMITIVE_ITERATIONS); ++i) {
    uint triangle_index = laneID + i * WORKGROUP_SIZE;
    if (triangle_index <= PARTICLE_TRIANGLE_SIZE) {
      output_indices[triangle_index] = triangles[triangle_index];
    }
  }
}
