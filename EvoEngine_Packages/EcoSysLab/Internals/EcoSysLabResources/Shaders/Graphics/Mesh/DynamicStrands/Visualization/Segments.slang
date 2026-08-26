import EcoSysLab.DynamicStrands;
import EcoSysLab.DynamicStrandsSet1;
import EvoEngine.PerFrame;
import EcoSysLab.Math;

static const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
#define SEGMENT_VERTICES_SIZE 8
#define SEGMENT_TRIANGLE_SIZE 12



static const uint SEGMENT_VERTEX_ITERATIONS = ((SEGMENT_VERTICES_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);
static const uint SEGMENT_PRIMITIVE_ITERATIONS = ((SEGMENT_TRIANGLE_SIZE + WORKGROUP_SIZE - 1) / WORKGROUP_SIZE);

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
  uint segments_size;
  uint color_mode;
  float multiplier;
  float factor;
  float length_multiplier;
};



// group_id.x runs from [0 .. parentTask.groupCountX - 1]
float4 computeRotCompetitionColor(uint segment_id) {
  float b = clamp(segments[segment_id].RB, 0.0, 1.0);
  float w = clamp(segments[segment_id].RW, 0.0, 1.0);

  float sum = b + w;
  float eps = 1e-6;

  float dominance = b / (sum + eps);


  float intensity = clamp(sum, 0.0, 1.0);

  float3 healthyWoodColor = float3(0.2, 0.22, 0.24);
  float3 whiteRotColor = float3(0.80, 0.85, 0.90);
  float3 brownRotColor = float3(0.60, 0.35, 0.15);
  float3 rotMixColor = lerp(whiteRotColor, brownRotColor, dominance);


  float competitionGate = smoothstep(0.10, 0.50, intensity);

  float3 baseColor = lerp(healthyWoodColor, rotMixColor, competitionGate);


  //float brightness = lerp(0.35, 1.0, smoothstep(0.05, 1.0, intensity));

  float3 finalColor = baseColor;

  return float4(finalColor, 1.0);
}


[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload Task ts_in, uint3 group_id : SV_GroupID, uint lane_id : SV_GroupIndex,
          OutputVertices<EcoSysLabMeshVertex, SEGMENT_VERTICES_SIZE> vertices,
          OutputIndices<uint3, SEGMENT_TRIANGLE_SIZE> output_indices) {
  const uint laneID = lane_id;
  uint segment_id = ts_in.baseID + ts_in.deltaIDs[group_id.x];

  if (segment_id >= segments_size)
    return;

  SetMeshOutputCounts(SEGMENT_VERTICES_SIZE, SEGMENT_TRIANGLE_SIZE);

  Segment segment = segments[segment_id];
  Particle segment_particle0 = segment_particle0s[segment_id];
  Particle segment_particle1 = segment_particle1s[segment_id];

  Particle particle0 = segment_particle0;
  Particle particle1 = segment_particle1;

  float3 start_position = particle0.x;
  float3 end_position = particle1.x;

  float3 center_position = (start_position + end_position) * 0.5f;
  float center_thickness = segment.radius * 2.0f * multiplier;

  // float4 rotation = segment.q;

  float3 front = normalize(end_position - start_position);
  float4 rotation = quatLookAt(front, float3(front.y, front.z, front.x));

  float length = length_multiplier * distance(start_position, end_position);

  float4x4 instance_matrix =
      mul(mul(scale(float3(center_thickness, center_thickness, length)), mat4_cast(rotation)),
          translate(center_position));

  float4x4 transform = mul(instance_matrix, EE_CAMERAS[camera_index].projection_view);

  float4 color;
  if (color_mode == 0) {
    color = color0;
  } else if (color_mode == 1) {
    color = segments[segment.node_handle].color;
  } else if (color_mode == 2) {
    color = segment.group_index == 0 ? float4(0.6, 0.3, 0.0, 1.0) : segments[segment.group_index].color;
  } else if (color_mode == 3) {
    color = lerp(color0, color1, clamp(fmod(segment.boundary_distance, factor) / factor, 0.0, 1.0));
  } else if (color_mode == 4) {
    color = lerp(color0, color1, clamp(segment.strength, 0.0f, 1.0f));
  } else if (color_mode == 5) {
    color = lerp(color0, color1,
                clamp(abs(segment.shear_stretch_strain) * 200.f / segment.shear_stretch_strain_limit, 0.0, 1.0));
  } else if (color_mode == 6) {
    color = lerp(color0, color1, clamp(segment.shear_stretch_strain_limit / factor, 0.0, 1.0));
  } else if (color_mode == 7) {
    color = segments[segment_id].color;
  } else if (color_mode == 8) {
    color = segments[segment.strand_handle].color;
  } else if (color_mode == 9) {  // fungus density
    //color = float4(0.0, clamp(segments[segment_id].RW, 0.0, 1.0), clamp(segments[segment_id].RW, 0.0, 1.0), 1.0);
    //color = float4(clamp(segments[segment_id].HL, 0.0, 1.0), clamp(segments[segment_id].HL, 0.0, 1.0), 0.0, 1.0);
    //color = float4(0.2, 0.2, 0.2, 1.0) * float(segments[segment_id].reach_ground);
    //color = float4(float3(1.0, 1.0, 1.0) *
    //                 (clamp(segments[segment_id].profile_polar_coordinate.y / (2.0 * 3.1415926), -0.5, 0.5) + 0.5),
    //             1.0);
    //color = float4(float3(1.0, 1.0, 1.0) * (clamp(segments[segment_id].profile_position.x / 80.0, -0.5, 0.5) + 0.5), 1.0);
    color = computeRotCompetitionColor(segment_id);
  } else if (color_mode == 10) {  // health
    color = float4(clamp(segments[segment_id].HL, 0.0, 1.0), clamp(segments[segment_id].HL, 0.0, 1.0), 0.0, 1.0);
    //color = float4(0.0, clamp(segments[segment_id].RB, 0.0, 1.0), clamp(segments[segment_id].RB, 0.0, 1.0), 1.0);
    //color = float4(0.2, 0.2, 0.2, 1.0) * float(segments[segment_id].reach_ground);
    //color = float4(clamp(segment_particle0s[segment_id].root_distance, 0.0, 1.0),
                  //clamp(segment_particle0s[segment_id].root_distance, 0.0, 1.0),
                  //clamp(segment_particle0s[segment_id].root_distance, 0.0, 1.0), 1.0);
    //color = float4(float3(1.0, 1.0, 1.0) * (clamp(segments[segment_id].profile_position.y / 80.0, -0.5, 0.5) + 0.5), 1.0);
    //color = float4(float3(1.0, 1.0, 1.0) * (clamp(segments[segment_id].boundary_distance / 0.3, 0.0, 1.0)), 1.0);
  } else if (color_mode == 11) {  // fungus moisture
    color =
        float4(lerp(float3(1.00, 0.90, 0.20), float3(0.00, 0.45, 0.80), clamp(segments[segment_id].moisture, 0.0, 1.0)), 1.0);
    if (segments[segment_id].profile_position[1] > 0.0) {
      instance_matrix = mul(mul(scale(float3(0.0, 0.0, 0.0)), mat4_cast(rotation)), translate(center_position));
      transform = mul(instance_matrix, EE_CAMERAS[camera_index].projection_view);
    }
  } else if (color_mode == 12) {  // screen depth
    if (segments[segment_id].screen_depth > 0.0f) {
      color = float4(lerp(float3(1.0), float3(1.0, 0.0, 0.0), clamp(segments[segment_id].screen_depth, 0.0, 1.0)), 1.0);
    } else {
      color = float4(lerp(float3(1.0), float3(0.0, 0.0, 1.0), 1.0 - clamp(segments[segment_id].screen_depth, 0.0, 1.0)), 1.0);
    }
  }

  float highlight1_ratio = 0.0f;
  float highlight2_ratio = 0.0f;
  float selected_ratio = 0.0f;

  if (particle0.highlighted == 1) {
    highlight1_ratio += 0.4f;
  }
  if (particle0.highlighted == 2) {
    highlight2_ratio += 0.4f;
  }
  if (particle0.selected == 1) {
    selected_ratio += 0.4f;
  }
  if (particle1.highlighted == 1) {
    highlight1_ratio += 0.4f;
  }
  if (particle1.highlighted == 2) {
    highlight2_ratio += 0.4f;
  }
  if (particle1.selected == 1) {
    selected_ratio += 0.4f;
  }

  if (highlight2_ratio != 0.f) {
    color = lerp(color, float4(1, 0, 1, 1), highlight2_ratio);
  } else if (selected_ratio != 0.f) {
    color = lerp(color, float4(1, 0, 0, 1), selected_ratio);
  } else if (highlight1_ratio != 0.f) {
    color = lerp(color, float4(0, 0, 1, 1), highlight1_ratio);
  }

  [[unroll]]
  for (uint i = 0; i < uint(SEGMENT_VERTEX_ITERATIONS); ++i) {
    uint vertex_index = laneID + i * WORKGROUP_SIZE;
    if (vertex_index > SEGMENT_VERTICES_SIZE)
      break;
    float3 vertex_position = positions[vertex_index];

    vertices[vertex_index].frag_pos = mul(float4(vertex_position, 1.0), instance_matrix).xyz;
    vertices[vertex_index].normal = float3(0, 1, 0);
    vertices[vertex_index].tangent = float3(0, 0, 1);
    vertices[vertex_index].color = color;
    vertices[vertex_index].position = mul(float4(vertex_position, 1.0), transform);
  }

  [[unroll]]
  for (uint i = 0; i < uint(SEGMENT_PRIMITIVE_ITERATIONS); ++i) {
    uint triangle_index = laneID + i * WORKGROUP_SIZE;
    if (triangle_index <= SEGMENT_TRIANGLE_SIZE) {
      output_indices[triangle_index] = triangles[triangle_index];
    }
  }
}
