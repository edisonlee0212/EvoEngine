#extension GL_EXT_mesh_shader : require
#extension GL_EXT_control_flow_attributes : require
#extension GL_ARB_shading_language_include : enable

#include "BasicConstants.slangh"
#include "PerFrame.slangh"
#include "Strands.slangh"
#include "StrandMeshlet.slangh"

const uint WORKGROUP_SIZE = EXT_MESH_SUBGROUP_COUNT * SUBGROUP_SIZE;
const uint STRAND_RING_MAX = 15;
const float PI2 = 6.28318530718;

struct EEStrandRasterOutput {
  vec4 position : SV_Position;
  [[vk::location(0)]]
  vec3 FragPos;
  [[vk::location(1)]]
  vec3 Normal;
  [[vk::location(2)]]
  vec3 Tangent;
  [[vk::location(3)]]
  nointerpolation float TangentHandedness;
  [[vk::location(4)]]
  vec4 TexCoord01;
  [[vk::location(5)]]
  vec4 TexCoord23;
  [[vk::location(6)]]
  vec4 Color;
  [[vk::location(7)]]
  nointerpolation uint currentInstanceIndex;
  [[vk::location(8)]]
  nointerpolation float transformHandedness;
};

[outputtopology("triangle")]
[numthreads(WORKGROUP_SIZE, 1, 1)]
[shader("mesh")]
void main(in payload StrandTaskPayload strand_task,
          OutputVertices<EEStrandRasterOutput, 32> vertices,
          OutputIndices<uint3, 30> triangles) {
  uint interval_index = gl_WorkGroupID.x;
  uint segment_index = 0;
  for (; segment_index + 1 < strand_task.segment_count &&
         interval_index >= strand_task.interval_offsets[segment_index + 1];
       ++segment_index) {
  }

  uint interval_begin = strand_task.interval_offsets[segment_index];
  uint subdivision = strand_task.interval_offsets[segment_index + 1] - interval_begin;
  float u0 = float(interval_index - interval_begin) / float(subdivision);
  float u1 = float(interval_index - interval_begin + 1) / float(subdivision);

  StrandMeshlet meshlet = EE_STRAND_MESHLETS[strand_task.meshlet_index];
  u8vec4 segment = meshlet.segments[segment_index];
  StrandPoint p0 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.x];
  StrandPoint p1 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.y];
  StrandPoint p2 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.z];
  StrandPoint p3 = EE_STRAND_POINT_DATA_CHUNKS[meshlet.strand_point_chunk_index].strand_points[segment.w];

  vec3 position0, position1, tangent0, tangent1;
  vec3 normal0, normal1, unused_vec3;
  vec4 color0, color1, unused_vec4;
  float thickness0, thickness1, tex_coord0, tex_coord1, unused_float;
  EE_SPLINE_INTERPOLATION(p0.position, p1.position, p2.position, p3.position, position0, tangent0, u0);
  EE_SPLINE_INTERPOLATION(p0.position, p1.position, p2.position, p3.position, position1, tangent1, u1);
  EE_SPLINE_INTERPOLATION(p0.normal, p1.normal, p2.normal, p3.normal, normal0, unused_vec3, u0);
  EE_SPLINE_INTERPOLATION(p0.normal, p1.normal, p2.normal, p3.normal, normal1, unused_vec3, u1);
  EE_SPLINE_INTERPOLATION(p0.thickness, p1.thickness, p2.thickness, p3.thickness, thickness0, unused_float, u0);
  EE_SPLINE_INTERPOLATION(p0.thickness, p1.thickness, p2.thickness, p3.thickness, thickness1, unused_float, u1);
  EE_SPLINE_INTERPOLATION(p0.tex_coord, p1.tex_coord, p2.tex_coord, p3.tex_coord, tex_coord0, unused_float, u0);
  EE_SPLINE_INTERPOLATION(p0.tex_coord, p1.tex_coord, p2.tex_coord, p3.tex_coord, tex_coord1, unused_float, u1);
  EE_SPLINE_INTERPOLATION(p0.color, p1.color, p2.color, p3.color, color0, unused_vec4, u0);
  EE_SPLINE_INTERPOLATION(p0.color, p1.color, p2.color, p3.color, color1, unused_vec4, u1);

  vec3 frame_normal0, frame_normal1, frame_bitangent0, frame_bitangent1;
  EE_STRAND_FRAME(tangent0, normal0, frame_normal0, frame_bitangent0);
  EE_STRAND_FRAME(tangent1, normal1, frame_normal1, frame_bitangent1);

  uint instance_index = strand_task.instance_index;
  mat4 model = EE_INSTANCES[instance_index].model;
  mat3 normal_matrix = transpose(inverse(mat3(model)));
  vec3 world_position0 = vec3(model * vec4(position0, 1.0));
  vec3 world_position1 = vec3(model * vec4(position1, 1.0));
  uint ring0 = uint(EE_STRANDS_RING_SUBDIVISION(model, world_position0, position0, thickness0));
  uint ring1 = uint(EE_STRANDS_RING_SUBDIVISION(model, world_position1, position1, thickness1));
  uint ring_size = min(max(ring0, ring1), STRAND_RING_MAX);
  uint vertex_count = (ring_size + 1) * 2;
  uint primitive_count = ring_size * 2;
  SetMeshOutputCounts(vertex_count, primitive_count);

  mat4 projection_view = EE_CAMERAS[EE_CAMERA_INDEX].projection_view;
  float handedness = EE_TRANSFORM_HANDEDNESS(model);
  for (uint vertex = gl_LocalInvocationID.x; vertex < vertex_count; vertex += WORKGROUP_SIZE) {
    bool endpoint = (vertex & 1u) != 0u;
    uint ring_vertex = vertex >> 1u;
    uint endpoint_ring = endpoint ? ring1 : ring0;
    uint ring_step = ring_vertex * endpoint_ring / ring_size;
    float angle = -PI2 * float(ring_step) / float(endpoint_ring);
    vec3 position = endpoint ? position1 : position0;
    vec3 tangent = endpoint ? tangent1 : tangent0;
    vec3 frame_normal = endpoint ? frame_normal1 : frame_normal0;
    vec3 frame_bitangent = endpoint ? frame_bitangent1 : frame_bitangent0;
    float thickness = endpoint ? thickness1 : thickness0;
    vec3 radial = frame_normal * sin(angle) + frame_bitangent * cos(angle);
    vec3 world_position = vec3(model * vec4(position + radial * thickness, 1.0));
    vec2 tex_coord = vec2(float(ring_step) / float(endpoint_ring), endpoint ? tex_coord1 : tex_coord0);

    vertices[vertex].FragPos = world_position;
    vertices[vertex].Normal = normalize(normal_matrix * radial);
    vertices[vertex].Tangent = mat3(model) * tangent;
    vertices[vertex].TangentHandedness = handedness;
    vertices[vertex].TexCoord01 = vec4(tex_coord, tex_coord);
    vertices[vertex].TexCoord23 = vec4(0.0);
    vertices[vertex].Color = endpoint ? color1 : color0;
    vertices[vertex].currentInstanceIndex = instance_index;
    vertices[vertex].transformHandedness = handedness;
    vertices[vertex].position = projection_view * vec4(world_position, 1.0);
  }

  for (uint primitive = gl_LocalInvocationID.x; primitive < primitive_count; primitive += WORKGROUP_SIZE) {
    uint ring_index = primitive >> 1u;
    uint source = ring_index * 2;
    uint target = source + 1;
    uint next_source = (ring_index + 1) * 2;
    uint next_target = next_source + 1;
    triangles[primitive] = (primitive & 1u) == 0u ? uint3(source, target, next_source)
                                                  : uint3(next_source, target, next_target);
  }
}
