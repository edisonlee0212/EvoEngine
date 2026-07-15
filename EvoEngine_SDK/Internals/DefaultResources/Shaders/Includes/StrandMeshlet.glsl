#ifndef EE_STRAND_MESHLET_GLSL
#define EE_STRAND_MESHLET_GLSL

#extension GL_EXT_shader_explicit_arithmetic_types_int8 : require

struct StrandPoint {
  vec3 position;
  float thickness;
  vec3 normal;
  float tex_coord;
  vec4 color;
};

struct StrandPointDataChunk {
  StrandPoint strand_points[MESHLET_MAX_VERTICES_SIZE];
};

layout(std430, set = 1, binding = 0) readonly buffer EE_STRAND_POINTS_BLOCK {
  StrandPointDataChunk EE_STRAND_POINT_DATA_CHUNKS[];
};

struct StrandMeshlet {
  u8vec4 segments[MESHLET_MAX_TRIANGLES_SIZE];
  uint strand_points_size;
  uint segment_size;
  uint strand_point_chunk_index;
};

layout(std430, set = 1, binding = 1) readonly buffer EE_STRAND_MESHLETS_BLOCK {
  StrandMeshlet EE_STRAND_MESHLETS[];
};

struct StrandTaskPayload {
  uint instance_index;
  uint meshlet_index;
  uint segment_count;
  uint interval_offsets[MESHLET_MAX_TRIANGLES_SIZE + 1];
};

void EE_STRAND_FRAME(in vec3 tangent, in vec3 normal, out vec3 frame_normal, out vec3 frame_bitangent) {
  vec3 t = normalize(tangent);
  frame_normal = normal - t * dot(normal, t);
  if (dot(frame_normal, frame_normal) < 0.000001) {
    vec3 axis = abs(t.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(0.0, 1.0, 0.0);
    frame_normal = cross(axis, t);
  }
  frame_normal = normalize(frame_normal);
  frame_bitangent = normalize(cross(frame_normal, t));
}

#endif
