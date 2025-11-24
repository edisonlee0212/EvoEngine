struct SegmentMeshletVertex {
  vec3 x0;
  int segment_index;
  vec3 x;
  int padding;
};

struct SegmentMeshletTriangle {
  uint[3] vertex_indices;
  int neighbor_segment_index;
  // TODO: perhaps split these off into separate buffers with indices
  vec4[3] normal;
  vec4[3] normal0;
  vec2[4] uv;
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 8) buffer SEGMENT_MESHLET_VERTICES_BLOCK {
  SegmentMeshletVertex segment_meshlet_vertices[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 9) buffer SEGMENT_MESHLET_TRIANGLES_BLOCK {
  SegmentMeshletTriangle segment_meshlet_triangles[];
};
