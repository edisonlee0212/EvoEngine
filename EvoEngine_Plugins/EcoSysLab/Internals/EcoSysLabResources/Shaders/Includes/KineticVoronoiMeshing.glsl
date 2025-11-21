struct SegmentMeshletVertex {
  float relative_position_x;
  float relative_position_y;
  float relative_position_z;
  int segment_index;
};

struct SegmentMeshletTriangle {
  uint[4] vertex_indices_twin_triangle;
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 8) buffer SEGMENT_MESHLET_VERTICES_BLOCK {
  SegmentMeshletVertex segment_meshlet_vertices[];
};

layout(std430, set = DYNAMIC_STRANDS_SET, binding = 9) buffer SEGMENT_MESHLET_TRIANGLES_BLOCK {
  SegmentMeshletTriangle segment_meshlet_triangles[];
};
