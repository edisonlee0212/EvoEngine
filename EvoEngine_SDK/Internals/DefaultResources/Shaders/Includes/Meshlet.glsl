#include "Vertex.glsl"

struct VertexDataChunk {
  Vertex vertices[MESHLET_MAX_VERTICES_SIZE];
};

layout(set = EE_MESHLETS_BLOCK_SET, binding = EE_VERTICES_BLOCK_BINDING) readonly buffer EE_VERTICES_BLOCK {
  VertexDataChunk EE_VERTEX_DATA_CHUNKS[];
};

struct Meshlet {
  uint8_t indices[MESHLET_MAX_INDICES_SIZE];
  uint verticesSize;
  uint triangleSize;
  uint chunkIndex;
};

layout(set = EE_MESHLETS_BLOCK_SET, binding = EE_MESHLETS_BLOCK_BINDING) readonly buffer EE_MESHLETS_BLOCK {
  Meshlet EE_MESHLETS[];
};