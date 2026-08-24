// @evoengine-dialect native
__exported import EvoEngine.Vertex;

public static const uint EE_MESHLET_MAX_VERTICES_SIZE = 64u;
public static const uint EE_MESHLET_MAX_TRIANGLES_SIZE = 40u;
public static const uint EE_MESHLET_MAX_INDICES_SIZE = EE_MESHLET_MAX_TRIANGLES_SIZE * 3u;

public struct VertexDataChunk {
  public Vertex vertices[EE_MESHLET_MAX_VERTICES_SIZE];
};

public struct Meshlet {
  public uint8_t indices[EE_MESHLET_MAX_INDICES_SIZE];
  public uint verticesSize;
  public uint triangleSize;
  public uint chunkIndex;
  public float4 boundingSphere;
  public float4 normalCone;
  public float4 normalConeApex;
};

[[vk::binding(0, 1)]]
public StructuredBuffer<VertexDataChunk> EE_VERTEX_DATA_CHUNKS;

[[vk::binding(1, 1)]]
public StructuredBuffer<Meshlet> EE_MESHLETS;
