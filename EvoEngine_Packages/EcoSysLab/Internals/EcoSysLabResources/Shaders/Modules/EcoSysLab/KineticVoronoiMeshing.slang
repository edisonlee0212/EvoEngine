public struct SegmentMeshletVertex {
  public float3 x0;
  public int segment_index;
  public float3 x;
  public int padding0;
  public float3 shift;
  public int padding1;
};

public struct SegmentMeshletTriangle {
  public uint[3] vertex_indices;
  public int neighbor_segment_index;
  // TODO: perhaps split these off into separate buffers with indices
  public float4[3] normal;
  public float4[3] normal0;
  public float4[3] uv;
  public int segment_pair_index;
  public int padding0;
  public int padding1;
  public int padding2;
};
