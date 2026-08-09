// @evoengine-dialect native
import EvoEngine.Meshlet;

public struct StrandPoint {
  public float3 position;
  public float thickness;
  public float3 normal;
  public float tex_coord;
  public float4 color;
};

public struct StrandPointDataChunk {
  public StrandPoint strand_points[EE_MESHLET_MAX_VERTICES_SIZE];
};

public struct StrandMeshlet {
  public uint8_t4 segments[EE_MESHLET_MAX_TRIANGLES_SIZE];
  public uint strand_points_size;
  public uint segment_size;
  public uint strand_point_chunk_index;
};

[[vk::binding(0, 1)]]
public StructuredBuffer<StrandPointDataChunk> EE_STRAND_POINT_DATA_CHUNKS;
[[vk::binding(1, 1)]]
public StructuredBuffer<StrandMeshlet> EE_STRAND_MESHLETS;

public struct StrandTaskPayload {
  public uint instance_index;
  public uint meshlet_index;
  public uint segment_count;
  public uint interval_offsets[EE_MESHLET_MAX_TRIANGLES_SIZE + 1u];
};

public struct StrandShadowTaskPayload {
  public uint instance_index;
  public uint meshlet_index;
};

public void EE_STRAND_FRAME(float3 tangent, float3 normal, out float3 frame_normal,
                            out float3 frame_bitangent) {
  float3 normalized_tangent = normalize(tangent);
  frame_normal = normal - normalized_tangent * dot(normal, normalized_tangent);
  if (dot(frame_normal, frame_normal) < 0.000001f) {
    float3 axis = abs(normalized_tangent.z) < 0.999f ? float3(0.0f, 0.0f, 1.0f)
                                                      : float3(0.0f, 1.0f, 0.0f);
    frame_normal = cross(axis, normalized_tangent);
  }
  frame_normal = normalize(frame_normal);
  frame_bitangent = normalize(cross(frame_normal, normalized_tangent));
}
