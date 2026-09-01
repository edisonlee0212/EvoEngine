public static const uint EE_GBUFFER_MATERIAL_INDEX_MASK = 0x3fffffffu;
public static const uint EE_GBUFFER_VERTEX_COLOR_REPLACES_BASE_COLOR_BIT = 0x40000000u;
public static const uint EE_GBUFFER_NEGATIVE_TANGENT_HANDEDNESS_BIT = 0x80000000u;
public static const uint EE_GBUFFER_CLEAR_VALUE = 0xffffffffu;

public struct EeGBufferMetadata {
  public uint instance_index;
  public uint material_index;
  public uint info_index;
  public float4 vertex_color;
  public bool vertex_color_replaces_base_color;
  public bool negative_tangent_handedness;
}

public uint EE_GBUFFER_PACK_UNORM8(float4 value) {
  const uint4 quantized = uint4(floor(clamp(value, float4(0.0f), float4(1.0f)) * 255.0f + float4(0.5f)));
  return quantized.r | quantized.g << 8u | quantized.b << 16u | quantized.a << 24u;
}

public float4 EE_GBUFFER_UNPACK_UNORM8(uint value) {
  return float4(value & 0xffu, value >> 8u & 0xffu, value >> 16u & 0xffu, value >> 24u & 0xffu) / 255.0f;
}

public uint4 EE_GBUFFER_PACK_METADATA(uint instance_index, uint material_index, uint info_index, float4 vertex_color,
                                      bool vertex_color_replaces_base_color, bool negative_tangent_handedness) {
  uint material_and_flags = material_index & EE_GBUFFER_MATERIAL_INDEX_MASK;
  material_and_flags |= vertex_color_replaces_base_color ? EE_GBUFFER_VERTEX_COLOR_REPLACES_BASE_COLOR_BIT : 0u;
  material_and_flags |= negative_tangent_handedness ? EE_GBUFFER_NEGATIVE_TANGENT_HANDEDNESS_BIT : 0u;
  return uint4(instance_index, material_and_flags, info_index, EE_GBUFFER_PACK_UNORM8(vertex_color));
}

public EeGBufferMetadata EE_GBUFFER_UNPACK_METADATA(uint4 value) {
  EeGBufferMetadata metadata;
  metadata.instance_index = value.x;
  metadata.material_index = value.y & EE_GBUFFER_MATERIAL_INDEX_MASK;
  metadata.info_index = value.z;
  metadata.vertex_color = EE_GBUFFER_UNPACK_UNORM8(value.w);
  metadata.vertex_color_replaces_base_color =
      (value.y & EE_GBUFFER_VERTEX_COLOR_REPLACES_BASE_COLOR_BIT) != 0u;
  metadata.negative_tangent_handedness = (value.y & EE_GBUFFER_NEGATIVE_TANGENT_HANDEDNESS_BIT) != 0u;
  return metadata;
}

public float2 EE_GBUFFER_OCT_ENCODE(float3 direction) {
  const float length_squared = dot(direction, direction);
  direction = length_squared > 0.0f ? direction * rsqrt(length_squared) : float3(0.0f, 0.0f, 1.0f);
  direction /= abs(direction.x) + abs(direction.y) + abs(direction.z);
  float2 encoded = direction.xy;
  if (direction.z < 0.0f) {
    const float2 sign_value = float2(encoded.x >= 0.0f ? 1.0f : -1.0f, encoded.y >= 0.0f ? 1.0f : -1.0f);
    encoded = (1.0f - abs(encoded.yx)) * sign_value;
  }
  return encoded;
}

public float3 EE_GBUFFER_OCT_DECODE(float2 encoded) {
  float3 direction = float3(encoded, 1.0f - abs(encoded.x) - abs(encoded.y));
  if (direction.z < 0.0f) {
    const float2 sign_value = float2(direction.x >= 0.0f ? 1.0f : -1.0f, direction.y >= 0.0f ? 1.0f : -1.0f);
    direction.xy = (1.0f - abs(direction.yx)) * sign_value;
  }
  return normalize(direction);
}
