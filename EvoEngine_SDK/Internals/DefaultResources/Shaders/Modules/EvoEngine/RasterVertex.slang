import EvoEngine.Instances;

public struct StandardRasterOutput {
  public float4 position : SV_Position;
  [[vk::location(0)]] public float3 world_position : TEXCOORD0;
  [[vk::location(1)]] public float3 normal : NORMAL;
  [[vk::location(2)]] public float3 tangent : TANGENT;
  [[vk::location(3)]] public nointerpolation float tangent_handedness : TEXCOORD1;
  [[vk::location(4)]] public float4 tex_coord01 : TEXCOORD2;
  [[vk::location(5)]] public float4 tex_coord23 : TEXCOORD3;
  [[vk::location(6)]] public float4 color : COLOR0;
  [[vk::location(7)]] public nointerpolation uint instance_index : TEXCOORD4;
  [[vk::location(8)]] public nointerpolation float transform_handedness : TEXCOORD5;
};

public float3 EE_RASTER_WORLD_NORMAL(float4x4 transform, float3 object_normal) {
  float3x3 linear = float3x3(transform);
  float3 world_normal = cross(linear[1], linear[2]) * object_normal.x +
                        cross(linear[2], linear[0]) * object_normal.y +
                        cross(linear[0], linear[1]) * object_normal.z;
  world_normal *= determinant(linear) < 0.0f ? -1.0f : 1.0f;
  return normalize(world_normal);
}

public StandardRasterOutput EE_BUILD_STANDARD_RASTER_OUTPUT(
    float3 position, float3 normal, float3 tangent, float tangent_handedness, float2 tex_coord0, float2 tex_coord1,
    float2 tex_coord2, float2 tex_coord3, float4 color, uint instance_index, float4x4 transform,
    float4x4 projection_view) {
  StandardRasterOutput output;
  output.world_position = mul(float4(position, 1.0f), transform).xyz;
  output.normal = EE_RASTER_WORLD_NORMAL(transform, normal);
  output.tangent = mul(tangent, float3x3(transform));
  output.transform_handedness = EE_TRANSFORM_HANDEDNESS(transform);
  output.tangent_handedness = (tangent_handedness < 0.0f ? -1.0f : 1.0f) * output.transform_handedness;
  output.tex_coord01 = float4(tex_coord0, tex_coord1);
  output.tex_coord23 = float4(tex_coord2, tex_coord3);
  output.color = color;
  output.instance_index = instance_index;
  output.position = mul(float4(output.world_position, 1.0f), projection_view);
  return output;
}
