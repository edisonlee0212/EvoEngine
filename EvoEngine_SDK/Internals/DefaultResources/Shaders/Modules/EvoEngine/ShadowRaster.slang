public struct ShadowRasterOutput {
  public float4 position : SV_Position;
  [[vk::location(0)]] public float2 tex_coord0 : TEXCOORD0;
  [[vk::location(1)]] public float2 tex_coord1 : TEXCOORD1;
  [[vk::location(2)]] public float4 color : COLOR0;
  [[vk::location(3)]] public nointerpolation uint instance_index : TEXCOORD2;
};

public ShadowRasterOutput EE_BUILD_SHADOW_RASTER_OUTPUT(float4 position, float2 tex_coord0, float2 tex_coord1,
                                                         float4 color, uint instance_index) {
  ShadowRasterOutput output;
  output.position = position;
  output.tex_coord0 = tex_coord0;
  output.tex_coord1 = tex_coord1;
  output.color = color;
  output.instance_index = instance_index;
  return output;
}
