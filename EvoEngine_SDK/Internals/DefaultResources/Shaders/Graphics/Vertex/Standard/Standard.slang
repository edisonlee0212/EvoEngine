// @evoengine-dialect native
import EvoEngine.PerFrame;
import EvoEngine.BasicConstants;
import EvoEngine.RasterVertex;

struct StandardVertexInput {
  [[vk::location(0)]] float3 position : POSITION;
  [[vk::location(1)]] float3 normal : NORMAL;
  [[vk::location(2)]] float3 tangent : TANGENT;
  [[vk::location(3)]] float2 tex_coord0 : TEXCOORD0;
  [[vk::location(4)]] float4 color : COLOR0;
  [[vk::location(9)]] float tangent_handedness : TEXCOORD4;
  [[vk::location(10)]] float2 tex_coord1 : TEXCOORD1;
  [[vk::location(11)]] float2 tex_coord2 : TEXCOORD2;
  [[vk::location(12)]] float2 tex_coord3 : TEXCOORD3;
};

[shader("vertex")]
StandardRasterOutput main(StandardVertexInput input, uint draw_id : SV_DrawIndex) {
  uint instance_index = draw_id + uint(EE_BASIC_CONSTANTS.instance_index);
  return EE_BUILD_STANDARD_RASTER_OUTPUT(
      input.position, input.normal, input.tangent, input.tangent_handedness, input.tex_coord0, input.tex_coord1,
      input.tex_coord2, input.tex_coord3, input.color, instance_index, EE_INSTANCES[instance_index].model,
      EE_CAMERAS[EE_BASIC_CONSTANTS.camera_index].projection_view);
}
