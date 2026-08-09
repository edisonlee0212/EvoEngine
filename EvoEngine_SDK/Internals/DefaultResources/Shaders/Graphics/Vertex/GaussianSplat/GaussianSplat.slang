// @evoengine-dialect native
import EvoEngine.GaussianSplat;

struct GaussianSplatVertexOutput {
  float4 position : SV_Position;
  [[vk::location(0)]]
  float3 color : COLOR0;
  [[vk::location(1)]]
  float2 local_coord : TEXCOORD0;
  [[vk::location(2)]]
  nointerpolation float opacity : TEXCOORD1;
  [[vk::location(3)]]
  nointerpolation float cutoff_radius_squared : TEXCOORD2;
};

[shader("vertex")]
GaussianSplatVertexOutput main(uint vertex_index : SV_VertexID, uint instance_index : SV_InstanceID) {
  GaussianSplatVertexOutput output;
  EE_GAUSSIAN_SPLAT_BUILD_VERTEX(instance_index, EE_GAUSSIAN_SPLAT_TRIANGLE_CORNERS[vertex_index], output.position,
                                 output.color, output.local_coord, output.opacity, output.cutoff_radius_squared);
  return output;
}
