// @evoengine-dialect native
import EvoEngine.Atmosphere;

struct AtmosphereFragmentInput {
  [[vk::location(0)]] float3 world_position : TEXCOORD0;
};

struct AtmosphereFragmentConstants {
  float4x4 projection_view;
  Atmosphere atmosphere;
  float3 sun_direction;
  float gamma;
  float3 ground_color;
  float ground_transmittance;
};

[[vk::push_constant]] ConstantBuffer<AtmosphereFragmentConstants> constants;

[shader("fragment")]
float4 main(AtmosphereFragmentInput input) : SV_Target {
  float3 ray_direction = normalize(input.world_position);
  float3 color = NishitaSkyIncidentLight(constants.atmosphere, float3(0.0f), ray_direction,
                                         constants.sun_direction);
  if (ray_direction.y <= 0.0f)
    color = lerp(constants.ground_color, color, constants.ground_transmittance);
  return float4(pow(color, float3(constants.gamma)), 1.0f);
}
