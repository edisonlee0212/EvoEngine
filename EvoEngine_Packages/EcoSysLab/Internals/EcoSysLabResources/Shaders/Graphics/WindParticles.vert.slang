import EvoEngine.PerFrame;

struct WindParticle {
  float4 position_age;
  float4 velocity_lifetime;
  float4 collision;
};

struct Parameters {
  uint camera_index;
  float2 viewport_size;
  float point_radius;
};

struct Output {
  float4 position : SV_Position;
  [[vk::location(0)]] float2 disc : TEXCOORD0;
  [[vk::location(1)]] float3 color : COLOR0;
};

[[vk::binding(0, 1)]] StructuredBuffer<WindParticle> particles;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

static const float2 corners[6] = {
  float2(-1.0, -1.0), float2(1.0, -1.0), float2(1.0, 1.0),
  float2(-1.0, -1.0), float2(1.0, 1.0), float2(-1.0, 1.0)
};

[shader("vertex")]
Output main(uint vertex_id : SV_VertexID, uint instance_id : SV_InstanceID) {
  WindParticle particle = particles[instance_id];
  Output output;
  output.disc = corners[vertex_id];
  output.color = lerp(float3(0.15, 0.55, 1.0), float3(1.0, 0.8, 0.2),
                      saturate(length(particle.velocity_lifetime.xyz) / 20.0));
  if (particle.velocity_lifetime.w <= 0.0 || particle.position_age.w >= particle.velocity_lifetime.w) {
    output.position = float4(2.0, 2.0, 2.0, 1.0);
    return output;
  }
  output.position = mul(float4(particle.position_age.xyz, 1.0),
                        EE_CAMERAS[parameters.camera_index].projection_view);
  output.position.xy += corners[vertex_id] * parameters.point_radius * 2.0 /
                        max(parameters.viewport_size, float2(1.0)) * output.position.w;
  return output;
}
