import EcoSysLab.AirGrid;

struct WindParticle {
  float4 position_age;
  float4 velocity_lifetime;
  float4 collision;
};

struct Parameters {
  uint cell_count;
  uint bucket_count;
  float cell_size;
  float time_step;
  float coupling_rate;
  float friction;
  float ambient_temperature;
  float ambient_humidity;
  float ambient_pressure;
  float particle_cell_size;
};

[[vk::binding(0, 0)]] StructuredBuffer<WindParticle> particles;
[[vk::binding(1, 0)]] StructuredBuffer<uint> particle_heads;
[[vk::binding(2, 0)]] StructuredBuffer<uint> particle_next;
[[vk::binding(3, 0)]] StructuredBuffer<uint> air_heads;
[[vk::binding(4, 0)]] StructuredBuffer<AirGridAddresses> air_addresses;
[[vk::push_constant]] ConstantBuffer<Parameters> parameters;

[numthreads(64, 1, 1)]
void main(uint3 id : SV_DispatchThreadID) {
  uint index = id.x;
  if (index >= parameters.cell_count) return;
  int3 coordinate = AirGridReadCell(air_addresses, index).coordinate_next.xyz;
  float3 center = (float3(coordinate) + 0.5) * parameters.cell_size;
  AirGridValue state = AirGridReadValue(air_addresses, index);
  if (state.velocity_temperature.w <= 0.0) {
    state.velocity_temperature = float4(0, 0, 0, parameters.ambient_temperature);
    state.humidity_strength_density = float4(parameters.ambient_humidity, 1.0, 0, 0);
  }
  float3 velocity_sum = 0;
  float temperature_sum = 0;
  float humidity_sum = 0;
  float strength_sum = 0;
  float weight_sum = 0;
  float support = parameters.cell_size * 1.75 + 1.0;
  int3 first = int3(floor((center - support) / parameters.particle_cell_size));
  int3 last = int3(floor((center + support) / parameters.particle_cell_size));
  for (int z = first.z; z <= last.z; ++z)
    for (int y = first.y; y <= last.y; ++y)
      for (int x = first.x; x <= last.x; ++x) {
        int3 neighbor = int3(x, y, z);
        uint particle_index = particle_heads[AirGridHash(neighbor, parameters.bucket_count)];
        while (particle_index != 0xffffffffu) {
          WindParticle particle = particles[particle_index];
          if (particle.velocity_lifetime.w > particle.position_age.w &&
              all(int3(floor(particle.position_age.xyz / parameters.particle_cell_size)) == neighbor)) {
            float distance = length(particle.position_age.xyz - center);
            float weight = saturate(1.0 - distance / (parameters.cell_size * 1.75 + particle.collision.x));
            velocity_sum += weight * particle.velocity_lifetime.xyz;
            temperature_sum += weight * particle.collision.w;
            humidity_sum += weight * particle.collision.z;
            strength_sum += weight * particle.collision.y;
            weight_sum += weight;
          }
          particle_index = particle_next[particle_index];
        }
      }
  float dt = parameters.time_step;
  if (weight_sum > 0.0) {
    float mix = 1.0 - exp(-parameters.coupling_rate * dt);
    state.velocity_temperature.xyz += dt * parameters.coupling_rate * velocity_sum / weight_sum;
    state.velocity_temperature.w = lerp(state.velocity_temperature.w, temperature_sum / weight_sum, mix);
    state.humidity_strength_density.x = lerp(state.humidity_strength_density.x, humidity_sum / weight_sum, mix);
    state.humidity_strength_density.y = lerp(state.humidity_strength_density.y, strength_sum / weight_sum, mix);
  }
  state.velocity_temperature.xyz *= exp(-parameters.friction * dt);
  state.humidity_strength_density.z = parameters.ambient_pressure /
      (287.05 * max(state.velocity_temperature.w, 1.0) *
       (1.0 + 0.61 * max(state.humidity_strength_density.x, 0.0)));
  AirGridWriteValue(air_addresses, index, state);
}
