import EvoEngine.RestirSss;
import EvoEngine.Random;

[[vk::binding(0, 0)]] RWStructuredBuffer<float> output;
[[vk::binding(1, 0)]] RWByteAddressBuffer samples;
[[vk::binding(2, 0)]] RWByteAddressBuffer candidate_buffer;

[shader("compute")][numthreads(1, 1, 1)]
void main() {
  EE_RESTIR_SSS_STORE(samples, 1u, EE_RESTIR_SSS_LOAD(samples, 0u));
  EE_RESTIR_SSS_STORE(samples, 2u, EE_RESTIR_SSS_EMPTY_SAMPLE());
  EE_RESTIR_SSS_STORE(candidate_buffer, 1u, EE_RESTIR_SSS_LOAD(candidate_buffer, 0u, false), false);
  EE_RESTIR_SSS_STORE(candidate_buffer, 2u, EE_RESTIR_SSS_LOAD(samples, 0u), false);
  for (uint i = 0; i < 16u; ++i) {
    const float u = (float(i) + 0.5f) / 16.0f;
    const float radius = EE_SSS_SAMPLE_RADIUS(u, 0.001f);
    output[i] = radius;
    output[16u + i] = 1.0f - 0.25f * exp(-radius / 0.001f) - 0.75f * exp(-radius / 0.003f);
  }
  SubsurfaceDirectReservoir empty = EE_SSS_DIRECT_EMPTY_RESERVOIR();
  EE_SSS_DIRECT_UPDATE(empty, 0u, float3(0), 0.0f);
  EE_SSS_DIRECT_UPDATE(empty, 1u, float3(asfloat(0x7fc00000u)), 0.0f);
  output[32] = EE_SSS_DIRECT_RESOLVE_WEIGHT(empty);
  output[33] = float(empty.candidate_index == 0xffffffffu);

  const float3 candidates[3] = {float3(2, 0, 0), float3(0, 1, 0), float3(0, 0, 3)};
  float3 total = float3(0);
  float3 visible_total = float3(0);
  for (uint sample = 0u; sample < 262144u; ++sample) {
    uint seed = EE_XXHASH32(uint3(sample, 1337u, 0u));
    SubsurfaceDirectReservoir reservoir = EE_SSS_DIRECT_EMPTY_RESERVOIR();
    for (uint i = 0u; i < 3u; ++i)
      EE_SSS_DIRECT_UPDATE(reservoir, i, candidates[i], EE_PCG_RANDOM(seed));
    const float3 resolved = candidates[reservoir.candidate_index] * EE_SSS_DIRECT_RESOLVE_WEIGHT(reservoir);
    total += resolved;
    if (reservoir.candidate_index != 2u) visible_total += resolved;
  }
  for (uint channel = 0u; channel < 3u; ++channel) {
    output[34u + channel] = total[channel] / 262144.0f;
    output[37u + channel] = visible_total[channel] / 262144.0f;
  }
  SubsurfaceDirectReservoir single = EE_SSS_DIRECT_EMPTY_RESERVOIR();
  EE_SSS_DIRECT_UPDATE(single, 7u, float3(2, 1, 3), 0.5f);
  EE_SSS_DIRECT_UPDATE(single, 8u, float3(0), 0.0f);
  output[40] = EE_SSS_DIRECT_RESOLVE_WEIGHT(single);
  output[41] = float(single.candidate_index);
  const float3 distance = float3(0.001f, 0.002f, 0.003f);
  const float3 delta = float3(0.003f, 0.004f, 0.012f);
  const float3 normal = float3(2, -3, 6) / 7.0f;
  for (uint axis = 0u; axis < 3u; ++axis) {
    float3 axis_normal = float3(0);
    axis_normal[axis] = 1.0f;
    output[42u + axis] = EE_SSS_PROJECTED_DENSITY(delta, axis_normal, distance);
  }
  output[45] = EE_SSS_PROJECTED_DENSITY(delta, normal, distance);
  output[46] = EE_SSS_PROJECTED_DENSITY(delta * 7.0f, normal, distance * 7.0f);
  output[47] = EE_SSS_PROJECTED_DENSITY(-delta, -normal, distance);
  output[48] = EE_SSS_PROJECTED_DENSITY(delta.xzy, normal.xzy, distance);
  output[49] = EE_SSS_PROJECTED_DENSITY(delta, float3(0), distance);
  // Different proposals, different receiver targets, and a 25% null-draw probability.
  const float3 previous_candidates[3] = {float3(1, 0, 0), float3(0, 4, 0), float3(0, 0, 2)};
  const float fresh_pdf[4] = {0.2f, 0.3f, 0.25f, 0.25f};
  const float history_pdf[4] = {0.45f, 0.2f, 0.1f, 0.25f};
  float3 reused_total = float3(0);
  for (uint trial = 0u; trial < 262144u; ++trial) {
    uint seed = EE_XXHASH32(uint3(trial, 0x53535354u, 0u));
    const float2 draw = EE_PCG_RANDOM_2(seed);
    const uint a = draw.x < 0.2f ? 0u : (draw.x < 0.5f ? 1u : (draw.x < 0.75f ? 2u : 3u));
    const uint b = draw.y < 0.45f ? 0u : (draw.y < 0.65f ? 1u : (draw.y < 0.75f ? 2u : 3u));
    const float3 fresh_value = a < 3u ? candidates[a] : float3(0);
    const float3 history_value = b < 3u ? candidates[b] : float3(0);
    const float4 targets = float4(EE_RESTIR_SSS_TARGET(fresh_value), EE_RESTIR_SSS_TARGET(history_value),
        a < 3u ? EE_RESTIR_SSS_TARGET(previous_candidates[a]) : 0.0f,
        b < 3u ? EE_RESTIR_SSS_TARGET(previous_candidates[b]) : 0.0f);
    const float2 weights = EE_RESTIR_SSS_BALANCE_WEIGHTS(targets, float2(1, 7), float2(1.0f / fresh_pdf[a], 1.0f / history_pdf[b]));
    const float sum = weights.x + weights.y;
    if (sum > 0.0f) {
      const bool history_selected = EE_PCG_RANDOM(seed) * sum < weights.y;
      reused_total += (history_selected ? history_value : fresh_value) *
          (sum / (history_selected ? targets.y : targets.x));
    }
  }
  for (uint channel = 0u; channel < 3u; ++channel) output[50u + channel] = reused_total[channel] / 262144.0f;
  output[53] = EE_RESTIR_SSS_BALANCE_WEIGHTS(float4(0), float2(1), float2(0)).x;
  output[54] = EE_RESTIR_SSS_BALANCE_WEIGHTS(float4(1, 0, 0, 0), float2(1, 0), float2(1, 0)).x;
  const float component = EE_SSS_COMPONENT_DENSITY(delta, normal, distance, 0u, 2u);
  output[55] = component;
  output[56] = EE_SSS_COMPONENT_DENSITY(delta * 100.0f, normal, distance, 0u, 2u);
  output[57] = EE_SSS_COMPONENT_DENSITY(float3(0, EE_SSS_SAMPLE_RADIUS(0.999f, distance.x), 0),
      float3(1, 0, 0), distance, 0u, 0u);
  output[58] = EE_SSS_COMPONENT_DENSITY(delta, float3(0), distance, 0u, 2u);
  output[59] = (3.0f / component) * (component / output[45]);
  output[60] = EE_RESTIR_SSS_LOAD(candidate_buffer, 0u, false).resolve_weight;
  RestirSssDirectSample stored = EE_RESTIR_SSS_LOAD(samples, 0u);
  for (uint channel = 0u; channel < 3u; ++channel) {
    output[61u + channel] = stored.receiver_position[channel];
    output[65u + channel] = stored.receiver_normal[channel];
    output[69u + channel] = stored.receiver_tangent[channel];
  }
  output[64] = stored.confidence;
  output[68] = asfloat(stored.receiver_material);
  output[72] = stored.target_density;
  bool selectors_match = true;
  for (uint light = 0u; light < 4u; ++light)
    for (uint channel = 0u; channel < 3u; ++channel)
      for (uint axis = 0u; axis < 3u; ++axis) {
        stored.axis_channel = axis | (channel << 2u);
        stored.light_technique = light == 3u ? 0xffffffffu : light;
        EE_RESTIR_SSS_STORE(samples, 6u, stored);
        const RestirSssDirectSample decoded = EE_RESTIR_SSS_LOAD(samples, 6u);
        selectors_match = selectors_match && decoded.axis_channel == stored.axis_channel &&
            decoded.light_technique == stored.light_technique;
      }
  output[73] = float(selectors_match);
  const uint requested[3] = {1u, 3u, 16u};
  const uint valid[3] = {1u, 2u, 13u};
  for (uint c = 0u; c < 3u; ++c) {
    float3 expected = float3(0);
    for (uint i = 0u; i < 4u; ++i) {
      const float3 value = i < 3u ? candidates[i] : float3(0);
      const float current = EE_RESTIR_SSS_TARGET(value);
      const float previous = i < 3u ? EE_RESTIR_SSS_TARGET(previous_candidates[i]) : 0.0f;
      const float2 mis = EE_RESTIR_SSS_PAIRWISE_MIS(float4(current, current, previous, previous), float2(1, 7), requested[c]);
      expected += fresh_pdf[i] * value * ((1.0f + float(valid[c]) * mis.x) / fresh_pdf[i]) / float(valid[c] + 1u);
      expected += float(valid[c]) * history_pdf[i] * value * (mis.y / history_pdf[i]) / float(valid[c] + 1u);
    }
    for (uint channel = 0u; channel < 3u; ++channel) output[74u + c * 3u + channel] = expected[channel];
  }
  const float4x4 perspective = float4x4(1,0,0,0, 0,1,0,0, 0,0,1,1, 0,0,0,0);
  const float4x4 identity = float4x4(1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1);
  const float depths[5] = {2, 4, 0.1f, 100, -1};
  for (uint i = 0u; i < 5u; ++i)
    output[83u + i] = EE_RESTIR_SSS_SPATIAL_RADIUS(float3(0.02f), float3(0,0,depths[i]), perspective, identity, float2(1920,1080));
  const int2 right_offset = EE_RESTIR_SSS_SPATIAL_OFFSET(10, float2(0.25f, 0));
  const int2 up_offset = EE_RESTIR_SSS_SPATIAL_OFFSET(10, float2(0.25f, 0.25f));
  output[88] = float(right_offset.x);
  output[89] = float(right_offset.y);
  output[90] = float(up_offset.x);
  output[91] = float(up_offset.y);
  float3 moments = float3(0);
  for (uint i = 0u; i < 262144u; ++i) {
    uint seed = EE_XXHASH32(uint3(i, 0x53535350u, 0));
    const float2 offset = float2(EE_RESTIR_SSS_SPATIAL_OFFSET(10, EE_PCG_RANDOM_2(seed)));
    moments += float3(offset, dot(offset, offset));
  }
  for (uint c = 0u; c < 3u; ++c) output[92u + c] = moments[c] / 262144.0f;
  const float2 empty_mis = EE_RESTIR_SSS_PAIRWISE_MIS(float4(0), float2(1), 3u);
  output[95] = empty_mis.x;
  output[96] = empty_mis.y;
  const float3 distances = float3(1, 2, 3);
  const float3 transmission[5] = {
      EE_SSS_SOLID_TRANSMISSION(0, 0, distances),
      EE_SSS_SOLID_TRANSMISSION(3, 1, distances),
      EE_SSS_SOLID_TRANSMISSION(3, 6, distances),
      EE_SSS_SOLID_TRANSMISSION(0.003f, 0.006f, distances * 0.001f),
      EE_SSS_SOLID_TRANSMISSION(3, 1000, distances)};
  for (uint i = 0u; i < 5u; ++i)
    for (uint channel = 0u; channel < 3u; ++channel) output[97u + i * 3u + channel] = transmission[i][channel];
  const float a = 10.0f / 9.99f;
  const float b = -0.1f / 9.99f;
  const float4x4 light = float4x4(1,0,0,0, 0,1,0,0, 0,0,a,1, 0,0,b,0);
  output[112] = EE_SSS_SHADOW_THICKNESS(light, 4, 4, a + b / 2);
  output[113] = EE_SSS_SHADOW_THICKNESS(light, 4, 4 * sqrt(2.0f), a + b / 2);
  output[114] = EE_SSS_SHADOW_THICKNESS(light, 2, 2, a + b / 4);
  const float4x4 moved_light = float4x4(0,0,a,1, 0,1,0,0, 1,0,0,0, 0,0,5 * a + b,5);
  output[115] = EE_SSS_SHADOW_THICKNESS(moved_light, 4, 4, a + b / 2);
  output[116] = EE_SSS_SHADOW_THICKNESS(light, 4, 4, 1);
  output[117] = EE_SSS_SHADOW_THICKNESS(light, 0, 4, a + b / 2);
  output[118] = float(EE_SSS_BACKFACE_MATCH(7, 7, 0.2f, 0.4f, false));
  output[119] = float(EE_SSS_BACKFACE_MATCH(7, 8, 0.2f, 0.4f, false));
  output[120] = float(EE_SSS_BACKFACE_MATCH(0xffffffffu, 0xffffffffu, 0.2f, 0.4f, false));
  output[121] = float(EE_SSS_BACKFACE_MATCH(7, 7, 0.2f, 0.4f, true));
  output[122] = float(EE_SSS_BACKFACE_MATCH(7, 7, 0.4f, 0.2f, false));
  output[123] = float(EE_SSS_BACKFACE_MATCH(7, 7, 0.2f, 0.2f, false));
  output[124] = float(EE_SSS_BACKFACE_MATCH(7, 7, 0.2f, 1.0f, false));
  output[125] = float(EE_SSS_BACKFACE_MATCH(7, 7, asfloat(0x7fc00000u), 0.4f, false));
  output[126] = float(EE_SSS_BACKFACE_MATCH(7, 7, 0.2f, asfloat(0x7f800000u), false));
  output[127] = EE_SSS_BACKFACE_THICKNESS(float3(0,0,2), float3(0,0,0), float3(0,0,1));
  output[128] = EE_SSS_BACKFACE_THICKNESS(float3(0,0,0), float3(0,0,2), float3(0,0,1));
  output[129] = EE_SSS_BACKFACE_THICKNESS(float3(0,0,2), float3(3,0,0), float3(0,0,1));
  const float3 front = float3(2,4,6), back = float3(1,2,3), sharp = float3(0.1f,0.2f,0.3f);
  for (uint i = 0u; i < 3u; ++i) {
    const float weight = float(i) * 0.5f;
    const float3 color = sharp + front + weight * back +
        EE_SSS_DIFFUSION_ADJUSTMENT(front + back, front + back, float3(0.5f,0.25f,0.75f), weight);
    for (uint channel = 0u; channel < 3u; ++channel) output[130u + i * 3u + channel] = color[channel];
  }
  const float2 filter_offsets[5] = {float2(0), float2(1), float2(1.5f, 0), float2(-1, 0.5f), float2(0)};
  for (uint i = 0u; i < 5u; ++i)
    output[139u + i] = EE_SSS_SHADOW_FILTER_WEIGHT(filter_offsets[i], i == 4u ? 1.0f : 0.5f);
  float3 uniform_sum = float3(0);
  float uniform_weight = 0;
  for (int y = -1; y <= 1; ++y)
    for (int x = -1; x <= 1; ++x) {
      const float weight = EE_SSS_SHADOW_FILTER_WEIGHT(float2(x, y) - float2(0.25f, -0.4f), a + b / 2);
      uniform_sum += weight * EE_SSS_SOLID_TRANSMISSION(0, 2, distances);
      uniform_weight += weight;
    }
  for (uint channel = 0u; channel < 3u; ++channel) output[144u + channel] = uniform_sum[channel] / uniform_weight;
  float3 mixed_sum = float3(0);
  float mixed_weight = 0;
  const float filter_depths[3] = {a + b, a + b / 3, 1};
  for (uint i = 0u; i < 3u; ++i) {
    const float weight = EE_SSS_SHADOW_FILTER_WEIGHT(float2(0), filter_depths[i]);
    mixed_sum += weight * EE_SSS_SOLID_TRANSMISSION(0,
        EE_SSS_SHADOW_THICKNESS(light, 4, 4, filter_depths[i]), distances);
    mixed_weight += weight;
  }
  for (uint channel = 0u; channel < 3u; ++channel) output[147u + channel] = mixed_sum[channel] / mixed_weight;
  output[150] = EE_RESTIR_SSS_DELAYED_JACOBIAN(float2(20, 20), float2(1, 1));
  output[151] = EE_RESTIR_SSS_DELAYED_JACOBIAN(float2(20, 5), float2(1, 4.5f));
  output[152] = EE_RESTIR_SSS_DELAYED_JACOBIAN(float2(5, 20), float2(4.5f, 1));
  output[153] = output[151] * 4.5f;
  output[154] = EE_RESTIR_SSS_DELAYED_JACOBIAN(float2(20, 0), float2(1));
  output[155] = EE_RESTIR_SSS_DELAYED_JACOBIAN(float2(asfloat(0x7fc00000u), 5), float2(1));
  output[156] = EE_RESTIR_SSS_CONFIDENCE_CAP(20, 0);
  output[157] = EE_RESTIR_SSS_CONFIDENCE_CAP(20, 1);
  output[158] = EE_RESTIR_SSS_CONFIDENCE_CAP(20, 0.5f);
  output[159] = EE_RESTIR_SSS_CONFIDENCE_CAP(1, 0.5f);
  float3 shifted_total = float3(0);
  for (uint trial = 0u; trial < 262144u; ++trial) {
    uint seed = EE_XXHASH32(uint3(trial, 0x5353534au, 0u));
    const float x = EE_PCG_RANDOM(seed);
    const float y = max(EE_PCG_RANDOM(seed), 1e-8f);
    const float shifted_x = sqrt(y);
    const float inverse_y = x * x;
    const float3 fresh_value = float3(1 + x, 2 * x, 1 - x * x);
    const float3 history_value = float3(1 + shifted_x, 2 * shifted_x, 1 - y);
    const float3 old_fresh = float3(0.5f + inverse_y, 1 + 3 * inverse_y, 1.5f - inverse_y * inverse_y);
    const float3 old_history = float3(0.5f + y, 1 + 3 * y, 1.5f - y * y);
    const float jacobian = EE_RESTIR_SSS_DELAYED_JACOBIAN(float2(1, 2 * shifted_x), float2(1));
    const float4 targets = float4(EE_RESTIR_SSS_TARGET(fresh_value),
        EE_RESTIR_SSS_TARGET(history_value) * jacobian,
        EE_RESTIR_SSS_TARGET(old_fresh) * (2 * x), EE_RESTIR_SSS_TARGET(old_history));
    const float2 weights = EE_RESTIR_SSS_BALANCE_WEIGHTS(targets, float2(1, 7), float2(1));
    const bool history_selected = EE_PCG_RANDOM(seed) * (weights.x + weights.y) < weights.y;
    const float3 value = history_selected ? history_value : fresh_value;
    shifted_total += value * ((weights.x + weights.y) / EE_RESTIR_SSS_TARGET(value));
  }
  for (uint channel = 0u; channel < 3u; ++channel)
    output[160u + channel] = shifted_total[channel] / 262144.0f;
}
