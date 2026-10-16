__exported import EvoEngine.Subsurface;

public static const uint EE_RESTIR_SSS_HISTORY_BYTES = 112u;
public static const uint EE_RESTIR_SSS_RESIDUAL_BYTES = 64u;

public struct RestirSssDirectSample {
  public uint receiver_instance;
  public uint receiver_primitive;
  public float2 receiver_barycentrics;
  public uint sample_instance;
  public uint sample_primitive;
  public float2 sample_barycentrics;
  public uint sampling_seed;
  public uint axis_channel;
  public uint intersection_index;
  public uint intersection_count;
  public float3 probe_direction;
  public float area_pdf;
  public uint light_technique;
  public uint light_seed;
  public uint bsdf_seed;
  public uint shadow_seed;
  public float3 throughput;
  public float resolve_weight;
  public float3 receiver_position;
  public float confidence;
  public float3 receiver_normal;
  public uint receiver_material;
  public float3 receiver_tangent;
  public float target_density;
  public uint origin_pixel;
  public uint origin_frame;
};

public RestirSssDirectSample EE_RESTIR_SSS_EMPTY_SAMPLE() {
  RestirSssDirectSample sample = {};
  sample.receiver_instance = sample.sample_instance = sample.light_technique = 0xffffffffu;
  sample.receiver_material = sample.origin_pixel = 0xffffffffu;
  return sample;
}

public void EE_RESTIR_SSS_STORE(RWByteAddressBuffer buffer, uint index, RestirSssDirectSample sample, bool history_format = true) {
  const uint offset = index * (history_format ? EE_RESTIR_SSS_HISTORY_BYTES : 96u);
  if (history_format) {
    // Axis/channel occupy four bits; light tags are 0=empty, 1=direct, 2=emissive.
    buffer.Store4(offset, uint4(sample.receiver_instance, sample.sample_instance, sample.sample_primitive,
        sample.axis_channel | ((sample.light_technique + 1u) << 4u)));
    buffer.Store4(offset + 16u, uint4(asuint(sample.sample_barycentrics), sample.light_seed, sample.bsdf_seed));
    buffer.Store4(offset + 32u, uint4(sample.receiver_material, asuint(sample.confidence),
        asuint(sample.target_density), asuint(sample.resolve_weight)));
    buffer.Store4(offset + 48u, asuint(float4(sample.throughput, sample.receiver_position.x)));
    buffer.Store4(offset + 64u, asuint(float4(sample.receiver_position.yz, sample.receiver_normal.xy)));
    buffer.Store4(offset + 80u, asuint(float4(sample.receiver_normal.z, sample.receiver_tangent)));
    buffer.Store4(offset + 96u, uint4(sample.sampling_seed, sample.intersection_index, sample.origin_pixel, sample.origin_frame));
    return;
  }
  buffer.Store4(offset, uint4(sample.receiver_instance, sample.receiver_primitive, asuint(sample.receiver_barycentrics)));
  buffer.Store4(offset + 16u, uint4(sample.sample_instance, sample.sample_primitive, asuint(sample.sample_barycentrics)));
  buffer.Store4(offset + 32u, uint4(sample.sampling_seed, sample.axis_channel, sample.intersection_index, sample.intersection_count));
  buffer.Store4(offset + 48u, asuint(float4(sample.probe_direction, sample.area_pdf)));
  buffer.Store4(offset + 64u, uint4(sample.light_technique, sample.light_seed, sample.bsdf_seed, sample.shadow_seed));
  buffer.Store4(offset + 80u, asuint(float4(sample.throughput, sample.resolve_weight)));
}

public RestirSssDirectSample EE_RESTIR_SSS_LOAD(RWByteAddressBuffer buffer, uint index, bool history_format = true) {
  const uint offset = index * (history_format ? EE_RESTIR_SSS_HISTORY_BYTES : 96u);
  RestirSssDirectSample sample = EE_RESTIR_SSS_EMPTY_SAMPLE();
  if (history_format) {
    const uint4 point = buffer.Load4(offset);
    const uint4 light = buffer.Load4(offset + 16u);
    const uint4 weights = buffer.Load4(offset + 32u);
    const float4 throughput = asfloat(buffer.Load4(offset + 48u));
    const float4 position_normal = asfloat(buffer.Load4(offset + 64u));
    const float4 normal_tangent = asfloat(buffer.Load4(offset + 80u));
    sample.receiver_instance = point.x;
    sample.sample_instance = point.y;
    sample.sample_primitive = point.z;
    sample.axis_channel = point.w & 15u;
    sample.light_technique = (point.w >> 4u) - 1u;
    sample.sample_barycentrics = asfloat(light.xy);
    sample.light_seed = light.z;
    sample.bsdf_seed = light.w;
    sample.receiver_material = weights.x;
    sample.confidence = asfloat(weights.y);
    sample.target_density = asfloat(weights.z);
    sample.resolve_weight = asfloat(weights.w);
    sample.throughput = throughput.xyz;
    sample.receiver_position = float3(throughput.w, position_normal.xy);
    sample.receiver_normal = float3(position_normal.zw, normal_tangent.x);
    sample.receiver_tangent = normal_tangent.yzw;
    const uint4 replay = buffer.Load4(offset + 96u);
    sample.sampling_seed = replay.x;
    sample.intersection_index = replay.y;
    sample.origin_pixel = replay.z;
    sample.origin_frame = replay.w;
    return sample;
  }
  const uint4 receiver = buffer.Load4(offset);
  const uint4 point = buffer.Load4(offset + 16u);
  const uint4 sampling = buffer.Load4(offset + 32u);
  const float4 probe = asfloat(buffer.Load4(offset + 48u));
  const uint4 light = buffer.Load4(offset + 64u);
  const float4 weight = asfloat(buffer.Load4(offset + 80u));
  sample.receiver_instance = receiver.x;
  sample.receiver_primitive = receiver.y;
  sample.receiver_barycentrics = asfloat(receiver.zw);
  sample.sample_instance = point.x;
  sample.sample_primitive = point.y;
  sample.sample_barycentrics = asfloat(point.zw);
  sample.sampling_seed = sampling.x;
  sample.axis_channel = sampling.y;
  sample.intersection_index = sampling.z;
  sample.intersection_count = sampling.w;
  sample.probe_direction = probe.xyz;
  sample.area_pdf = probe.w;
  sample.light_technique = light.x;
  sample.light_seed = light.y;
  sample.bsdf_seed = light.z;
  sample.shadow_seed = light.w;
  sample.throughput = weight.xyz;
  sample.resolve_weight = weight.w;
  return sample;
}

public float EE_RESTIR_SSS_TARGET(float3 contribution) {
  const float target = dot(contribution, float3(0.2126f, 0.7152f, 0.0722f));
  return all(isfinite(contribution)) && isfinite(target) ? max(target, 0.0f) : 0.0f;
}

public float2 EE_RESTIR_SSS_BALANCE_WEIGHTS(float4 targets, float2 confidence, float2 contribution_weight) {
  // Targets: fresh at fresh, history at fresh, fresh at history, history at history.
  const float2 numerator = confidence * targets.xw;
  const float2 denominator = numerator + confidence.yx * targets.zy;
  const float2 mis = float2(denominator.x > 0.0f ? numerator.x / denominator.x : 0.0f,
                           denominator.y > 0.0f ? numerator.y / denominator.y : 0.0f);
  const float2 weights = mis * targets.xy * contribution_weight;
  return float2(isfinite(weights.x) ? max(weights.x, 0.0f) : 0.0f,
                isfinite(weights.y) ? max(weights.y, 0.0f) : 0.0f);
}

public float2 EE_RESTIR_SSS_PAIRWISE_MIS(float4 targets, float2 confidence, uint requested_neighbors) {
  const float canonical = confidence.x / float(max(requested_neighbors, 1u));
  const float2 denominator = confidence.y * targets.zw + canonical * targets.xy;
  const float2 neighbor = float2(denominator.x > 0.0f ? confidence.y * targets.z / denominator.x : 0.0f,
                                 denominator.y > 0.0f ? confidence.y * targets.w / denominator.y : 0.0f);
  return float2(1.0f - (isfinite(neighbor.x) ? saturate(neighbor.x) : 0.0f),
                isfinite(neighbor.y) ? saturate(neighbor.y) : 0.0f);
}

public float EE_RESTIR_SSS_SPATIAL_RADIUS(float3 distance, float3 position,
    float4x4 projection_view, float4x4 inverse_view, float2 image_size) {
  const float radius = 2.0f * max(distance.x, max(distance.y, distance.z));
  const float4 center = mul(float4(position, 1), projection_view);
  const float4 right = mul(float4(position + radius * inverse_view[0].xyz, 1), projection_view);
  const float4 up = mul(float4(position + radius * inverse_view[1].xyz, 1), projection_view);
  if (!(center.w > 0.0f) || !(right.w > 0.0f) || !(up.w > 0.0f)) return 5.0f;
  const float2 center_pixel = center.xy / center.w;
  const float projected = max(length((right.xy / right.w - center_pixel) * image_size * 0.5f),
                              length((up.xy / up.w - center_pixel) * image_size * 0.5f));
  return isfinite(projected) ? clamp(projected, 5.0f, 30.0f) : 5.0f;
}

public int2 EE_RESTIR_SSS_SPATIAL_OFFSET(float radius, float2 random) {
  const float angle = 6.28318530718f * random.y;
  return int2(round(radius * sqrt(random.x) * float2(cos(angle), sin(angle))));
}

public float EE_RESTIR_SSS_DELAYED_JACOBIAN(float2 bssrdf_pdf, float2 light_pdf) {
  if (any(bssrdf_pdf <= float2(0)) || any(light_pdf <= float2(0)) ||
      !all(isfinite(bssrdf_pdf)) || !all(isfinite(light_pdf))) return 0.0f;
  const float jacobian = (bssrdf_pdf.x / bssrdf_pdf.y) * (light_pdf.x / light_pdf.y);
  return isfinite(jacobian) ? jacobian : 0.0f;
}

public float EE_RESTIR_SSS_CONFIDENCE_CAP(float cap, float duplication) {
  return lerp(cap, 1.0f, pow(saturate(duplication), 0.1f));
}
