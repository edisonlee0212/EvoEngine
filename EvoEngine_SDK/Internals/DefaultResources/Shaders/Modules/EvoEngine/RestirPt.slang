static const uint EE_RESTIR_PT_PAIRING_SIZES[16] = {
    254u, 230u, 210u, 190u, 178u, 166u, 154u, 142u,
    246u, 238u, 222u, 206u, 198u, 182u, 174u, 162u};
static const uint EE_RESTIR_PT_PAIRING_OFFSETS[16] = {
    0u, 64516u, 117416u, 161516u, 197616u, 229300u, 256856u, 280572u,
    300736u, 361252u, 417896u, 467180u, 509616u, 548820u, 581944u, 612220u};
__exported import EvoEngine.CameraRaySamples;
#if !EE_ENABLE_RESTIR
#error ReSTIR is disabled in this build.
#endif
[[vk::binding(20, 2)]] public ByteAddressBuffer ee_restir_pt_pairing_map;

uint EE_RESTIR_PT_PAIR_HASH(uint value) {
  value ^= value >> 16u;
  value *= 0x7feb352du;
  value ^= value >> 15u;
  value *= 0x846ca68bu;
  return value ^ (value >> 16u);
}

public uint2 EE_RESTIR_PT_PAIR(const uint2 pixel, const uint2 image_size, const uint frame_id,
                              const uint neighbor_index) {
  const uint size = EE_RESTIR_PT_PAIRING_SIZES[neighbor_index];
  const uint offset = EE_RESTIR_PT_PAIRING_OFFSETS[neighbor_index];
  const bool transpose = (EE_RESTIR_PT_PAIR_HASH(frame_id ^ (neighbor_index * 8128u)) & 1u) != 0u;
  const bool flip = (EE_RESTIR_PT_PAIR_HASH(frame_id ^ (neighbor_index * 31337u)) & 1u) != 0u;
  int2 coordinate = transpose ? pixel.yx : pixel;
  if (flip) coordinate.x = int(size) - 1 - coordinate.x;
  const uint hash_x = EE_RESTIR_PT_PAIR_HASH(frame_id ^ (neighbor_index * 1973u));
  const uint hash_y = EE_RESTIR_PT_PAIR_HASH(hash_x);
  coordinate += int2(hash_x % size, hash_y % size);
  coordinate = (coordinate % int(size) + int(size)) % int(size);
  const uint packed = ee_restir_pt_pairing_map.Load((offset + uint(coordinate.y) * size + uint(coordinate.x)) * 4u);
  int2 delta = int2(int(packed << 16u) >> 16u, int(packed) >> 16u);
  if (flip) delta.x = -delta.x;
  if (transpose) delta = delta.yx;
  const int2 partner = int2(pixel) + delta;
  return any(partner < int2(0)) || any(partner >= int2(image_size))
             ? uint2(0xffffffffu) : uint2(partner);
}





public float EE_RESTIR_PT_TARGET_DENSITY(const float3 contribution) {
  const float target = dot(max(contribution, float3(0.0f)), float3(0.2126f, 0.7152f, 0.0722f));
  return isfinite(target) ? target : 0.0f;
}

public bool EE_RESTIR_PT_SURFACES_COMPATIBLE(const RestirPtPrimarySurface lhs,
                                             const RestirPtPrimarySurface rhs) {
  if (lhs.material_id != rhs.material_id) {
    return false;
  }
  if (lhs.material_id == EE_RESTIR_PT_INVALID_INSTANCE) {
    return true;
  }
  return dot(lhs.normal, rhs.normal) >= 0.9f &&
         distance(lhs.position, rhs.position) <= 0.1f * min(lhs.depth, rhs.depth);
}

public bool EE_RESTIR_PT_ENHANCED_SPATIAL_PAIR_ACCEPTED(const RestirPtPrimarySurface lhs,
                                                        const RestirPtPrimarySurface rhs,
                                                        const float3 camera_position) {
  if (lhs.material_id == EE_RESTIR_PT_INVALID_INSTANCE || lhs.material_id != rhs.material_id ||
      dot(lhs.normal, rhs.normal) < 0.9f) return false;
  const float3 delta = rhs.position - lhs.position;
  const float lhs_tolerance = 0.005f * (1.0f + distance(lhs.position, camera_position));
  const float rhs_tolerance = 0.005f * (1.0f + distance(rhs.position, camera_position));
  return length(delta) <= 30.0f * min(lhs_tolerance, rhs_tolerance) &&
         abs(dot(delta, lhs.normal)) <= lhs_tolerance &&
         abs(dot(delta, rhs.normal)) <= rhs_tolerance;
}

public void EE_RESTIR_PT_ADD_CANDIDATE(inout RestirPtPathReservoir reservoir,
                                        const RestirPtPathReservoir candidate, const float random_value) {
  const float weight = EE_RESTIR_PT_TARGET_DENSITY(candidate.contribution);
  if (weight <= 0.0f) {
    return;
  }
  const float next_sum = reservoir.weight_sum + weight;
  if (!isfinite(next_sum)) {
    return;
  }
  if (random_value * next_sum < weight) {
    const float previous_count = reservoir.effective_count;
    const float previous_sum = reservoir.weight_sum;
    reservoir = candidate;
    reservoir.effective_count = previous_count;
    reservoir.weight_sum = previous_sum;
    reservoir.target_density = weight;
  }
  reservoir.weight_sum = next_sum;
}

public void EE_RESTIR_PT_FINALIZE_CANDIDATES(inout RestirPtPathReservoir reservoir) {
  reservoir.effective_count = 1.0f;
  reservoir.selected_weight = reservoir.target_density > 0.0f
                                  ? reservoir.weight_sum / reservoir.target_density : 0.0f;
}

public bool EE_RESTIR_PT_MERGE_INITIAL_RESERVOIR(inout RestirPtPathReservoir reservoir,
                                                  const RestirPtPathReservoir source,
                                                  const float random_value) {
  const float next_count = reservoir.effective_count + source.effective_count;
  const float next_sum = reservoir.weight_sum + source.weight_sum;
  if (!isfinite(next_count) || !isfinite(next_sum)) {
    return false;
  }
  bool selected = false;
  if (source.weight_sum > 0.0f && random_value * next_sum < source.weight_sum) {
    reservoir = source;
    selected = true;
  }
  reservoir.effective_count = next_count;
  reservoir.weight_sum = next_sum;
  reservoir.selected_weight = reservoir.target_density > 0.0f && next_count > 0.0f
                                  ? next_sum / (reservoir.target_density * next_count) : 0.0f;
  return selected;
}

public float3 EE_RESTIR_PT_RESOLVE_CANDIDATE(const RestirPtPathReservoir reservoir) {
  if (reservoir.version != EE_RESTIR_PT_RESERVOIR_VERSION ||
      reservoir.target_density <= 0.0f || !isfinite(reservoir.selected_weight)) {
    return float3(0.0f);
  }
  return reservoir.contribution * reservoir.selected_weight;
}

public uint EE_RESTIR_PT_SIGNAL_CLASS(const RestirPtPathReservoir reservoir) {
  if (reservoir.path_type == EE_RESTIR_PT_PATH_PRIMARY_BACKGROUND ||
      reservoir.path_type == EE_RESTIR_PT_PATH_PRIMARY_EMISSION ||
      (reservoir.path_type == EE_RESTIR_PT_PATH_UNLIT && reservoir.path_length == 1u))
    return 2u;
  return (reservoir.flags & EE_RESTIR_PT_FLAG_PRIMARY_DIFFUSE) != 0u ? 0u : 1u;
}

public void EE_RESTIR_PT_ADD_SIGNAL(const RestirPtPathReservoir reservoir, const float3 contribution,
                                    inout float3 diffuse, inout float3 specular, inout float3 excluded) {
  const uint signal_class = EE_RESTIR_PT_SIGNAL_CLASS(reservoir);
  if (signal_class == 0u)
    diffuse += contribution;
  else if (signal_class == 1u)
    specular += contribution;
  else
    excluded += contribution;
}

public RestirPtPathReservoir EE_RESTIR_PT_LOAD(RWByteAddressBuffer buffer, const uint pixel_index) {
  const uint base = pixel_index * EE_RESTIR_PT_RESERVOIR_BYTES;
  RestirPtPathReservoir r;
  const uint4 header = buffer.Load4(base);
  r.version = header.x;
  r.path_type = header.y;
  r.sample_seed = header.z;
  r.path_length = header.w;
  const uint4 geometry = buffer.Load4(base + 16u);
  r.reconnection_length = geometry.x;
  r.instance_id = geometry.y;
  r.primitive_id = geometry.z;
  r.flags = geometry.w;
  const float4 sample = asfloat(buffer.Load4(base + 32u));
  r.barycentrics = sample.xy;
  r.light_pdf = sample.z;
  r.bsdf_pdf = sample.w;
  const float4 contribution = asfloat(buffer.Load4(base + 48u));
  r.contribution = contribution.xyz;
  r.target_density = contribution.w;
  const float4 direction = asfloat(buffer.Load4(base + 64u));
  r.reconnection_direction = direction.xyz;
  r.jacobian = direction.w;
  const float4 position = asfloat(buffer.Load4(base + 80u));
  r.reconnection_position = position.xyz;
  r.effective_count = position.w;
  const float4 normal = asfloat(buffer.Load4(base + 96u));
  r.reconnection_normal = normal.xyz;
  r.weight_sum = normal.w;
  const float4 primary = asfloat(buffer.Load4(base + 112u));
  r.primary_position = primary.xyz;
  r.selected_weight = primary.w;
  return r;
}

public void EE_RESTIR_PT_STORE(RWByteAddressBuffer buffer, const uint pixel_index,
                               const RestirPtPathReservoir r) {
  const uint base = pixel_index * EE_RESTIR_PT_RESERVOIR_BYTES;
  buffer.Store4(base, uint4(r.version, r.path_type, r.sample_seed, r.path_length));
  buffer.Store4(base + 16u, uint4(r.reconnection_length, r.instance_id, r.primitive_id, r.flags));
  buffer.Store4(base + 32u, asuint(float4(r.barycentrics, r.light_pdf, r.bsdf_pdf)));
  buffer.Store4(base + 48u, asuint(float4(r.contribution, r.target_density)));
  buffer.Store4(base + 64u, asuint(float4(r.reconnection_direction, r.jacobian)));
  buffer.Store4(base + 80u, asuint(float4(r.reconnection_position, r.effective_count)));
  buffer.Store4(base + 96u, asuint(float4(r.reconnection_normal, r.weight_sum)));
  buffer.Store4(base + 112u, asuint(float4(r.primary_position, r.selected_weight)));
}





public RestirPtSpatialShift EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(RWByteAddressBuffer buffer, const uint pixel_index) {
  const uint base = pixel_index * EE_RESTIR_PT_SPATIAL_SHIFT_BYTES;
  RestirPtSpatialShift shift;
  const float4 contribution = asfloat(buffer.Load4(base));
  const uint4 metadata = buffer.Load4(base + 16u);
  shift.contribution = contribution.xyz;
  shift.weight_sum = contribution.w;
  shift.partner_index = metadata.x;
  shift.status = metadata.y;
  shift.ray_count = metadata.z;
  shift.reserved = metadata.w;
  return shift;
}

public void EE_RESTIR_PT_STORE_SPATIAL_SHIFT(RWByteAddressBuffer buffer, const uint pixel_index,
                                              const RestirPtSpatialShift shift) {
  const uint base = pixel_index * EE_RESTIR_PT_SPATIAL_SHIFT_BYTES;
  buffer.Store4(base, asuint(float4(shift.contribution, shift.weight_sum)));
  buffer.Store4(base + 16u, uint4(shift.partner_index, shift.status, shift.ray_count, shift.reserved));
}
