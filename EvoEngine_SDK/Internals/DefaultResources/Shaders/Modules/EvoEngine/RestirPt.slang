// @evoengine-dialect native
public static const uint EE_RESTIR_PT_RESERVOIR_VERSION = 1u;
public static const uint EE_RESTIR_PT_RESERVOIR_BYTES = 128u;
public static const uint EE_RESTIR_PT_PRIMARY_SURFACE_BYTES = 32u;
public static const uint EE_RESTIR_PT_SPATIAL_SHIFT_BYTES = 32u;
public static const uint EE_RESTIR_PT_GENERATED_MAX_EVENTS = 32u;
public static const uint EE_RESTIR_PT_GENERATED_PIXEL_BYTES = 4u + 8u * EE_RESTIR_PT_GENERATED_MAX_EVENTS;
public static const uint EE_RESTIR_PT_INVALID_INSTANCE = 0xffffffffu;
public static const uint EE_RESTIR_PT_PATH_INVALID = 0u;
public static const uint EE_RESTIR_PT_PATH_PRIMARY_EMISSION = 1u;
public static const uint EE_RESTIR_PT_PATH_BSDF_EMISSION = 2u;
public static const uint EE_RESTIR_PT_PATH_BSDF_ENVIRONMENT = 3u;
public static const uint EE_RESTIR_PT_PATH_NEE_EMISSION = 4u;
public static const uint EE_RESTIR_PT_PATH_NEE_ENVIRONMENT = 5u;
public static const uint EE_RESTIR_PT_PATH_PRIMARY_BACKGROUND = 6u;
public static const uint EE_RESTIR_PT_PATH_NEE_PUNCTUAL = 7u;
public static const uint EE_RESTIR_PT_PATH_UNLIT = 8u;
public static const uint EE_RESTIR_PT_PATH_VOLUME_SCATTER_NEE = 9u;
public static const uint EE_RESTIR_PT_FLAG_DELTA_PREFIX = 1u;
public static const uint EE_RESTIR_PT_FLAG_SECONDARY_EMISSION_RECONNECTION = 4u;
public static const uint EE_RESTIR_PT_FLAG_FIRST_DIFFUSE_EVENT = 8u;
public static const uint EE_RESTIR_PT_PROFILE_SOURCE_REASON_SHIFT = 8u;
public static const uint EE_RESTIR_PT_SHIFT_VALID = 0u;
public static const uint EE_RESTIR_PT_SHIFT_NO_PARTNER = 1u;
public static const uint EE_RESTIR_PT_SHIFT_NO_SOURCE = 2u;
public static const uint EE_RESTIR_PT_SHIFT_SURFACE_MISMATCH = 3u;
public static const uint EE_RESTIR_PT_SHIFT_ZERO_TARGET = 4u;
public static const uint EE_RESTIR_PT_SHIFT_SKIPPED_BACKGROUND = 5u;

public struct RestirPtPathReservoir {
  public uint version;
  public uint path_type;
  public uint sample_seed;
  public uint path_length;
  public uint reconnection_length;
  public uint instance_id;
  public uint primitive_id;
  public uint flags;
  public float2 barycentrics;
  public float light_pdf;
  public float bsdf_pdf;
  public float3 contribution;
  public float target_density;
  public float3 reconnection_direction;
  public float jacobian;
  public float3 reconnection_position;
  public float effective_count;
  public float3 reconnection_normal;
  public float weight_sum;
  public float3 primary_position;
  public float selected_weight;
};

public struct RestirPtPrimarySurface {
  public float3 position;
  public float depth;
  public float3 normal;
  public uint material_id;
};

public struct RestirPtSpatialShift {
  public float3 contribution;
  public float weight_sum;
  public uint partner_index;
  public uint status;
  public uint ray_count;
  public uint reserved;
};

public uint2 EE_RESTIR_PT_PAIR(const uint2 pixel, const uint2 image_size, const uint frame_id,
                              const uint neighbor_index) {
  const uint offset = neighbor_index < 2u ? 1u : 2u;
  const uint2 partner = ((frame_id + neighbor_index) & 1u) == 0u
                            ? uint2(pixel.x ^ offset, pixel.y) : uint2(pixel.x, pixel.y ^ offset);
  return any(partner >= image_size) ? uint2(0xffffffffu) : partner;
}

public RestirPtPrimarySurface EE_RESTIR_PT_EMPTY_PRIMARY_SURFACE() {
  RestirPtPrimarySurface surface;
  surface.position = float3(0.0f);
  surface.depth = 0.0f;
  surface.normal = float3(0.0f);
  surface.material_id = 0xffffffffu;
  return surface;
}

public RestirPtPathReservoir EE_RESTIR_PT_EMPTY_RESERVOIR() {
  RestirPtPathReservoir r;
  r.version = EE_RESTIR_PT_RESERVOIR_VERSION;
  r.path_type = 0u;
  r.sample_seed = 0u;
  r.path_length = 0u;
  r.reconnection_length = 0u;
  r.instance_id = EE_RESTIR_PT_INVALID_INSTANCE;
  r.primitive_id = 0u;
  r.flags = 0u;
  r.barycentrics = float2(0.0f);
  r.light_pdf = 0.0f;
  r.bsdf_pdf = 0.0f;
  r.contribution = float3(0.0f);
  r.target_density = 0.0f;
  r.reconnection_direction = float3(0.0f);
  r.jacobian = 0.0f;
  r.reconnection_position = float3(0.0f);
  r.effective_count = 0.0f;
  r.reconnection_normal = float3(0.0f);
  r.weight_sum = 0.0f;
  r.primary_position = float3(0.0f);
  r.selected_weight = 0.0f;
  return r;
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

public RestirPtPrimarySurface EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(RWByteAddressBuffer buffer,
                                                                 const uint pixel_index) {
  const uint base = pixel_index * EE_RESTIR_PT_PRIMARY_SURFACE_BYTES;
  RestirPtPrimarySurface surface;
  const float4 position = asfloat(buffer.Load4(base));
  const uint4 normal = buffer.Load4(base + 16u);
  surface.position = position.xyz;
  surface.depth = position.w;
  surface.normal = asfloat(normal.xyz);
  surface.material_id = normal.w;
  return surface;
}

public void EE_RESTIR_PT_STORE_PRIMARY_SURFACE(RWByteAddressBuffer buffer, const uint pixel_index,
                                                const RestirPtPrimarySurface surface) {
  const uint base = pixel_index * EE_RESTIR_PT_PRIMARY_SURFACE_BYTES;
  buffer.Store4(base, asuint(float4(surface.position, surface.depth)));
  buffer.Store4(base + 16u, uint4(asuint(surface.normal), surface.material_id));
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
