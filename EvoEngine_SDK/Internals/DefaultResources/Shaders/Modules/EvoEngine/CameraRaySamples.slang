// @evoengine-dialect native
public static const uint EE_RESTIR_PT_RESERVOIR_VERSION = 1u;
public static const uint EE_RESTIR_PT_RESERVOIR_BYTES = 128u;
public static const uint EE_RESTIR_PT_RESOLVED_BYTES = 16u;
public static const uint EE_RESTIR_PT_PRIMARY_SURFACE_BYTES = 48u;
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
public static const uint EE_RESTIR_PT_FLAG_INITIAL_CAMERA_JITTER = 16u;
public static const uint EE_RESTIR_PT_FLAG_SUFFIX_DESCRIPTOR = 32u;
public static const uint EE_RESTIR_PT_FLAG_LONG_SUFFIX_DESCRIPTOR = 64u;
public static const uint EE_RESTIR_PT_FLAG_PRIMARY_DIFFUSE = 128u;
public static const uint EE_RESTIR_PT_RECONNECTION_SECONDARY_INDEX = 2u;
public static const float EE_RESTIR_PT_RECON_FOOTPRINT_C_CONSTANT = 0.02f;
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
  public uint instance_index;
  public uint entity_index;
  public float2 screen_motion;
};

public struct RestirPtSpatialShift {
  public float3 contribution;
  public float weight_sum;
  public uint partner_index;
  public uint status;
  public uint ray_count;
  public uint reserved;
};


public RestirPtPrimarySurface EE_RESTIR_PT_EMPTY_PRIMARY_SURFACE() {
  RestirPtPrimarySurface surface;
  surface.position = float3(0.0f);
  surface.depth = 0.0f;
  surface.normal = float3(0.0f);
  surface.material_id = 0xffffffffu;
  surface.instance_index = 0xffffffffu;
  surface.entity_index = 0u;
  surface.screen_motion = float2(0.0f);
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
  const uint2 instance = buffer.Load2(base + 32u);
  surface.instance_index = instance.x;
  surface.entity_index = instance.y;
  surface.screen_motion = asfloat(buffer.Load2(base + 40u));
  return surface;
}

public void EE_RESTIR_PT_STORE_PRIMARY_SURFACE(RWByteAddressBuffer buffer, const uint pixel_index,
                                                const RestirPtPrimarySurface surface) {
  const uint base = pixel_index * EE_RESTIR_PT_PRIMARY_SURFACE_BYTES;
  buffer.Store4(base, asuint(float4(surface.position, surface.depth)));
  buffer.Store4(base + 16u, uint4(asuint(surface.normal), surface.material_id));
  buffer.Store2(base + 32u, uint2(surface.instance_index, surface.entity_index));
  buffer.Store2(base + 40u, asuint(surface.screen_motion));
}
