// @evoengine-dialect native
public static const uint EE_DDGI_MAX_VOLUME_COUNT = 8u;
public static const uint EE_REFLECTION_PROBE_MAX_COUNT = 32u;

public struct DdgiVolumeInfo {
  public float4 first_probe;
  public float4 probe_step_x;
  public float4 probe_step_y;
  public float4 probe_step_z;
  public float4 probe_counts;
  public int4 probe_scroll_and_priority;
  public uint4 atlas_parameters;
  public float4 volume_parameters;
  public float4 lighting_parameters;
  public uint4 identity_and_flags;
};

public struct ReflectionProbeInfo {
  public float4x4 world_to_probe;
  public float4 shape_parameters;
  public float4 projection_parameters;
  public float4 lighting_parameters;
  public uint4 identity_and_flags;
  public uint4 transition_parameters;
};

public struct RenderInfo {
  public float shadow_split_0;
  public float shadow_split_1;
  public float shadow_split_2;
  public float shadow_split_3;

  public int shadow_sample_size;
  public int debug_visualization;
  public float shadow_cascade_transition_width;
  public float indirect_lighting_intensity;

  public float strand_subdivision_x;
  public float strand_subdivision_y;
  public int strand_subdivision_max_x;
  public int strand_subdivision_max_y;

  public int directional_light_size;
  public int point_light_size;
  public int spot_light_size;
  public int brdf_lut_map_index;

  public int4 shadow_debug_parameters;
  public float4 shadow_fade_parameters;
  public uint4 emissive_triangle_parameters;
  public uint4 ddgi_volume_header;
  public DdgiVolumeInfo ddgi_volumes[EE_DDGI_MAX_VOLUME_COUNT];
  public uint4 reflection_probe_header;
  public ReflectionProbeInfo reflection_probes[EE_REFLECTION_PROBE_MAX_COUNT];
};

[[vk::binding(0, 0)]]
public ConstantBuffer<RenderInfo, Std140DataLayout> EE_RENDER_INFO;
