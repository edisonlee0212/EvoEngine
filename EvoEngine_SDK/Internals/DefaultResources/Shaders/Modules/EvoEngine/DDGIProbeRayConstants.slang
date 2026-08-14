// @evoengine-dialect native
public struct DdgiProbeRayConstants {
  public float4 first_probe;
  public float4 probe_step_x;
  public float4 probe_step_y;
  public float4 probe_step_z;
  public uint4 probe_counts_and_ray_count;
  public uint4 selected_probe_volume_flags_environment;
  public float4 trace_parameters;
  public int4 probe_scroll_offset;
};

[[vk::push_constant]] public ConstantBuffer<DdgiProbeRayConstants> EE_DDGI_PROBE_RAY_CONSTANTS;

public uint EE_DDGI_TOTAL_RAY_COUNT() {
  return max(EE_DDGI_PROBE_RAY_CONSTANTS.probe_counts_and_ray_count.w, 1u);
}

public uint EE_DDGI_FIXED_RAY_COUNT() {
  return asuint(EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.z) & 0x3fu;
}

public uint EE_DDGI_EMISSIVE_RAY_COUNT() {
  return (asuint(EE_DDGI_PROBE_RAY_CONSTANTS.trace_parameters.z) >> 6u) & 0x1fffu;
}

public uint EE_DDGI_UNIFORM_RAY_COUNT() {
  return EE_DDGI_TOTAL_RAY_COUNT() - min(EE_DDGI_EMISSIVE_RAY_COUNT(), EE_DDGI_TOTAL_RAY_COUNT());
}
