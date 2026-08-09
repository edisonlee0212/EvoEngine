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
