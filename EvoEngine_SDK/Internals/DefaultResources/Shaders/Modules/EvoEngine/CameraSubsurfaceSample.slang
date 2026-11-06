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
