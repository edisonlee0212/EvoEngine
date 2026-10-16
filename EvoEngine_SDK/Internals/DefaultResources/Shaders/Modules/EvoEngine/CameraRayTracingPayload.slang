public struct CameraRayTracingPayload {
  public float3 shadow_transmission;
  public uint seed;

  public float2 barycentrics;
  public float hit_t;
  public uint instance_index;

  public float shadow_previous_hit_t;
  public uint primitive_id;
  public uint type;
  public uint shadow_is_inside;
  public uint camera_index;

  public float3 object_geometric_normal;
  public uint geometry_type;
#if EE_CAMERA_SUBSURFACE_HIT_GROUP
  // Primary-ray inputs and direct SSS output; nested geometry probes clear sss_request.
  public uint sss_request;
  public uint sss_sample_seed;
  public float sss_ray_spread;
  public float3 sss_radiance;
  public uint sss_ray_count;
  public float sss_sample_pdf;
  public uint2 sss_shadow_seeds;
  public uint sss_suppress_light;
#endif
};
