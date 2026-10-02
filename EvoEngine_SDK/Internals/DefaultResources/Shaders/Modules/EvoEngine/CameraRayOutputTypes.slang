__exported import EvoEngine.Cameras;

public struct CameraRayOutputDiagnostics {
  public float3 albedo;
  public float3 normal;
  public float roughness;
  public float3 debug;
  public float time;
  public uint ray_count;
  public uint path_length;
  public uint primary_hit;
  public float secondary_hit_distance;
  public uint secondary_lobe;
  public float3 nrd_diffuse_factor;
  public float3 nrd_specular_factor;
  public float3 nrd_diffuse_radiance;
  public float3 nrd_specular_radiance;
  public float3 nrd_excluded_radiance;
};

public CameraRayOutputDiagnostics EE_CAMERA_INIT_RAY_OUTPUT_DIAGNOSTICS() {
  CameraRayOutputDiagnostics diagnostics = {};
  diagnostics.normal = float3(0.0f, 0.0f, 1.0f);
  diagnostics.roughness = 1.0f;
  diagnostics.secondary_lobe = 2u;
  diagnostics.nrd_diffuse_factor = float3(1.0f);
  diagnostics.nrd_specular_factor = float3(1.0f);
  return diagnostics;
}
