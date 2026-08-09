// @evoengine-dialect native
__exported import EvoEngine.Cameras;

public struct CameraRayOutputDiagnostics {
  public float3 albedo;
  public float3 normal;
  public float3 debug;
  public float time;
  public uint ray_count;
  public uint path_length;
  public uint primary_hit;
};

public CameraRayOutputDiagnostics EE_CAMERA_INIT_RAY_OUTPUT_DIAGNOSTICS() {
  CameraRayOutputDiagnostics diagnostics = {};
  diagnostics.normal = float3(0.0f, 0.0f, 1.0f);
  return diagnostics;
}
