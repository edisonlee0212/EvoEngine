#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
import EvoEngine.CameraRayIntegrator;
import EvoEngine.CameraRayTraversalPolicies;

[shader("raygeneration")] void main() {
  EE_CAMERA_RENDER_PIXEL<CameraRayTracingTraversalPolicy, EE_GLTF_COMPILED_FEATURE_MASK>(
      DispatchRaysIndex().xy, DispatchRaysDimensions().xy);
}
