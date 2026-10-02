import EvoEngine.CameraRayIntegrator;

public bool EE_RESTIR_PT_TEMPORAL_SURFACES_MATCH(const RestirPtPrimarySurface current,
                                                 const RestirPtPrimarySurface previous,
                                                 const float3 camera_position) {
  if (current.material_id == EE_RESTIR_PT_INVALID_INSTANCE ||
      current.material_id != previous.material_id ||
      (current.entity_index != 0u && previous.entity_index != 0u &&
       current.entity_index != previous.entity_index) ||
      dot(current.normal, previous.normal) < 0.9f) return false;
  const float3 delta = previous.position - current.position;
  const float tolerance = 0.005f * (1.0f + distance(current.position, camera_position));
  return abs(dot(delta, current.normal)) <= tolerance && length(delta) <= 30.0f * tolerance;
}

public bool EE_RESTIR_PT_REPROJECT_SURFACE(const RestirPtPrimarySurface surface, const float3 previous_position,
                                          const Camera camera,
                                          const uint2 image_size, out uint2 previous_pixel) {
  previous_pixel = uint2(0xffffffffu);
  if (surface.material_id == EE_RESTIR_PT_INVALID_INSTANCE) return false;
  const float4 clip = mul(float4(previous_position, 1.0f), camera.previous_projection_view);
  if (clip.w <= 0.0f || any(!isfinite(clip))) return false;
  const float2 projected = (clip.xy / clip.w * 0.5f + 0.5f) * float2(image_size);
  const int2 coordinate = int2(floor(projected));
  if (any(coordinate < int2(0)) || any(coordinate >= int2(image_size))) return false;
  previous_pixel = uint2(coordinate);
  return true;
}
