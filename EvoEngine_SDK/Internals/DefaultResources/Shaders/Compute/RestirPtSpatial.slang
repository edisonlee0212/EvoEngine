#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
import EvoEngine.CameraRayIntegrator;
import EvoEngine.CameraRayTraversalPolicies;

[[vk::binding(10, 2)]] RWByteAddressBuffer candidate_buffer;
[[vk::binding(11, 2)]] RWByteAddressBuffer primary_surface_buffer;
[[vk::binding(13, 2)]] RWByteAddressBuffer shift_buffer;

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  const uint2 image_size = uint2(width, height);
  if (any(pixel >= image_size)) return;
  const uint index = pixel.y * width + pixel.x;
  RestirPtSpatialShift shift;
  shift.contribution = 0.0f;
  shift.weight_sum = 0.0f;
  shift.partner_index = 0xffffffffu;
  shift.status = EE_RESTIR_PT_SHIFT_NO_PARTNER;
  shift.ray_count = 0u;
  shift.reserved = 0u;
  const uint2 partner = EE_RESTIR_PT_PAIR(pixel, image_size, EE_CAMERA_RESTIR_FRAME_ID());
  if (any(partner >= image_size)) {
    EE_RESTIR_PT_STORE_SPATIAL_SHIFT(shift_buffer, index, shift);
    return;
  }
  shift.partner_index = partner.y * width + partner.x;
  const RestirPtPathReservoir source = EE_RESTIR_PT_LOAD(candidate_buffer, index);
  shift.status = EE_RESTIR_PT_SHIFT_NO_SOURCE;
  if (source.target_density <= 0.0f || source.weight_sum <= 0.0f) {
    EE_RESTIR_PT_STORE_SPATIAL_SHIFT(shift_buffer, index, shift);
    return;
  }
  float3 contribution;
  RestirPtPathReservoir replay;
  RestirPtPrimarySurface target_surface;
  EE_CAMERA_REPLAY_RESTIR_EVENT<CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>,
                                EE_GLTF_COMPILED_FEATURE_MASK>(
      partner, image_size, source, contribution, replay, target_surface, shift.ray_count);
  const RestirPtPrimarySurface source_surface = EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(primary_surface_buffer, index);
  shift.status = EE_RESTIR_PT_SHIFT_SURFACE_MISMATCH;
  if (EE_RESTIR_PT_SURFACES_COMPATIBLE(source_surface, target_surface)) {
    shift.contribution = contribution;
    shift.weight_sum = replay.weight_sum;
    shift.status = EE_RESTIR_PT_TARGET_DENSITY(contribution) > 0.0f
                       ? EE_RESTIR_PT_SHIFT_VALID : EE_RESTIR_PT_SHIFT_ZERO_TARGET;
  }
  EE_RESTIR_PT_STORE_SPATIAL_SHIFT(shift_buffer, index, shift);
}
