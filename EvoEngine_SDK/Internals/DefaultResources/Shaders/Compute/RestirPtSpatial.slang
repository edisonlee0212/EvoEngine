#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
#ifndef EE_RESTIR_PT_PROFILE_ELIGIBILITY
#  define EE_RESTIR_PT_PROFILE_ELIGIBILITY 0
#endif
import EvoEngine.CameraRayIntegrator;
import EvoEngine.CameraRayTraversalPolicies;

[[vk::binding(10, 2)]] RWByteAddressBuffer candidate_buffer;
[[vk::binding(11, 2)]] RWByteAddressBuffer primary_surface_buffer;
[[vk::binding(13, 2)]] RWByteAddressBuffer shift_buffer;

uint EE_RESTIR_PT_SECONDARY_NEE_ELIGIBILITY(const RestirPtPathReservoir source,
                                             const RestirPtPrimarySurface source_surface,
                                             const RestirPtPrimarySurface target_surface) {
  if (source.path_type != EE_RESTIR_PT_PATH_NEE_EMISSION || source.path_length != 3u ||
      (source.flags & EE_RESTIR_PT_FLAG_DELTA_PREFIX) != 0u) return 0u;
  if (source.instance_id == EE_RESTIR_PT_INVALID_INSTANCE || source.light_pdf <= 0.0f ||
      source.bsdf_pdf <= 0.0f || any(!isfinite(source.reconnection_position)) ||
      any(!isfinite(source.reconnection_normal))) return 1u;
  if (!EE_RESTIR_PT_SURFACES_COMPATIBLE(source_surface, target_surface)) return 2u;
  const float3 source_offset = source.reconnection_position - source_surface.position;
  const float3 target_offset = source.reconnection_position - target_surface.position;
  const float source_distance_squared = dot(source_offset, source_offset);
  const float target_distance_squared = dot(target_offset, target_offset);
  if (source_distance_squared <= 1e-8f || target_distance_squared <= 1e-8f) return 3u;
  const float source_facing = dot(source.reconnection_normal, -source_offset);
  const float target_facing = dot(source.reconnection_normal, -target_offset);
  if (source_facing * target_facing <= 0.0f || abs(source_facing) <= 1e-6f || abs(target_facing) <= 1e-6f) return 4u;
  return 5u;
}

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
  if (source.path_type == EE_RESTIR_PT_PATH_PRIMARY_BACKGROUND) {
    shift.status = EE_RESTIR_PT_SHIFT_SKIPPED_BACKGROUND;
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
  if (EE_RESTIR_PT_PROFILE_ELIGIBILITY != 0) {
    shift.reserved = EE_RESTIR_PT_SECONDARY_NEE_ELIGIBILITY(source, source_surface, target_surface);
  }
  EE_RESTIR_PT_STORE_SPATIAL_SHIFT(shift_buffer, index, shift);
}
