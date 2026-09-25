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
  const uint neighbor_count = EE_CAMERA_RESTIR_NEIGHBOR_COUNT();
  for (uint neighbor_index = 0u; neighbor_index < neighbor_count; ++neighbor_index) {
    const RestirPtPathReservoir source = EE_RESTIR_PT_LOAD(candidate_buffer, index);
    if ((source.flags & EE_RESTIR_PT_FLAG_SECONDARY_EMISSION_RECONNECTION) == 0u) continue;
    const uint shift_index = index * neighbor_count + neighbor_index;
    const uint2 partner = EE_RESTIR_PT_PAIR(pixel, image_size, EE_CAMERA_RESTIR_FRAME_ID(), neighbor_index);
    if (any(partner >= image_size)) continue;
    RestirPtSpatialShift shift = EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(shift_buffer, shift_index);
    shift.ray_count = 0u;
    float3 contribution;
    RestirPtPrimarySurface target_surface;
    const RestirPtPrimarySurface source_surface = EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(primary_surface_buffer, index);
    float jacobian = 1.0f;
    uint rejection_reason;
    EE_CAMERA_RECONNECT_RESTIR_SECONDARY_EMISSION<CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>,
                                                   EE_GLTF_COMPILED_FEATURE_MASK>(
        partner, image_size, source, source_surface, contribution, target_surface, jacobian, shift.ray_count,
        rejection_reason);
    shift.reserved = (shift.reserved & 0xffu) | 0x10000u | (rejection_reason << 8u);
    shift.status = EE_RESTIR_PT_SHIFT_SURFACE_MISMATCH;
    if (EE_RESTIR_PT_SURFACES_COMPATIBLE(source_surface, target_surface)) {
      shift.contribution = contribution * jacobian;
      shift.weight_sum = jacobian;
      shift.status = EE_RESTIR_PT_TARGET_DENSITY(contribution) > 0.0f
                         ? EE_RESTIR_PT_SHIFT_VALID : EE_RESTIR_PT_SHIFT_ZERO_TARGET;
    }
    EE_RESTIR_PT_STORE_SPATIAL_SHIFT(shift_buffer, shift_index, shift);
  }
}
