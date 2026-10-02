#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
import EvoEngine.CameraRayIntegrator;
import EvoEngine.CameraRayTraversalPolicies;

[[vk::binding(10, 2)]] RWByteAddressBuffer candidate_buffer;
[[vk::binding(11, 2)]] RWByteAddressBuffer primary_surface_buffer;
[[vk::binding(13, 2)]] RWByteAddressBuffer shift_buffer;

[shader("compute")][numthreads(8, 8, 4)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  const uint2 image_size = uint2(width, height);
  const uint pair = dispatch_thread_id.z;
  const uint neighbor_count = EE_CAMERA_RESTIR_NEIGHBOR_COUNT();
  if (any(pixel >= image_size) || pair >= neighbor_count) return;
  const uint index = pixel.y * width + pixel.x;
  const RestirPtPathReservoir source = EE_RESTIR_PT_LOAD(candidate_buffer, index);
  if (source.reconnection_length < 2u) return;
  const bool long_suffix = (source.flags & EE_RESTIR_PT_FLAG_LONG_SUFFIX_DESCRIPTOR) != 0u;
  if (!long_suffix && source.reconnection_length != 2u) return;
  if (!long_suffix &&
      (source.flags & (EE_RESTIR_PT_FLAG_SECONDARY_EMISSION_RECONNECTION | EE_RESTIR_PT_FLAG_SUFFIX_DESCRIPTOR)) !=
          (EE_RESTIR_PT_FLAG_SECONDARY_EMISSION_RECONNECTION | EE_RESTIR_PT_FLAG_SUFFIX_DESCRIPTOR)) return;
  const RestirPtPrimarySurface source_surface = EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(primary_surface_buffer, index);
  const uint2 partner = EE_RESTIR_PT_PAIR(pixel, image_size, EE_CAMERA_RESTIR_FRAME_ID(), pair);
  if (any(partner >= image_size)) return;
  const RestirPtPrimarySurface partner_surface =
      EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(primary_surface_buffer, partner.y * width + partner.x);
  if (!EE_RESTIR_PT_ENHANCED_SPATIAL_PAIR_ACCEPTED(
          source_surface, partner_surface, EE_CAMERA_RESTIR_ACTIVE_CAMERA().inverse_view[3].xyz)) return;
  const uint shift_index = index * neighbor_count + pair;
  RestirPtSpatialShift shift = EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(shift_buffer, shift_index);
  float3 contribution;
  RestirPtPrimarySurface target_surface;
  float jacobian;
  uint rejection_reason;
  float3 first_bsdf_factor;
  float first_bsdf_pdf;
  float2 first_roughness;
  if (long_suffix) {
    EE_CAMERA_RECONNECT_RESTIR_LONG_SUFFIX_AT_CAMERA<
        CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>, EE_GLTF_COMPILED_FEATURE_MASK>(
        partner, image_size, source, source_surface, false, contribution, target_surface, jacobian,
        shift.ray_count, rejection_reason);
  } else {
    EE_CAMERA_RECONNECT_RESTIR_SECONDARY_EMISSION_AT_CAMERA<
        CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>, EE_GLTF_COMPILED_FEATURE_MASK, true>(
        partner, image_size, source, source_surface, false, contribution, target_surface, jacobian,
        shift.ray_count, rejection_reason, first_bsdf_factor, first_bsdf_pdf, first_roughness);
  }
  shift.status = EE_RESTIR_PT_SHIFT_SURFACE_MISMATCH;
  const float new_cached_jacobian = source.jacobian * jacobian;
  if (EE_RESTIR_PT_SURFACES_COMPATIBLE(source_surface, target_surface) && jacobian > 0.0f &&
      new_cached_jacobian > 0.0f && isfinite(new_cached_jacobian)) {
    shift.contribution = contribution;
    shift.weight_sum = jacobian;
    shift.reserved = asuint(new_cached_jacobian) | 0x80000000u;
    shift.status = EE_RESTIR_PT_TARGET_DENSITY(contribution) > 0.0f
                       ? EE_RESTIR_PT_SHIFT_VALID : EE_RESTIR_PT_SHIFT_ZERO_TARGET;
  }
  EE_RESTIR_PT_STORE_SPATIAL_SHIFT(shift_buffer, shift_index, shift);
}
