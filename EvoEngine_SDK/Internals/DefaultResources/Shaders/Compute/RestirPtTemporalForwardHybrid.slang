#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
import EvoEngine.RestirPtTemporal;
import EvoEngine.CameraRayIntegrator;
import EvoEngine.CameraRayTraversalPolicies;
import EvoEngine.Instances;
import EvoEngine.RestirPtPreviousSurface;

[[vk::binding(11, 2)]] RWByteAddressBuffer primary_surface_buffer;
[[vk::binding(15, 2)]] RWByteAddressBuffer previous_history_buffer;
[[vk::binding(16, 2)]] RWByteAddressBuffer previous_surface_buffer;
[[vk::binding(17, 2)]] RWByteAddressBuffer temporal_forward_buffer;

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  if (any(pixel >= uint2(width, height))) return;
  const uint index = pixel.y * width + pixel.x;
  RestirPtSpatialShift shift = EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(temporal_forward_buffer, index);
  if (shift.partner_index >= width * height) return;
  const RestirPtPathReservoir history = EE_RESTIR_PT_LOAD(previous_history_buffer, shift.partner_index);
  const bool long_suffix = (history.flags & EE_RESTIR_PT_FLAG_LONG_SUFFIX_DESCRIPTOR) != 0u;
  if (history.target_density <= 0.0f || history.selected_weight <= 0.0f ||
      history.reconnection_length < 2u ||
      (!long_suffix && history.reconnection_length != 2u) ||
      (!long_suffix &&
       (history.flags & (EE_RESTIR_PT_FLAG_SECONDARY_EMISSION_RECONNECTION | EE_RESTIR_PT_FLAG_SUFFIX_DESCRIPTOR)) !=
           (EE_RESTIR_PT_FLAG_SECONDARY_EMISSION_RECONNECTION | EE_RESTIR_PT_FLAG_SUFFIX_DESCRIPTOR))) return;
  const RestirPtPrimarySurface current_surface = EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(primary_surface_buffer, index);
  const RestirPtPrimarySurface previous_surface =
      EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(previous_surface_buffer, shift.partner_index);
  if (current_surface.instance_index != previous_surface.instance_index ||
      !EE_RESTIR_PT_INSTANCE_UNMOVED(current_surface.instance_index) ||
      !EE_RESTIR_PT_INSTANCE_STABLE(history.instance_id) ||
      (!long_suffix && !EE_RESTIR_PT_INSTANCE_UNMOVED(history.instance_id))) return;

  float3 contribution;
  RestirPtPrimarySurface replay_surface;
  float jacobian;
  uint ray_count;
  uint rejection_reason;
  float3 first_bsdf_factor;
  float first_bsdf_pdf;
  float2 first_roughness;
  if (long_suffix) {
    EE_CAMERA_RECONNECT_RESTIR_LONG_SUFFIX_AT_CAMERA<
        CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>, EE_GLTF_COMPILED_FEATURE_MASK>(
        pixel, uint2(width, height), history, previous_surface, false, contribution, replay_surface, jacobian,
        ray_count, rejection_reason);
  } else {
    EE_CAMERA_RECONNECT_RESTIR_SECONDARY_EMISSION_AT_CAMERA<
        CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>, EE_GLTF_COMPILED_FEATURE_MASK, true>(
        pixel, uint2(width, height), history, previous_surface, false, contribution, replay_surface, jacobian,
        ray_count, rejection_reason, first_bsdf_factor, first_bsdf_pdf, first_roughness);
  }
  shift.contribution = float3(0.0f);
  shift.weight_sum = 0.0f;
  shift.ray_count += ray_count;
  shift.reserved |= 0x10000u;
  shift.status = EE_RESTIR_PT_SHIFT_SURFACE_MISMATCH;
  const Camera camera = EE_CAMERA_RESTIR_ACTIVE_CAMERA();
  if (jacobian > 0.0f && EE_RESTIR_PT_TEMPORAL_SURFACES_MATCH(
                             current_surface, replay_surface, camera.inverse_view[3].xyz)) {
    shift.contribution = contribution;
    shift.weight_sum = jacobian;
    shift.status = EE_RESTIR_PT_TARGET_DENSITY(contribution) > 0.0f
                       ? EE_RESTIR_PT_SHIFT_VALID : EE_RESTIR_PT_SHIFT_ZERO_TARGET;
  }
  EE_RESTIR_PT_STORE_SPATIAL_SHIFT(temporal_forward_buffer, index, shift);
}
