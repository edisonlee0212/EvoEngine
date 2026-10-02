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
  const uint2 image_size = uint2(width, height);
  if (any(pixel >= image_size)) return;
  const uint index = pixel.y * width + pixel.x;
  RestirPtSpatialShift shift;
  shift.contribution = float3(0.0f);
  shift.weight_sum = 0.0f;
  shift.partner_index = 0xffffffffu;
  shift.status = EE_RESTIR_PT_SHIFT_NO_PARTNER;
  shift.ray_count = 0u;
  shift.reserved = 0u;

  const Camera camera = EE_CAMERA_RESTIR_ACTIVE_CAMERA();
  RestirPtPrimarySurface current_surface = EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(primary_surface_buffer, index);
  RestirPtPrimarySurface reprojected_surface;
  if (current_surface.material_id == EE_RESTIR_PT_INVALID_INSTANCE) {
    EE_RESTIR_PT_STORE_SPATIAL_SHIFT(temporal_forward_buffer, index, shift);
    return;
  }
  if (!EE_RESTIR_PT_PREVIOUS_SURFACE(current_surface, reprojected_surface)) {
    EE_RESTIR_PT_STORE_SPATIAL_SHIFT(temporal_forward_buffer, index, shift);
    return;
  }
  uint2 previous_pixel;
  if (!EE_RESTIR_PT_REPROJECT_SURFACE(current_surface, reprojected_surface.position,
                                    camera, image_size, previous_pixel)) {
    EE_RESTIR_PT_STORE_SPATIAL_SHIFT(temporal_forward_buffer, index, shift);
    return;
  }
  current_surface.screen_motion = float2(pixel) - float2(previous_pixel);
  EE_RESTIR_PT_STORE_PRIMARY_SURFACE(primary_surface_buffer, index, current_surface);
  uint previous_index = previous_pixel.y * width + previous_pixel.x;
  RestirPtPrimarySurface previous_surface =
      EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(previous_surface_buffer, previous_index);
  if (!EE_RESTIR_PT_TEMPORAL_SURFACES_MATCH(reprojected_surface, previous_surface,
                                          camera.inverse_view[3].xyz)) {
    const int2 dual_pixel = int2(round(float2(pixel) - previous_surface.screen_motion));
    if (any(dual_pixel < int2(0)) || any(dual_pixel >= int2(image_size))) {
      shift.status = EE_RESTIR_PT_SHIFT_SURFACE_MISMATCH;
      EE_RESTIR_PT_STORE_SPATIAL_SHIFT(temporal_forward_buffer, index, shift);
      return;
    }
    const uint dual_index = uint(dual_pixel.y) * width + uint(dual_pixel.x);
    const RestirPtPrimarySurface dual_surface =
        EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(previous_surface_buffer, dual_index);
    if (!EE_RESTIR_PT_TEMPORAL_SURFACES_MATCH(reprojected_surface, dual_surface,
                                             camera.inverse_view[3].xyz)) {
      shift.status = EE_RESTIR_PT_SHIFT_SURFACE_MISMATCH;
      EE_RESTIR_PT_STORE_SPATIAL_SHIFT(temporal_forward_buffer, index, shift);
      return;
    }
    previous_index = dual_index;
    previous_surface = dual_surface;
    shift.reserved = 1u;
  }
  shift.partner_index = previous_index;
  const RestirPtPathReservoir history = EE_RESTIR_PT_LOAD(previous_history_buffer, previous_index);
  shift.status = EE_RESTIR_PT_SHIFT_NO_SOURCE;
  if (history.target_density <= 0.0f || history.selected_weight <= 0.0f ||
      history.path_type == EE_RESTIR_PT_PATH_PRIMARY_BACKGROUND) {
    EE_RESTIR_PT_STORE_SPATIAL_SHIFT(temporal_forward_buffer, index, shift);
    return;
  }

  const bool long_suffix = (history.flags & EE_RESTIR_PT_FLAG_LONG_SUFFIX_DESCRIPTOR) != 0u;
  if (camera.restir_spatial_hybrid != 0u &&
      (long_suffix ||
       ((history.flags & EE_RESTIR_PT_FLAG_SECONDARY_EMISSION_RECONNECTION) != 0u &&
        (history.flags & EE_RESTIR_PT_FLAG_SUFFIX_DESCRIPTOR) != 0u)) &&
      history.reconnection_length >= 2u &&
      current_surface.instance_index == previous_surface.instance_index &&
      EE_RESTIR_PT_INSTANCE_UNMOVED(current_surface.instance_index) &&
      EE_RESTIR_PT_INSTANCE_STABLE(history.instance_id) &&
      (long_suffix || EE_RESTIR_PT_INSTANCE_UNMOVED(history.instance_id))) {
    EE_RESTIR_PT_STORE_SPATIAL_SHIFT(temporal_forward_buffer, index, shift);
    return;
  }

  float3 contribution;
  RestirPtPathReservoir replay;
  RestirPtPrimarySurface replay_surface;
  EE_CAMERA_REPLAY_RESTIR_EVENT<CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>,
                                EE_GLTF_COMPILED_FEATURE_MASK, true>(
      pixel, image_size, history, contribution, replay, replay_surface, shift.ray_count);
  shift.status = EE_RESTIR_PT_SHIFT_SURFACE_MISMATCH;
  if (EE_RESTIR_PT_TEMPORAL_SURFACES_MATCH(current_surface, replay_surface,
                                         camera.inverse_view[3].xyz)) {
    shift.contribution = contribution;
    shift.weight_sum = 1.0f;
    shift.status = EE_RESTIR_PT_TARGET_DENSITY(contribution) > 0.0f
                       ? EE_RESTIR_PT_SHIFT_VALID : EE_RESTIR_PT_SHIFT_ZERO_TARGET;
  }
  EE_RESTIR_PT_STORE_SPATIAL_SHIFT(temporal_forward_buffer, index, shift);
}
