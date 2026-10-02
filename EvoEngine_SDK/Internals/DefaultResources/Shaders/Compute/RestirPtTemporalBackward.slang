#ifndef EE_GLTF_COMPILED_FEATURE_MASK
#  define EE_GLTF_COMPILED_FEATURE_MASK 32767u
#endif
import EvoEngine.RestirPtTemporal;
import EvoEngine.CameraRayIntegrator;
import EvoEngine.CameraRayTraversalPolicies;
import EvoEngine.Random;

[[vk::binding(10, 2)]] RWByteAddressBuffer candidate_buffer;
[[vk::binding(13, 2)]] RWByteAddressBuffer shift_buffer;
[[vk::binding(15, 2)]] RWByteAddressBuffer previous_history_buffer;
[[vk::binding(16, 2)]] RWByteAddressBuffer previous_surface_buffer;
[[vk::binding(17, 2)]] RWByteAddressBuffer temporal_forward_buffer;
[[vk::binding(18, 2)]] RWByteAddressBuffer duplication_buffer;

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  const uint2 image_size = uint2(width, height);
  if (any(pixel >= image_size)) return;
  const uint index = pixel.y * width + pixel.x;
  const RestirPtSpatialShift forward = EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(temporal_forward_buffer, index);
  if (forward.partner_index == 0xffffffffu) return;
  const uint previous_index = forward.partner_index;
  const uint2 previous_pixel = uint2(previous_index % width, previous_index / width);
  const RestirPtPathReservoir history = EE_RESTIR_PT_LOAD(previous_history_buffer, previous_index);
  RestirPtPathReservoir current = EE_RESTIR_PT_LOAD(candidate_buffer, index);
  const Camera camera = EE_CAMERA_RESTIR_ACTIVE_CAMERA();
  float count_cap = float(camera.restir_temporal_history_cap);
  if (camera.restir_temporal_adaptive_cap != 0u) {
    const float duplication = float(duplication_buffer.Load(previous_index * 4u)) / 255.0f;
    count_cap = lerp(count_cap, 1.0f, pow(duplication, 0.1f));
  }
  const float history_count = history.target_density > 0.0f && history.selected_weight > 0.0f
                                  ? min(count_cap, max(history.effective_count, 0.0f)) : 0.0f;
  const float current_count = max(current.effective_count, 0.0f);
  const float history_pdf = history.target_density;
  const float current_pdf = current.target_density;
  const float forward_pdf = (forward.status == EE_RESTIR_PT_SHIFT_VALID ||
                             forward.status == EE_RESTIR_PT_SHIFT_ZERO_TARGET)
                                ? EE_RESTIR_PT_TARGET_DENSITY(forward.contribution) : 0.0f;
  const float forward_jacobian = (forward.reserved & 0x10000u) != 0u ? forward.weight_sum : 1.0f;

  float backward_pdf = 0.0f;
  float backward_jacobian = 1.0f;
  if (history_count > 0.0f && current_pdf > 0.0f &&
      current.path_type != EE_RESTIR_PT_PATH_PRIMARY_BACKGROUND) {
    bool hybrid_shift = false;
    if (camera.restir_spatial_hybrid != 0u) {
      const RestirPtSpatialShift hybrid = EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(shift_buffer, index);
      hybrid_shift = (hybrid.reserved & 0x10000u) != 0u;
      if (hybrid_shift && (hybrid.status == EE_RESTIR_PT_SHIFT_VALID ||
                           hybrid.status == EE_RESTIR_PT_SHIFT_ZERO_TARGET)) {
        backward_pdf = EE_RESTIR_PT_TARGET_DENSITY(hybrid.contribution);
        backward_jacobian = hybrid.weight_sum;
      }
    }
    if (!hybrid_shift) {
      float3 backward_contribution;
      RestirPtPathReservoir replay;
      RestirPtPrimarySurface replay_surface;
      uint ray_count;
      EE_CAMERA_REPLAY_RESTIR_EVENT_AT_CAMERA<CameraRayQueryTraversalPolicy<EE_GLTF_COMPILED_FEATURE_MASK>,
                                              EE_GLTF_COMPILED_FEATURE_MASK, true>(
          previous_pixel, image_size, current, true, backward_contribution, replay, replay_surface, ray_count);
      const RestirPtPrimarySurface previous_surface =
          EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(previous_surface_buffer, previous_index);
      if (EE_RESTIR_PT_TEMPORAL_SURFACES_MATCH(previous_surface, replay_surface,
                                             camera.previous_inverse_view[3].xyz)) {
        backward_pdf = EE_RESTIR_PT_TARGET_DENSITY(backward_contribution);
      }
    }
  }

  float history_weight = 0.0f;
  if (forward_pdf > 0.0f && history_pdf > 0.0f) {
    const float denominator = current_count * forward_pdf * forward_jacobian + history_count * history_pdf;
    if (denominator > 0.0f) {
      history_weight = history_count * history_pdf / denominator *
                       forward_pdf * history.selected_weight * forward_jacobian;
    }
  }
  float current_mis = 1.0f;
  if (backward_pdf > 0.0f) {
    const float denominator = current_count * current_pdf + history_count * backward_pdf * backward_jacobian;
    if (denominator > 0.0f) current_mis = current_count * current_pdf / denominator;
  }
  const float current_weight = current_mis * current_pdf * current.selected_weight;
  if (!isfinite(history_weight) || !isfinite(current_weight)) return;
  const float weight_sum = history_weight + current_weight;
  uint seed = EE_XXHASH32(uint3(pixel, EE_CAMERA_RESTIR_FRAME_ID() ^ 0x95f24d7bu));
  const bool history_won = history_weight > 0.0f && EE_PCG_RANDOM(seed) * weight_sum < history_weight;
  if (history_won) {
    current = history;
    current.contribution = forward.contribution;
    current.target_density = forward_pdf;
    if ((current.flags & (EE_RESTIR_PT_FLAG_SUFFIX_DESCRIPTOR | EE_RESTIR_PT_FLAG_LONG_SUFFIX_DESCRIPTOR)) != 0u &&
        (forward.reserved & 0x10000u) != 0u && forward_jacobian > 0.0f &&
        isfinite(current.jacobian * forward_jacobian)) {
      current.jacobian *= forward_jacobian;
    } else {
      current.flags &= ~(EE_RESTIR_PT_FLAG_SECONDARY_EMISSION_RECONNECTION | EE_RESTIR_PT_FLAG_SUFFIX_DESCRIPTOR |
                         EE_RESTIR_PT_FLAG_LONG_SUFFIX_DESCRIPTOR);
    }
  } else if (current_count <= 0.0f || weight_sum <= 0.0f) {
    current = EE_RESTIR_PT_EMPTY_RESERVOIR();
    current.effective_count = 1.0f;
    EE_RESTIR_PT_STORE(candidate_buffer, index, current);
    return;
  }
  current.selected_weight = current.target_density > 0.0f ? weight_sum / current.target_density : 0.0f;
  current.weight_sum = weight_sum;
  current.effective_count = min(current_count + floor(history_count), 255.0f);
  EE_RESTIR_PT_STORE(candidate_buffer, index, current);
}
