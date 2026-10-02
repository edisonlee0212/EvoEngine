import EvoEngine.CameraRayIntegrator;
import EvoEngine.Random;

[[vk::binding(10, 2)]] RWByteAddressBuffer candidate_buffer;
[[vk::binding(11, 2)]] RWByteAddressBuffer primary_surface_buffer;
[[vk::binding(12, 2)]] RWByteAddressBuffer resolved_buffer;
[[vk::binding(13, 2)]] RWByteAddressBuffer shift_buffer;
[[vk::binding(19, 2)]] RWByteAddressBuffer next_history_buffer;
#ifdef EE_ENABLE_NRD_SIGNALS
[[vk::binding(25, 2)]] RWTexture2D<float4> nrd_diffuse_signal_image;
[[vk::binding(26, 2)]] RWTexture2D<float4> nrd_specular_signal_image;
[[vk::binding(27, 2)]] RWTexture2D<float4> nrd_excluded_signal_image;

void EE_RESTIR_PT_STORE_SIGNALS(const uint2 pixel, const float3 diffuse, const float3 specular,
                                const float3 excluded) {
  nrd_diffuse_signal_image[pixel] = float4(diffuse, 0.0f);
  nrd_specular_signal_image[pixel] = float4(specular, 0.0f);
  nrd_excluded_signal_image[pixel] = float4(excluded, 0.0f);
}
#endif

bool EE_RESTIR_PT_ENHANCED_SHIFT_VALID(const RestirPtSpatialShift shift, const uint partner_index) {
  return shift.partner_index == partner_index &&
         (shift.status == EE_RESTIR_PT_SHIFT_VALID || shift.status == EE_RESTIR_PT_SHIFT_ZERO_TARGET);
}

float EE_RESTIR_PT_ENHANCED_SHIFT_JACOBIAN(const RestirPtSpatialShift shift) {
  return (shift.reserved & 0x80000000u) != 0u ? shift.weight_sum : 1.0f;
}

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  const uint2 image_size = uint2(width, height);
  if (any(pixel >= image_size)) return;
  const uint index = pixel.y * width + pixel.x;
  const RestirPtPathReservoir central = EE_RESTIR_PT_LOAD(candidate_buffer, index);
  const RestirPtPrimarySurface surface = EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(primary_surface_buffer, index);
  const uint neighbor_count = EE_CAMERA_RESTIR_NEIGHBOR_COUNT();
  const float3 camera_position = EE_CAMERA_RESTIR_ACTIVE_CAMERA().inverse_view[3].xyz;
  if (surface.material_id == EE_RESTIR_PT_INVALID_INSTANCE) {
    const float3 radiance = EE_RESTIR_PT_RESOLVE_CANDIDATE(central);
    resolved_buffer.Store4(index * EE_RESTIR_PT_RESOLVED_BYTES, asuint(float4(radiance, 0.0f)));
#ifdef EE_ENABLE_NRD_SIGNALS
    EE_RESTIR_PT_STORE_SIGNALS(pixel, float3(0.0f), float3(0.0f), radiance);
#endif
    RestirPtPathReservoir empty = EE_RESTIR_PT_EMPTY_RESERVOIR();
    empty.effective_count = 0.0f;
    EE_RESTIR_PT_STORE(next_history_buffer, index, empty);
    return;
  }

  const float self_count = min(max(central.effective_count, 0.0f), 255.0f);
  if (self_count == 0.0f) {
    EE_RESTIR_PT_STORE(next_history_buffer, index, EE_RESTIR_PT_EMPTY_RESERVOIR());
    resolved_buffer.Store4(index * EE_RESTIR_PT_RESOLVED_BYTES, uint4(0u));
#ifdef EE_ENABLE_NRD_SIGNALS
    EE_RESTIR_PT_STORE_SIGNALS(pixel, float3(0.0f), float3(0.0f), float3(0.0f));
#endif
    return;
  }
  float neighbor_count_sum = 0.0f;
  for (uint pair = 0u; pair < neighbor_count; ++pair) {
    const uint2 partner = EE_RESTIR_PT_PAIR(pixel, image_size, EE_CAMERA_RESTIR_FRAME_ID(), pair);
    if (any(partner >= image_size)) continue;
    const uint partner_index = partner.y * width + partner.x;
    const RestirPtPrimarySurface partner_surface =
        EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(primary_surface_buffer, partner_index);
    if (!EE_RESTIR_PT_ENHANCED_SPATIAL_PAIR_ACCEPTED(surface, partner_surface, camera_position)) continue;
    const RestirPtPathReservoir neighbor = EE_RESTIR_PT_LOAD(candidate_buffer, partner_index);
    neighbor_count_sum += min(max(neighbor.effective_count, 0.0f), 255.0f);
  }
  const float total_count = self_count + neighbor_count_sum;
  float canonical_mis = total_count > 0.0f ? self_count / total_count : 1.0f;
  float3 radiance_sum = float3(0.0f);
#ifdef EE_ENABLE_NRD_SIGNALS
  float3 diffuse = float3(0.0f);
  float3 specular = float3(0.0f);
  float3 excluded = float3(0.0f);
#endif
  float selection_sum = 0.0f;
  RestirPtPathReservoir winner = central;
  float3 winner_contribution = central.contribution;
  bool winner_from_neighbor = false;
  bool winner_hybrid = false;
  float winner_cached_jacobian = 0.0f;
  uint seed = EE_XXHASH32(uint3(pixel, EE_CAMERA_RESTIR_FRAME_ID() ^ 0xa5a5a5a5u));

  for (uint pair = 0u; pair < neighbor_count; ++pair) {
    const uint2 partner = EE_RESTIR_PT_PAIR(pixel, image_size, EE_CAMERA_RESTIR_FRAME_ID(), pair);
    if (any(partner >= image_size)) continue;
    const uint partner_index = partner.y * width + partner.x;
    const RestirPtPrimarySurface partner_surface =
        EE_RESTIR_PT_LOAD_PRIMARY_SURFACE(primary_surface_buffer, partner_index);
    if (!EE_RESTIR_PT_ENHANCED_SPATIAL_PAIR_ACCEPTED(surface, partner_surface, camera_position)) continue;
    const RestirPtPathReservoir neighbor = EE_RESTIR_PT_LOAD(candidate_buffer, partner_index);
    const float neighbor_count_value = min(max(neighbor.effective_count, 0.0f), 255.0f);
    if (neighbor_count_value <= 0.0f || total_count <= 0.0f) continue;

    const RestirPtSpatialShift backward =
        EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(shift_buffer, index * neighbor_count + pair);
    const float backward_pdf = EE_RESTIR_PT_ENHANCED_SHIFT_VALID(backward, partner_index)
                                   ? EE_RESTIR_PT_TARGET_DENSITY(backward.contribution) : 0.0f;
    const float backward_jacobian = EE_RESTIR_PT_ENHANCED_SHIFT_JACOBIAN(backward);
    const float current_denominator = self_count * central.target_density +
                                      neighbor_count_sum * backward_pdf * backward_jacobian;
    if (current_denominator > 0.0f) {
      canonical_mis += neighbor_count_value / total_count *
                       (self_count * central.target_density / current_denominator);
    }

    const RestirPtSpatialShift forward =
        EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(shift_buffer, partner_index * neighbor_count + pair);
    if (!EE_RESTIR_PT_ENHANCED_SHIFT_VALID(forward, index)) continue;
    const float shifted_pdf = EE_RESTIR_PT_TARGET_DENSITY(forward.contribution);
    const float jacobian = EE_RESTIR_PT_ENHANCED_SHIFT_JACOBIAN(forward);
    const float denominator = self_count * shifted_pdf * jacobian +
                              neighbor_count_sum * neighbor.target_density;
    if (denominator <= 0.0f || shifted_pdf <= 0.0f || jacobian <= 0.0f) continue;
    const float mis = neighbor_count_value / total_count *
                      (neighbor_count_sum * neighbor.target_density / denominator);
    const float weight = mis * shifted_pdf * neighbor.selected_weight * jacobian;
    if (!(weight > 0.0f) || !isfinite(weight)) continue;
    selection_sum += weight;
    const float3 contribution = mis * forward.contribution * neighbor.selected_weight * jacobian;
    radiance_sum += contribution;
#ifdef EE_ENABLE_NRD_SIGNALS
    EE_RESTIR_PT_ADD_SIGNAL(neighbor, contribution, diffuse, specular, excluded);
#endif
    if (EE_PCG_RANDOM(seed) * selection_sum < weight) {
      winner = neighbor;
      winner_contribution = forward.contribution;
      winner_from_neighbor = true;
      winner_hybrid = (forward.reserved & 0x80000000u) != 0u;
      winner_cached_jacobian = asfloat(forward.reserved & 0x7fffffffu);
    }
  }

  const float central_weight = canonical_mis * central.target_density * central.selected_weight;
  if (central_weight > 0.0f && isfinite(central_weight)) {
    selection_sum += central_weight;
    const float3 contribution = canonical_mis * central.contribution * central.selected_weight;
    radiance_sum += contribution;
#ifdef EE_ENABLE_NRD_SIGNALS
    EE_RESTIR_PT_ADD_SIGNAL(central, contribution, diffuse, specular, excluded);
#endif
    if (EE_PCG_RANDOM(seed) * selection_sum < central_weight) {
      winner = central;
      winner_contribution = central.contribution;
      winner_from_neighbor = false;
      winner_hybrid = false;
    }
  }
  winner.contribution = winner_contribution;
  winner.target_density = EE_RESTIR_PT_TARGET_DENSITY(winner_contribution);
  winner.selected_weight = winner.target_density > 0.0f ? selection_sum / winner.target_density : 0.0f;
  winner.weight_sum = selection_sum;
  winner.effective_count = min(total_count, 255.0f);
  if (winner_from_neighbor) {
    if ((winner.flags & (EE_RESTIR_PT_FLAG_SUFFIX_DESCRIPTOR | EE_RESTIR_PT_FLAG_LONG_SUFFIX_DESCRIPTOR)) != 0u &&
        winner_hybrid &&
        winner_cached_jacobian > 0.0f &&
        isfinite(winner_cached_jacobian)) {
      winner.jacobian = winner_cached_jacobian;
    } else {
      winner.flags &= ~(EE_RESTIR_PT_FLAG_SECONDARY_EMISSION_RECONNECTION | EE_RESTIR_PT_FLAG_SUFFIX_DESCRIPTOR |
                        EE_RESTIR_PT_FLAG_LONG_SUFFIX_DESCRIPTOR);
    }
  }
  EE_RESTIR_PT_STORE(next_history_buffer, index, winner);
  resolved_buffer.Store4(index * EE_RESTIR_PT_RESOLVED_BYTES, asuint(float4(radiance_sum, 0.0f)));
#ifdef EE_ENABLE_NRD_SIGNALS
  EE_RESTIR_PT_STORE_SIGNALS(pixel, diffuse, specular, excluded);
#endif
}
