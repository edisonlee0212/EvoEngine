import EvoEngine.CameraRayIntegrator;

[[vk::binding(10, 2)]] RWByteAddressBuffer candidate_buffer;
[[vk::binding(12, 2)]] RWByteAddressBuffer resolved_buffer;
[[vk::binding(13, 2)]] RWByteAddressBuffer shift_buffer;
#ifdef EE_ENABLE_NRD_SIGNALS
[[vk::binding(25, 2)]] RWTexture2D<float4> nrd_diffuse_signal_image;
[[vk::binding(26, 2)]] RWTexture2D<float4> nrd_specular_signal_image;
[[vk::binding(27, 2)]] RWTexture2D<float4> nrd_excluded_signal_image;
#endif

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  const uint2 image_size = uint2(width, height);
  if (any(pixel >= image_size)) return;
  const uint index = pixel.y * width + pixel.x;
  const RestirPtPathReservoir central = EE_RESTIR_PT_LOAD(candidate_buffer, index);
  const uint neighbor_count = EE_CAMERA_RESTIR_NEIGHBOR_COUNT();
  float3 combined_radiance = float3(0.0f);
#ifdef EE_ENABLE_NRD_SIGNALS
  float3 diffuse = float3(0.0f);
  float3 specular = float3(0.0f);
  float3 excluded = float3(0.0f);
#endif
  for (uint neighbor_index = 0u; neighbor_index < neighbor_count; ++neighbor_index) {
    const RestirPtSpatialShift outgoing =
        EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(shift_buffer, index * neighbor_count + neighbor_index);
    float3 pair_radiance = float3(0.0f);
    const uint2 partner = EE_RESTIR_PT_PAIR(pixel, image_size, EE_CAMERA_RESTIR_FRAME_ID(), neighbor_index);
    const bool has_partner = all(partner < image_size);
    const uint partner_index = has_partner ? partner.y * width + partner.x : 0xffffffffu;

    if (central.target_density > 0.0f && central.weight_sum > 0.0f) {
      float mis = 1.0f;
      if (has_partner && outgoing.partner_index == partner_index &&
          (outgoing.status == EE_RESTIR_PT_SHIFT_VALID || outgoing.status == EE_RESTIR_PT_SHIFT_ZERO_TARGET)) {
        const float self_pdf = central.target_density;
        const float neighbor_pdf = EE_RESTIR_PT_TARGET_DENSITY(outgoing.contribution);
        mis = self_pdf / (self_pdf + neighbor_pdf);
      }
      const float3 contribution = central.contribution * central.selected_weight * mis;
      pair_radiance += contribution;
#ifdef EE_ENABLE_NRD_SIGNALS
      EE_RESTIR_PT_ADD_SIGNAL(central, contribution / float(neighbor_count), diffuse, specular, excluded);
#endif
    }
    if (has_partner) {
      const RestirPtPathReservoir neighbor = EE_RESTIR_PT_LOAD(candidate_buffer, partner_index);
      const RestirPtSpatialShift incoming =
          EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(shift_buffer, partner_index * neighbor_count + neighbor_index);
      if (neighbor.target_density > 0.0f && neighbor.weight_sum > 0.0f &&
          incoming.partner_index == index &&
          (incoming.status == EE_RESTIR_PT_SHIFT_VALID || incoming.status == EE_RESTIR_PT_SHIFT_ZERO_TARGET)) {
        const float source_pdf = neighbor.target_density;
        const float target_pdf = EE_RESTIR_PT_TARGET_DENSITY(incoming.contribution);
        if (target_pdf > 0.0f) {
          const float3 contribution = incoming.contribution * neighbor.selected_weight *
                                      (source_pdf / (source_pdf + target_pdf));
          pair_radiance += contribution;
#ifdef EE_ENABLE_NRD_SIGNALS
          EE_RESTIR_PT_ADD_SIGNAL(neighbor, contribution / float(neighbor_count), diffuse, specular, excluded);
#endif
        }
      }
    }
    // The RGB sum is the conditional expectation of selecting one pair sample by luminance.
    combined_radiance += pair_radiance / float(neighbor_count);
  }
  resolved_buffer.Store4(index * EE_RESTIR_PT_RESOLVED_BYTES, asuint(float4(combined_radiance, 0.0f)));
#ifdef EE_ENABLE_NRD_SIGNALS
  nrd_diffuse_signal_image[pixel] = float4(diffuse, 0.0f);
  nrd_specular_signal_image[pixel] = float4(specular, 0.0f);
  nrd_excluded_signal_image[pixel] = float4(excluded, 0.0f);
#endif
}
