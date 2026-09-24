import EvoEngine.CameraRayIntegrator;
import EvoEngine.Random;

[[vk::binding(10, 2)]] RWByteAddressBuffer candidate_buffer;
[[vk::binding(12, 2)]] RWByteAddressBuffer resolved_buffer;
[[vk::binding(13, 2)]] RWByteAddressBuffer shift_buffer;

[shader("compute")][numthreads(8, 8, 1)] void main(uint3 dispatch_thread_id : SV_DispatchThreadID) {
  uint width;
  uint height;
  result_image.GetDimensions(width, height);
  const uint2 pixel = dispatch_thread_id.xy;
  const uint2 image_size = uint2(width, height);
  if (any(pixel >= image_size)) return;
  const uint index = pixel.y * width + pixel.x;
  const RestirPtPathReservoir central = EE_RESTIR_PT_LOAD(candidate_buffer, index);
  const RestirPtSpatialShift outgoing = EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(shift_buffer, index);
  RestirPtPathReservoir resolved = EE_RESTIR_PT_EMPTY_RESERVOIR();
  uint seed = EE_XXHASH32(uint3(pixel, central.sample_seed ^ 0x91e10da5u));
  const uint2 partner = EE_RESTIR_PT_PAIR(pixel, image_size, EE_CAMERA_RESTIR_FRAME_ID());
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
    RestirPtPathReservoir sample = central;
    sample.contribution = central.contribution * central.selected_weight * mis;
    EE_RESTIR_PT_ADD_CANDIDATE(resolved, sample, EE_PCG_RANDOM(seed));
  }
  if (has_partner) {
    const RestirPtPathReservoir neighbor = EE_RESTIR_PT_LOAD(candidate_buffer, partner_index);
    const RestirPtSpatialShift incoming = EE_RESTIR_PT_LOAD_SPATIAL_SHIFT(shift_buffer, partner_index);
    if (neighbor.target_density > 0.0f && neighbor.weight_sum > 0.0f &&
        incoming.partner_index == index &&
        (incoming.status == EE_RESTIR_PT_SHIFT_VALID || incoming.status == EE_RESTIR_PT_SHIFT_ZERO_TARGET)) {
      const float source_pdf = neighbor.target_density;
      const float target_pdf = EE_RESTIR_PT_TARGET_DENSITY(incoming.contribution);
      RestirPtPathReservoir sample = neighbor;
      sample.contribution = incoming.contribution * neighbor.selected_weight *
                            (source_pdf / (source_pdf + target_pdf));
      EE_RESTIR_PT_ADD_CANDIDATE(resolved, sample, EE_PCG_RANDOM(seed));
    }
  }
  EE_RESTIR_PT_FINALIZE_CANDIDATES(resolved);
  EE_RESTIR_PT_STORE(resolved_buffer, index, resolved);
}
