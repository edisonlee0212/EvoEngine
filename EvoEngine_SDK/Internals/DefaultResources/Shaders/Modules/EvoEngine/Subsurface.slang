public float EE_SSS_SHADOW_FILTER_WEIGHT(float2 delta, float depth) {
  // Empty depth must not brighten a blocked receiver at a silhouette.
  if (depth >= 1.0f) return 0.0f;
  const float2 tent = max(float2(1.5f) - abs(delta), float2(0));
  return tent.x * tent.y;
}

public float3 EE_SSS_DIFFUSION_ADJUSTMENT(float3 unfiltered, float3 filtered, float3 albedo, float weight) {
  return weight * (filtered * saturate(albedo) - unfiltered);
}

public bool EE_SSS_BACKFACE_MATCH(uint front_instance, uint back_instance, float front_depth, float back_depth,
                                  bool front_facing) {
  return !front_facing && front_instance == back_instance && front_instance != 0xffffffffu &&
         isfinite(front_depth) && isfinite(back_depth) && front_depth >= 0 && front_depth < back_depth && back_depth < 1;
}

public float EE_SSS_BACKFACE_THICKNESS(float3 front_position, float3 back_position, float3 front_normal) {
  return max(dot(front_position - back_position, front_normal), 0);
}

public float3 EE_SSS_AREA_PROFILE(float radius, float3 distance) {
  distance = max(distance, float3(1e-6f));
  return (exp(-radius / distance) + exp(-radius / (3.0f * distance))) /
         (8.0f * 3.14159265359f * distance * max(radius, 1e-8f));
}

public float EE_SSS_PROJECTED_DENSITY(float3 projected_delta, float3 projected_normal, float3 distance) {
  const float3 radii = float3(length(projected_delta.yz), length(projected_delta.xz), length(projected_delta.xy));
  const float3 axis_probability = float3(0.5f, 0.25f, 0.25f);
  float density = 0.0f;
  for (uint axis = 0u; axis < 3u; ++axis) {
    const float3 channel_density = EE_SSS_AREA_PROFILE(radii[axis], distance);
    density += (channel_density.r + channel_density.g + channel_density.b) / 3.0f *
               abs(projected_normal[axis]) * axis_probability[axis];
  }
  return density;
}

public float EE_SSS_SAMPLE_RADIUS(float u, float distance) {
  u = clamp(u, 0.0f, 1.0f - 1e-7f);
  const float c = 1.0f - u;
  const float a = log(2.0f * c + sqrt(4.0f * c * c + 1.0f)) / 3.0f;
  return -3.0f * distance * log(clamp(exp(a) - exp(-a), 1e-7f, 1.0f));
}

public float EE_SSS_COMPONENT_DENSITY(float3 projected_delta, float3 projected_normal,
    float3 distance, uint axis, uint channel) {
  if (length(projected_delta) >= EE_SSS_SAMPLE_RADIUS(0.999f, distance[channel])) return 0.0f;
  const float3 radii = float3(length(projected_delta.yz), length(projected_delta.xz), length(projected_delta.xy));
  return EE_SSS_AREA_PROFILE(radii[axis], distance)[channel] * abs(projected_normal[axis]) *
      (axis == 0u ? 0.5f : 0.25f) / 3.0f;
}

public float3 EE_SSS_THIN_TRANSMISSION(float thickness, float3 distance) {
  return exp(-max(thickness, 0.0f) / max(distance, float3(1e-6f)));
}

public float3 EE_SSS_SOLID_TRANSMISSION(float authored_thickness, float shadow_thickness, float3 distance) {
  const float thickness = max(authored_thickness, 0.0f);
  distance = max(distance, float3(1e-6f));
  const float3 tail = exp(-thickness / (3.0f * distance));
  return 0.25f * tail * (tail * tail + 3.0f) *
      exp(-max(shadow_thickness - thickness, 0.0f) / (3.0f * distance));
}

public float EE_SSS_SHADOW_THICKNESS(float4x4 light_matrix, float receiver_clip_w,
    float receiver_distance, float shadow_depth) {
  if (!(receiver_clip_w > 0.0f)) return 0.0f;
  const float3 w_axis = float3(light_matrix[0][3], light_matrix[1][3], light_matrix[2][3]);
  const float3 z_axis = float3(light_matrix[0][2], light_matrix[1][2], light_matrix[2][2]);
  // Perspective clip_z = A * clip_w + B, including the light's view transform.
  const float a = dot(z_axis, w_axis) / dot(w_axis, w_axis);
  const float b = light_matrix[3][2] - a * light_matrix[3][3];
  const float shadow_w = b / (shadow_depth - a);
  const float shadow_distance = receiver_distance * shadow_w / receiver_clip_w;
  return isfinite(shadow_distance) && shadow_distance > 0.0f
      ? max(receiver_distance - shadow_distance, 0.0f) : 0.0f;
}

public struct SubsurfaceDirectReservoir {
  public uint candidate_index;
  public float target_density;
  public float weight_sum;
};

public SubsurfaceDirectReservoir EE_SSS_DIRECT_EMPTY_RESERVOIR() {
  SubsurfaceDirectReservoir reservoir;
  reservoir.candidate_index = 0xffffffffu;
  reservoir.target_density = 0.0f;
  reservoir.weight_sum = 0.0f;
  return reservoir;
}

public void EE_SSS_DIRECT_UPDATE(inout SubsurfaceDirectReservoir reservoir, uint index,
                                 float3 contribution, float random_value) {
  const float target = dot(contribution, float3(0.2126f, 0.7152f, 0.0722f));
  if (!all(isfinite(contribution)) || target <= 0.0f || !isfinite(target)) return;
  reservoir.weight_sum += target;
  if (reservoir.candidate_index == 0xffffffffu || random_value < target / reservoir.weight_sum) {
    reservoir.candidate_index = index;
    reservoir.target_density = target;
  }
}

public float EE_SSS_DIRECT_RESOLVE_WEIGHT(SubsurfaceDirectReservoir reservoir) {
  // Candidates are complementary MIS-weighted estimates; their expectation is summed, not averaged.
  return reservoir.target_density > 0.0f ? reservoir.weight_sum / reservoir.target_density : 0.0f;
}
