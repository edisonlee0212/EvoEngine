// @evoengine-dialect native
import EvoEngine.BasicConstants;
public static const uint EE_MESHLET_CULL_FRUSTUM = 1u;
public static const uint EE_MESHLET_CULL_CONE_REQUESTED = 2u;
public static const uint EE_MESHLET_CULL_CONE_BACK = 4u;
public static const uint EE_MESHLET_CULL_CONE_FRONT = 8u;
public static const uint EE_MESHLET_VIEW_DIRECTIONAL = 256u;
public static const uint EE_MESHLET_VIEW_POINT = 512u;
public static const uint EE_MESHLET_VIEW_SPOT = 768u;

public bool EE_MESHLET_SPHERE_INTERSECTS_CLIP(float4 local_sphere, float4x4 model, float4x4 projection_view,
                                               bool zero_near_plane) {
  if (local_sphere.w < 0.0f || !all(isfinite(local_sphere))) return true;
  float3 world_center = mul(float4(local_sphere.xyz, 1.0f), model).xyz;
  float3 basis_x = mul(float4(1.0f, 0.0f, 0.0f, 0.0f), model).xyz;
  float3 basis_y = mul(float4(0.0f, 1.0f, 0.0f, 0.0f), model).xyz;
  float3 basis_z = mul(float4(0.0f, 0.0f, 1.0f, 0.0f), model).xyz;
  float3 basis_lengths = float3(length(basis_x), length(basis_y), length(basis_z));
  bool orthogonal = abs(dot(basis_x, basis_y)) <= 1e-5f * basis_lengths.x * basis_lengths.y &&
                    abs(dot(basis_x, basis_z)) <= 1e-5f * basis_lengths.x * basis_lengths.z &&
                    abs(dot(basis_y, basis_z)) <= 1e-5f * basis_lengths.y * basis_lengths.z;
  float world_scale = orthogonal ? max(basis_lengths.x, max(basis_lengths.y, basis_lengths.z))
                                 : length(basis_lengths);
  float world_radius = local_sphere.w * world_scale;
  if (!all(isfinite(world_center)) || !isfinite(world_radius)) return true;
  uint outside[6] = {0u, 0u, 0u, 0u, 0u, 0u};
  for (uint corner = 0u; corner < 8u; ++corner) {
    float3 sign = float3((corner & 1u) != 0u ? 1.0f : -1.0f, (corner & 2u) != 0u ? 1.0f : -1.0f,
                         (corner & 4u) != 0u ? 1.0f : -1.0f);
    float4 clip = mul(float4(world_center + sign * world_radius, 1.0f), projection_view);
    if (!all(isfinite(clip))) return true;
    outside[0] += clip.x < -clip.w ? 1u : 0u;
    outside[1] += clip.x > clip.w ? 1u : 0u;
    outside[2] += clip.y < -clip.w ? 1u : 0u;
    outside[3] += clip.y > clip.w ? 1u : 0u;
    outside[4] += clip.z < (zero_near_plane ? 0.0f : -clip.w) ? 1u : 0u;
    outside[5] += clip.z > clip.w ? 1u : 0u;
  }
  for (uint plane = 0u; plane < 6u; ++plane) {
    if (outside[plane] == 8u) return false;
  }
  return true;
}

public bool EE_MESHLET_FRUSTUM_VISIBLE(float4 local_sphere, float4x4 model, float4x4 projection_view,
                                        bool zero_near_plane) {
  return EE_MESHLET_SPHERE_INTERSECTS_CLIP(local_sphere, model, projection_view, zero_near_plane);
}

public bool EE_MESHLET_CONE_VISIBLE(float4 local_cone, float4 local_apex, float4x4 model,
                                    float3 observer_or_direction, bool directional_view) {
  uint flags = EE_BASIC_CONSTANTS.meshlet_culling_flags;
  if ((flags & EE_MESHLET_CULL_CONE_REQUESTED) == 0u) return true;
  if ((flags & EE_MESHLET_CULL_CONE_BACK) == 0u) return true;
  if (!all(isfinite(local_cone)) || !all(isfinite(local_apex)) || local_cone.w >= 1.0f) {
    return true;
  }
  float3x3 linear = float3x3(model);
  float3 lengths = float3(length(linear[0]), length(linear[1]), length(linear[2]));
  float maximum_length = max(lengths.x, max(lengths.y, lengths.z));
  float minimum_length = min(lengths.x, min(lengths.y, lengths.z));
  bool uniform_orthogonal = minimum_length > 1e-6f && maximum_length <= minimum_length * 1.0001f &&
                            abs(dot(linear[0], linear[1])) <= 1e-5f * lengths.x * lengths.y &&
                            abs(dot(linear[0], linear[2])) <= 1e-5f * lengths.x * lengths.z &&
                            abs(dot(linear[1], linear[2])) <= 1e-5f * lengths.y * lengths.z &&
                            determinant(linear) > 0.0f;
  if (!uniform_orthogonal) {
    return true;
  }
  float3 world_axis = normalize(cross(linear[1], linear[2]) * local_cone.x +
                                cross(linear[2], linear[0]) * local_cone.y +
                                cross(linear[0], linear[1]) * local_cone.z);
  float3 world_apex = mul(float4(local_apex.xyz, 1.0f), model).xyz;
  float3 to_apex = directional_view ? normalize(observer_or_direction)
                                    : normalize(world_apex - observer_or_direction);
  if (!all(isfinite(world_axis)) || !all(isfinite(to_apex))) {
    return true;
  }
  return dot(to_apex, world_axis) < local_cone.w;
}
