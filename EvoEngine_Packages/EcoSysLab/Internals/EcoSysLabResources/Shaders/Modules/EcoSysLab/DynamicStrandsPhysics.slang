import EcoSysLab.DynamicStrandsSet0;
import EcoSysLab.DynamicStrands;
import EcoSysLab.Math;

// Constraint Solvers Decl
public void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle);
public void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle, out float3 x0_correction,
                                      out float3 x1_correction, out float4 q_correction);
public void project_shear_stretch_constraint(in float inv_time_step, in float3 p0, in float3 p1, in float4 q, in float inv_mass_p0,
                                      in float inv_mass_p1, in float inv_mass_q, in float3 alpha, in float rest_length,
                                      out float3 x0_correction, out float3 x1_correction, out float4 q_correction);

public void project_shear_stretch_constraint_pre_strain(in float inv_time_step, in float bd, in float HC, in float3 p0,
                                                 in float3 p1, in float4 q, in float inv_mass_p0, in float inv_mass_p1,
                                                 in float inv_mass_q, in float3 alpha, in float rest_length,
                                                 out float3 x0_correction, out float3 x1_correction, out float4 q_correction);

public void project_bend_twist_constraint(in float inv_time_step, in int segment_pair_handle);
public void project_bend_twist_constraint(in float inv_time_step, in int segment_pair_handle, out float4 q0_correction,
                                   out float4 q1_correction);
public void project_bend_twist_constraint(in float inv_time_step, in float4 q0, in float inv_mass_q0, in float4 q1,
                                   in float inv_mass_q1, in float3 alpha, in float4 rest_darboux_vector,
                                   out float4 q0_correction, out float4 q1_correction);

public void BundleSegmentPosition(in uint segment_handle, in float inv_time_step, in float over_relaxation, in float crack_bd_shrinkage_offset, in float crack_R_scale, in float crack_T_scale);
public void BundleSegmentRotation(in uint segment_handle, in float inv_time_step, in float over_relaxation);
public void BundleSegmentBendTwist(in uint segment_handle, in float inv_time_step, in float over_relaxation);
public void BundleSegmentShearStretch(in uint segment_handle, in float inv_time_step);

public float AdaptAlphaToLigninHealth(in float lignin_health, in float alpha) {
  return alpha / clamp(2.0 * lignin_health - 1.0, 0.001, 1.0);
}

// Constraint Solvers Impl
public void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle) {
  float3 x0_correction, x1_correction;
  float4 q_correction;

  float3 p0 = segments[segment_handle].particle0.x;
  float3 p1 = segments[segment_handle].particle1.x;

  float bd = segments[segment_handle].boundary_distance;
  float HC = segments[segment_handle].HC;

  int prev_handle = segments[segment_handle].prev_handle;
  int next_handle = segments[segment_handle].next_handle;

  float inv_mass_q = segments[segment_handle].inv_mass;
  float inv_mass_p0 = prev_handle != -1 ? segments[prev_handle].inv_mass : inv_mass_q;
  float inv_mass_p1 = next_handle != -1 ? segments[next_handle].inv_mass : inv_mass_q;

  float4 q = segments[segment_handle].q;
  float3 alpha = float3(segments[segment_handle].shear_stretch_alpha);
  float rest_length = segments[segment_handle].rest_length;

  project_shear_stretch_constraint_pre_strain(inv_time_step, bd, HC, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q,
                                              alpha, rest_length, x0_correction, x1_correction, q_correction);

  float3 particle0_new_position = p0 + x0_correction;
  float3 particle1_new_position = p1 + x1_correction;

  if (prev_handle != -1 &&
      segment_pairs[segment_data_list[segment_handle].pair_handles[0]].connectivity_integrity > 0.f) {
    float neighbor_inv_mass = segments[prev_handle].inv_mass;
    particle0_new_position =
        (particle0_new_position * neighbor_inv_mass + segments[prev_handle].particle1.x * inv_mass_q) /
        (inv_mass_q + neighbor_inv_mass);
    segments[prev_handle].particle1.x = particle0_new_position;
  }
  if (next_handle != -1 &&
      segment_pairs[segment_data_list[segment_handle].pair_handles[1]].connectivity_integrity > 0.f) {
    float neighbor_inv_mass = segments[next_handle].inv_mass;
    particle1_new_position =
        (particle1_new_position * neighbor_inv_mass + segments[next_handle].particle0.x * inv_mass_q) /
        (inv_mass_q + neighbor_inv_mass);
    segments[next_handle].particle0.x = particle1_new_position;
  }

  segments[segment_handle].particle0.x = particle0_new_position;
  segments[segment_handle].particle1.x = particle1_new_position;
  segments[segment_handle].q = normalize(q_correction + q);
}

public void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle, out float3 x0_correction,
                                      out float3 x1_correction, out float4 q_correction) {
  float3 p0 = segments[segment_handle].particle0.x;
  float3 p1 = segments[segment_handle].particle1.x;

  float bd = segments[segment_handle].boundary_distance;
  float HC = segments[segment_handle].HC;

  int prev_handle = segments[segment_handle].prev_handle;
  int next_handle = segments[segment_handle].next_handle;

  float inv_mass_q = segments[segment_handle].inv_mass;
  float inv_mass_p0 = prev_handle != -1 ? segments[prev_handle].inv_mass : inv_mass_q;
  float inv_mass_p1 = next_handle != -1 ? segments[next_handle].inv_mass : inv_mass_q;

  float4 q = segments[segment_handle].q;
  float3 alpha = float3(segments[segment_handle].shear_stretch_alpha);
  float rest_length = segments[segment_handle].rest_length;

  project_shear_stretch_constraint_pre_strain(inv_time_step, bd, HC, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q,
                                              alpha, rest_length, x0_correction, x1_correction, q_correction);
}

public void project_shear_stretch_constraint(in float inv_time_step, in float3 p0, in float3 p1, in float4 q, in float inv_mass_p0,
                                      in float inv_mass_p1, in float inv_mass_q, in float3 alpha, in float rest_length,
                                      out float3 x0_correction, out float3 x1_correction, out float4 q_correction) {
  float3 d3;
  d3[0] = -2.0f * (q.x * q.z + q.w * q.y);
  d3[1] = -2.0f * (q.y * q.z - q.w * q.x);
  d3[2] = -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z;

  // rest_length = rest_length * 0.5f;
  float3 lambda = p1 - p0 - d3 * rest_length;
  float3x3 r = mat3_cast(q);
  lambda = mul(transpose(r), lambda);
  float factor = max(1e-9f, inv_mass_p0 + inv_mass_p1 + 4.0f * inv_mass_q * rest_length * rest_length);

  float t2 = inv_time_step * inv_time_step;
  float3 alpha_factor = float3(t2 * alpha.x, t2 * alpha.y, t2 * alpha.z);
  lambda.x /= factor + alpha_factor.x;
  lambda.y /= factor + alpha_factor.y;
  lambda.z /= factor + alpha_factor.z;

  lambda = mul(r, lambda);

  x0_correction = inv_mass_p0 * lambda;
  x1_correction = -inv_mass_p1 * lambda;

  float4 q_e_3_bar = float4(q.y, -q.x, q.w, -q.z);
  q_correction = quat_mul(float4(lambda.x, lambda.y, lambda.z, 0.0f), q_e_3_bar);
  q_correction *= 2.f * inv_mass_q * rest_length;  // From PositionBasedDynamics repo.
  // q_correction *= inv_mass_q * rest_length; //Derived from original paper.
}

public void project_shear_stretch_constraint_pre_strain(in float inv_time_step, in float bd, in float HC, in float3 p0,
                                                 in float3 p1, in float4 q, in float inv_mass_p0, in float inv_mass_p1,
                                                 in float inv_mass_q, in float3 alpha, in float rest_length,
                                                 out float3 x0_correction, out float3 x1_correction,
                                                 out float4 q_correction) {
  float3 d3;
  d3[0] = -2.0f * (q.x * q.z + q.w * q.y);
  d3[1] = -2.0f * (q.y * q.z - q.w * q.x);
  d3[2] = -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z;

  float3 abs_dis = abs(p1 - p0);
  float bd_factor = 6.0f * max(0.03f - bd, 0.0f) * (1.0f - HC);
  bd_factor = 12.0f * max(0.03f - bd, 0.0f);
  bd_factor = 0.0f;

  rest_length = rest_length * (1.0f - bd_factor);
  float3 lambda = p1 - p0 - d3 * rest_length;
  float3x3 r = mat3_cast(q);
  lambda = mul(transpose(r), lambda);
  float factor = max(1e-9f, inv_mass_p0 + inv_mass_p1 + 4.0f * inv_mass_q * rest_length * rest_length);

  float t2 = inv_time_step * inv_time_step;
  float3 alpha_factor = float3(t2 * alpha.x, t2 * alpha.y, t2 * alpha.z);
  lambda.x /= factor + alpha_factor.x;
  lambda.y /= factor + alpha_factor.y;
  lambda.z /= factor + alpha_factor.z;

  lambda = mul(r, lambda);

  x0_correction = inv_mass_p0 * lambda;
  x1_correction = -inv_mass_p1 * lambda;

  float4 q_e_3_bar = float4(q.y, -q.x, q.w, -q.z);
  q_correction = quat_mul(float4(lambda.x, lambda.y, lambda.z, 0.0f), q_e_3_bar);
  q_correction *= 2.f * inv_mass_q * rest_length;  // From PositionBasedDynamics repo.
  // q_correction *= inv_mass_q * rest_length; //Derived from original paper.
}

public void project_bend_twist_constraint(in float inv_time_step, in int segment_pair_handle) {
  if (segment_pairs[segment_pair_handle].bend_twist_bundle_integrity <= 0.f)
    return;
  float4 q0_correction, q1_correction;
  int segment0_handle = segment_pairs[segment_pair_handle].segment0_handle;
  int segment1_handle = segment_pairs[segment_pair_handle].segment1_handle;

  float rest_length = (segments[segment0_handle].rest_length + segments[segment1_handle].rest_length) * 0.5f;
  float3 alpha = float3(segment_pairs[segment_pair_handle].bending_alpha, segment_pairs[segment_pair_handle].bending_alpha,
                    segment_pairs[segment_pair_handle].twisting_alpha);
  project_bend_twist_constraint(inv_time_step, segments[segment0_handle].q, segments[segment0_handle].inv_mass,
                                segments[segment1_handle].q, segments[segment1_handle].inv_mass, alpha,
                                segment_pairs[segment_pair_handle].rest_darboux_vector, q0_correction, q1_correction);

  segments[segment0_handle].q = normalize(q0_correction + segments[segment0_handle].q);
  segments[segment1_handle].q = normalize(q1_correction + segments[segment1_handle].q);
}

public void project_bend_twist_constraint(in float inv_time_step, in int segment_pair_handle, out float4 q0_correction,
                                   out float4 q1_correction) {
  q0_correction = 0.0f;
  q1_correction = 0.0f;
  if (segment_pairs[segment_pair_handle].bend_twist_bundle_integrity <= 0.f)
    return;
  int segment0_handle = segment_pairs[segment_pair_handle].segment0_handle;
  int segment1_handle = segment_pairs[segment_pair_handle].segment1_handle;

  float rest_length = (segments[segment0_handle].rest_length + segments[segment1_handle].rest_length) * 0.5f;
  float3 alpha = float3(segment_pairs[segment_pair_handle].bending_alpha, segment_pairs[segment_pair_handle].bending_alpha,
                    segment_pairs[segment_pair_handle].twisting_alpha);
  project_bend_twist_constraint(inv_time_step, segments[segment0_handle].q, segments[segment0_handle].inv_mass,
                                segments[segment1_handle].q, segments[segment1_handle].inv_mass, alpha,
                                segment_pairs[segment_pair_handle].rest_darboux_vector, q0_correction, q1_correction);
}

public void project_bend_twist_constraint(in float inv_time_step, in float4 q0, in float inv_mass_q0, in float4 q1,
                                   in float inv_mass_q1, in float3 alpha, in float4 rest_darboux_vector,
                                   out float4 q0_correction, out float4 q1_correction) {
  float4 lambda = quat_mul(conjugate(q0), q1);
  float4 lambda_plus = lambda + rest_darboux_vector;
  lambda -= rest_darboux_vector;
  lambda = squared_norm(lambda) > squared_norm(lambda_plus) ? lambda_plus : lambda;

  float factor = max(1e-9f, inv_mass_q0 + inv_mass_q1);

  float t2 = inv_time_step * inv_time_step;
  float3 alpha_factor = float3(t2 * alpha.x, t2 * alpha.y, t2 * alpha.z);
  lambda.x /= factor + alpha_factor.x;
  lambda.y /= factor + alpha_factor.y;
  lambda.z /= factor + alpha_factor.z;

  lambda.w = 0.0f;

  q0_correction = quat_mul(q1, lambda) * inv_mass_q0;
  q1_correction = quat_mul(q0, lambda) * inv_mass_q1 * -1.0f;
}

public float shear_stretch_strain(in float3 p0, in float3 p1, in float4 q, in float rest_length) {
  float3 d3;
  d3[0] = -2.0f * (q.x * q.z + q.w * q.y);
  d3[1] = -2.0f * (q.y * q.z - q.w * q.x);
  d3[2] = -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z;

  float3 lambda = (p1 - p0) / rest_length - d3;
  return length(lambda);
}

public float2 bend_twist_strain(in float4 q0, in float4 q1, in float4 rest_darboux_vector) {
  float4 lambda = quat_mul(conjugate(q0), q1);
  float4 lambda_plus = lambda + rest_darboux_vector;
  lambda -= rest_darboux_vector;
  lambda = squared_norm(lambda) > squared_norm(lambda_plus) ? lambda_plus : lambda;

  return float2(max(abs(lambda.x), abs(lambda.y)), abs(lambda.z));
}

public void BundleSegmentPosition(in uint segment_handle, in float inv_time_step, in float over_relaxation, in float crack_bd_shrinkage_offset, in float crack_R_scale, in float crack_T_scale, in uint treespace, in uint internal_pattern) {
  float3 movement0_sum = float3(0.0f, 0.0f, 0.0f);
  float3 movement1_sum = float3(0.0f, 0.0f, 0.0f);

  float sum = 0.f;
  float bundle_alpha_sum = 0.0f;

  float3 segment0_particle0_position = segments[segment_handle].particle0.x;
  float3 segment0_particle1_position = segments[segment_handle].particle1.x;
  float3 segment0_center_position = (segment0_particle0_position + segment0_particle1_position) * 0.5f;

  float bd = segments[segment_handle].boundary_distance;
  float HC = segments[segment_handle].HC;
  float RW = segments[segment_handle].RW;
  float moisture = segments[segment_handle].moisture;
  int cube_pattern = segments[segment_handle].cube_pattern;


  float2 tree_profile_position = segments[segment_handle].profile_position;


  //float3 R_normal_world =
  //    float3(0.0, segments[segment_handle].profile_position /
  //                  (length(segments[segment_handle].profile_position) + 0.000001));  // TODO: Rewrite for General Case
  //float3 T_normal_world = float3(0.0, float2x2(0.0, -1.0, 1.0, 0.0) * segments[segment_handle].profile_position /
  //                                    (length(segments[segment_handle].profile_position) + 0.000001));
  //float3 L_normal_world = float3(1.0, 0.0, 0.0);  //for lying log


  float3 L_normal_world = normalize(segments[segment_handle].particle1.x - segments[segment_handle].particle0.x);// for tree (For log case, localX and localY need to be inverse)
  float3 up = float3(0.0, 0.0, 1.0);
  if (length(dot(L_normal_world, up)) > 0.99) {
    up = float3(0.0, -1.0, 0.0);
  }
  float3 localX = -normalize(cross(up, L_normal_world));
  float3 localY = normalize(cross(localX, L_normal_world));
  float3 R_normal_world =
      normalize(segments[segment_handle].profile_position.x * localX + segments[segment_handle].profile_position.y * localY);
  float3 T_normal_world = normalize( - segments[segment_handle].profile_position.y * localX +
                        segments[segment_handle].profile_position.x * localY);

  //float2 profile_corrected =
  //    segments[segment_handle].profile_position / (length(segments[segment_handle].profile_position) + 0.000001);
  //float3 R_normal_world =
  //    float3(profile_corrected.x, 0.0, profile_corrected.y);
  // float3 T_normal_world = float3(-profile_corrected.y, 0.0, profile_corrected.x);
  // float3 L_normal_world = float3(0.0, 1.0, 0.0);


  float t2 = inv_time_step * inv_time_step;


  [[unroll]] for (uint i = 0; i < BUNDLE_MAX_CONNECTION; i++) {
    int pair_handle = segment_data_list[segment_handle].pair_handles[i];
    if (pair_handle < 0)
      continue;
    if (segment_pairs[pair_handle].bend_twist_bundle_integrity <= 0.f)
      continue;
    bool is_segment0 = segment_handle == segment_pairs[pair_handle].segment0_handle;
    int neighbor_segment_handle =
        is_segment0 ? segment_pairs[pair_handle].segment1_handle : segment_pairs[pair_handle].segment0_handle;
    float3 neighbor_segment_particle0_position = segments[neighbor_segment_handle].particle0.x;
    float3 neighbor_segment_particle1_position = segments[neighbor_segment_handle].particle1.x;
    float3 neighbor_segment_center_position =
        (neighbor_segment_particle0_position + neighbor_segment_particle1_position) * 0.5f;

    float bd_factor = 12.0f * max(0.03f - bd, 0.0f) * (1.0f - HC);

    float3 offset_local =
        is_segment0 ? segment_pairs[pair_handle].segment0_offset.xyz : segment_pairs[pair_handle].segment1_offset.xyz;

    //bd_factor = min(max(bd - 0.1f, 0.0f), 0.1) * 2.0; //for log inner crack
    bd_factor = min(max(bd - crack_bd_shrinkage_offset, 0.0f), 0.1) * 2.0;  // for trunk inner crack
    //bd_factor = min(max(bd, 0.0f), 0.04) * 5.0;
    float moist_factor = 1.0f - moisture;
    //moist_factor = RW;  // For test only
    moist_factor = 1.0f;
    if (internal_pattern == 0) {
      moist_factor = 0.0f;
    }
    float3 offset_world = rotate_vec3(segments[neighbor_segment_handle].q, offset_local);
    float3 scaled_offset_world = offset_world;

    if (treespace == 1)
    {
      scaled_offset_world = scaleAlong(scaled_offset_world, R_normal_world,
                                       max(1.0 - 0.6 * bd_factor * moist_factor * crack_R_scale, 0.0));
      scaled_offset_world = scaleAlong(scaled_offset_world, T_normal_world,
                                       max(1.0 - 0.6 * bd_factor * moist_factor * crack_T_scale, 0.0));
    } else {
      scaled_offset_world =
          scaleAlong(scaled_offset_world, float3(0.0, 0.0, 1.0), max(1.0 - 0.1 * moist_factor * crack_R_scale, 0.0));
      scaled_offset_world =
          scaleAlong(scaled_offset_world, float3(0.0, 1.0, 0.0), max(1.0 - 0.1 * moist_factor * crack_T_scale, 0.0));
    }

    //scaled_offset_world = scaleAlong(scaled_offset_world, float3(0.0, 0.0, 1.0), max(1.0 - 3.0 * bd_factor * moist_factor, 0.0));
    //scaled_offset_world.z = -offset_world.z;

    float3 target_segment0_center_position = neighbor_segment_center_position + scaled_offset_world;
    // Always enforce inf stiffness.
    bundle_alpha_sum += 0.0f;  // t2 * segment_pairs[pair_handle].bending_alpha;
    float lambda = max(1e-9f, segments[segment_handle].inv_mass + segments[neighbor_segment_handle].inv_mass);

    float factor0 = segments[segment_handle].inv_mass / lambda;

    float segment_length = distance(segment0_particle0_position, segment0_particle1_position);
    float3 front_direction = normalize(rotate_vec3(segments[segment_handle].q, float3(0, 0, -1)));
    float3 target_segment0_particle0_position = target_segment0_center_position - front_direction * segment_length * .5f;
    float3 target_segment0_particle1_position = target_segment0_center_position + front_direction * segment_length * .5f;

    float3 segment0_particle0_position_correction =
        (target_segment0_particle0_position - segment0_particle0_position) * factor0;
    float3 segment0_particle1_position_correction =
        (target_segment0_particle1_position - segment0_particle1_position) * factor0;

    sum += 1.0f;
    movement0_sum += segment0_particle0_position_correction;
    movement1_sum += segment0_particle1_position_correction;
  }

  segment_data_list[segment_handle].particle0_position_correction.xyz =
      sum != 0 ? movement0_sum / sum / (1.0f + bundle_alpha_sum / sum) : float3(0.0f, 0.0f, 0.0f);
  segment_data_list[segment_handle].particle1_position_correction.xyz =
      sum != 0 ? movement1_sum / sum / (1.0f + bundle_alpha_sum / sum) : float3(0.0f, 0.0f, 0.0f);
}

// Assume vectors are normalized.
public float4 compute_rotation_between(in float3 v1, in float3 v2) {
  float dot_product = dot(v1, v2);
  float3 axis = cross(v1, v2);
  if (abs(1.f - dot_product) < 1e-6f) {
    return float4(0, 0, 0, 0);
  }
  if (abs(1.f + dot_product) < 1e-6f) {
    float3 ortho = v1.x < 0.9f ? float3(1.0f, 0.0f, 0.0f) : float3(0.0f, 1.0f, 0.0f);
    axis = normalize(cross(v1, ortho));
    return angle_axis(3.1415926f, axis);
  }
  float angle = acos(clamp(dot_product, -1.0f, 1.0f));
  return angle_axis(angle, normalize(axis));
}

public float4 compute_rotation_between(in float3 v1, in float3 v2, in float3 axis) {
  float3 v1p = v1 - dot(v1, axis) * axis;
  float3 v2p = v2 - dot(v2, axis) * axis;
  float len1p = length(v1p);
  float len2p = length(v2p);
  if (len1p < 1e-6f || len2p < 1e-6f) {
    // handle edge case (vectors parallel to axis or near zero)
    return float4(0, 0, 0, 0);
  }
  v1p /= len1p;
  v2p /= len2p;
  float dot_product = dot(v1p, v2p);
  if (abs(1.f - dot_product) < 1e-6f) {
    return float4(0, 0, 0, 0);
  }
  if (abs(-1.f - dot_product) < 1e-6f) {
    return angle_axis(3.1415926f, axis);
  }
  float3 cross_product = cross(v1p, v2p);
  float sine = length(cross_product);
  dot_product = clamp(dot_product, -1.0f, 1.0f);

  float angle = atan(sine / dot_product);
  angle += dot_product < 0 ? (sine >= 0 ? 3.1415926f : -3.1415926f) : 0.f;
  angle = dot(cross_product, axis) < 0 ? -angle : angle;

  float half_angle = angle * .5f;
  float sin_angle = sin(half_angle);
  return float4(axis * sin_angle, cos(half_angle));
}

public void BundleSegmentRotation(in uint segment_handle, in float inv_time_step, in float over_relaxation) {
  float rotation_sum = 0.f;
  float4 q_correction_sum = float4(0.0f, 0.0f, 0.0f, 0.0f);
  float4 q = segments[segment_handle].q;
  float inv_mass = segments[segment_handle].inv_mass;

  float3 segment0_center_position = (segments[segment_handle].particle0.x + segments[segment_handle].particle1.x) * 0.5f;
  [[unroll]] for (uint i = 0; i < BUNDLE_MAX_CONNECTION; i++) {
    int pair_handle = segment_data_list[segment_handle].pair_handles[i];
    if (pair_handle < 0)
      continue;
    if (segment_pairs[pair_handle].bend_twist_bundle_integrity <= 0.f)
      continue;
    int segment0_handle = segment_pairs[pair_handle].segment0_handle;
    int segment1_handle = segment_pairs[pair_handle].segment1_handle;

    bool is_segment0 = segment_handle == segment0_handle;
    int neighbor_segment_handle = is_segment0 ? segment1_handle : segment0_handle;
    float3 segment1_center_position =
        (segments[neighbor_segment_handle].particle0.x + segments[neighbor_segment_handle].particle1.x) * 0.5f;

    float t2 = inv_time_step * inv_time_step;
    float bending_alpha = segment_pairs[pair_handle].bending_alpha;
    float twisting_alpha = segment_pairs[pair_handle].twisting_alpha;

    float3 alpha_factor = t2 * float3(bending_alpha, bending_alpha, twisting_alpha);

    float factor = max(1e-9f, inv_mass + segments[neighbor_segment_handle].inv_mass);

    if (segments[segment_handle].strand_handle != segments[neighbor_segment_handle].strand_handle) {
      float3 current_offset = rotate_vec3(conjugate(q), segment1_center_position - segment0_center_position);
      float3 expected_offset =
          is_segment0 ? segment_pairs[pair_handle].segment1_offset.xyz : segment_pairs[pair_handle].segment0_offset.xyz;
      float4 expected_rotation =
          quat_mul(q, compute_rotation_between(normalize(expected_offset), normalize(current_offset)));

      float4 lambda = q;
      float4 lambda_plus = lambda + expected_rotation;
      lambda -= expected_rotation;
      if (squared_norm(lambda) > squared_norm(lambda_plus))
        lambda = lambda_plus;

      lambda.x /= factor;
      lambda.y /= factor;
      lambda.z /= factor;

      lambda.w = 0.0f;

      float4 rotation_correction = quat_mul(q, lambda) * inv_mass;
      q_correction_sum += expected_rotation;
      rotation_sum += 1.0f;
    }
  }
  segment_data_list[segment_handle].q_correction =
      rotation_sum != 0 ? q_correction_sum / rotation_sum : float4(0.0f, 0.0f, 0.0f, 0.0f);
}

public void BundleSegmentBendTwist(in uint segment_handle, in float inv_time_step, in float over_relaxation) {
  float4 q_correction_sum = float4(0.0f, 0.0f, 0.0f, 0.0f);
  float sum = 0.f;

  float4 q = segments[segment_handle].q;
  float inv_mass = segments[segment_handle].inv_mass;
  float t2 = inv_time_step * inv_time_step;

  [[unroll]] for (uint i = 0; i < BUNDLE_MAX_CONNECTION; i++) {
    int pair_handle = segment_data_list[segment_handle].pair_handles[i];
    if (pair_handle < 0)
      continue;
    if (segment_pairs[pair_handle].bend_twist_bundle_integrity <= 0.f)
      continue;

    int segment0_handle = segment_pairs[pair_handle].segment0_handle;
    int segment1_handle = segment_pairs[pair_handle].segment1_handle;

    bool is_segment0 = segment_handle == segment0_handle;
    int neighbor_segment_handle = is_segment0 ? segment1_handle : segment0_handle;

    float4 neighbor_q = segments[neighbor_segment_handle].q;
    float neighbor_inv_mass = segments[neighbor_segment_handle].inv_mass;

    float4 rest_darboux_vector = is_segment0 ? segment_pairs[pair_handle].rest_darboux_vector
                                           : conjugate(segment_pairs[pair_handle].rest_darboux_vector);

    float4 q0_correction, q1_correction;
    float bending_alpha = 0.0f;   // segment_pairs[pair_handle].bending_alpha;
    float twisting_alpha = 0.0f;  // segment_pairs[pair_handle].twisting_alpha;

    float4 lambda = quat_mul(conjugate(q), neighbor_q);
    float4 lambda_plus = lambda + rest_darboux_vector;
    lambda -= rest_darboux_vector;
    lambda = squared_norm(lambda) > squared_norm(lambda_plus) ? lambda_plus : lambda;
    float factor = max(1e-9f, inv_mass + neighbor_inv_mass);

    lambda.x /= factor + bending_alpha * t2;
    lambda.y /= factor + twisting_alpha * t2;
    lambda.z /= factor + twisting_alpha * t2;

    lambda.w = 0.0f;

    float4 q_correction = quat_mul(neighbor_q, lambda) * inv_mass;
    q_correction_sum += q_correction;
    sum += 1.f;
  }

  float4 q_correction = q_correction_sum / sum;
  segment_data_list[segment_handle].q_correction = sum != 0 ? q_correction : float4(0.0f, 0.0f, 0.0f, 0.0f);
}

public void BundleSegmentShearStretch(in uint segment_handle, in float inv_time_step) {
  float3 x0_correction, x1_correction;
  float4 q_correction;

  int prev_handle = segments[segment_handle].prev_handle;
  int next_handle = segments[segment_handle].next_handle;

  float3 p0 = segments[segment_handle].particle0.x;
  float3 p1 = segments[segment_handle].particle1.x;

  float bd = segments[segment_handle].boundary_distance;
  float HC = segments[segment_handle].HC;

  float inv_mass_q = segments[segment_handle].inv_mass;
  float inv_mass_p0 = prev_handle != -1 ? segments[prev_handle].inv_mass : inv_mass_q;
  float inv_mass_p1 = next_handle != -1 ? segments[next_handle].inv_mass : inv_mass_q;

  float4 q = segments[segment_handle].q;

  float3 alpha = float3(segments[segment_handle].shear_stretch_alpha);

  float rest_length = segments[segment_handle].rest_length;

  project_shear_stretch_constraint_pre_strain(inv_time_step, bd, HC, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q,
                                              alpha, rest_length, x0_correction, x1_correction, q_correction);

  segment_data_list[segment_handle].particle0_position_correction.xyz = x0_correction;
  segment_data_list[segment_handle].particle1_position_correction.xyz = x1_correction;
  segment_data_list[segment_handle].q_correction = q_correction;
}

public void update_inertia_w(in float4 q, in float3 inertia_tensor, in float3 inv_inertia_tensor, inout float4x4 inertia_w,
                      inout float4x4 inv_inertia_w) {
  // Update w
  float3x3 rot = mat3_cast(q);
  float3x3 inertia_tensor_diag = float3x3(inertia_tensor.x, 0.0, 0.0, 0.0, inertia_tensor.y, 0.0, 0.0, 0.0, inertia_tensor.z);
  float3x3 inertia_w33 = rot * inertia_tensor_diag * transpose(rot);

  float3x3 inverse_inertia_tensor_diag =
      float3x3(inv_inertia_tensor.x, 0.0, 0.0, 0.0, inv_inertia_tensor.y, 0.0, 0.0, 0.0, inv_inertia_tensor.z);
  float3x3 inverse_inertia_w33 = rot * inverse_inertia_tensor_diag * transpose(rot);

  inertia_w =
      float4x4(inertia_w33[0][0], inertia_w33[0][1], inertia_w33[0][2], 0.0, inertia_w33[1][0], inertia_w33[1][1],
           inertia_w33[1][2], 0.0, inertia_w33[2][0], inertia_w33[2][1], inertia_w33[2][2], 0.0, 0.0, 0.0, 0.0, 0.0);
  inv_inertia_w =
      float4x4(inverse_inertia_w33[0][0], inverse_inertia_w33[0][1], inverse_inertia_w33[0][2], 0.0,
           inverse_inertia_w33[1][0], inverse_inertia_w33[1][1], inverse_inertia_w33[1][2], 0.0,
           inverse_inertia_w33[2][0], inverse_inertia_w33[2][1], inverse_inertia_w33[2][2], 0.0, 0.0, 0.0, 0.0, 0.0);
}
