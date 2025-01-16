
#include "DynamicStrands.glsl"
#include "Math.glsl"

// Constraint Solvers Decl
void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle);
void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle, out vec3 x0_correction,
                                      out vec3 x1_correction, out vec4 q_correction);
void project_shear_stretch_constraint(in float inv_time_step, in vec3 p0, in vec3 p1, in vec4 q, in float inv_mass_p0,
                                      in float inv_mass_p1, in float inv_mass_q, in vec3 alpha, in float rest_length,
                                      out vec3 x0_correction, out vec3 x1_correction, out vec4 q_correction);

void project_bend_twist_constraint(in float inv_time_step, in int segment_pair_handle);
void project_bend_twist_constraint(in float inv_time_step, in int segment_pair_handle, out vec4 q0_correction,
                                   out vec4 q1_correction);
void project_bend_twist_constraint(in float inv_time_step, in vec4 q0, in float inv_mass_q0, in vec4 q1,
                                   in float inv_mass_q1, in vec3 alpha, in vec4 rest_darboux_vector,
                                   out vec4 q0_correction, out vec4 q1_correction);

void BundleSegmentPosition(in uint segment_handle, in float inv_time_step, in float over_relaxation);
void BundleSegmentRotation(in uint segment_handle, in float inv_time_step, in float over_relaxation);
void BundleSegmentBendTwist(in uint segment_handle, in float inv_time_step, in float over_relaxation);
void BundleSegmentShearStretch(in uint segment_handle, in float inv_time_step);

// Constraint Solvers Impl
void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle) {
  vec3 x0_correction, x1_correction;
  vec4 q_correction;

  vec3 p0 = segments[segment_handle].particle0.x;
  vec3 p1 = segments[segment_handle].particle1.x;

  int prev_handle = segments[segment_handle].prev_handle;
  int next_handle = segments[segment_handle].next_handle;

  float inv_mass_q = segments[segment_handle].inv_mass;
  float inv_mass_p0 = prev_handle != -1 ? segments[prev_handle].inv_mass : inv_mass_q;
  float inv_mass_p1 = next_handle != -1 ? segments[next_handle].inv_mass : inv_mass_q;

  vec4 q = segments[segment_handle].q;
  vec3 alpha = vec3(segments[segment_handle].shear_stretch_alpha);
  float rest_length = segments[segment_handle].rest_length;

  project_shear_stretch_constraint(inv_time_step, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q, alpha, rest_length,
                                   x0_correction, x1_correction, q_correction);

  vec3 particle0_new_position = p0 + x0_correction;
  vec3 particle1_new_position = p1 + x1_correction;

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

void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle, out vec3 x0_correction,
                                      out vec3 x1_correction, out vec4 q_correction) {
  vec3 p0 = segments[segment_handle].particle0.x;
  vec3 p1 = segments[segment_handle].particle1.x;

  int prev_handle = segments[segment_handle].prev_handle;
  int next_handle = segments[segment_handle].next_handle;

  float inv_mass_q = segments[segment_handle].inv_mass;
  float inv_mass_p0 = prev_handle != -1 ? segments[prev_handle].inv_mass : inv_mass_q;
  float inv_mass_p1 = next_handle != -1 ? segments[next_handle].inv_mass : inv_mass_q;

  vec4 q = segments[segment_handle].q;
  vec3 alpha = vec3(segments[segment_handle].shear_stretch_alpha);
  float rest_length = segments[segment_handle].rest_length;

  project_shear_stretch_constraint(inv_time_step, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q, alpha, rest_length,
                                   x0_correction, x1_correction, q_correction);
}

void project_shear_stretch_constraint(in float inv_time_step, in vec3 p0, in vec3 p1, in vec4 q, in float inv_mass_p0,
                                      in float inv_mass_p1, in float inv_mass_q, in vec3 alpha, in float rest_length,
                                      out vec3 x0_correction, out vec3 x1_correction, out vec4 q_correction) {
  vec3 d3;
  d3[0] = -2.0f * (q.x * q.z + q.w * q.y);
  d3[1] = -2.0f * (q.y * q.z - q.w * q.x);
  d3[2] = -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z;

  vec3 lambda = p1 - p0 - d3 * rest_length;
  mat3 r = mat3_cast(q);
  lambda = transpose(r) * lambda;
  float factor = max(1e-9f, inv_mass_p0 + inv_mass_p1 + 4.0f * inv_mass_q * rest_length * rest_length);

  float t2 = inv_time_step * inv_time_step;
  vec3 alpha_factor = vec3(t2 * alpha.x, t2 * alpha.y, t2 * alpha.z);
  lambda.x /= factor + alpha_factor.x;
  lambda.y /= factor + alpha_factor.y;
  lambda.z /= factor + alpha_factor.z;

  lambda = r * lambda;

  x0_correction = inv_mass_p0 * lambda;
  x1_correction = -inv_mass_p1 * lambda;

  vec4 q_e_3_bar = vec4(q.y, -q.x, q.w, -q.z);
  q_correction = quat_mul(vec4(lambda.x, lambda.y, lambda.z, 0.0f), q_e_3_bar);
  q_correction *= 2.f * inv_mass_q * rest_length;  // From PositionBasedDynamics repo.
  // q_correction *= inv_mass_q * rest_length; //Derived from original paper.
}

void project_bend_twist_constraint(in float inv_time_step, in int segment_pair_handle) {
  if (segment_pairs[segment_pair_handle].bend_twist_bundle_integrity <= 0.f)
    return;
  vec4 q0_correction, q1_correction;
  int segment0_handle = segment_pairs[segment_pair_handle].segment0_handle;
  int segment1_handle = segment_pairs[segment_pair_handle].segment1_handle;

  float rest_length = (segments[segment0_handle].rest_length + segments[segment1_handle].rest_length) * 0.5f;
  vec3 alpha = vec3(segment_pairs[segment_pair_handle].bending_alpha, segment_pairs[segment_pair_handle].bending_alpha,
                    segment_pairs[segment_pair_handle].twisting_alpha);
  project_bend_twist_constraint(inv_time_step, segments[segment0_handle].q, segments[segment0_handle].inv_mass,
                                segments[segment1_handle].q, segments[segment1_handle].inv_mass, alpha,
                                segment_pairs[segment_pair_handle].rest_darboux_vector, q0_correction, q1_correction);

  segments[segment0_handle].q = normalize(q0_correction + segments[segment0_handle].q);
  segments[segment1_handle].q = normalize(q1_correction + segments[segment1_handle].q);
}

void project_bend_twist_constraint(in float inv_time_step, in int segment_pair_handle, out vec4 q0_correction,
                                   out vec4 q1_correction) {
  if (segment_pairs[segment_pair_handle].bend_twist_bundle_integrity <= 0.f)
    return;
  int segment0_handle = segment_pairs[segment_pair_handle].segment0_handle;
  int segment1_handle = segment_pairs[segment_pair_handle].segment1_handle;

  float rest_length = (segments[segment0_handle].rest_length + segments[segment1_handle].rest_length) * 0.5f;
  vec3 alpha = vec3(segment_pairs[segment_pair_handle].bending_alpha, segment_pairs[segment_pair_handle].bending_alpha,
                    segment_pairs[segment_pair_handle].twisting_alpha);
  project_bend_twist_constraint(inv_time_step, segments[segment0_handle].q, segments[segment0_handle].inv_mass,
                                segments[segment1_handle].q, segments[segment1_handle].inv_mass, alpha,
                                segment_pairs[segment_pair_handle].rest_darboux_vector, q0_correction, q1_correction);
}

void project_bend_twist_constraint(in float inv_time_step, in vec4 q0, in float inv_mass_q0, in vec4 q1,
                                   in float inv_mass_q1, in vec3 alpha, in vec4 rest_darboux_vector,
                                   out vec4 q0_correction, out vec4 q1_correction) {
  vec4 lambda = quat_mul(conjugate(q0), q1);
  vec4 lambda_plus = lambda + rest_darboux_vector;
  lambda -= rest_darboux_vector;
  lambda = squared_norm(lambda) > squared_norm(lambda_plus) ? lambda_plus : lambda;

  float factor = max(1e-9f, inv_mass_q0 + inv_mass_q1);

  float t2 = inv_time_step * inv_time_step;
  vec3 alpha_factor = vec3(t2 * alpha.x, t2 * alpha.y, t2 * alpha.z);
  lambda.x /= factor + alpha_factor.x;
  lambda.y /= factor + alpha_factor.y;
  lambda.z /= factor + alpha_factor.z;

  lambda.w = 0.0f;

  q0_correction = quat_mul(q1, lambda) * inv_mass_q0;
  q1_correction = quat_mul(q0, lambda) * inv_mass_q1 * -1.0f;
}

float shear_stretch_strain(in vec3 p0, in vec3 p1, in vec4 q, in float rest_length) {
  vec3 d3;
  d3[0] = -2.0f * (q.x * q.z + q.w * q.y);
  d3[1] = -2.0f * (q.y * q.z - q.w * q.x);
  d3[2] = -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z;

  vec3 lambda = (p1 - p0) / rest_length - d3;
  return length(lambda);
}

vec2 bend_twist_strain(in vec4 q0, in vec4 q1, in vec4 rest_darboux_vector) {
  vec4 lambda = quat_mul(conjugate(q0), q1);
  vec4 lambda_plus = lambda + rest_darboux_vector;
  lambda -= rest_darboux_vector;
  lambda = squared_norm(lambda) > squared_norm(lambda_plus) ? lambda_plus : lambda;

  return vec2(max(abs(lambda.x), abs(lambda.y)), abs(lambda.z));
}

void BundleSegmentPosition(in uint segment_handle, in float inv_time_step, in float over_relaxation) {
  vec3 movement0_sum = vec3(0.0f, 0.0f, 0.0f);
  vec3 movement1_sum = vec3(0.0f, 0.0f, 0.0f);

  float sum = 0.f;
  float bundle_alpha_sum = 0.0f;

  vec3 segment0_particle0_position = segments[segment_handle].particle0.x;
  vec3 segment0_particle1_position = segments[segment_handle].particle1.x;
  vec3 segment0_center_position = (segment0_particle0_position + segment0_particle1_position) * 0.5f;

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
    vec3 neighbor_segment_particle0_position = segments[neighbor_segment_handle].particle0.x;
    vec3 neighbor_segment_particle1_position = segments[neighbor_segment_handle].particle1.x;
    vec3 neighbor_segment_center_position =
        (neighbor_segment_particle0_position + neighbor_segment_particle1_position) * 0.5f;

    
    vec3 target_segment0_center_position =
        neighbor_segment_center_position +
        rotate_vec3(segments[neighbor_segment_handle].q, is_segment0 ? segment_pairs[pair_handle].segment0_offset.xyz
                                                                     : segment_pairs[pair_handle].segment1_offset.xyz);
    
    /*
    float current_distance = distance(segment0_center_position, neighbor_segment_center_position);
    float target_distance = length(segment_pairs[pair_handle].segment0_offset);
    vec3 target_segment0_center_position =
        segment0_center_position + (target_distance - current_distance) * 0.5f *
                                       normalize(segment0_center_position - neighbor_segment_center_position);
    */
    // Always enforce inf stiffness.
    bundle_alpha_sum += 0.0f; // t2 * segment_pairs[pair_handle].bending_alpha;
    float lambda = max(1e-9f, segments[segment_handle].inv_mass + segments[neighbor_segment_handle].inv_mass);
    
    float factor0 = segments[segment_handle].inv_mass / lambda;

    float segment_length = distance(segment0_particle0_position, segment0_particle1_position);
    vec3 front_direction = normalize(rotate_vec3(segments[segment_handle].q, vec3(0, 0, -1)));
    vec3 target_segment0_particle0_position = target_segment0_center_position - front_direction * segment_length * .5f;
    vec3 target_segment0_particle1_position = target_segment0_center_position + front_direction * segment_length * .5f;

    vec3 segment0_particle0_position_correction =
        (target_segment0_particle0_position - segment0_particle0_position) * factor0;
    vec3 segment0_particle1_position_correction =
        (target_segment0_particle1_position - segment0_particle1_position) * factor0;

    sum += 1.0f;
    movement0_sum += segment0_particle0_position_correction;
    movement1_sum += segment0_particle1_position_correction;
  }

  segment_data_list[segment_handle].particle0_position_correction.xyz =
      sum != 0 ? movement0_sum / sum / (1.0f + bundle_alpha_sum / sum) : vec3(0.0f, 0.0f, 0.0f);
  segment_data_list[segment_handle].particle1_position_correction.xyz =
      sum != 0 ? movement1_sum / sum / (1.0f + bundle_alpha_sum / sum) : vec3(0.0f, 0.0f, 0.0f);
}

// Assume vectors are normalized.
vec4 compute_rotation_between(in vec3 v1, in vec3 v2) {
  float dot_product = dot(v1, v2);
  vec3 axis = cross(v1, v2);
  if (abs(1.f - dot_product) < 1e-6f) {
    return vec4(0, 0, 0, 0);
  }
  if (abs(1.f + dot_product) < 1e-6f) {
    vec3 ortho = v1.x < 0.9f ? vec3(1.0f, 0.0f, 0.0f) : vec3(0.0f, 1.0f, 0.0f);
    axis = normalize(cross(v1, ortho));
    return angle_axis(3.1415926f, axis);
  }
  float angle = acos(clamp(dot_product, -1.0f, 1.0f));
  return angle_axis(angle, normalize(axis));
}

vec4 compute_rotation_between(in vec3 v1, in vec3 v2, in vec3 axis) {
  vec3 v1p = v1 - dot(v1, axis) * axis;
  vec3 v2p = v2 - dot(v2, axis) * axis;
  float len1p = length(v1p);
  float len2p = length(v2p);
  if (len1p < 1e-6f || len2p < 1e-6f) {
    // handle edge case (vectors parallel to axis or near zero)
    return vec4(0, 0, 0, 0);
  }
  v1p /= len1p;
  v2p /= len2p;
  float dot_product = dot(v1p, v2p);
  if (abs(1.f - dot_product) < 1e-6f) {
    return vec4(0, 0, 0, 0);
  }
  if (abs(-1.f - dot_product) < 1e-6f) {
    return angle_axis(3.1415926f, axis);
  }
  vec3 cross_product = cross(v1p, v2p);
  float sine = length(cross_product);
  dot_product = clamp(dot_product, -1.0f, 1.0f);

  float angle = atan(sine / dot_product);
  angle += dot_product < 0 ? (sine >= 0 ? 3.1415926f : -3.1415926f) : 0.f;
  angle = dot(cross_product, axis) < 0 ? -angle : angle;

  float half_angle = angle * .5f;
  float sin_angle = sin(half_angle);
  return vec4(axis * sin_angle, cos(half_angle));
}

void BundleSegmentRotation(in uint segment_handle, in float inv_time_step, in float over_relaxation) {
  float rotation_sum = 0.f;
  vec4 q_correction_sum = vec4(0.0f, 0.0f, 0.0f, 0.0f);
  vec4 q = segments[segment_handle].q;
  float inv_mass = segments[segment_handle].inv_mass;

  vec3 segment0_center_position = (segments[segment_handle].particle0.x + segments[segment_handle].particle1.x) * 0.5f;
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
    vec3 segment1_center_position =
        (segments[neighbor_segment_handle].particle0.x + segments[neighbor_segment_handle].particle1.x) * 0.5f;

    float t2 = inv_time_step * inv_time_step;
    float bending_alpha = segment_pairs[pair_handle].bending_alpha;
    float twisting_alpha = segment_pairs[pair_handle].twisting_alpha;

    vec3 alpha_factor = t2 * vec3(bending_alpha, bending_alpha, twisting_alpha);

    float factor = max(1e-9f, inv_mass + segments[neighbor_segment_handle].inv_mass);

    if (segments[segment_handle].strand_handle != segments[neighbor_segment_handle].strand_handle) {
      vec3 current_offset = rotate_vec3(conjugate(q), segment1_center_position - segment0_center_position);
      vec3 expected_offset =
          is_segment0 ? segment_pairs[pair_handle].segment1_offset.xyz : segment_pairs[pair_handle].segment0_offset.xyz;
      vec4 expected_rotation =
          quat_mul(q, compute_rotation_between(normalize(expected_offset), normalize(current_offset)));

      vec4 lambda = q;
      vec4 lambda_plus = lambda + expected_rotation;
      lambda -= expected_rotation;
      if (squared_norm(lambda) > squared_norm(lambda_plus))
        lambda = lambda_plus;

      lambda.x /= factor;
      lambda.y /= factor;
      lambda.z /= factor;

      lambda.w = 0.0f;

      vec4 rotation_correction = quat_mul(q, lambda) * inv_mass;
      q_correction_sum += expected_rotation;
      rotation_sum += 1.0f;
    }
  }
  segment_data_list[segment_handle].q_correction =
      rotation_sum != 0 ? q_correction_sum / rotation_sum : vec4(0.0f, 0.0f, 0.0f, 0.0f);
}

void BundleSegmentBendTwist(in uint segment_handle, in float inv_time_step, in float over_relaxation) {
  vec4 q_correction_sum = vec4(0.0f, 0.0f, 0.0f, 0.0f);
  float sum = 0.f;

  vec4 q = segments[segment_handle].q;
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

    vec4 neighbor_q = segments[neighbor_segment_handle].q;
    float neighbor_inv_mass = segments[neighbor_segment_handle].inv_mass;

    vec4 rest_darboux_vector = is_segment0 ? segment_pairs[pair_handle].rest_darboux_vector
                                           : conjugate(segment_pairs[pair_handle].rest_darboux_vector);

    vec4 q0_correction, q1_correction;
    float bending_alpha = 0.0f; //segment_pairs[pair_handle].bending_alpha;
    float twisting_alpha = 0.0f; // segment_pairs[pair_handle].twisting_alpha;
    
    vec4 lambda = quat_mul(conjugate(q), neighbor_q);
    vec4 lambda_plus = lambda + rest_darboux_vector;
    lambda -= rest_darboux_vector;
    lambda = squared_norm(lambda) > squared_norm(lambda_plus) ? lambda_plus : lambda;
    float factor = max(1e-9f, inv_mass + neighbor_inv_mass);

    lambda.x /= factor + bending_alpha * t2;
    lambda.y /= factor + twisting_alpha * t2;
    lambda.z /= factor + twisting_alpha * t2;

    lambda.w = 0.0f;

    vec4 q_correction = quat_mul(neighbor_q, lambda) * inv_mass;
    q_correction_sum += q_correction;
    sum += 1.f;
  }

  vec4 q_correction = q_correction_sum / sum;
  segment_data_list[segment_handle]
          .q_correction = sum != 0 ? q_correction : vec4(0.0f, 0.0f, 0.0f, 0.0f);
}

void BundleSegmentShearStretch(in uint segment_handle, in float inv_time_step) {
  vec3 x0_correction, x1_correction;
  vec4 q_correction;

  int prev_handle = segments[segment_handle].prev_handle;
  int next_handle = segments[segment_handle].next_handle;

  vec3 p0 = segments[segment_handle].particle0.x;
  vec3 p1 = segments[segment_handle].particle1.x;

  float inv_mass_q = segments[segment_handle].inv_mass;
  float inv_mass_p0 = prev_handle != -1 ? segments[prev_handle].inv_mass : inv_mass_q;
  float inv_mass_p1 = next_handle != -1 ? segments[next_handle].inv_mass : inv_mass_q;

  vec4 q = segments[segment_handle].q;

  vec3 alpha = vec3(segments[segment_handle].shear_stretch_alpha);

  float rest_length = segments[segment_handle].rest_length;

  project_shear_stretch_constraint(inv_time_step, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q, alpha, rest_length,
                                   x0_correction, x1_correction, q_correction);

  segment_data_list[segment_handle].particle0_position_correction.xyz = x0_correction;
  segment_data_list[segment_handle].particle1_position_correction.xyz = x1_correction;
  segment_data_list[segment_handle].q_correction = q_correction;
}

void update_inertia_w(in vec4 q, in vec3 inertia_tensor, in vec3 inv_inertia_tensor, inout mat4 inertia_w,
                      inout mat4 inv_inertia_w) {
  // Update w
  mat3 rot = mat3_cast(q);
  mat3 inertia_tensor_diag = mat3(inertia_tensor.x, 0.0, 0.0, 0.0, inertia_tensor.y, 0.0, 0.0, 0.0, inertia_tensor.z);
  mat3 inertia_w33 = rot * inertia_tensor_diag * transpose(rot);

  mat3 inverse_inertia_tensor_diag =
      mat3(inv_inertia_tensor.x, 0.0, 0.0, 0.0, inv_inertia_tensor.y, 0.0, 0.0, 0.0, inv_inertia_tensor.z);
  mat3 inverse_inertia_w33 = rot * inverse_inertia_tensor_diag * transpose(rot);

  inertia_w =
      mat4(inertia_w33[0][0], inertia_w33[0][1], inertia_w33[0][2], 0.0, inertia_w33[1][0], inertia_w33[1][1],
           inertia_w33[1][2], 0.0, inertia_w33[2][0], inertia_w33[2][1], inertia_w33[2][2], 0.0, 0.0, 0.0, 0.0, 0.0);
  inv_inertia_w =
      mat4(inverse_inertia_w33[0][0], inverse_inertia_w33[0][1], inverse_inertia_w33[0][2], 0.0,
           inverse_inertia_w33[1][0], inverse_inertia_w33[1][1], inverse_inertia_w33[1][2], 0.0,
           inverse_inertia_w33[2][0], inverse_inertia_w33[2][1], inverse_inertia_w33[2][2], 0.0, 0.0, 0.0, 0.0, 0.0);
}
