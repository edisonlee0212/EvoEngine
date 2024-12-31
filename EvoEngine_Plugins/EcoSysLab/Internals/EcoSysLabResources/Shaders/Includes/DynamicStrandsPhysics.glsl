
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

  Segment segment = segments[segment_handle];

  Particle particle0 = segment.particle0;
  Particle particle1 = segment.particle1;

  vec3 p0 = particle0.x;
  vec3 p1 = particle1.x;

  float inv_mass_q = segment.inv_mass;
  float inv_mass_p0 = inv_mass_q;
  float inv_mass_p1 = inv_mass_q;
  if (segment.prev_handle != -1) {
    inv_mass_p0 = segments[segment.prev_handle].inv_mass;
  }
  if (segment.next_handle != -1) {
    inv_mass_p1 = segments[segment.next_handle].inv_mass;
  }
  vec4 q = segment.q;
  vec3 alpha = vec3(segment.shearing_alpha, segment.shearing_alpha, segment.stretching_alpha);
  float rest_length = segment.rest_length;

  project_shear_stretch_constraint(inv_time_step, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q, alpha, rest_length,
                                   x0_correction, x1_correction, q_correction);

  vec3 particle0_new_position = p0 + x0_correction;
  vec3 particle1_new_position = p1 + x1_correction;
  segments[segment_handle].particle0.x = particle0_new_position;
  segments[segment_handle].particle1.x = particle1_new_position;
  segments[segment_handle].q = normalize(q_correction + segment.q);

  SegmentData segment_data = segment_data_list[segment_handle];

  if (segment.prev_handle != -1) {
    if (segment_pairs[segment_data.pair_handles[0]].connectivity_integrity > 0.f) {
      segments[segment.prev_handle].particle1.x = particle0_new_position;
    }
  }
  if (segment.next_handle != -1) {
    if (segment_pairs[segment_data.pair_handles[1]].connectivity_integrity > 0.f) {
      segments[segment.next_handle].particle0.x = particle1_new_position;
    }
  }
}

void project_shear_stretch_constraint(in float inv_time_step, in int segment_handle, out vec3 x0_correction,
                                      out vec3 x1_correction, out vec4 q_correction) {
  Segment segment = segments[segment_handle];

  Particle particle0 = segment.particle0;
  Particle particle1 = segment.particle1;

  vec3 p0 = particle0.x;
  vec3 p1 = particle1.x;

  float inv_mass_q = segment.inv_mass;
  float inv_mass_p0 = inv_mass_q;
  float inv_mass_p1 = inv_mass_q;
  if (segment.prev_handle != -1) {
    inv_mass_p0 = segments[segment.prev_handle].inv_mass;
  }
  if (segment.next_handle != -1) {
    inv_mass_p1 = segments[segment.next_handle].inv_mass;
  }
  vec4 q = segment.q;
  vec3 alpha = vec3(segment.shearing_alpha, segment.shearing_alpha, segment.stretching_alpha);
  float rest_length = segment.rest_length;

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
  SegmentPair segment_pair = segment_pairs[segment_pair_handle];
  if (segment_pair.bend_twist_bundle_integrity <= 0.f)
    return;

  vec4 q0_correction, q1_correction;
  int segment0_handle = segment_pair.segment0_handle;
  int segment1_handle = segment_pair.segment1_handle;
  Segment segment0 = segments[segment0_handle];
  Segment segment1 = segments[segment1_handle];

  float rest_length = (segment0.rest_length + segment1.rest_length) * 0.5f;
  vec3 alpha = vec3(segment_pair.bending_alpha, segment_pair.bending_alpha, segment_pair.twisting_alpha);
  project_bend_twist_constraint(inv_time_step, segment0.q, segment0.inv_mass, segment1.q, segment1.inv_mass, alpha,
                                segment_pair.rest_darboux_vector, q0_correction, q1_correction);

  segments[segment0_handle].q = normalize(q0_correction + segment0.q);
  segments[segment1_handle].q = normalize(q1_correction + segment1.q);
}

void project_bend_twist_constraint(in float inv_time_step, in int segment_pair_handle, out vec4 q0_correction,
                                   out vec4 q1_correction) {
  SegmentPair segment_pair = segment_pairs[segment_pair_handle];
  if (segment_pair.bend_twist_bundle_integrity <= 0.f)
    return;
  int segment0_handle = segment_pair.segment0_handle;
  int segment1_handle = segment_pair.segment1_handle;
  Segment segment0 = segments[segment0_handle];
  Segment segment1 = segments[segment1_handle];

  float rest_length = (segment0.rest_length + segment1.rest_length) * 0.5f;
  vec3 alpha = vec3(segment_pair.bending_alpha, segment_pair.bending_alpha, segment_pair.twisting_alpha);
  project_bend_twist_constraint(inv_time_step, segment0.q, segment0.inv_mass, segment1.q, segment1.inv_mass, alpha,
                                segment_pair.rest_darboux_vector, q0_correction, q1_correction);
}

void project_bend_twist_constraint(in float inv_time_step, in vec4 q0, in float inv_mass_q0, in vec4 q1,
                                   in float inv_mass_q1, in vec3 alpha, in vec4 rest_darboux_vector,
                                   out vec4 q0_correction, out vec4 q1_correction) {
  vec4 lambda = quat_mul(conjugate(q0), q1);
  vec4 lambda_plus = lambda + rest_darboux_vector;
  lambda -= rest_darboux_vector;
  if (squared_norm(lambda) > squared_norm(lambda_plus))
    lambda = lambda_plus;
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

vec2 shear_stretch_strain(in vec3 p0, in vec3 p1, in vec4 q, in float rest_length) {
  vec3 d3;
  d3[0] = -2.0f * (q.x * q.z + q.w * q.y);
  d3[1] = -2.0f * (q.y * q.z - q.w * q.x);
  d3[2] = -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z;

  vec3 lambda = (p1 - p0) / rest_length - d3;
  return vec2(max(abs(lambda.x), abs(lambda.y)), lambda.z);
}

vec2 bend_twist_strain(in vec4 q0, in vec4 q1, in vec4 rest_darboux_vector) {
  vec4 lambda = quat_mul(conjugate(q0), q1);
  vec4 lambda_plus = lambda + rest_darboux_vector;
  lambda -= rest_darboux_vector;
  if (squared_norm(lambda) > squared_norm(lambda_plus))
    lambda = lambda_plus;
  return vec2(max(abs(lambda.x), abs(lambda.y)), lambda.z);
}

void BundleSegmentPosition(in uint segment_handle, in float inv_time_step, in float over_relaxation) {
  Segment segment0 = segments[segment_handle];
  Particle segment0_particle0 = segment0.particle0;
  Particle segment0_particle1 = segment0.particle1;

  SegmentData segment_data = segment_data_list[segment_handle];
  vec3 movement0_sum = vec3(0.0f, 0.0f, 0.0f);
  vec3 movement1_sum = vec3(0.0f, 0.0f, 0.0f);

  float sum = 0.f;
  float bundle_alpha_sum = 0.0f;
  vec3 segment0_center_position = (segment0_particle0.x + segment0_particle1.x) * 0.5f;

  [[unroll]] for (uint i = 0; i < BUNDLE_MAX_CONNECTION; i++) {
    int pair_handle = segment_data.pair_handles[i];
    if (pair_handle < 0)
      continue;
    SegmentPair segment_pair = segment_pairs[pair_handle];
    if (segment_pair.bend_twist_bundle_integrity <= 0.f)
      continue;
    bool is_segment0 = segment_handle == segment_pair.segment0_handle;
    Segment segment1 = segments[is_segment0 ? segment_pair.segment1_handle : segment_pair.segment0_handle];

    Particle segment1_particle0 = segment1.particle0;
    Particle segment1_particle1 = segment1.particle1;

    vec3 segment1_center_position = (segment1_particle0.x + segment1_particle1.x) * 0.5f;

    vec3 target_segment0_center_position =
        segment1_center_position +
        rotate_vec3(segment1.q, is_segment0 ? segment_pair.segment0_offset.xyz : segment_pair.segment1_offset.xyz);

    float t2 = inv_time_step * inv_time_step;
    // bundle_alpha_sum += t2 * segment_pair.bundle_alpha;
    float lambda = max(1e-9f, segment0.inv_mass + segment1.inv_mass);
    // Always enforce inf stiffness.
    float factor0 = segment0.inv_mass / lambda;

    float segment_length = distance(segment0_particle0.x, segment0_particle1.x);
    vec3 front_direction = normalize(rotate_vec3(segment0.q, vec3(0, 0, -1)));
    vec3 target_segment0_particle0_position = target_segment0_center_position - front_direction * segment_length * .5f;
    vec3 target_segment0_particle1_position = target_segment0_center_position + front_direction * segment_length * .5f;

    vec3 segment0_particle0_position_correction = (target_segment0_particle0_position - segment0_particle0.x) * factor0;
    vec3 segment0_particle1_position_correction = (target_segment0_particle1_position - segment0_particle1.x) * factor0;

    sum += 1.0f;
    movement0_sum += segment0_particle0_position_correction;
    movement1_sum += segment0_particle1_position_correction;
  }

  if (sum != 0) {
    float bundle_alpha = bundle_alpha_sum / sum;
    segment_data_list[segment_handle].particle0_position_correction.xyz = movement0_sum / sum / (1.0f + bundle_alpha);
    segment_data_list[segment_handle].particle1_position_correction.xyz = movement1_sum / sum / (1.0f + bundle_alpha);
  } else {
    segment_data_list[segment_handle].particle0_position_correction.xyz = vec3(0.0f, 0.0f, 0.0f);
    segment_data_list[segment_handle].particle1_position_correction.xyz = vec3(0.0f, 0.0f, 0.0f);
  }
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
  if (dot_product < 0) {
    angle += (sine >= 0 ? 3.1415926f : -3.1415926f);
  }

  if (dot(cross_product, axis) < 0) {
    angle = -angle;
  }
  float half_angle = angle * .5f;
  float sin_angle = sin(half_angle);
  return vec4(axis * sin_angle, cos(half_angle));
}

void BundleSegmentRotation(in uint segment_handle, in float inv_time_step, in float over_relaxation) {
  Segment segment0 = segments[segment_handle];
  Particle segment0_particle0 = segment0.particle0;
  Particle segment0_particle1 = segment0.particle1;

  SegmentData segment_data = segment_data_list[segment_handle];

  float rotation_sum = 0.f;
  vec4 q_correction_sum = vec4(0.0f, 0.0f, 0.0f, 0.0f);

  vec3 segment0_center_position = (segment0_particle0.x + segment0_particle1.x) * 0.5f;
  [[unroll]] for (uint i = 0; i < BUNDLE_MAX_CONNECTION; i++) {
    int pair_handle = segment_data.pair_handles[i];
    if (pair_handle < 0)
      continue;
    SegmentPair segment_pair = segment_pairs[pair_handle];
    if (segment_pair.bend_twist_bundle_integrity <= 0.f)
      continue;
    bool is_segment0 = segment_handle == segment_pair.segment0_handle;
    Segment segment1 = segments[is_segment0 ? segment_pair.segment1_handle : segment_pair.segment0_handle];

    Particle segment1_particle0 = segment1.particle0;
    Particle segment1_particle1 = segment1.particle1;

    vec3 segment1_center_position = (segment1_particle0.x + segment1_particle1.x) * 0.5f;

    float t2 = inv_time_step * inv_time_step;
    // bundle_alpha_sum += t2 * segment_pair.bundle_alpha;
    float factor = max(1e-9f, segment0.inv_mass + segment1.inv_mass);

    if (segment0.strand_handle != segment1.strand_handle) {
      vec3 current_offset = rotate_vec3(conjugate(segment0.q), segment1_center_position - segment0_center_position);
      vec3 expected_offset = is_segment0 ? segment_pair.segment1_offset.xyz : segment_pair.segment0_offset.xyz;
      vec4 expected_rotation = quat_mul(
          segment0.q, compute_rotation_between(normalize(expected_offset), normalize(current_offset)));

      vec4 lambda = segment0.q;
      vec4 lambda_plus = lambda + expected_rotation;
      lambda -= expected_rotation;
      if (squared_norm(lambda) > squared_norm(lambda_plus))
        lambda = lambda_plus;

      lambda.x /= factor;
      lambda.y /= factor;
      lambda.z /= factor;
      lambda.w = 0.0f;
      vec4 rotation_correction = quat_mul(segment0.q, lambda) * segment0.inv_mass;
      q_correction_sum += expected_rotation;
      rotation_sum += 1.0f;
    }
  }

  if (rotation_sum != 0) {
    segment_data_list[segment_handle].q_correction = q_correction_sum / rotation_sum;
  } else {
    segment_data_list[segment_handle].q_correction = vec4(0.0f, 0.0f, 0.0f, 0.0f);
  }
}

void BundleSegmentBendTwist(in uint segment_handle, in float inv_time_step, in float over_relaxation) {
  SegmentData segment_data = segment_data_list[segment_handle];
  vec4 q_correction_sum = vec4(0.0f, 0.0f, 0.0f, 0.0f);
  // vec3 alpha_sum;
  float sum = 0.f;
  [[unroll]] for (uint i = 0; i < BUNDLE_MAX_CONNECTION; i++) {
    int pair_handle = segment_data.pair_handles[i];
    if (pair_handle < 0)
      continue;
    SegmentPair segment_pair = segment_pairs[pair_handle];
    if (segment_pair.bend_twist_bundle_integrity <= 0.f)
      continue;
    bool is_segment0 = segment_handle == segment_pair.segment0_handle;
    vec4 q0_correction, q1_correction;
    vec3 bend_twist_alpha = vec3(0.0f, 0.0f, 0.0f);
    // alpha_sum += vec3(segment_pair.bending_alpha, segment_pair.bending_alpha, segment_pair.twisting_alpha);
    project_bend_twist_constraint(
        inv_time_step, segments[segment_pair.segment0_handle].q, segments[segment_pair.segment0_handle].inv_mass,
        segments[segment_pair.segment1_handle].q, segments[segment_pair.segment1_handle].inv_mass, bend_twist_alpha,
        segment_pair.rest_darboux_vector, q0_correction, q1_correction);

    vec4 q_correction = is_segment0 ? q0_correction : q1_correction;
    q_correction_sum += q_correction;
    sum += 1.0f;
  }

  if (sum != 0) {
    vec4 q_correction = q_correction_sum / sum;
    // vec3 alpha = alpha_sum / sum;
    // float t2 = inv_time_step * inv_time_step;
    // vec3 alpha_factor = vec3(t2 * alpha.x, t2 * alpha.y, t2 * alpha.z);
    // q_correction.x /= (1.f + alpha_factor.x);
    // q_correction.y /= (1.f + alpha_factor.y);
    // q_correction.z /= (1.f + alpha_factor.z);
    segment_data_list[segment_handle].q_correction = q_correction;
  } else {
    segment_data_list[segment_handle].q_correction = vec4(0.0f, 0.0f, 0.0f, 0.0f);
  }
}

void BundleSegmentShearStretch(in uint segment_handle, in float inv_time_step) {
  vec3 x0_correction, x1_correction;
  vec4 q_correction;

  Segment segment = segments[segment_handle];
  Particle particle0 = segment.particle0;
  Particle particle1 = segment.particle1;

  vec3 p0 = particle0.x;
  vec3 p1 = particle1.x;

  float inv_mass_q = segment.inv_mass;
  float inv_mass_p0 = inv_mass_q;
  float inv_mass_p1 = inv_mass_q;
  if (segment.prev_handle != -1) {
    inv_mass_p0 = segments[segment.prev_handle].inv_mass;
  }
  if (segment.next_handle != -1) {
    inv_mass_p1 = segments[segment.next_handle].inv_mass;
  }
  vec4 q = segment.q;

  vec3 alpha = vec3(segment.shearing_alpha, segment.shearing_alpha, segment.stretching_alpha);

  // vec3 alpha = vec3(1.0, 1.0, 0.0);

  float rest_length = segment.rest_length;

  project_shear_stretch_constraint(inv_time_step, p0, p1, q, inv_mass_p0, inv_mass_p1, inv_mass_q, alpha, rest_length,
                                   x0_correction, x1_correction, q_correction);

  segment_data_list[segment_handle].particle0_position_correction.xyz = x0_correction;
  segment_data_list[segment_handle].particle1_position_correction.xyz = x1_correction;
  segment_data_list[segment_handle].q_correction = q_correction;
}