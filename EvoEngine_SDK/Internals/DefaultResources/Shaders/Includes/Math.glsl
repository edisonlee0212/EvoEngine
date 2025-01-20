
#ifndef _MATH_GLSL_
#define _MATH_GLSL_

mat4 translate(in vec3 d) {
  return mat4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, d.x, d.y, d.z, 1);
}

mat4 scale(in vec3 s) {
  return mat4(s.x, 0, 0, 0, 0, s.y, 0, 0, 0, 0, s.z, 0, 0, 0, 0, 1);
}

mat4 mat4_cast(in vec4 q) {
  float qxx = q.x * q.x;
  float qyy = q.y * q.y;
  float qzz = q.z * q.z;
  float qxz = q.x * q.z;
  float qxy = q.x * q.y;
  float qyz = q.y * q.z;
  float qwx = q.w * q.x;
  float qwy = q.w * q.y;
  float qwz = q.w * q.z;
  mat4 ret_val = mat4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
  ret_val[0][0] = 1.0 - 2.0 * (qyy + qzz);
  ret_val[0][1] = 2.0 * (qxy + qwz);
  ret_val[0][2] = 2.0 * (qxz - qwy);

  ret_val[1][0] = 2.0 * (qxy - qwz);
  ret_val[1][1] = 1.0 - 2.0 * (qxx + qzz);
  ret_val[1][2] = 2.0 * (qyz + qwx);

  ret_val[2][0] = 2.0 * (qxz + qwy);
  ret_val[2][1] = 2.0 * (qyz - qwx);
  ret_val[2][2] = 1.0 - 2.0 * (qxx + qyy);

  return ret_val;
}

mat4 mat4_cast(in mat3 m) {
  return mat4(m[0][0], m[0][1], m[0][2], 0, m[1][0], m[1][1], m[1][2], 0, m[2][0], m[2][1], m[2][2], 0, 0, 0, 0, 0);
}

mat3 mat3_cast(in vec4 q) {
  float qxx = q.x * q.x;
  float qyy = q.y * q.y;
  float qzz = q.z * q.z;
  float qxz = q.x * q.z;
  float qxy = q.x * q.y;
  float qyz = q.y * q.z;
  float qwx = q.w * q.x;
  float qwy = q.w * q.y;
  float qwz = q.w * q.z;
  mat3 ret_val;
  ret_val[0][0] = 1.0 - 2.0 * (qyy + qzz);
  ret_val[0][1] = 2.0 * (qxy + qwz);
  ret_val[0][2] = 2.0 * (qxz - qwy);

  ret_val[1][0] = 2.0 * (qxy - qwz);
  ret_val[1][1] = 1.0 - 2.0 * (qxx + qzz);
  ret_val[1][2] = 2.0 * (qyz + qwx);

  ret_val[2][0] = 2.0 * (qxz + qwy);
  ret_val[2][1] = 2.0 * (qyz - qwx);
  ret_val[2][2] = 1.0 - 2.0 * (qxx + qyy);

  return ret_val;
}

vec4 quat_cast(in mat3 m) {
  float fourXSquaredMinus1 = m[0][0] - m[1][1] - m[2][2];
  float fourYSquaredMinus1 = m[1][1] - m[0][0] - m[2][2];
  float fourZSquaredMinus1 = m[2][2] - m[0][0] - m[1][1];
  float fourWSquaredMinus1 = m[0][0] + m[1][1] + m[2][2];

  int biggestIndex = 0;
  float fourBiggestSquaredMinus1 = fourWSquaredMinus1;
  if (fourXSquaredMinus1 > fourBiggestSquaredMinus1) {
    fourBiggestSquaredMinus1 = fourXSquaredMinus1;
    biggestIndex = 1;
  }
  if (fourYSquaredMinus1 > fourBiggestSquaredMinus1) {
    fourBiggestSquaredMinus1 = fourYSquaredMinus1;
    biggestIndex = 2;
  }
  if (fourZSquaredMinus1 > fourBiggestSquaredMinus1) {
    fourBiggestSquaredMinus1 = fourZSquaredMinus1;
    biggestIndex = 3;
  }

  float biggestVal = sqrt(fourBiggestSquaredMinus1 + 1.0) * 0.5;
  float mult = 0.25 / biggestVal;

  switch (biggestIndex) {
    case 0:
      return vec4((m[1][2] - m[2][1]) * mult, (m[2][0] - m[0][2]) * mult, (m[0][1] - m[1][0]) * mult, biggestVal);
    case 1:
      return vec4(biggestVal, (m[0][1] + m[1][0]) * mult, (m[2][0] + m[0][2]) * mult, (m[1][2] - m[2][1]) * mult);
    case 2:
      return vec4((m[0][1] + m[1][0]) * mult, biggestVal, (m[1][2] + m[2][1]) * mult, (m[2][0] - m[0][2]) * mult);
    case 3:
      return vec4((m[2][0] + m[0][2]) * mult, (m[1][2] + m[2][1]) * mult, biggestVal, (m[0][1] - m[1][0]) * mult);
    default:
      return vec4(0, 0, 0, 1);
  }
}

vec4 quatLookAt(in vec3 direction, in vec3 up) {
  mat3 Result;

  Result[2] = -direction;
  vec3 Right = cross(up, Result[2]);
  Result[0] = Right * inversesqrt(max(0.00001, dot(Right, Right)));
  Result[1] = cross(Result[2], Result[0]);

  return quat_cast(Result);
}

vec4 conjugate(in vec4 q) {
  return vec4(-q.x, -q.y, -q.z, q.w);
}

vec4 quat_mul(in vec4 a, in vec4 b) {
  return vec4(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
              a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

vec3 compute_darboux_vector(in vec4 q0, in vec4 q1, float length) {
  return 2.0 / length * quat_mul(conjugate(q0), q1).xyz;  // Eq. 27.5
}

float squared_norm(in vec4 q) {
  return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

vec3 rotate_vec3(in vec4 q, in vec3 v) {
  vec3 q_vec = q.xyz;
  vec3 t = cross(q_vec, v) * 2.0;
  return v + t * q.w + cross(q_vec, t);
}

vec4 angle_axis(in float angle, in vec3 axis) {
  float s = sin(angle * .5f);
  return vec4(axis.xyz * s, cos(angle * .5f));
}

// From https://www.adriancourreges.com/blog/2018/12/02/ue4-optimized-post-effects/#screen-space-ambient-occlusion

#define PI_OVER_2 1.5707963f
#define PI_OVER_4 0.785398f

// Maps a unit square in [-1, 1] to a unit disk in [-1, 1]. Shirley 97 "A Low Distortion Map Between Disk and Square"
// Inputs: cartesian coordinates
// Return: new circle-mapped polar coordinates (radius, angle)
vec2 UnitSquareToUnitDiskPolar(float a, float b) {
  float radius, angle;
  if (abs(a) > abs(b)) {  // First region (left and right quadrants of the disk)
    radius = a;
    angle = b / (a + 1e-6f) * 0.785398f;
  } else {  // Second region (top and bottom quadrants of the disk)
    radius = b;
    angle = PI_OVER_2 - (a / (b + 1e-6f) * 0.785398f);
  }
  if (radius < 0) {  // Always keep radius positive
    radius *= -1.0f;
    angle += 3.1415926f;
  }
  return vec2(radius, angle);
}

// Maps a unit square in [-1, 1] to a unit disk in [-1, 1]
// Inputs: cartesian coordinates
// Return: new circle-mapped cartesian coordinates
vec2 SquareToDiskMapping(float a, float b) {
  vec2 polar_coord = UnitSquareToUnitDiskPolar(a, b);
  return vec2(polar_coord.x * cos(polar_coord.y), polar_coord.x * sin(polar_coord.y));
}

// Remap a unit square in [0, 1] to a polygon in [-1, 1] with <edgeCount> edges rotated by <shapeRotation> radians
// Inputs: cartesian coordinates
// Return: new polygon-mapped cartesian coordinates
vec2 SquareToPolygonMapping(vec2 uv, float edgeCount, float shapeRotation) {
  vec2 polar_coord = UnitSquareToUnitDiskPolar(uv.x, uv.y);  // (radius, angle)

  // Re-scale radius to match a polygon shape
  polar_coord.x *=
      cos(3.1415926f /
          edgeCount) /  //----------------------------------------------------------------------------------------------
      cos(polar_coord.y -
          (2.0f * 3.1415926f / edgeCount) * floor((edgeCount * polar_coord.y + 3.1415926f) / 2.0f / 3.1415926f));

  // Apply a rotation to the polygon shape
  polar_coord.y += shapeRotation;

  return vec2(polar_coord.x * cos(polar_coord.y), polar_coord.x * sin(polar_coord.y));
}

void CalculateAlignUpDirectionRotation(in vec4 q, in vec3 direction, out float angle, out vec3 axis) {
  vec3 up = normalize(rotate_vec3(q, vec3(0, 1, 0)));
  float dot_product = dot(up, direction);
  bool valid = dot_product <= 1.f - 1e-6f;
  axis = valid ? normalize(cross(up, direction)) : direction;
  angle = valid ? acos(dot_product) : 0.f;
}

void CalculateAlignDownDirectionRotation(in vec4 q, in vec3 direction, out float angle, out vec3 axis) {
  vec3 down = normalize(rotate_vec3(q, vec3(0, -1, 0)));
  float dot_product = dot(down, direction);
  bool valid = dot_product <= 1.f - 1e-6f;
  axis = valid ? normalize(cross(down, direction)) : direction;
  angle = valid ? acos(dot_product) : 0.f;
}

void CalculateAlignFrontDirectionRotation(in vec4 q, in vec3 direction, out float angle, out vec3 axis) {
  vec3 front = normalize(rotate_vec3(q, vec3(0, 0, -1)));
  float dot_product = dot(front, direction);
  bool valid = dot_product <= 1.f - 1e-6f;
  axis = valid ? normalize(cross(front, direction)) : direction;
  angle = valid ? acos(dot_product) : 0.f;
}

void CalculateAlignRightDirectionRotation(in vec4 q, in vec3 direction, out float angle, out vec3 axis) {
  vec3 right = normalize(rotate_vec3(q, vec3(1, 0, 0)));
  float dot_product = dot(right, direction);
  bool valid = dot_product <= 1.f - 1e-6f;
  axis = valid ? normalize(cross(right, direction)) : direction;
  angle = valid ? acos(dot_product) : 0.f;
}

float DistSquared(in vec3 A, in vec3 B) {
  vec3 C = A - B;
  return dot(C, C);
}

vec3 ProjectOntoPlane(in vec3 vector, in vec3 normalizedPlaneNormal) {
  // Compute the dot product of the vector and the plane normal
  float dotProduct = dot(vector, normalizedPlaneNormal);

  // Subtract the component of the vector that is parallel to the plane normal
  return vector - dotProduct * normalizedPlaneNormal;
}

#endif  // _MATH_GLSL_
