#define _MATH_GLSL_

public float4x4 translate(in float3 d) {
  return float4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, d.x, d.y, d.z, 1);
}
public float4x4 scale(in float3 s) {
  return float4x4(s.x, 0, 0, 0, 0, s.y, 0, 0, 0, 0, s.z, 0, 0, 0, 0, 1);
}

public float4x4 mat4_cast(in float4 q) {
  float qxx = q.x * q.x;
  float qyy = q.y * q.y;
  float qzz = q.z * q.z;
  float qxz = q.x * q.z;
  float qxy = q.x * q.y;
  float qyz = q.y * q.z;
  float qwx = q.w * q.x;
  float qwy = q.w * q.y;
  float qwz = q.w * q.z;
  float4x4 ret_val = float4x4(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1);
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

public float4x4 mat4_cast(in float3x3 m) {
  return float4x4(m[0][0], m[0][1], m[0][2], 0, m[1][0], m[1][1], m[1][2], 0, m[2][0], m[2][1], m[2][2], 0, 0, 0, 0, 0);
}

public float3x3 mat3_cast(in float4 q) {
  float qxx = q.x * q.x;
  float qyy = q.y * q.y;
  float qzz = q.z * q.z;
  float qxz = q.x * q.z;
  float qxy = q.x * q.y;
  float qyz = q.y * q.z;
  float qwx = q.w * q.x;
  float qwy = q.w * q.y;
  float qwz = q.w * q.z;
  float3x3 ret_val;
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

public float4 quat_cast(in float3x3 m) {
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
      return float4((m[1][2] - m[2][1]) * mult, (m[2][0] - m[0][2]) * mult, (m[0][1] - m[1][0]) * mult, biggestVal);
    case 1:
      return float4(biggestVal, (m[0][1] + m[1][0]) * mult, (m[2][0] + m[0][2]) * mult, (m[1][2] - m[2][1]) * mult);
    case 2:
      return float4((m[0][1] + m[1][0]) * mult, biggestVal, (m[1][2] + m[2][1]) * mult, (m[2][0] - m[0][2]) * mult);
    case 3:
      return float4((m[2][0] + m[0][2]) * mult, (m[1][2] + m[2][1]) * mult, biggestVal, (m[0][1] - m[1][0]) * mult);
    default:
      return float4(0, 0, 0, 1);
  }
}

public float4 quatLookAt(in float3 direction, in float3 up) {
  float3x3 Result;

  Result[2] = -direction;
  float3 Right = cross(up, Result[2]);
  Result[0] = Right * rsqrt(max(0.00001, dot(Right, Right)));
  Result[1] = cross(Result[2], Result[0]);

  return quat_cast(Result);
}

public float4 conjugate(in float4 q) {
  return float4(-q.x, -q.y, -q.z, q.w);
}

public float4 quat_mul(in float4 a, in float4 b) {
  return float4(a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y + a.y * b.w + a.z * b.x - a.x * b.z,
              a.w * b.z + a.z * b.w + a.x * b.y - a.y * b.x, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z);
}

public float3 compute_darboux_vector(in float4 q0, in float4 q1, float length) {
  return 2.0 / length * quat_mul(conjugate(q0), q1).xyz;  // Eq. 27.5
}

public float squared_norm(in float4 q) {
  return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
}

public float3 rotate_vec3(in float4 q, in float3 v) {
  float3 q_vec = q.xyz;
  float3 t = cross(q_vec, v) * 2.0;
  return v + t * q.w + cross(q_vec, t);
}

public float4 angle_axis(in float angle, in float3 axis) {
  float s = sin(angle * .5f);
  return float4(axis.xyz * s, cos(angle * .5f));
}

// From https://www.adriancourreges.com/blog/2018/12/02/ue4-optimized-post-effects/#screen-space-ambient-occlusion

#define PI_OVER_2 1.5707963f
#define PI_OVER_4 0.785398f

// Maps a unit square in [-1, 1] to a unit disk in [-1, 1]. Shirley 97 "A Low Distortion Map Between Disk and Square"
// Inputs: cartesian coordinates
// Return: new circle-mapped polar coordinates (radius, angle)
public float2 UnitSquareToUnitDiskPolar(float a, float b) {
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
  return float2(radius, angle);
}

// Maps a unit square in [-1, 1] to a unit disk in [-1, 1]
// Inputs: cartesian coordinates
// Return: new circle-mapped cartesian coordinates
public float2 SquareToDiskMapping(float a, float b) {
  float2 polar_coord = UnitSquareToUnitDiskPolar(a, b);
  return float2(polar_coord.x * cos(polar_coord.y), polar_coord.x * sin(polar_coord.y));
}

// Remap a unit square in [0, 1] to a polygon in [-1, 1] with <edgeCount> edges rotated by <shapeRotation> radians
// Inputs: cartesian coordinates
// Return: new polygon-mapped cartesian coordinates
public float2 SquareToPolygonMapping(float2 uv, float edgeCount, float shapeRotation) {
  float2 polar_coord = UnitSquareToUnitDiskPolar(uv.x, uv.y);  // (radius, angle)

  // Re-scale radius to match a polygon shape
  polar_coord.x *=
      cos(3.1415926f /
          edgeCount) /  //----------------------------------------------------------------------------------------------
      cos(polar_coord.y -
          (2.0f * 3.1415926f / edgeCount) * floor((edgeCount * polar_coord.y + 3.1415926f) / 2.0f / 3.1415926f));

  // Apply a rotation to the polygon shape
  polar_coord.y += shapeRotation;

  return float2(polar_coord.x * cos(polar_coord.y), polar_coord.x * sin(polar_coord.y));
}

public void CalculateAlignUpDirectionRotation(in float4 q, in float3 direction, out float angle, out float3 axis) {
  float3 up = normalize(rotate_vec3(q, float3(0, 1, 0)));
  float dot_product = dot(up, direction);
  bool valid = dot_product <= 1.f - 1e-6f;
  axis = valid ? normalize(cross(up, direction)) : direction;
  angle = valid ? acos(dot_product) : 0.f;
}

public void CalculateAlignDownDirectionRotation(in float4 q, in float3 direction, out float angle, out float3 axis) {
  float3 down = normalize(rotate_vec3(q, float3(0, -1, 0)));
  float dot_product = dot(down, direction);
  bool valid = dot_product <= 1.f - 1e-6f;
  axis = valid ? normalize(cross(down, direction)) : direction;
  angle = valid ? acos(dot_product) : 0.f;
}

public void CalculateAlignFrontDirectionRotation(in float4 q, in float3 direction, out float angle, out float3 axis) {
  float3 front = normalize(rotate_vec3(q, float3(0, 0, -1)));
  float dot_product = dot(front, direction);
  bool valid = dot_product <= 1.f - 1e-6f;
  axis = valid ? normalize(cross(front, direction)) : direction;
  angle = valid ? acos(dot_product) : 0.f;
}

public void CalculateAlignRightDirectionRotation(in float4 q, in float3 direction, out float angle, out float3 axis) {
  float3 right = normalize(rotate_vec3(q, float3(1, 0, 0)));
  float dot_product = dot(right, direction);
  bool valid = dot_product <= 1.f - 1e-6f;
  axis = valid ? normalize(cross(right, direction)) : direction;
  angle = valid ? acos(dot_product) : 0.f;
}

public float DistSquared(in float3 A, in float3 B) {
  float3 C = A - B;
  return dot(C, C);
}

public float3 scaleAlong(in float3 v, in float3 d, in float k) {
  if (abs(k-1.0) < 0.0001)
  {
        return v;
  }
  float3 v_dir = dot(v, d) * d;
  float3 v_perp = v - v_dir;
  return v_perp + k * v_dir;
}

public float3 scaleAlongDirectional(in float3 v, in float3 d, in float k) {
  if (dot(v, d) < 0)
  {
    return v;
  }
  float3 v_dir = dot(v, d) * d;
  float3 v_perp = v - v_dir;
  return v_perp + k * v_dir;
}

public float3 ProjectOntoPlane(in float3 vector, in float3 normalizedPlaneNormal) {
  // Compute the dot product of the vector and the plane normal
  float dotProduct = dot(vector, normalizedPlaneNormal);

  // Subtract the component of the vector that is parallel to the plane normal
  return vector - dotProduct * normalizedPlaneNormal;
}

public float rand01(uint seed) {
  seed ^= seed >> 16;
  seed *= 0x7feb352d;
  seed ^= seed >> 15;
  seed *= 0x846ca68b;
  seed ^= seed >> 16;
  return float(seed) * (1.0 / 4294967296.0);
}

public float rand_ab(uint seed, float a, float b) {
  return lerp(a, b, rand01(seed));
}
