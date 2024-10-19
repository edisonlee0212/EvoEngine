

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