#ifndef EE_GLTF_RAY_TRACING_MATH_GLSL
#define EE_GLTF_RAY_TRACING_MATH_GLSL

const float EE_GLTF_RT_MATH_PI = 3.14159265359f;

vec3 EE_GLTF_RT_SAFE_NORMALIZE(const vec3 value, const vec3 fallback) {
  const float length_squared = dot(value, value);
  return length_squared > 1e-8f ? value * inversesqrt(length_squared) : fallback;
}

vec3 EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(const vec2 xi) {
  const float r = sqrt(clamp(xi.x, 0.0f, 1.0f));
  const float phi = 2.0f * EE_GLTF_RT_MATH_PI * xi.y;
  return vec3(cos(phi) * r, sin(phi) * r, sqrt(max(0.0f, 1.0f - r * r)));
}

float EE_GLTF_RT_COSINE_HEMISPHERE_PDF(const vec3 normal, const vec3 direction) {
  return max(dot(EE_GLTF_RT_SAFE_NORMALIZE(normal, vec3(0.0f, 1.0f, 0.0f)),
                 EE_GLTF_RT_SAFE_NORMALIZE(direction, vec3(0.0f, 1.0f, 0.0f))),
             0.0f) /
         EE_GLTF_RT_MATH_PI;
}

vec3 EE_GLTF_RT_FRESNEL_SCHLICK(const float cos_theta, const vec3 f0) {
  return f0 + (vec3(1.0f) - f0) * pow(max(1.0f - cos_theta, 0.0f), 5.0f);
}

float EE_GLTF_RT_IOR_TO_F0(const float incident_ior, const float transmitted_ior) {
  const float ratio = (transmitted_ior - incident_ior) / max(transmitted_ior + incident_ior, 0.000001f);
  return ratio * ratio;
}

float EE_GLTF_RT_F0_TO_IOR(const float f0) {
  const float root = sqrt(clamp(f0, 0.0f, 0.999999f));
  return (1.0f + root) / max(1.0f - root, 0.000001f);
}

#endif
