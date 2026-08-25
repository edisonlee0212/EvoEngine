public static const float EE_GLTF_RT_MATH_PI = 3.14159265359f;

public float3 EE_GLTF_RT_SAFE_NORMALIZE(const float3 value, const float3 fallback) {
  const float length_squared = dot(value, value);
  return length_squared > 1e-8f ? value * rsqrt(length_squared) : fallback;
}

public float3 EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(const float2 xi) {
  const float r = sqrt(clamp(xi.x, 0.0f, 1.0f));
  const float phi = 2.0f * EE_GLTF_RT_MATH_PI * xi.y;
  return float3(cos(phi) * r, sin(phi) * r, sqrt(max(0.0f, 1.0f - r * r)));
}

public float EE_GLTF_RT_COSINE_HEMISPHERE_PDF(const float3 normal, const float3 direction) {
  return max(dot(EE_GLTF_RT_SAFE_NORMALIZE(normal, float3(0.0f, 1.0f, 0.0f)),
                 EE_GLTF_RT_SAFE_NORMALIZE(direction, float3(0.0f, 1.0f, 0.0f))),
             0.0f) /
         EE_GLTF_RT_MATH_PI;
}

public float3 EE_GLTF_RT_FRESNEL_SCHLICK(const float cos_theta, const float3 f0, const float3 f90) {
  return f0 + (f90 - f0) * pow(max(1.0f - clamp(cos_theta, 0.0f, 1.0f), 0.0f), 5.0f);
}

public float3 EE_GLTF_RT_FRESNEL_SCHLICK(const float cos_theta, const float3 f0) {
  return EE_GLTF_RT_FRESNEL_SCHLICK(cos_theta, f0, float3(1.0f));
}

public float EE_GLTF_RT_IOR_TO_F0(const float incident_ior, const float transmitted_ior) {
  const float ratio = (transmitted_ior - incident_ior) / max(transmitted_ior + incident_ior, 0.000001f);
  return ratio * ratio;
}

public float EE_GLTF_RT_F0_TO_IOR(const float f0) {
  const float root = sqrt(clamp(f0, 0.0f, 0.999999f));
  return (1.0f + root) / max(1.0f - root, 0.000001f);
}
