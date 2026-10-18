import EvoEngine.GltfRayTracingBsdf;
import EvoEngine.Random;

public float2 EE_SSR_RANDOM(uint2 pixel, uint frame_index, uint camera_index) {
  uint seed = EE_XXHASH32(uint3(pixel, frame_index ^ (camera_index * 747796405u)));
  return EE_PCG_RANDOM_2(seed);
}

public float3 EE_SSR_SAMPLE_GGX(float3 view, float3 normal, float roughness, float2 xi,
                               float3 f0, float f90, out float3 weight) {
  weight = float3(0.0f);
  float n_dot_v = dot(normal, view);
  if (n_dot_v <= 0.0f) return float3(0.0f);
  if (roughness <= 0.01f) {
    weight = EE_GLTF_RT_FRESNEL_SCHLICK(n_dot_v, f0, float3(f90));
    return reflect(-view, normal);
  }
  float3 tangent = normalize(cross(abs(normal.z) < 0.999f ? float3(0, 0, 1) : float3(0, 1, 0), normal));
  float3 bitangent = cross(normal, tangent);
  float3 local_view = float3(dot(view, tangent), dot(view, bitangent), n_dot_v);
  float2 alpha = float2(max(roughness * roughness, EE_GLTF_RT_BSDF_MIN_ROUGHNESS));
  float3 half_vector = EE_GLTF_RT_HVD_GGX_SAMPLE_VNDF(local_view, alpha, xi);
  float3 local_light = reflect(-local_view, half_vector);
  if (local_light.z <= 0.0f) return float3(0.0f);
  // With visible-normal sampling, BRDF * NdotL / PDF reduces to Fresnel * G1(light).
  weight = EE_GLTF_RT_FRESNEL_SCHLICK(dot(local_view, half_vector), f0, float3(f90)) *
           EE_GLTF_RT_SMITH_SHADOW_OR_MASK(local_light, alpha);
  return normalize(tangent * local_light.x + bitangent * local_light.y + normal * local_light.z);
}

public float4 EE_SSR_WEIGHTED_CORRECTION(float3 radiance, float3 probe_radiance, float3 weight, float confidence) {
  return float4((radiance - probe_radiance) * weight * confidence, confidence);
}
