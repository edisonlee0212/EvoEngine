import EvoEngine.GltfRayTracingMath;
import EvoEngine.CameraRayGeometry;

[[vk::binding(0, 0)]]
RWStructuredBuffer<float> values;

[numthreads(1, 1, 1)] void main() {
  const float f0 = EE_GLTF_RT_IOR_TO_F0(1.0f, 1.5f);
  const float3 fresnel_normal = EE_GLTF_RT_FRESNEL_SCHLICK(1.0f, float3(f0));
  const float3 fresnel_grazing = EE_GLTF_RT_FRESNEL_SCHLICK(0.0f, float3(f0));
  const float3 sample_direction = EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(float2(0.25f, 0.5f));
  const float pdf = EE_GLTF_RT_COSINE_HEMISPHERE_PDF(float3(0.0f, 0.0f, 1.0f), sample_direction);
  const float3 weighted_f0 = min(float3(f0) * float3(2.0f, 0.5f, 1.0f), float3(1.0f)) * 0.5f;
  const float3 weighted_normal = EE_GLTF_RT_FRESNEL_SCHLICK(1.0f, weighted_f0, float3(0.5f));
  const float3 weighted_grazing = EE_GLTF_RT_FRESNEL_SCHLICK(0.0f, weighted_f0, float3(0.5f));
  float3 scaled_clearcoat_normal = float3(0.6f, -0.6f, 1.0f);
  scaled_clearcoat_normal.xy *= 0.5f;
  scaled_clearcoat_normal = EE_GLTF_RT_SAFE_NORMALIZE(scaled_clearcoat_normal, float3(0.0f, 0.0f, 1.0f));
  const float coated_normal_emission = 2.0f * (1.0f - EE_GLTF_RT_FRESNEL_SCHLICK(1.0f, float3(0.04f)).x);
  const float coated_grazing_emission = 2.0f * (1.0f - EE_GLTF_RT_FRESNEL_SCHLICK(0.0f, float3(0.04f)).x);
  const float3 normal_fallback = normalize(float3(1.0f, 2.0f, 3.0f));
  const float3 tiny_face_normal =
      EE_CAMERA_GEOMETRIC_NORMAL(float3(0.00001f, 0.0f, 0.0f), float3(0.0f, 0.00001f, 0.0f), normal_fallback);
  const float3 large_face_normal =
      EE_CAMERA_GEOMETRIC_NORMAL(float3(100000.0f, 0.0f, 0.0f), float3(0.0f, 100000.0f, 0.0f), normal_fallback);
  const float3 minimum_scale_face_normal =
      EE_CAMERA_GEOMETRIC_NORMAL(float3(1.0e-30f, 0.0f, 0.0f), float3(0.0f, 1.0e-30f, 0.0f), normal_fallback);
  const float3 maximum_scale_face_normal =
      EE_CAMERA_GEOMETRIC_NORMAL(float3(1.0e30f, 0.0f, 0.0f), float3(0.0f, 1.0e30f, 0.0f), normal_fallback);
  const float3 degenerate_face_normal =
      EE_CAMERA_GEOMETRIC_NORMAL(float3(1.0f, 0.0f, 0.0f), float3(2.0f, 0.0f, 0.0f), normal_fallback);
  const float3 invalid_face_normal =
      EE_CAMERA_SCALE_INDEPENDENT_NORMALIZE(float3(asfloat(0x7f800000u), 0.0f, 0.0f), normal_fallback);

  values[0] = f0;
  values[1] = fresnel_normal.x;
  values[2] = fresnel_grazing.x;
  values[3] = length(sample_direction);
  values[4] = sample_direction.z;
  values[5] = pdf;
  values[6] = sample_direction.z / EE_GLTF_RT_MATH_PI;
  values[7] = EE_GLTF_RT_F0_TO_IOR(f0);
  values[8] = !any(isnan(sample_direction)) && !any(isinf(sample_direction)) ? 1.0f : 0.0f;
  values[9] = weighted_normal.x;
  values[10] = weighted_normal.y;
  values[11] = weighted_normal.z;
  values[12] = weighted_grazing.x;
  values[13] = max(weighted_grazing.y, weighted_grazing.z);
  values[14] = scaled_clearcoat_normal.x;
  values[15] = scaled_clearcoat_normal.y;
  values[16] = length(scaled_clearcoat_normal);
  values[17] = coated_normal_emission;
  values[18] = coated_grazing_emission;
  values[19] = all(weighted_normal >= float3(0.0f)) && all(weighted_normal <= float3(0.5f)) &&
                       !any(isnan(weighted_grazing)) && !any(isinf(weighted_grazing))
                   ? 1.0f
                   : 0.0f;
  values[20] = tiny_face_normal.x;
  values[21] = tiny_face_normal.y;
  values[22] = tiny_face_normal.z;
  values[23] = large_face_normal.x;
  values[24] = large_face_normal.y;
  values[25] = large_face_normal.z;
  values[26] = degenerate_face_normal.x;
  values[27] = degenerate_face_normal.y;
  values[28] = degenerate_face_normal.z;
  values[29] = invalid_face_normal.x;
  values[30] = invalid_face_normal.y;
  values[31] = invalid_face_normal.z;
  values[32] = minimum_scale_face_normal.x;
  values[33] = minimum_scale_face_normal.y;
  values[34] = minimum_scale_face_normal.z;
  values[35] = maximum_scale_face_normal.x;
  values[36] = maximum_scale_face_normal.y;
  values[37] = maximum_scale_face_normal.z;
}
