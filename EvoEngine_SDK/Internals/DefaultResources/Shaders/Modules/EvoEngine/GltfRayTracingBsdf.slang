__exported import EvoEngine.GltfRayTracingMaterial;
__exported import EvoEngine.GltfRayTracingMath;
public
static const float EE_GLTF_RT_BSDF_PI = 3.14159265359f;
public
static const float EE_GLTF_RT_BSDF_EPSILON = 1e-6f;
public
static const float EE_GLTF_RT_BSDF_MIN_PDF = 0.00001f;
public
static const float EE_GLTF_RT_BSDF_MIN_ROUGHNESS = 0.0014142f;
public
static const float EE_GLTF_RT_BSDF_DIRAC_PDF = -1.0f;
public
static const float EE_GLTF_RT_IOR_INFINITY_SURROGATE = 1000000.0f;
public
static const int EE_GLTF_RT_BSDF_EVENT_ABSORB = 0;
public
static const int EE_GLTF_RT_BSDF_EVENT_DIFFUSE = 1;
public
static const int EE_GLTF_RT_BSDF_EVENT_GLOSSY = 2;
public
static const int EE_GLTF_RT_BSDF_EVENT_IMPULSE = 4;
public
static const int EE_GLTF_RT_BSDF_EVENT_REFLECTION = 8;
public
static const int EE_GLTF_RT_BSDF_EVENT_TRANSMISSION = 16;
public
static const int EE_GLTF_RT_BSDF_EVENT_DIFFUSE_REFLECTION =
    EE_GLTF_RT_BSDF_EVENT_DIFFUSE | EE_GLTF_RT_BSDF_EVENT_REFLECTION;
public
static const int EE_GLTF_RT_BSDF_EVENT_DIFFUSE_TRANSMISSION =
    EE_GLTF_RT_BSDF_EVENT_DIFFUSE | EE_GLTF_RT_BSDF_EVENT_TRANSMISSION;
public
static const int EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION =
    EE_GLTF_RT_BSDF_EVENT_GLOSSY | EE_GLTF_RT_BSDF_EVENT_REFLECTION;
public
static const int EE_GLTF_RT_BSDF_EVENT_GLOSSY_TRANSMISSION =
    EE_GLTF_RT_BSDF_EVENT_GLOSSY | EE_GLTF_RT_BSDF_EVENT_TRANSMISSION;
public
static const int EE_GLTF_RT_BSDF_EVENT_IMPULSE_REFLECTION =
    EE_GLTF_RT_BSDF_EVENT_IMPULSE | EE_GLTF_RT_BSDF_EVENT_REFLECTION;
public
static const int EE_GLTF_RT_BSDF_EVENT_IMPULSE_TRANSMISSION =
    EE_GLTF_RT_BSDF_EVENT_IMPULSE | EE_GLTF_RT_BSDF_EVENT_TRANSMISSION;
public
static const int EE_GLTF_RT_BSDF_LOBE_DIFFUSE_REFLECTION = 0;
public
static const int EE_GLTF_RT_BSDF_LOBE_SPECULAR_TRANSMISSION = 1;
public
static const int EE_GLTF_RT_BSDF_LOBE_SPECULAR_REFLECTION = 2;
public
static const int EE_GLTF_RT_BSDF_LOBE_METAL_REFLECTION = 3;
public
static const int EE_GLTF_RT_BSDF_LOBE_SHEEN_REFLECTION = 4;
public
static const int EE_GLTF_RT_BSDF_LOBE_CLEARCOAT_REFLECTION = 5;
public
static const int EE_GLTF_RT_BSDF_LOBE_DIFFUSE_TRANSMISSION = 6;
public
struct GltfRayTracingPbrMaterial {
 public
  float3 base_color;
 public
  float opacity;
 public
  float2 roughness;
 public
  float metallic;
 public
  float3 emissive;
 public
  float occlusion;
 public
  float3 normal;
 public
  float3 tangent;
 public
  float3 bitangent;
 public
  float3 geometric_normal;
 public
  float ior1;
 public
  float ior2;
 public
  float dispersion;
 public
  float specular;
 public
  float3 specular_color;
 public
  float3 specular_f0;
 public
  float transmission;
 public
  float3 attenuation_color;
 public
  float attenuation_distance;
 public
  float thickness;
 public
  float3 scatter_coefficient;
 public
  float scatter_anisotropy;
 public
  float clearcoat;
 public
  float clearcoat_roughness;
 public
  float3 clearcoat_normal;
 public
  float iridescence;
 public
  float iridescence_ior;
 public
  float iridescence_thickness;
 public
  float3 sheen_color;
 public
  float sheen_roughness;
 public
  float diffuse_transmission_factor;
 public
  float3 diffuse_transmission_color;
 public
  float retroreflection;
};
public
struct GltfRayTracingBsdfEvaluateData {
 public
  float3 k1;
 public
  float3 k2;
 public
  float3 xi;
 public
  float3 bsdf_diffuse;
 public
  float3 bsdf_glossy;
 public
  float pdf;
 public
  int event_type;
};
public
struct GltfRayTracingBsdfSampleData {
 public
  float3 k1;
 public
  float3 k2;
 public
  float3 xi;
 public
  float pdf;
 public
  float3 bsdf_over_pdf;
 public
  int event_type;
};
public
struct GltfRayTracingBsdfLobeWeights {
 public
  float diffuse_reflection;
 public
  float specular_transmission;
 public
  float specular_reflection;
 public
  float metal_reflection;
 public
  float sheen_reflection;
 public
  float clearcoat_reflection;
 public
  float diffuse_transmission;
 public
  float dielectric_fresnel_weight;
 public
  float3 tint;
 public
  float3 specular_tint;
};
public
float3 EE_GLTF_RT_SANITIZE(const float3 value) {
  return float3(value.x >= 0.0f && value.x < 3.402823466e+38f ? value.x : 0.0f,
                value.y >= 0.0f && value.y < 3.402823466e+38f ? value.y : 0.0f,
                value.z >= 0.0f && value.z < 3.402823466e+38f ? value.z : 0.0f);
}
public
float3 EE_GLTF_RT_MULTI_TO_SINGLE_SCATTER_ALBEDO(const float3 rho_ms) {
  const float3 clamped_rho = clamp(rho_ms, float3(0.0f), float3(1.0f));
  const float3 t = 4.09712f + 4.20863f * clamped_rho -
                   sqrt(9.59217f + 41.6808f * clamped_rho + 17.7126f * clamped_rho * clamped_rho);
  return clamp(float3(1.0f) - t * t, float3(0.0f), float3(1.0f));
}
public
float3 EE_GLTF_RT_VOLUME_EXTINCTION_COEFFICIENT(const GltfRayTracingPbrMaterial material) {
  if (material.attenuation_distance <= EE_GLTF_RT_BSDF_EPSILON) {
    return float3(0.0f);
  }
  return -log(max(material.attenuation_color, float3(0.001f))) / max(material.attenuation_distance, 0.001f);
}
public
float3 EE_GLTF_RT_WEIGHTED_SPECULAR_FRESNEL(const GltfRayTracingPbrMaterial material, const float cos_theta) {
  const float weight = clamp(material.specular, 0.0f, 1.0f);
  return EE_GLTF_RT_FRESNEL_SCHLICK(cos_theta, weight * clamp(material.specular_f0, float3(0.0f), float3(1.0f)),
                                    float3(weight));
}
public
float EE_GLTF_RT_CLEARCOAT_FRESNEL(const float3 clearcoat_normal, const float3 outgoing_direction) {
  const float3 safe_normal = EE_GLTF_RT_SAFE_NORMALIZE(clearcoat_normal, float3(0.0f, 1.0f, 0.0f));
  const float cosine =
      clamp(abs(dot(EE_GLTF_RT_SAFE_NORMALIZE(outgoing_direction, safe_normal), safe_normal)), 0.0f, 1.0f);
  return EE_GLTF_RT_FRESNEL_SCHLICK(cosine, float3(0.04f)).x;
}
public
float3 EE_GLTF_RT_COATED_EMISSION_VALUE(const float3 emissive, const float clearcoat, const float3 clearcoat_normal,
                                        const float3 outgoing_direction) {
  const float coat = clamp(clearcoat, 0.0f, 1.0f);
  if (coat <= 0.0f) {
    return emissive;
  }
  const float fresnel = EE_GLTF_RT_CLEARCOAT_FRESNEL(clearcoat_normal, outgoing_direction);
  return emissive * max(1.0f - coat * fresnel, 0.0f);
}
public
float3 EE_GLTF_RT_COATED_EMISSION(const GltfRayTracingPbrMaterial material, const float3 outgoing_direction) {
  return EE_GLTF_RT_COATED_EMISSION_VALUE(material.emissive, material.clearcoat, material.clearcoat_normal,
                                          outgoing_direction);
}
public
float3 EE_GLTF_RT_COATED_EMISSION_LOD0(const uint material_index, const GltfTexCoords tex_coords, const float3 normal,
                                       const float3 tangent, const float3 bitangent, const float3 outgoing_direction) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  const float3 emissive = max(material.emissive_factor * EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(
                                                             material.emissive_texture, EE_GLTF_RASTER_TEXTURE_EMISSIVE,
                                                             tex_coords, float4(1.0f))
                                                             .rgb,
                              float3(0.0f));
  const float clearcoat = material.clearcoat_factor * EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(material.clearcoat_texture,
                                                                                       EE_GLTF_RASTER_TEXTURE_CLEARCOAT,
                                                                                       tex_coords, float4(1.0f))
                                                          .r;
  if (clamp(clearcoat, 0.0f, 1.0f) <= 0.0f) {
    return emissive;
  }
  float3 clearcoat_normal = EE_GLTF_RT_SAFE_NORMALIZE(normal, float3(0.0f, 1.0f, 0.0f));
  if (EE_GLTF_HAS_TEXTURE(material.clearcoat_normal_texture)) {
    const float3 safe_tangent = EE_GLTF_RT_SAFE_NORMALIZE(tangent, float3(1.0f, 0.0f, 0.0f));
    const float3 safe_bitangent = EE_GLTF_RT_SAFE_NORMALIZE(bitangent, cross(clearcoat_normal, safe_tangent));
    float3 normal_vector =
        EE_GLTF_SAMPLE_TEXTURE_SLOT_LOD0(material.clearcoat_normal_texture, EE_GLTF_RASTER_TEXTURE_CLEARCOAT_NORMAL,
                                         tex_coords, float4(0.5f, 0.5f, 1.0f, 1.0f))
                .xyz *
            2.0f -
        1.0f;
    normal_vector.xy *= material.clearcoat_normal_texture_scale;
    clearcoat_normal = EE_GLTF_RT_SAFE_NORMALIZE(
        mul(normal_vector, float3x3(safe_tangent, safe_bitangent, clearcoat_normal)), clearcoat_normal);
  }
  return EE_GLTF_RT_COATED_EMISSION_VALUE(emissive, clearcoat, clearcoat_normal, outgoing_direction);
}
public
float3 EE_GLTF_RT_COATED_EMISSION_LOD0(const uint material_index, const float2 tex_coord_0, const float2 tex_coord_1,
                                       const float3 normal, const float3 tangent, const float3 bitangent,
                                       const float3 outgoing_direction) {
  return EE_GLTF_RT_COATED_EMISSION_LOD0(material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1), normal,
                                         tangent, bitangent, outgoing_direction);
}
public
float3 EE_GLTF_RT_COATED_EMISSION_LOD0(const uint material_index, const float2 tex_coord_0, const float2 tex_coord_1,
                                       const float2 tex_coord_2, const float2 tex_coord_3, const float3 normal,
                                       const float3 tangent, const float3 bitangent, const float3 outgoing_direction) {
  return EE_GLTF_RT_COATED_EMISSION_LOD0(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, float4(0.0)), normal,
      tangent, bitangent, outgoing_direction);
}
public
float EE_GLTF_RT_FRESNEL_COSINE_APPROXIMATION(const float v_dot_n, const float roughness) {
  return lerp(v_dot_n, sqrt(0.5f + 0.5f * v_dot_n), sqrt(clamp(roughness, 0.0f, 1.0f)));
}
public
float EE_GLTF_RT_IOR_FRESNEL(const float eta, const float kh) {
  float costheta = 1.0f - (1.0f - kh * kh) / (eta * eta);
  if (costheta <= 0.0f) {
    return 1.0f;
  }
  costheta = sqrt(costheta);
  const float n1t1 = kh;
  const float n1t2 = costheta;
  const float n2t1 = kh * eta;
  const float n2t2 = costheta * eta;
  const float r_p = (n1t2 - n2t1) / (n1t2 + n2t1);
  const float r_o = (n1t1 - n2t2) / (n1t1 + n2t2);
  return clamp(0.5f * (r_p * r_p + r_o * r_o), 0.0f, 1.0f);
}
public
float3 EE_GLTF_RT_IOR_TO_F0(const float3 transmitted_ior, const float incident_ior) {
  const float3 ratio =
      (transmitted_ior - incident_ior) / max(transmitted_ior + incident_ior, float3(EE_GLTF_RT_BSDF_EPSILON));
  return ratio * ratio;
}
public
float3 EE_GLTF_RT_F0_TO_IOR(const float3 f0) {
  const float3 sqrt_f0 = sqrt(clamp(f0, float3(0.0f), float3(0.9999f)));
  return (float3(1.0f) + sqrt_f0) / max(float3(1.0f) - sqrt_f0, float3(EE_GLTF_RT_BSDF_EPSILON));
}
public
float3 EE_GLTF_RT_IRIDESCENCE_SENSITIVITY(const float optical_path_difference, const float3 shift) {
  const float phase = 2.0f * EE_GLTF_RT_BSDF_PI * optical_path_difference * 1.0e-9f;
  const float3 value = float3(5.4856e-13f, 4.4201e-13f, 5.2481e-13f);
  const float3 position = float3(1.6810e+06f, 1.7953e+06f, 2.2084e+06f);
  const float3 variance = float3(4.3278e+09f, 9.3046e+09f, 6.6121e+09f);
  float3 xyz = value * sqrt(2.0f * EE_GLTF_RT_BSDF_PI * variance) * cos(position * phase + shift) *
               exp(-(phase * phase) * variance);
  xyz.x += 9.7470e-14f * sqrt(2.0f * EE_GLTF_RT_BSDF_PI * 4.5282e+09f) * cos(2.2399e+06f * phase + shift.x) *
           exp(-4.5282e+09f * phase * phase);
  xyz /= 1.0685e-7f;
  const float3x3 xyz_to_rec709 = float3x3(3.2404542f, -0.9692660f, 0.0556434f, -1.5371385f, 1.8760108f, -0.2040259f,
                                          -0.4985314f, 0.0415560f, 1.0572252f);
  return mul(xyz, xyz_to_rec709);
}
public
float3 EE_GLTF_RT_THIN_FILM_FACTOR(const float coating_thickness, const float coating_ior, const float3 base_f0,
                                   const float incoming_ior, const float kh) {
  const float thickness = max(coating_thickness, 0.0f);
  const float film_ior = lerp(max(incoming_ior, 1.0f), max(coating_ior, 1.0f), smoothstep(0.0f, 0.03f, thickness));
  const float cos_theta_1 = clamp(kh, 0.0f, 1.0f);
  const float eta = max(incoming_ior, 1.0f) / film_ior;
  const float sin_theta_2_squared = eta * eta * (1.0f - cos_theta_1 * cos_theta_1);
  const float cos_theta_2_squared = 1.0f - sin_theta_2_squared;
  if (cos_theta_2_squared < 0.0f) {
    return float3(1.0f);
  }
  const float cos_theta_2 = sqrt(cos_theta_2_squared);
  const float r0 = EE_GLTF_RT_IOR_TO_F0(film_ior, max(incoming_ior, 1.0f));
  const float r12 = EE_GLTF_RT_FRESNEL_SCHLICK(cos_theta_1, float3(r0)).x;
  const float transmission_121 = 1.0f - r12;
  const float phi_12 = film_ior < incoming_ior ? EE_GLTF_RT_BSDF_PI : 0.0f;
  const float phi_21 = EE_GLTF_RT_BSDF_PI - phi_12;
  const float3 base_ior = EE_GLTF_RT_F0_TO_IOR(base_f0);
  const float3 r1 = EE_GLTF_RT_IOR_TO_F0(base_ior, film_ior);
  const float3 r23 = EE_GLTF_RT_FRESNEL_SCHLICK(cos_theta_2, r1);
  const float3 phi_23 =
      float3(base_ior.x < film_ior ? EE_GLTF_RT_BSDF_PI : 0.0f, base_ior.y < film_ior ? EE_GLTF_RT_BSDF_PI : 0.0f,
             base_ior.z < film_ior ? EE_GLTF_RT_BSDF_PI : 0.0f);
  const float optical_path_difference = 2.0f * film_ior * thickness * cos_theta_2;
  const float3 phase = float3(phi_21) + phi_23;
  const float3 r123 = clamp(r12 * r23, float3(1.0e-5f), float3(0.9999f));
  const float3 root_r123 = sqrt(r123);
  const float3 multiple_scatter =
      transmission_121 * transmission_121 * r23 / max(float3(1.0f) - r123, float3(EE_GLTF_RT_BSDF_EPSILON));
  float3 result = float3(r12) + multiple_scatter;
  float3 harmonic = multiple_scatter - float3(transmission_121);
  for (int order = 1; order <= 2; ++order) {
    harmonic *= root_r123;
    result += harmonic * 2.0f *
              EE_GLTF_RT_IRIDESCENCE_SENSITIVITY(float(order) * optical_path_difference, float(order) * phase);
  }
  return max(result, float3(0.0f));
}
public
float EE_GLTF_RT_HVD_GGX_EVAL(const float2 inv_roughness, const float3 h) {
  const float x = h.x * inv_roughness.x;
  const float y = h.y * inv_roughness.y;
  const float aniso = x * x + y * y;
  const float f = aniso + h.z * h.z;
  return (1.0f / EE_GLTF_RT_BSDF_PI) * inv_roughness.x * inv_roughness.y * h.z / max(f * f, EE_GLTF_RT_BSDF_EPSILON);
}
public
float3 EE_GLTF_RT_HVD_GGX_SAMPLE_VNDF(const float3 k, const float2 roughness, const float2 xi) {
  const float3 v =
      EE_GLTF_RT_SAFE_NORMALIZE(float3(k.x * roughness.x, k.y * roughness.y, k.z), float3(0.0f, 0.0f, 1.0f));
  const float3 t1 = v.z < 0.99999f
                        ? EE_GLTF_RT_SAFE_NORMALIZE(cross(v, float3(0.0f, 0.0f, 1.0f)), float3(1.0f, 0.0f, 0.0f))
                        : float3(1.0f, 0.0f, 0.0f);
  const float3 t2 = cross(t1, v);
  const float a = 1.0f / max(1.0f + v.z, EE_GLTF_RT_BSDF_EPSILON);
  const float r = sqrt(clamp(xi.x, 0.0f, 1.0f));
  const float phi = xi.y < a
                        ? xi.y / a * EE_GLTF_RT_BSDF_PI
                        : EE_GLTF_RT_BSDF_PI + (xi.y - a) / max(1.0f - a, EE_GLTF_RT_BSDF_EPSILON) * EE_GLTF_RT_BSDF_PI;
  const float p1 = r * cos(phi);
  const float p2 = r * sin(phi) * (xi.y < a ? 1.0f : v.z);
  float3 h = p1 * t1 + p2 * t2 + sqrt(max(0.0f, 1.0f - p1 * p1 - p2 * p2)) * v;
  h.x *= roughness.x;
  h.y *= roughness.y;
  h.z = max(0.0f, h.z);
  return EE_GLTF_RT_SAFE_NORMALIZE(h, float3(0.0f, 0.0f, 1.0f));
}
public
float EE_GLTF_RT_SMITH_SHADOW_OR_MASK(const float3 k, const float2 roughness) {
  const float kz2 = k.z * k.z;
  if (kz2 <= EE_GLTF_RT_BSDF_EPSILON) {
    return 0.0f;
  }
  const float ax = k.x * roughness.x;
  const float ay = k.y * roughness.y;
  const float inv_a2 = (ax * ax + ay * ay) / kz2;
  return 2.0f / (1.0f + sqrt(1.0f + inv_a2));
}
public
float EE_GLTF_RT_GGX_SMITH_SHADOW_MASK(out float g1, out float g2, const float3 k1, const float3 k2,
                                       const float2 roughness) {
  g1 = EE_GLTF_RT_SMITH_SHADOW_OR_MASK(k1, roughness);
  g2 = EE_GLTF_RT_SMITH_SHADOW_OR_MASK(k2, roughness);
  return g1 * g2;
}
public
float EE_GLTF_RT_HVD_SHEEN_EVAL(const float inv_roughness, const float n_dot_h) {
  const float sin_theta = sqrt(max(0.0f, 1.0f - n_dot_h * n_dot_h));
  return (inv_roughness + 2.0f) * pow(sin_theta, inv_roughness) * 0.5f * (1.0f / EE_GLTF_RT_BSDF_PI) * n_dot_h;
}
public
float3 EE_GLTF_RT_HVD_SHEEN_SAMPLE(const float2 xi, const float inv_roughness) {
  const float phi = 2.0f * EE_GLTF_RT_BSDF_PI * xi.x;
  const float sin_theta = pow(max(1.0f - xi.y, 0.0f), 1.0f / (inv_roughness + 2.0f));
  const float cos_theta = sqrt(max(0.0f, 1.0f - sin_theta * sin_theta));
  return EE_GLTF_RT_SAFE_NORMALIZE(float3(cos(phi) * sin_theta, sin(phi) * sin_theta, cos_theta),
                                   float3(0.0f, 0.0f, 1.0f));
}
public
float EE_GLTF_RT_VCAVITIES_MASK(const float n_dot_h, const float k_dot_h, const float n_dot_k) {
  return min(2.0f * n_dot_h * n_dot_k / max(k_dot_h, EE_GLTF_RT_BSDF_EPSILON), 1.0f);
}
public
float EE_GLTF_RT_VCAVITIES_SHADOW_MASK(out float g1, out float g2, const float n_dot_h, const float3 k1,
                                       const float k1_dot_h, const float3 k2, const float k2_dot_h) {
  g1 = EE_GLTF_RT_VCAVITIES_MASK(n_dot_h, k1_dot_h, k1.z);
  g2 = EE_GLTF_RT_VCAVITIES_MASK(n_dot_h, k2_dot_h, k2.z);
  return min(g1, g2);
}
public
float3 EE_GLTF_RT_FLIP_SHEEN_HALF_VECTOR(const float3 h, const float3 k, const float xi) {
  const float a = h.z * k.z;
  const float b = h.x * k.x + h.y * k.y;
  const float kh = max(0.0f, a + b);
  const float kh_flipped = max(0.0f, a - b);
  const float flip_probability = kh_flipped / max(kh + kh_flipped, EE_GLTF_RT_BSDF_EPSILON);
  return xi < flip_probability ? float3(-h.x, -h.y, h.z) : h;
}
public
float EE_GLTF_RT_RERANDOMIZE(const float value) {
  uint word = asuint(value);
  word = ((word >> ((word >> 28u) + 4u)) ^ word) * 277803737u;
  word = (word >> 22u) ^ word;
  return float(word) / asfloat(0x4f800000u);
}
public
float EE_GLTF_RT_COMPUTE_DISPERSED_IOR(const float base_ior, const float dispersion, const float wavelength_nm) {
  const float abbe_number = 20.0f / max(dispersion, EE_GLTF_RT_BSDF_EPSILON);
  return max(base_ior + (base_ior - 1.0f) *
                            (523655.0f / max(wavelength_nm * wavelength_nm, EE_GLTF_RT_BSDF_EPSILON) - 1.5168f) /
                            abbe_number,
             1.0f);
}
public
float3 EE_GLTF_RT_WAVELENGTH_TO_RGB(const float wavelength) {
  float3 rgb = float3(0.0f);
  if (399.43862850585765f < wavelength) {
    if (wavelength < 435.3450352446586f) {
      rgb.r = 2.6268757476158464e-05f * wavelength + -0.010492756458829732f;
    } else if (wavelength < 452.7741480943567f) {
      rgb.r = -5.383671438883332e-05f * wavelength + 0.024380763013525125f;
    } else if (wavelength < 550.5919453498173f) {
      rgb.r = 1.2536207000814165e-07f * wavelength + -5.187018452935683e-05f;
    } else if (wavelength < 600.8694441891222f) {
      rgb.r = 0.00032842519537482f * wavelength + -0.18081111406184644f;
    } else if (wavelength < 668.6617899434457f) {
      rgb.r = -0.0002438262071743009f * wavelength + 0.16303726812428945f;
    }
  }
  if (467.41924217251835f < wavelength) {
    if (wavelength < 532.3927928594046f) {
      rgb.g = 0.00020126149345609334f * wavelength + -0.0940734947497564f;
    } else if (wavelength < 552.5312202450474f) {
      rgb.g = -4.3718474429905034e-05f * wavelength + 0.03635207454767751f;
    } else if (wavelength < 605.5304635656746f) {
      rgb.g = -0.00023012125757884968f * wavelength + 0.13934543177803685f;
    }
  }
  if (400.68666327204835f < wavelength) {
    if (wavelength < 447.59688835108466f) {
      rgb.b = 0.00042519082480799777f * wavelength + -0.1703682928462067f;
    } else if (wavelength < 501.2110070695423f) {
      rgb.b = -0.00037202508909921054f * wavelength + 0.18646306956262593f;
    }
  }
  return max(rgb, float3(0.0f));
}
public
float2 EE_GLTF_RT_TRANSMISSION_IOR(const GltfRayTracingPbrMaterial material, const float xi, inout float3 tint) {
  const float wavelength_min = 399.43862850585765f;
  const float wavelength_max = 668.6617899434457f;
  float2 ior = float2(max(material.ior1, 1.0f), max(material.ior2, 1.0f));
  if (material.dispersion > 0.0f) {
    const float wavelength = lerp(wavelength_min, wavelength_max, EE_GLTF_RT_RERANDOMIZE(xi));
    if (ior.x > ior.y) {
      ior.x = EE_GLTF_RT_COMPUTE_DISPERSED_IOR(ior.x, material.dispersion, wavelength);
    } else {
      ior.y = EE_GLTF_RT_COMPUTE_DISPERSED_IOR(ior.y, material.dispersion, wavelength);
    }
    tint *= (wavelength_max - wavelength_min) * EE_GLTF_RT_WAVELENGTH_TO_RGB(wavelength);
  }
  return ior;
}
public
bool EE_GLTF_RT_IS_TIR(const float2 ior, const float k_dot_h) {
  const float eta = ior.x / max(ior.y, EE_GLTF_RT_BSDF_EPSILON);
  return 1.0f < eta * eta * (1.0f - k_dot_h * k_dot_h);
}
public
float3 EE_GLTF_RT_COMPUTE_HALF_VECTOR(const float3 k1, const float3 k2, const float3 normal, const float2 ior,
                                      const float n_dot_k2, const bool transmission, const bool thin_walled) {
  float3 half_vector;
  if (transmission) {
    if (thin_walled) {
      half_vector = k1 + (normal * (n_dot_k2 + n_dot_k2) + k2);
    } else {
      half_vector = k2 * ior.y + k1 * ior.x;
      if (ior.y > ior.x) {
        half_vector *= -1.0f;
      }
    }
  } else {
    half_vector = k1 + k2;
  }
  return EE_GLTF_RT_SAFE_NORMALIZE(half_vector, normal);
}
public
float3 EE_GLTF_RT_REFRACT(const float3 k, const float3 normal, const float eta, const float n_dot_k, out bool tir) {
  const float refraction = eta * eta * (1.0f - n_dot_k * n_dot_k);
  tir = 1.0f <= refraction;
  return tir ? EE_GLTF_RT_SAFE_NORMALIZE(normal * (n_dot_k + n_dot_k) - k, normal)
             : EE_GLTF_RT_SAFE_NORMALIZE(-k * eta + normal * (eta * n_dot_k - sqrt(1.0f - refraction)), -normal);
}
public
void EE_GLTF_RT_EVALUATE_GGX_TRANSMISSION(inout GltfRayTracingBsdfEvaluateData data,
                                          const GltfRayTracingPbrMaterial material, float3 tint) {
  const bool thin_walled = material.thickness <= EE_GLTF_RT_BSDF_EPSILON;
  const float2 roughness = max(material.roughness, float2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS));
  float2 ior = EE_GLTF_RT_TRANSMISSION_IOR(material, data.xi.z, tint);
  const float n_dot_k1 = abs(dot(data.k1, material.normal));
  float n_dot_k2 = dot(data.k2, material.normal);
  const bool backside = n_dot_k2 < 0.0f;
  n_dot_k2 = abs(n_dot_k2);
  const float3 half_vector =
      EE_GLTF_RT_COMPUTE_HALF_VECTOR(data.k1, data.k2, material.normal, ior, n_dot_k2, backside, thin_walled);
  const float n_dot_h = dot(material.normal, half_vector);
  const float k1_dot_h = dot(data.k1, half_vector);
  const float k2_dot_h = dot(data.k2, half_vector) * (backside ? -1.0f : 1.0f);
  if (n_dot_k1 <= EE_GLTF_RT_BSDF_EPSILON || n_dot_h <= EE_GLTF_RT_BSDF_EPSILON || k1_dot_h < 0.0f || k2_dot_h < 0.0f) {
    return;
  }
  float fresnel;
  if (!backside) {
    if (!EE_GLTF_RT_IS_TIR(ior, k1_dot_h)) {
      return;
    }
    fresnel = 1.0f;
  } else {
    fresnel = 0.0f;
  }
  const float3 local_half = float3(dot(material.tangent, half_vector), dot(material.bitangent, half_vector), n_dot_h);
  float scattering_pdf = EE_GLTF_RT_HVD_GGX_EVAL(float2(1.0f) / roughness, local_half);
  float g1;
  float g2;
  const float g12 = EE_GLTF_RT_GGX_SMITH_SHADOW_MASK(
      g1, g2, float3(dot(material.tangent, data.k1), dot(material.bitangent, data.k1), n_dot_k1),
      float3(dot(material.tangent, data.k2), dot(material.bitangent, data.k2), n_dot_k2), roughness);
  if (!thin_walled && backside) {
    const float tmp = k1_dot_h * ior.x - k2_dot_h * ior.y;
    if (tmp <= EE_GLTF_RT_BSDF_EPSILON) {
      return;
    }
    scattering_pdf *= k1_dot_h * k2_dot_h / max(n_dot_k1 * n_dot_h * tmp * tmp, EE_GLTF_RT_BSDF_EPSILON);
  } else {
    scattering_pdf *= 0.25f / max(n_dot_k1 * n_dot_h, EE_GLTF_RT_BSDF_EPSILON);
  }
  const float probability = backside ? 1.0f - fresnel : fresnel;
  if (probability <= EE_GLTF_RT_BSDF_EPSILON) {
    return;
  }
  data.pdf = scattering_pdf * probability * g1;
  data.bsdf_glossy = float3(probability * g12 * scattering_pdf) * tint;
  data.event_type = backside ? EE_GLTF_RT_BSDF_EVENT_GLOSSY_TRANSMISSION : EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION;
}
public
void EE_GLTF_RT_SAMPLE_GGX_TRANSMISSION(inout GltfRayTracingBsdfSampleData data,
                                        const GltfRayTracingPbrMaterial material, float3 tint) {
  const bool thin_walled = material.thickness <= EE_GLTF_RT_BSDF_EPSILON;
  const float2 roughness = max(material.roughness, float2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS));
  float2 ior = EE_GLTF_RT_TRANSMISSION_IOR(material, data.xi.z, tint);
  const float n_dot_k1 = abs(dot(data.k1, material.normal));
  if (n_dot_k1 <= EE_GLTF_RT_BSDF_EPSILON) {
    return;
  }
  const float3 local_k1 = float3(dot(data.k1, material.tangent), dot(data.k1, material.bitangent), n_dot_k1);
  const float3 local_half = EE_GLTF_RT_HVD_GGX_SAMPLE_VNDF(local_k1, roughness, data.xi.xy);
  if (abs(local_half.z) <= EE_GLTF_RT_BSDF_EPSILON) {
    return;
  }
  const float3 half_vector = EE_GLTF_RT_SAFE_NORMALIZE(
      local_half.x * material.tangent + local_half.y * material.bitangent + local_half.z * material.normal,
      material.normal);
  const float k_dot_h = dot(data.k1, half_vector);
  if (k_dot_h <= EE_GLTF_RT_BSDF_EPSILON) {
    return;
  }
  bool tir = false;
  if (thin_walled) {
    data.k2 = (2.0f * k_dot_h) * half_vector - data.k1;
    data.k2 =
        EE_GLTF_RT_SAFE_NORMALIZE(data.k2 - 2.0f * material.normal * dot(data.k2, material.normal), -material.normal);
  } else {
    data.k2 = EE_GLTF_RT_REFRACT(data.k1, half_vector, ior.x / max(ior.y, EE_GLTF_RT_BSDF_EPSILON), k_dot_h, tir);
  }
  data.bsdf_over_pdf = float3(1.0f);
  data.event_type = tir ? EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION : EE_GLTF_RT_BSDF_EVENT_GLOSSY_TRANSMISSION;
  const float n_dot_k2 = dot(data.k2, material.normal) * (tir ? 1.0f : -1.0f);
  if (n_dot_k2 <= EE_GLTF_RT_BSDF_EPSILON || isnan(data.k2.x)) {
    data.k2 = float3(0.0f);
    data.bsdf_over_pdf = float3(0.0f);
    data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    return;
  }
  const float k2_dot_h = abs(dot(data.k2, half_vector));
  float g1;
  float g2;
  const float g12 = EE_GLTF_RT_GGX_SMITH_SHADOW_MASK(
      g1, g2, local_k1, float3(dot(data.k2, material.tangent), dot(data.k2, material.bitangent), n_dot_k2), roughness);
  if (g12 <= EE_GLTF_RT_BSDF_EPSILON) {
    data.k2 = float3(0.0f);
    data.bsdf_over_pdf = float3(0.0f);
    data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    return;
  }
  data.bsdf_over_pdf *= g2;
  data.pdf = EE_GLTF_RT_HVD_GGX_EVAL(float2(1.0f) / roughness, local_half) * g1;
  if (!thin_walled && data.event_type == EE_GLTF_RT_BSDF_EVENT_GLOSSY_TRANSMISSION) {
    const float tmp = k_dot_h * ior.x - k2_dot_h * ior.y;
    if (tmp <= EE_GLTF_RT_BSDF_EPSILON) {
      data.k2 = float3(0.0f);
      data.pdf = 0.0f;
      data.bsdf_over_pdf = float3(0.0f);
      data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
      return;
    }
    data.pdf *= k_dot_h * k2_dot_h / max(n_dot_k1 * local_half.z * tmp * tmp, EE_GLTF_RT_BSDF_EPSILON);
  } else {
    data.pdf *= 0.25f / max(n_dot_k1 * local_half.z, EE_GLTF_RT_BSDF_EPSILON);
  }
  data.bsdf_over_pdf *= tint;
}
public
GltfRayTracingBsdfLobeWeights EE_GLTF_RT_COMPUTE_LOBE_WEIGHTS(const GltfRayTracingPbrMaterial material,
                                                              const float v_dot_n) {
  GltfRayTracingBsdfLobeWeights weights;
  weights.diffuse_reflection = 0.0f;
  weights.specular_transmission = 0.0f;
  weights.specular_reflection = 0.0f;
  weights.metal_reflection = 0.0f;
  weights.sheen_reflection = 0.0f;
  weights.clearcoat_reflection = 0.0f;
  weights.diffuse_transmission = 0.0f;
  weights.dielectric_fresnel_weight = 0.0f;
  weights.tint = material.base_color;
  weights.specular_tint = float3(1.0f);
  float coat_fresnel = 0.0f;
  if (material.clearcoat > 0.0f) {
    const float coat_cosine = EE_GLTF_RT_FRESNEL_COSINE_APPROXIMATION(v_dot_n, material.clearcoat_roughness);
    coat_fresnel = clamp(
        material.clearcoat * EE_GLTF_RT_IOR_FRESNEL(1.5f / max(material.ior1, EE_GLTF_RT_BSDF_EPSILON), coat_cosine),
        0.0f, 1.0f);
  }
  float3 dielectric_fresnel = float3(0.0f);
  if (material.specular > 0.0f) {
    const float dielectric_cosine =
        EE_GLTF_RT_FRESNEL_COSINE_APPROXIMATION(v_dot_n, (material.roughness.x + material.roughness.y) * 0.5f);
    dielectric_fresnel = EE_GLTF_RT_WEIGHTED_SPECULAR_FRESNEL(material, dielectric_cosine);
  }
  if (material.iridescence > 0.0f && material.iridescence_thickness > 0.0f) {
    const float3 iridescence_fresnel =
        EE_GLTF_RT_THIN_FILM_FACTOR(material.iridescence_thickness, material.iridescence_ior,
                                    material.specular * material.specular_f0, material.ior1, v_dot_n);
    dielectric_fresnel = lerp(dielectric_fresnel, iridescence_fresnel, clamp(material.iridescence, 0.0f, 1.0f));
  }
  float sheen = 0.0f;
  if (max(material.sheen_color.x, max(material.sheen_color.y, material.sheen_color.z)) > 0.0f) {
    sheen = pow(max(1.0f - abs(v_dot_n), 0.0f), material.sheen_roughness);
    sheen = sheen / max(sheen + 0.5f, EE_GLTF_RT_BSDF_EPSILON);
  }
  float base_weight = 1.0f;
  weights.clearcoat_reflection = coat_fresnel;
  base_weight *= 1.0f - weights.clearcoat_reflection;
  weights.sheen_reflection = base_weight * sheen;
  base_weight *= 1.0f - sheen;
  weights.metal_reflection = base_weight * clamp(material.metallic, 0.0f, 1.0f);
  base_weight *= 1.0f - clamp(material.metallic, 0.0f, 1.0f);
  const float dielectric_fresnel_weight =
      clamp(max(dielectric_fresnel.x, max(dielectric_fresnel.y, dielectric_fresnel.z)), 0.0f, 1.0f);
  weights.dielectric_fresnel_weight = dielectric_fresnel_weight;
  weights.specular_tint = dielectric_fresnel_weight > EE_GLTF_RT_BSDF_EPSILON
                              ? dielectric_fresnel / dielectric_fresnel_weight
                              : float3(0.0f);
  weights.specular_reflection = base_weight * dielectric_fresnel_weight;
  base_weight *= 1.0f - dielectric_fresnel_weight;
  const float transmission = clamp(material.transmission, 0.0f, 1.0f);
  weights.specular_transmission = base_weight * transmission;
  const float remaining_weight = base_weight * (1.0f - transmission);
  const float diffuse_transmission = clamp(material.diffuse_transmission_factor, 0.0f, 1.0f);
  weights.diffuse_transmission = remaining_weight * diffuse_transmission;
  weights.diffuse_reflection = remaining_weight * (1.0f - diffuse_transmission);
  return weights;
}
public
float EE_GLTF_RT_LOBE_WEIGHT_SUM(const GltfRayTracingBsdfLobeWeights weights) {
  return weights.diffuse_reflection + weights.specular_transmission + weights.specular_reflection +
         weights.metal_reflection + weights.sheen_reflection + weights.clearcoat_reflection +
         weights.diffuse_transmission;
}
public
int EE_GLTF_RT_FIND_BSDF_LOBE(const GltfRayTracingBsdfLobeWeights weights, const float random_value) {
  const float sum = EE_GLTF_RT_LOBE_WEIGHT_SUM(weights);
  if (sum <= EE_GLTF_RT_BSDF_EPSILON) {
    return EE_GLTF_RT_BSDF_LOBE_DIFFUSE_REFLECTION;
  }
  const float target = clamp(random_value, 0.0f, 0.999999f) * sum;
  float weight = weights.diffuse_transmission;
  if (target < weight) {
    return EE_GLTF_RT_BSDF_LOBE_DIFFUSE_TRANSMISSION;
  }
  weight += weights.clearcoat_reflection;
  if (target < weight) {
    return EE_GLTF_RT_BSDF_LOBE_CLEARCOAT_REFLECTION;
  }
  weight += weights.sheen_reflection;
  if (target < weight) {
    return EE_GLTF_RT_BSDF_LOBE_SHEEN_REFLECTION;
  }
  weight += weights.metal_reflection;
  if (target < weight) {
    return EE_GLTF_RT_BSDF_LOBE_METAL_REFLECTION;
  }
  weight += weights.specular_reflection;
  if (target < weight) {
    return EE_GLTF_RT_BSDF_LOBE_SPECULAR_REFLECTION;
  }
  weight += weights.specular_transmission;
  if (target < weight) {
    return EE_GLTF_RT_BSDF_LOBE_SPECULAR_TRANSMISSION;
  }
  return EE_GLTF_RT_BSDF_LOBE_DIFFUSE_REFLECTION;
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL_SPECIALIZED<let feature_mask : uint>(
    uint material_index, GltfTexCoords tex_coords, float4 vertex_color, GltfRasterMaterial surface, float3 normal,
    float3 tangent, float3 bitangent, float3 geometric_normal, bool is_inside) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  GltfRayTracingPbrMaterial pbr;
  pbr.base_color = max(surface.base_color.rgb, float3(0.0f));
  pbr.opacity = clamp(surface.base_color.a, 0.0f, 1.0f);
  const float roughness = max(surface.roughness, EE_GLTF_RT_BSDF_MIN_ROUGHNESS);
  pbr.roughness = float2(roughness * roughness);
  pbr.metallic = clamp(surface.metallic, 0.0f, 1.0f);
  pbr.emissive = max(surface.emissive, float3(0.0f));
  pbr.occlusion = surface.occlusion;
  pbr.normal = EE_GLTF_RT_SAFE_NORMALIZE(normal, geometric_normal);
  pbr.geometric_normal = EE_GLTF_RT_SAFE_NORMALIZE(geometric_normal, pbr.normal);
  pbr.tangent = EE_GLTF_RT_SAFE_NORMALIZE(tangent, float3(1.0f, 0.0f, 0.0f));
  pbr.bitangent = EE_GLTF_RT_SAFE_NORMALIZE(bitangent, cross(pbr.normal, pbr.tangent));
  const float3 clearcoat_basis_normal = pbr.normal;
  const float3 clearcoat_basis_tangent = pbr.tangent;
  const float3 clearcoat_basis_bitangent = pbr.bitangent;
  const float basis_handedness = dot(cross(pbr.normal, pbr.tangent), pbr.bitangent) < 0.0f ? -1.0f : 1.0f;
  bool needs_tangent_update = false;
  if (EE_GLTF_HAS_TEXTURE(material.normal_texture)) {
    float3 normal_vector = EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(material.normal_texture, tex_coords,
                                                                            float4(0.5f, 0.5f, 1.0f, 1.0f))
                               .xyz;
    normal_vector = normal_vector * 2.0f - 1.0f;
    normal_vector.xy *= material.normal_texture_scale;
    pbr.normal =
        EE_GLTF_RT_SAFE_NORMALIZE(mul(normal_vector, float3x3(pbr.tangent, pbr.bitangent, pbr.normal)), pbr.normal);
    needs_tangent_update = true;
  }
  pbr.ior1 = 1.0f;
  pbr.ior2 = 1.5f;
  pbr.dispersion = 0.0f;
  pbr.specular = 1.0f;
  pbr.specular_color = float3(1.0f);
  pbr.specular_f0 = surface.specular_f0;
  pbr.transmission = 0.0f;
  pbr.attenuation_color = float3(1.0f);
  pbr.attenuation_distance = 1.0f;
  pbr.thickness = 0.0f;
  pbr.scatter_coefficient = float3(0.0f);
  pbr.scatter_anisotropy = 0.0f;
  pbr.clearcoat = 0.0f;
  pbr.clearcoat_roughness = EE_GLTF_RT_BSDF_MIN_ROUGHNESS;
  pbr.clearcoat_normal = clearcoat_basis_normal;
  pbr.iridescence = 0.0f;
  pbr.iridescence_ior = 1.3f;
  pbr.iridescence_thickness = 400.0f;
  pbr.sheen_color = float3(0.0f);
  pbr.sheen_roughness = 0.0f;
  pbr.diffuse_transmission_factor = 0.0f;
  pbr.diffuse_transmission_color = float3(1.0f);
  pbr.retroreflection = 0.0f;
  if ((feature_mask & EE_GLTF_FEATURE_VOLUME) != 0u) {
    pbr.attenuation_color = material.attenuation_color;
    pbr.attenuation_distance = material.attenuation_distance;
    pbr.thickness = material.thickness_factor;
  }
  float dielectric_ior = 1.5f;
  if ((feature_mask & EE_GLTF_FEATURE_IOR) != 0u) {
    dielectric_ior = material.ior == 0.0f ? 0.0f : max(material.ior, 1.0f);
    pbr.ior2 = material.ior == 0.0f ? EE_GLTF_RT_IOR_INFINITY_SURROGATE : max(material.ior, 1.0f);
  }
  if ((feature_mask & EE_GLTF_FEATURE_SPECULAR_GLOSSINESS) == 0u ||
      material.pbr_model != EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
    const float dielectric_f0 = EE_GLTF_RT_IOR_TO_F0(pbr.ior1, dielectric_ior);
    pbr.specular_f0 = float3(dielectric_f0);
  }
  if (is_inside && pbr.thickness > 0.0f) {
    pbr.ior1 = pbr.ior2;
    pbr.ior2 = 1.0f;
  }
  if ((feature_mask & EE_GLTF_FEATURE_SPECULAR) != 0u) {
    pbr.specular_color = material.specular_color_factor;
    if (EE_GLTF_HAS_TEXTURE(material.specular_color_texture)) {
      pbr.specular_color *=
          EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(material.specular_color_texture, tex_coords, float4(1.0f))
              .rgb;
    }
    pbr.specular = clamp(material.specular_factor, 0.0f, 1.0f);
    if (EE_GLTF_HAS_TEXTURE(material.specular_texture)) {
      pbr.specular *=
          EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(material.specular_texture, tex_coords, float4(1.0f)).a;
    }
    if ((feature_mask & EE_GLTF_FEATURE_SPECULAR_GLOSSINESS) == 0u ||
        material.pbr_model != EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS) {
      pbr.specular_f0 = clamp(pbr.specular_f0 * max(pbr.specular_color, float3(0.0f)), float3(0.0f), float3(1.0f));
    }
  }
  if ((feature_mask & EE_GLTF_FEATURE_TRANSMISSION) != 0u) {
    pbr.transmission = material.transmission_factor;
    if (EE_GLTF_HAS_TEXTURE(material.transmission_texture)) {
      pbr.transmission *=
          EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(material.transmission_texture, tex_coords, float4(1.0f)).r;
    }
  }
  if ((feature_mask & EE_GLTF_FEATURE_CLEARCOAT) != 0u) {
    pbr.clearcoat = material.clearcoat_factor;
    if (EE_GLTF_HAS_TEXTURE(material.clearcoat_texture)) {
      pbr.clearcoat *=
          EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(material.clearcoat_texture, tex_coords, float4(1.0f)).r;
    }
    pbr.clearcoat_roughness = max(material.clearcoat_roughness, EE_GLTF_RT_BSDF_MIN_ROUGHNESS);
    if (EE_GLTF_HAS_TEXTURE(material.clearcoat_roughness_texture)) {
      pbr.clearcoat_roughness *= EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(material.clearcoat_roughness_texture,
                                                                                  tex_coords, float4(1.0f))
                                     .g;
    }
    if (EE_GLTF_HAS_TEXTURE(material.clearcoat_normal_texture)) {
      float3 normal_vector = EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(
                                 material.clearcoat_normal_texture, tex_coords, float4(0.5f, 0.5f, 1.0f, 1.0f))
                                 .xyz;
      normal_vector = normal_vector * 2.0f - 1.0f;
      normal_vector.xy *= material.clearcoat_normal_texture_scale;
      pbr.clearcoat_normal = EE_GLTF_RT_SAFE_NORMALIZE(
          mul(normal_vector, float3x3(clearcoat_basis_tangent, clearcoat_basis_bitangent, clearcoat_basis_normal)),
          clearcoat_basis_normal);
    }
  }
  if ((feature_mask & EE_GLTF_FEATURE_IRIDESCENCE) != 0u) {
    pbr.iridescence = material.iridescence_factor;
    pbr.iridescence_ior = max(material.iridescence_ior, 1.0f);
    pbr.iridescence_thickness = material.iridescence_thickness_maximum;
    if (EE_GLTF_HAS_TEXTURE(material.iridescence_texture)) {
      pbr.iridescence *=
          EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(material.iridescence_texture, tex_coords, float4(1.0f)).r;
    }
    if (EE_GLTF_HAS_TEXTURE(material.iridescence_thickness_texture)) {
      const float thickness_mix = EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(
                                      material.iridescence_thickness_texture, tex_coords, float4(1.0f))
                                      .g;
      pbr.iridescence_thickness =
          lerp(material.iridescence_thickness_minimum, material.iridescence_thickness_maximum, thickness_mix);
    }
  }
  if ((feature_mask & EE_GLTF_FEATURE_ANISOTROPY) != 0u) {
    float anisotropy_strength = material.anisotropy_strength;
    if (anisotropy_strength > 0.0f) {
      float2 anisotropy_direction = float2(1.0f, 0.0f);
      if (EE_GLTF_HAS_TEXTURE(material.anisotropy_texture)) {
        const float4 anisotropy = EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(
            material.anisotropy_texture, tex_coords, float4(0.5f, 0.5f, 1.0f, 1.0f));
        anisotropy_direction =
            EE_GLTF_RT_SAFE_NORMALIZE(float3(anisotropy.xy * 2.0f - 1.0f, 0.0f), float3(1.0f, 0.0f, 0.0f)).xy;
        anisotropy_strength *= anisotropy.z;
      }
      pbr.roughness.x = lerp(pbr.roughness.y, 1.0f, anisotropy_strength * anisotropy_strength);
      const float c = material.anisotropy_rotation.x;
      const float s = material.anisotropy_rotation.y;
      anisotropy_direction = float2(c * anisotropy_direction.x - s * anisotropy_direction.y,
                                    s * anisotropy_direction.x + c * anisotropy_direction.y);
      pbr.tangent = EE_GLTF_RT_SAFE_NORMALIZE(
          pbr.tangent * anisotropy_direction.x + pbr.bitangent * anisotropy_direction.y, pbr.tangent);
      needs_tangent_update = true;
    }
  }
  if (needs_tangent_update) {
    pbr.bitangent = EE_GLTF_RT_SAFE_NORMALIZE(cross(pbr.normal, pbr.tangent), pbr.bitangent) * basis_handedness;
    pbr.tangent = EE_GLTF_RT_SAFE_NORMALIZE(cross(pbr.bitangent, pbr.normal) * basis_handedness, pbr.tangent);
  }
  if ((feature_mask & EE_GLTF_FEATURE_SHEEN) != 0u) {
    pbr.sheen_color = material.sheen_color_factor;
    if (EE_GLTF_HAS_TEXTURE(material.sheen_color_texture)) {
      pbr.sheen_color *=
          EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(material.sheen_color_texture, tex_coords, float4(1.0f)).rgb;
    }
    pbr.sheen_roughness = max(material.sheen_roughness_factor, EE_GLTF_RT_BSDF_MIN_ROUGHNESS);
    if (EE_GLTF_HAS_TEXTURE(material.sheen_roughness_texture)) {
      pbr.sheen_roughness *=
          EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(material.sheen_roughness_texture, tex_coords, float4(1.0f))
              .a;
    }
  }
  if ((feature_mask & EE_GLTF_FEATURE_DISPERSION) != 0u &&
      ((feature_mask & EE_GLTF_FEATURE_IOR) == 0u || material.ior != 0.0f)) {
    pbr.dispersion = max(material.dispersion, 0.0f);
  }
  if ((feature_mask & EE_GLTF_FEATURE_DIFFUSE_TRANSMISSION) != 0u) {
    pbr.diffuse_transmission_factor = material.diffuse_transmission_factor;
    if (EE_GLTF_HAS_TEXTURE(material.diffuse_transmission_texture)) {
      pbr.diffuse_transmission_factor *= EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(
                                             material.diffuse_transmission_texture, tex_coords, float4(1.0f))
                                             .a;
    }
    pbr.diffuse_transmission_color = material.diffuse_transmission_color;
    if (EE_GLTF_HAS_TEXTURE(material.diffuse_transmission_color_texture)) {
      pbr.diffuse_transmission_color *= EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(
                                            material.diffuse_transmission_color_texture, tex_coords, float4(1.0f))
                                            .rgb;
    }
  }
  if ((feature_mask & EE_GLTF_FEATURE_RETROREFLECTION) != 0u) {
    pbr.retroreflection = clamp(material.retroreflection_factor, 0.0f, 1.0f);
    if (EE_GLTF_HAS_TEXTURE(material.retroreflection_texture)) {
      pbr.retroreflection *=
          EE_GLTF_SAMPLE_TEXTURE_SPECIALIZED<feature_mask>(material.retroreflection_texture, tex_coords, float4(1.0f))
              .r;
    }
  }
  if ((feature_mask & EE_GLTF_FEATURE_VOLUME_SCATTER) != 0u) {
    pbr.scatter_anisotropy = material.scatter_anisotropy;
    if (max(max(material.multiscatter_color_factor.x, material.multiscatter_color_factor.y),
            material.multiscatter_color_factor.z) > 0.0f) {
      const float3 single_scatter_albedo =
          EE_GLTF_RT_MULTI_TO_SINGLE_SCATTER_ALBEDO(material.multiscatter_color_factor);
      const float3 attenuation_coefficient = EE_GLTF_RT_VOLUME_EXTINCTION_COEFFICIENT(pbr);
      pbr.scatter_coefficient = attenuation_coefficient * single_scatter_albedo;
    }
  }
  return pbr;
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(uint material_index, GltfTexCoords tex_coords,
                                                                    float4 vertex_color, GltfRasterMaterial surface,
                                                                    float3 normal, float3 tangent, float3 bitangent,
                                                                    float3 geometric_normal, bool is_inside) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL_SPECIALIZED<EE_GLTF_ALL_FEATURES>(
      material_index, tex_coords, vertex_color, surface, normal, tangent, bitangent, geometric_normal, is_inside);
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(uint material_index, GltfTexCoords tex_coords,
                                                                    float4 vertex_color, float3 normal, float3 tangent,
                                                                    float3 bitangent, float3 geometric_normal,
                                                                    bool is_inside) {
  const GltfRasterMaterial surface = EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coords, vertex_color);
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(material_index, tex_coords, vertex_color, surface, normal, tangent,
                                                   bitangent, geometric_normal, is_inside);
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL_SPECIALIZED<let feature_mask : uint>(
    uint material_index, float2 tex_coord_0, float2 tex_coord_1, float2 tex_coord_2, float2 tex_coord_3,
    float4 vertex_color, GltfRasterMaterial surface, float3 normal, float3 tangent, float3 bitangent,
    float3 geometric_normal, bool is_inside, float4 tex_gradients) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL_SPECIALIZED<feature_mask>(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, tex_gradients),
      vertex_color, surface, normal, tangent, bitangent, geometric_normal, is_inside);
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(
    uint material_index, float2 tex_coord_0, float2 tex_coord_1, float2 tex_coord_2, float2 tex_coord_3,
    float4 vertex_color, GltfRasterMaterial surface, float3 normal, float3 tangent, float3 bitangent,
    float3 geometric_normal, bool is_inside, float4 tex_gradients) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, tex_gradients),
      vertex_color, surface, normal, tangent, bitangent, geometric_normal, is_inside);
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(uint material_index, float2 tex_coord_0,
                                                                    float2 tex_coord_1, float2 tex_coord_2,
                                                                    float2 tex_coord_3, float4 vertex_color,
                                                                    float3 normal, float3 tangent, float3 bitangent,
                                                                    float3 geometric_normal, bool is_inside,
                                                                    float4 tex_gradients) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(
      material_index, EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, tex_coord_2, tex_coord_3, tex_gradients),
      vertex_color, normal, tangent, bitangent, geometric_normal, is_inside);
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(uint material_index, float2 tex_coord_0,
                                                                    float2 tex_coord_1, float4 vertex_color,
                                                                    float3 normal, float3 tangent, float3 bitangent,
                                                                    float3 geometric_normal, bool is_inside,
                                                                    float2 tex_grad) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(
      material_index,
      EE_GLTF_MAKE_TEX_COORDS(tex_coord_0, tex_coord_1, float2(0.0), float2(0.0), float4(tex_grad, 0.0, 0.0)),
      vertex_color, normal, tangent, bitangent, geometric_normal, is_inside);
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(uint material_index, float2 tex_coord_0,
                                                                    float2 tex_coord_1, float4 vertex_color,
                                                                    float3 normal, float3 tangent, float3 bitangent,
                                                                    float3 geometric_normal, bool is_inside,
                                                                    float tex_grad) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(material_index, tex_coord_0, tex_coord_1, vertex_color, normal,
                                                   tangent, bitangent, geometric_normal, is_inside, float2(tex_grad));
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(uint material_index, float2 tex_coord_0,
                                                                    float2 tex_coord_1, float3 normal, float3 tangent,
                                                                    float3 bitangent, float3 geometric_normal,
                                                                    bool is_inside, float2 tex_grad) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(material_index, tex_coord_0, tex_coord_1, float4(1.0), normal,
                                                   tangent, bitangent, geometric_normal, is_inside, tex_grad);
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(uint material_index, float2 tex_coord_0,
                                                                    float2 tex_coord_1, float3 normal, float3 tangent,
                                                                    float3 bitangent, float3 geometric_normal,
                                                                    bool is_inside, float tex_grad) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(material_index, tex_coord_0, tex_coord_1, float4(1.0), normal,
                                                   tangent, bitangent, geometric_normal, is_inside, float2(tex_grad));
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(uint material_index, float2 tex_coord_0,
                                                                    float2 tex_coord_1, float4 vertex_color,
                                                                    float3 normal, float3 tangent, float3 bitangent,
                                                                    float3 geometric_normal, bool is_inside) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(material_index, tex_coord_0, tex_coord_1, vertex_color, normal,
                                                   tangent, bitangent, geometric_normal, is_inside, float2(0.0f));
}
public
GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(uint material_index, float2 tex_coord_0,
                                                                    float2 tex_coord_1, float3 normal, float3 tangent,
                                                                    float3 bitangent, float3 geometric_normal,
                                                                    bool is_inside) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(material_index, tex_coord_0, tex_coord_1, float4(1.0), normal,
                                                   tangent, bitangent, geometric_normal, is_inside, float2(0.0f));
}
public
void EE_GLTF_RT_EVALUATE_DIFFUSE_REFLECTION(inout GltfRayTracingBsdfEvaluateData data,
                                            const GltfRayTracingPbrMaterial material, const float3 tint) {
  const float n_dot_l = dot(data.k2, material.normal);
  if (n_dot_l <= 0.0f) {
    return;
  }
  data.pdf = n_dot_l / EE_GLTF_RT_BSDF_PI;
  data.bsdf_diffuse = tint * data.pdf;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_DIFFUSE_REFLECTION;
}
public
void EE_GLTF_RT_EVALUATE_DIFFUSE_TRANSMISSION(inout GltfRayTracingBsdfEvaluateData data,
                                              const GltfRayTracingPbrMaterial material, const float3 tint) {
  const float n_dot_l = dot(data.k2, material.normal);
  if (-n_dot_l <= 0.0f) {
    return;
  }
  data.pdf = max(0.0f, -n_dot_l / EE_GLTF_RT_BSDF_PI);
  data.bsdf_diffuse = tint * data.pdf;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_DIFFUSE_TRANSMISSION;
}
public
void EE_GLTF_RT_EVALUATE_GGX_REFLECTION_LOBE(inout GltfRayTracingBsdfEvaluateData data,
                                             const GltfRayTracingPbrMaterial material, const int lobe, float3 tint,
                                             const float3 normal, const float3 tangent, const float3 bitangent,
                                             const float2 roughness, const float dielectric_fresnel_weight) {
  const float n_dot_v = abs(dot(data.k1, normal));
  const float n_dot_l = abs(dot(data.k2, normal));
  const float3 half_vector = EE_GLTF_RT_SAFE_NORMALIZE(data.k1 + data.k2, normal);
  const float n_dot_h = dot(normal, half_vector);
  const float v_dot_h = dot(data.k1, half_vector);
  const float l_dot_h = dot(data.k2, half_vector);
  if (n_dot_v <= EE_GLTF_RT_BSDF_EPSILON || n_dot_h <= EE_GLTF_RT_BSDF_EPSILON || v_dot_h < 0.0f || l_dot_h < 0.0f) {
    return;
  }
  const float3 local_half = float3(dot(tangent, half_vector), dot(bitangent, half_vector), n_dot_h);
  data.pdf = EE_GLTF_RT_HVD_GGX_EVAL(float2(1.0f) / max(roughness, float2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS)), local_half);
  float g1;
  float g2;
  const float g12 =
      EE_GLTF_RT_GGX_SMITH_SHADOW_MASK(g1, g2, float3(dot(tangent, data.k1), dot(bitangent, data.k1), n_dot_v),
                                       float3(dot(tangent, data.k2), dot(bitangent, data.k2), n_dot_l),
                                       max(roughness, float2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS)));
  data.pdf *= 0.25f / max(n_dot_v * n_dot_h, EE_GLTF_RT_BSDF_EPSILON);
  const float scattering_pdf = data.pdf;
  if (lobe == EE_GLTF_RT_BSDF_LOBE_SPECULAR_REFLECTION) {
    float3 fresnel = EE_GLTF_RT_WEIGHTED_SPECULAR_FRESNEL(material, v_dot_h);
    if (material.iridescence > 0.0f && material.iridescence_thickness > 0.0f) {
      const float3 film = EE_GLTF_RT_THIN_FILM_FACTOR(material.iridescence_thickness, material.iridescence_ior,
                                                      material.specular * material.specular_f0, material.ior1, v_dot_h);
      fresnel = lerp(fresnel, film, clamp(material.iridescence, 0.0f, 1.0f));
    }
    tint = fresnel / max(dielectric_fresnel_weight, EE_GLTF_RT_BSDF_EPSILON);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_METAL_REFLECTION) {
    const float3 base_fresnel =
        EE_GLTF_RT_FRESNEL_SCHLICK(v_dot_h, clamp(material.base_color, float3(0.0f), float3(1.0f)));
    tint = base_fresnel;
    if (material.iridescence > 0.0f && material.iridescence_thickness > 0.0f) {
      const float3 film = EE_GLTF_RT_THIN_FILM_FACTOR(material.iridescence_thickness, material.iridescence_ior,
                                                      material.base_color, material.ior1, v_dot_h);
      tint = lerp(base_fresnel, film, clamp(material.iridescence, 0.0f, 1.0f));
    }
  }
  data.pdf = scattering_pdf * g1;
  data.bsdf_glossy = float3(g12 * scattering_pdf) * tint;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION;
}
public
void EE_GLTF_RT_EVALUATE_SHEEN_REFLECTION(inout GltfRayTracingBsdfEvaluateData data,
                                          const GltfRayTracingPbrMaterial material) {
  const float n_dot_l = dot(data.k2, material.normal);
  if (n_dot_l <= 0.0f) {
    return;
  }
  const float n_dot_v = abs(dot(data.k1, material.normal));
  const float3 half_vector = EE_GLTF_RT_SAFE_NORMALIZE(data.k1 + data.k2, material.normal);
  const float n_dot_h = dot(material.normal, half_vector);
  const float v_dot_h = dot(data.k1, half_vector);
  const float l_dot_h = dot(data.k2, half_vector);
  if (n_dot_v <= EE_GLTF_RT_BSDF_EPSILON || n_dot_h <= EE_GLTF_RT_BSDF_EPSILON || v_dot_h < 0.0f || l_dot_h < 0.0f) {
    return;
  }
  const float inv_roughness = 1.0f / max(material.sheen_roughness * material.sheen_roughness, EE_GLTF_RT_BSDF_EPSILON);
  data.pdf = EE_GLTF_RT_HVD_SHEEN_EVAL(inv_roughness, n_dot_h);
  float g1;
  float g2;
  const float g12 = EE_GLTF_RT_VCAVITIES_SHADOW_MASK(
      g1, g2, n_dot_h, float3(dot(material.tangent, data.k1), dot(material.bitangent, data.k1), n_dot_v), v_dot_h,
      float3(dot(material.tangent, data.k2), dot(material.bitangent, data.k2), n_dot_l), l_dot_h);
  data.pdf *= 0.25f / max(n_dot_v * n_dot_h, EE_GLTF_RT_BSDF_EPSILON);
  const float scattering_pdf = data.pdf;
  data.pdf = scattering_pdf * g1;
  data.bsdf_glossy = float3(g12 * scattering_pdf) * material.sheen_color;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION;
}
public
void EE_GLTF_RT_SAMPLE_DIFFUSE_REFLECTION(inout GltfRayTracingBsdfSampleData data,
                                          const GltfRayTracingPbrMaterial material, const float3 tint) {
  const float3 sampled_dir = EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(data.xi.xy);
  data.k2 = EE_GLTF_RT_SAFE_NORMALIZE(
      material.tangent * sampled_dir.x + material.bitangent * sampled_dir.y + material.normal * sampled_dir.z,
      material.normal);
  data.pdf = dot(data.k2, material.normal) / EE_GLTF_RT_BSDF_PI;
  data.bsdf_over_pdf = tint;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_DIFFUSE_REFLECTION;
}
public
void EE_GLTF_RT_SAMPLE_DIFFUSE_TRANSMISSION(inout GltfRayTracingBsdfSampleData data,
                                            const GltfRayTracingPbrMaterial material, const float3 tint) {
  const float3 sampled_dir = EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(data.xi.xy);
  data.k2 = EE_GLTF_RT_SAFE_NORMALIZE(
      material.tangent * sampled_dir.x + material.bitangent * sampled_dir.y - material.normal * sampled_dir.z,
      -material.normal);
  data.pdf = dot(data.k2, -material.normal) / EE_GLTF_RT_BSDF_PI;
  data.bsdf_over_pdf = tint;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_DIFFUSE_TRANSMISSION;
}
public
void EE_GLTF_RT_SAMPLE_GGX_REFLECTION_LOBE(inout GltfRayTracingBsdfSampleData data,
                                           const GltfRayTracingPbrMaterial material, const int lobe, float3 tint,
                                           const float3 normal, const float3 tangent, const float3 bitangent,
                                           const float2 roughness, const float dielectric_fresnel_weight) {
  const float n_dot_v = dot(data.k1, normal);
  if (n_dot_v <= 0.0f) {
    return;
  }
  const float3 local_view = float3(dot(data.k1, tangent), dot(data.k1, bitangent), n_dot_v);
  const float3 local_half =
      EE_GLTF_RT_HVD_GGX_SAMPLE_VNDF(local_view, max(roughness, float2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS)), data.xi.xy);
  if (local_half.z <= 0.0f) {
    return;
  }
  const float3 half_vector =
      EE_GLTF_RT_SAFE_NORMALIZE(local_half.x * tangent + local_half.y * bitangent + local_half.z * normal, normal);
  const float v_dot_h = dot(data.k1, half_vector);
  if (v_dot_h <= 0.0f) {
    return;
  }
  data.k2 = EE_GLTF_RT_SAFE_NORMALIZE(2.0f * v_dot_h * half_vector - data.k1, reflect(-data.k1, normal));
  const float n_dot_l = dot(data.k2, normal);
  if (n_dot_l <= 0.0f) {
    return;
  }
  float g1;
  float g2;
  const float g12 = EE_GLTF_RT_GGX_SMITH_SHADOW_MASK(g1, g2, local_view,
                                                     float3(dot(data.k2, tangent), dot(data.k2, bitangent), n_dot_l),
                                                     max(roughness, float2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS)));
  if (g12 <= 0.0f) {
    return;
  }
  data.pdf = EE_GLTF_RT_HVD_GGX_EVAL(float2(1.0f) / max(roughness, float2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS)), local_half) *
             g1 * 0.25f / max(n_dot_v * local_half.z, EE_GLTF_RT_BSDF_EPSILON);
  if (lobe == EE_GLTF_RT_BSDF_LOBE_SPECULAR_REFLECTION) {
    float3 fresnel = EE_GLTF_RT_WEIGHTED_SPECULAR_FRESNEL(material, v_dot_h);
    if (material.iridescence > 0.0f && material.iridescence_thickness > 0.0f) {
      const float3 film = EE_GLTF_RT_THIN_FILM_FACTOR(material.iridescence_thickness, material.iridescence_ior,
                                                      material.specular * material.specular_f0, material.ior1, v_dot_h);
      fresnel = lerp(fresnel, film, clamp(material.iridescence, 0.0f, 1.0f));
    }
    tint = fresnel / max(dielectric_fresnel_weight, EE_GLTF_RT_BSDF_EPSILON);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_METAL_REFLECTION) {
    const float3 base_fresnel =
        EE_GLTF_RT_FRESNEL_SCHLICK(v_dot_h, clamp(material.base_color, float3(0.0f), float3(1.0f)));
    tint = base_fresnel;
    if (material.iridescence > 0.0f && material.iridescence_thickness > 0.0f) {
      const float3 film = EE_GLTF_RT_THIN_FILM_FACTOR(material.iridescence_thickness, material.iridescence_ior,
                                                      material.base_color, material.ior1, v_dot_h);
      tint = lerp(base_fresnel, film, clamp(material.iridescence, 0.0f, 1.0f));
    }
  }
  data.bsdf_over_pdf = float3(g2) * tint;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION;
}
public
void EE_GLTF_RT_SAMPLE_SHEEN_REFLECTION(inout GltfRayTracingBsdfSampleData data,
                                        const GltfRayTracingPbrMaterial material) {
  const float n_dot_v = dot(data.k1, material.normal);
  if (n_dot_v <= 0.0f) {
    return;
  }
  const float inv_roughness = 1.0f / max(material.sheen_roughness * material.sheen_roughness, EE_GLTF_RT_BSDF_EPSILON);
  const float3 local_view = float3(dot(data.k1, material.tangent), dot(data.k1, material.bitangent), n_dot_v);
  float3 local_half = EE_GLTF_RT_HVD_SHEEN_SAMPLE(data.xi.xy, inv_roughness);
  local_half = EE_GLTF_RT_FLIP_SHEEN_HALF_VECTOR(local_half, local_view, data.xi.z);
  if (local_half.z <= 0.0f) {
    return;
  }
  const float3 half_vector = EE_GLTF_RT_SAFE_NORMALIZE(
      local_half.x * material.tangent + local_half.y * material.bitangent + local_half.z * material.normal,
      material.normal);
  const float v_dot_h = dot(data.k1, half_vector);
  if (v_dot_h <= 0.0f) {
    return;
  }
  data.k2 = EE_GLTF_RT_SAFE_NORMALIZE(2.0f * v_dot_h * half_vector - data.k1, reflect(-data.k1, material.normal));
  const float n_dot_l = dot(data.k2, material.normal);
  if (n_dot_l <= 0.0f) {
    return;
  }
  const float l_dot_h = abs(dot(data.k2, half_vector));
  float g1;
  float g2;
  const float g12 = EE_GLTF_RT_VCAVITIES_SHADOW_MASK(
      g1, g2, local_half.z, local_view, v_dot_h,
      float3(dot(data.k2, material.tangent), dot(data.k2, material.bitangent), n_dot_l), l_dot_h);
  if (g12 <= 0.0f) {
    return;
  }
  data.pdf = EE_GLTF_RT_HVD_SHEEN_EVAL(inv_roughness, local_half.z) * g1 * 0.25f /
             max(n_dot_v * local_half.z, EE_GLTF_RT_BSDF_EPSILON);
  data.bsdf_over_pdf = float3(g12 / max(g1, EE_GLTF_RT_BSDF_EPSILON)) * material.sheen_color;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION;
}
public
bool EE_GLTF_RT_IS_REFLECTION_LOBE(const int lobe) {
  return lobe != EE_GLTF_RT_BSDF_LOBE_SPECULAR_TRANSMISSION && lobe != EE_GLTF_RT_BSDF_LOBE_DIFFUSE_TRANSMISSION;
}
public
void EE_GLTF_RT_BSDF_EVALUATE_LOBE(inout GltfRayTracingBsdfEvaluateData data, const GltfRayTracingPbrMaterial material,
                                   const int lobe, const GltfRayTracingBsdfLobeWeights weights) {
  if (lobe == EE_GLTF_RT_BSDF_LOBE_DIFFUSE_REFLECTION) {
    EE_GLTF_RT_EVALUATE_DIFFUSE_REFLECTION(data, material, weights.tint);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_DIFFUSE_TRANSMISSION) {
    EE_GLTF_RT_EVALUATE_DIFFUSE_TRANSMISSION(data, material, material.diffuse_transmission_color);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_SPECULAR_REFLECTION) {
    EE_GLTF_RT_EVALUATE_GGX_REFLECTION_LOBE(data, material, lobe, weights.specular_tint, material.normal,
                                            material.tangent, material.bitangent, material.roughness,
                                            weights.dielectric_fresnel_weight);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_SPECULAR_TRANSMISSION) {
    EE_GLTF_RT_EVALUATE_GGX_TRANSMISSION(data, material, weights.tint);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_METAL_REFLECTION) {
    EE_GLTF_RT_EVALUATE_GGX_REFLECTION_LOBE(data, material, lobe, material.base_color, material.normal,
                                            material.tangent, material.bitangent, material.roughness, 1.0f);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_CLEARCOAT_REFLECTION) {
    const float3 clearcoat_tangent = EE_GLTF_RT_SAFE_NORMALIZE(
        material.tangent - material.clearcoat_normal * dot(material.tangent, material.clearcoat_normal),
        material.tangent);
    const float3 clearcoat_bitangent =
        EE_GLTF_RT_SAFE_NORMALIZE(cross(material.clearcoat_normal, clearcoat_tangent), material.bitangent);
    EE_GLTF_RT_EVALUATE_GGX_REFLECTION_LOBE(
        data, material, lobe, float3(1.0f), material.clearcoat_normal, clearcoat_tangent, clearcoat_bitangent,
        float2(max(material.clearcoat_roughness * material.clearcoat_roughness, EE_GLTF_RT_BSDF_MIN_ROUGHNESS)), 1.0f);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_SHEEN_REFLECTION) {
    EE_GLTF_RT_EVALUATE_SHEEN_REFLECTION(data, material);
  }
}
public
void EE_GLTF_RT_FINALIZE_BSDF_EVALUATION(inout GltfRayTracingBsdfEvaluateData data,
                                         const GltfRayTracingPbrMaterial material) {
  data.bsdf_diffuse = EE_GLTF_RT_SANITIZE(data.bsdf_diffuse);
  data.bsdf_glossy = EE_GLTF_RT_SANITIZE(data.bsdf_glossy);
  data.bsdf_diffuse *= max(material.occlusion, 0.0f);
  data.bsdf_glossy *= max(material.occlusion, 0.0f);
}
public
void EE_GLTF_RT_BSDF_EVALUATE(inout GltfRayTracingBsdfEvaluateData data, const GltfRayTracingPbrMaterial material) {
  data.k1 = EE_GLTF_RT_SAFE_NORMALIZE(data.k1, material.normal);
  data.k2 = EE_GLTF_RT_SAFE_NORMALIZE(data.k2, material.normal);
  data.bsdf_diffuse = float3(0.0f);
  data.bsdf_glossy = float3(0.0f);
  data.pdf = 0.0f;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
  const GltfRayTracingBsdfLobeWeights weights =
      EE_GLTF_RT_COMPUTE_LOBE_WEIGHTS(material, dot(material.normal, data.k1));
  const int lobe = EE_GLTF_RT_FIND_BSDF_LOBE(weights, data.xi.z);
  EE_GLTF_RT_BSDF_EVALUATE_LOBE(data, material, lobe, weights);
  EE_GLTF_RT_FINALIZE_BSDF_EVALUATION(data, material);
  const float retroreflection = clamp(material.retroreflection, 0.0f, 1.0f);
  if (retroreflection > 0.0f && EE_GLTF_RT_IS_REFLECTION_LOBE(lobe)) {
    GltfRayTracingBsdfEvaluateData retro_data;
    retro_data.k1 = EE_GLTF_RT_SAFE_NORMALIZE(reflect(-data.k1, material.normal), material.normal);
    retro_data.k2 = data.k2;
    retro_data.xi = data.xi;
    retro_data.bsdf_diffuse = float3(0.0f);
    retro_data.bsdf_glossy = float3(0.0f);
    retro_data.pdf = 0.0f;
    retro_data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    const GltfRayTracingBsdfLobeWeights retro_weights =
        EE_GLTF_RT_COMPUTE_LOBE_WEIGHTS(material, dot(material.normal, retro_data.k1));
    EE_GLTF_RT_BSDF_EVALUATE_LOBE(retro_data, material, lobe, retro_weights);
    EE_GLTF_RT_FINALIZE_BSDF_EVALUATION(retro_data, material);
    data.bsdf_diffuse = lerp(data.bsdf_diffuse, retro_data.bsdf_diffuse, retroreflection);
    data.bsdf_glossy = lerp(data.bsdf_glossy, retro_data.bsdf_glossy, retroreflection);
    data.pdf = lerp(data.pdf, retro_data.pdf, retroreflection);
  }
  data.bsdf_diffuse = EE_GLTF_RT_SANITIZE(data.bsdf_diffuse);
  data.bsdf_glossy = EE_GLTF_RT_SANITIZE(data.bsdf_glossy);
}
public
void EE_GLTF_RT_BSDF_SAMPLE_LOBE(inout GltfRayTracingBsdfSampleData data, const GltfRayTracingPbrMaterial material,
                                 const int lobe, const GltfRayTracingBsdfLobeWeights weights) {
  data.k2 = float3(0.0f);
  data.pdf = 0.0f;
  data.bsdf_over_pdf = float3(0.0f);
  data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
  if (lobe == EE_GLTF_RT_BSDF_LOBE_DIFFUSE_REFLECTION) {
    EE_GLTF_RT_SAMPLE_DIFFUSE_REFLECTION(data, material, weights.tint);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_DIFFUSE_TRANSMISSION) {
    EE_GLTF_RT_SAMPLE_DIFFUSE_TRANSMISSION(data, material, material.diffuse_transmission_color);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_SPECULAR_REFLECTION) {
    EE_GLTF_RT_SAMPLE_GGX_REFLECTION_LOBE(data, material, lobe, weights.specular_tint, material.normal,
                                          material.tangent, material.bitangent, material.roughness,
                                          weights.dielectric_fresnel_weight);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_SPECULAR_TRANSMISSION) {
    EE_GLTF_RT_SAMPLE_GGX_TRANSMISSION(data, material, weights.tint);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_METAL_REFLECTION) {
    EE_GLTF_RT_SAMPLE_GGX_REFLECTION_LOBE(data, material, lobe, material.base_color, material.normal, material.tangent,
                                          material.bitangent, material.roughness, 1.0f);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_CLEARCOAT_REFLECTION) {
    const float3 clearcoat_tangent = EE_GLTF_RT_SAFE_NORMALIZE(
        material.tangent - material.clearcoat_normal * dot(material.tangent, material.clearcoat_normal),
        material.tangent);
    const float3 clearcoat_bitangent =
        EE_GLTF_RT_SAFE_NORMALIZE(cross(material.clearcoat_normal, clearcoat_tangent), material.bitangent);
    EE_GLTF_RT_SAMPLE_GGX_REFLECTION_LOBE(
        data, material, lobe, float3(1.0f), material.clearcoat_normal, clearcoat_tangent, clearcoat_bitangent,
        float2(max(material.clearcoat_roughness * material.clearcoat_roughness, EE_GLTF_RT_BSDF_MIN_ROUGHNESS)), 1.0f);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_SHEEN_REFLECTION) {
    EE_GLTF_RT_SAMPLE_SHEEN_REFLECTION(data, material);
  }
}
public
void EE_GLTF_RT_BSDF_SAMPLE(inout GltfRayTracingBsdfSampleData data, const GltfRayTracingPbrMaterial material) {
  const float3 forward_k1 = EE_GLTF_RT_SAFE_NORMALIZE(data.k1, material.normal);
  data.k1 = forward_k1;
  const GltfRayTracingBsdfLobeWeights weights =
      EE_GLTF_RT_COMPUTE_LOBE_WEIGHTS(material, dot(material.normal, data.k1));
  if (EE_GLTF_RT_LOBE_WEIGHT_SUM(weights) <= EE_GLTF_RT_BSDF_EPSILON) {
    data.k2 = float3(0.0f);
    data.pdf = 0.0f;
    data.bsdf_over_pdf = float3(0.0f);
    data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    return;
  }
  const int lobe = EE_GLTF_RT_FIND_BSDF_LOBE(weights, data.xi.z);
  const float retroreflection = clamp(material.retroreflection, 0.0f, 1.0f);
  const bool has_retroreflection = retroreflection > 0.0f && EE_GLTF_RT_IS_REFLECTION_LOBE(lobe);
  GltfRayTracingBsdfLobeWeights sample_weights = weights;
  if (has_retroreflection) {
    const float path_random = EE_GLTF_RT_RERANDOMIZE(data.xi.z);
    const bool use_retroreflection = path_random < retroreflection;
    data.xi.z = EE_GLTF_RT_RERANDOMIZE(path_random);
    if (use_retroreflection) {
      data.k1 = EE_GLTF_RT_SAFE_NORMALIZE(reflect(-forward_k1, material.normal), material.normal);
      sample_weights = EE_GLTF_RT_COMPUTE_LOBE_WEIGHTS(material, dot(material.normal, data.k1));
    }
  }
  EE_GLTF_RT_BSDF_SAMPLE_LOBE(data, material, lobe, sample_weights);
  if (has_retroreflection && data.event_type != EE_GLTF_RT_BSDF_EVENT_ABSORB && data.pdf != EE_GLTF_RT_BSDF_DIRAC_PDF) {
    GltfRayTracingBsdfEvaluateData forward_data;
    forward_data.k1 = forward_k1;
    forward_data.k2 = data.k2;
    forward_data.xi = data.xi;
    forward_data.bsdf_diffuse = float3(0.0f);
    forward_data.bsdf_glossy = float3(0.0f);
    forward_data.pdf = 0.0f;
    forward_data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    EE_GLTF_RT_BSDF_EVALUATE_LOBE(forward_data, material, lobe, weights);
    GltfRayTracingBsdfEvaluateData retro_data;
    retro_data.k1 = EE_GLTF_RT_SAFE_NORMALIZE(reflect(-forward_k1, material.normal), material.normal);
    retro_data.k2 = data.k2;
    retro_data.xi = data.xi;
    retro_data.bsdf_diffuse = float3(0.0f);
    retro_data.bsdf_glossy = float3(0.0f);
    retro_data.pdf = 0.0f;
    retro_data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    const GltfRayTracingBsdfLobeWeights retro_weights =
        EE_GLTF_RT_COMPUTE_LOBE_WEIGHTS(material, dot(material.normal, retro_data.k1));
    EE_GLTF_RT_BSDF_EVALUATE_LOBE(retro_data, material, lobe, retro_weights);
    data.pdf = lerp(forward_data.pdf, retro_data.pdf, retroreflection);
    const float3 mixture_bsdf = lerp(forward_data.bsdf_diffuse + forward_data.bsdf_glossy,
                                     retro_data.bsdf_diffuse + retro_data.bsdf_glossy, retroreflection);
    data.bsdf_over_pdf = data.pdf > EE_GLTF_RT_BSDF_EPSILON ? mixture_bsdf / data.pdf : float3(0.0f);
  }
  data.k1 = forward_k1;
  if (data.pdf <= EE_GLTF_RT_BSDF_MIN_PDF || any(isnan(data.bsdf_over_pdf))) {
    data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
  }
  if ((isnan(data.pdf) || isinf(data.pdf)) && data.event_type != EE_GLTF_RT_BSDF_EVENT_ABSORB) {
    data.event_type = (data.event_type & (~EE_GLTF_RT_BSDF_EVENT_GLOSSY)) | EE_GLTF_RT_BSDF_EVENT_IMPULSE;
    data.pdf = EE_GLTF_RT_BSDF_DIRAC_PDF;
  }
}
