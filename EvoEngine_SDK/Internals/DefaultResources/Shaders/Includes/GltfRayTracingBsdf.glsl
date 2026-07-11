#ifndef EE_GLTF_RAY_TRACING_BSDF_GLSL
#define EE_GLTF_RAY_TRACING_BSDF_GLSL

#include "GltfRasterMaterial.glsl"

/*
 * BSDF lobe weighting and microfacet helper formulas are derived from NVIDIA nvpro_core2
 * nvshaders/bsdf_functions.h.slang and pbr_ggx_microfacet.h.slang (Apache-2.0).
 */

const float EE_GLTF_RT_BSDF_PI = 3.14159265359f;
const float EE_GLTF_RT_BSDF_EPSILON = 1e-6f;
const float EE_GLTF_RT_BSDF_MIN_PDF = 0.00001f;
const float EE_GLTF_RT_BSDF_MIN_ROUGHNESS = 0.0014142f;
const float EE_GLTF_RT_BSDF_DIRAC_PDF = -1.0f;
const float EE_GLTF_RT_IOR_COMPATIBILITY_INFINITY = 1000000.0f;

const int EE_GLTF_RT_BSDF_EVENT_ABSORB = 0;
const int EE_GLTF_RT_BSDF_EVENT_DIFFUSE = 1;
const int EE_GLTF_RT_BSDF_EVENT_GLOSSY = 2;
const int EE_GLTF_RT_BSDF_EVENT_IMPULSE = 4;
const int EE_GLTF_RT_BSDF_EVENT_REFLECTION = 8;
const int EE_GLTF_RT_BSDF_EVENT_TRANSMISSION = 16;
const int EE_GLTF_RT_BSDF_EVENT_DIFFUSE_REFLECTION =
    EE_GLTF_RT_BSDF_EVENT_DIFFUSE | EE_GLTF_RT_BSDF_EVENT_REFLECTION;
const int EE_GLTF_RT_BSDF_EVENT_DIFFUSE_TRANSMISSION =
    EE_GLTF_RT_BSDF_EVENT_DIFFUSE | EE_GLTF_RT_BSDF_EVENT_TRANSMISSION;
const int EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION =
    EE_GLTF_RT_BSDF_EVENT_GLOSSY | EE_GLTF_RT_BSDF_EVENT_REFLECTION;
const int EE_GLTF_RT_BSDF_EVENT_GLOSSY_TRANSMISSION =
    EE_GLTF_RT_BSDF_EVENT_GLOSSY | EE_GLTF_RT_BSDF_EVENT_TRANSMISSION;
const int EE_GLTF_RT_BSDF_EVENT_IMPULSE_REFLECTION =
    EE_GLTF_RT_BSDF_EVENT_IMPULSE | EE_GLTF_RT_BSDF_EVENT_REFLECTION;
const int EE_GLTF_RT_BSDF_EVENT_IMPULSE_TRANSMISSION =
    EE_GLTF_RT_BSDF_EVENT_IMPULSE | EE_GLTF_RT_BSDF_EVENT_TRANSMISSION;

const int EE_GLTF_RT_BSDF_LOBE_DIFFUSE_REFLECTION = 0;
const int EE_GLTF_RT_BSDF_LOBE_SPECULAR_TRANSMISSION = 1;
const int EE_GLTF_RT_BSDF_LOBE_SPECULAR_REFLECTION = 2;
const int EE_GLTF_RT_BSDF_LOBE_METAL_REFLECTION = 3;
const int EE_GLTF_RT_BSDF_LOBE_SHEEN_REFLECTION = 4;
const int EE_GLTF_RT_BSDF_LOBE_CLEARCOAT_REFLECTION = 5;
const int EE_GLTF_RT_BSDF_LOBE_DIFFUSE_TRANSMISSION = 6;

struct GltfRayTracingPbrMaterial {
  vec3 base_color;
  float opacity;
  vec2 roughness;
  float metallic;
  vec3 emissive;
  float occlusion;

  vec3 normal;
  vec3 tangent;
  vec3 bitangent;
  vec3 geometric_normal;

  float ior1;
  float ior2;
  float dispersion;
  float specular;
  vec3 specular_color;
  vec3 specular_f0;
  float transmission;
  vec3 attenuation_color;
  float attenuation_distance;
  float thickness;
  vec3 scatter_coefficient;
  float scatter_anisotropy;
  float clearcoat;
  float clearcoat_roughness;
  vec3 clearcoat_normal;
  float iridescence;
  float iridescence_ior;
  float iridescence_thickness;
  vec3 sheen_color;
  float sheen_roughness;
  float diffuse_transmission_factor;
  vec3 diffuse_transmission_color;
  float retroreflection;
};

struct GltfRayTracingBsdfEvaluateData {
  vec3 k1;
  vec3 k2;
  vec3 xi;
  vec3 bsdf_diffuse;
  vec3 bsdf_glossy;
  float pdf;
  int event_type;
};

struct GltfRayTracingBsdfSampleData {
  vec3 k1;
  vec3 k2;
  vec3 xi;
  float pdf;
  vec3 bsdf_over_pdf;
  int event_type;
};

struct GltfRayTracingBsdfLobeWeights {
  float diffuse_reflection;
  float specular_transmission;
  float specular_reflection;
  float metal_reflection;
  float sheen_reflection;
  float clearcoat_reflection;
  float diffuse_transmission;
  float dielectric_fresnel_weight;
  vec3 tint;
  vec3 specular_tint;
};

vec3 EE_GLTF_RT_SAFE_NORMALIZE(const vec3 value, const vec3 fallback) {
  const float length_squared = dot(value, value);
  return length_squared > 1e-8f ? value * inversesqrt(length_squared) : fallback;
}

vec3 EE_GLTF_RT_SANITIZE(const vec3 value) {
  return vec3(value.x >= 0.0f && value.x < 3.402823466e+38f ? value.x : 0.0f,
              value.y >= 0.0f && value.y < 3.402823466e+38f ? value.y : 0.0f,
              value.z >= 0.0f && value.z < 3.402823466e+38f ? value.z : 0.0f);
}

float EE_GLTF_RT_LUMINANCE(const vec3 color) {
  return dot(color, vec3(0.2126f, 0.7152f, 0.0722f));
}

vec3 EE_GLTF_RT_MULTI_TO_SINGLE_SCATTER_ALBEDO(const vec3 rho_ms) {
  const vec3 clamped_rho = clamp(rho_ms, vec3(0.0f), vec3(1.0f));
  const vec3 t = 4.09712f + 4.20863f * clamped_rho -
                 sqrt(9.59217f + 41.6808f * clamped_rho + 17.7126f * clamped_rho * clamped_rho);
  return clamp(vec3(1.0f) - t * t, vec3(0.0f), vec3(1.0f));
}

vec3 EE_GLTF_RT_VOLUME_EXTINCTION_COEFFICIENT(const GltfRayTracingPbrMaterial material) {
  if (material.attenuation_distance <= EE_GLTF_RT_BSDF_EPSILON) {
    return vec3(0.0f);
  }
  return -log(max(material.attenuation_color, vec3(0.001f))) / max(material.attenuation_distance, 0.001f);
}

vec3 EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(const vec2 xi) {
  const float r = sqrt(clamp(xi.x, 0.0f, 1.0f));
  const float phi = 2.0f * EE_GLTF_RT_BSDF_PI * xi.y;
  return vec3(cos(phi) * r, sin(phi) * r, sqrt(max(0.0f, 1.0f - r * r)));
}

float EE_GLTF_RT_COSINE_HEMISPHERE_PDF(const vec3 normal, const vec3 direction) {
  return max(dot(EE_GLTF_RT_SAFE_NORMALIZE(normal, vec3(0.0f, 1.0f, 0.0f)),
                 EE_GLTF_RT_SAFE_NORMALIZE(direction, vec3(0.0f, 1.0f, 0.0f))),
             0.0f) /
         EE_GLTF_RT_BSDF_PI;
}

float EE_GLTF_RT_DISTRIBUTION_GGX(const vec3 normal, const vec3 half_vector, const float alpha) {
  const float a2 = max(alpha * alpha, EE_GLTF_RT_BSDF_EPSILON);
  const float n_dot_h = max(dot(normal, half_vector), 0.0f);
  const float n_dot_h2 = n_dot_h * n_dot_h;
  const float denominator = EE_GLTF_RT_BSDF_PI * pow(n_dot_h2 * (a2 - 1.0f) + 1.0f, 2.0f);
  return a2 / max(denominator, EE_GLTF_RT_BSDF_EPSILON);
}

float EE_GLTF_RT_GEOMETRY_SCHLICK_GGX(const float n_dot_v, const float alpha) {
  const float k = (alpha + 1.0f) * (alpha + 1.0f) / 8.0f;
  return n_dot_v / max(n_dot_v * (1.0f - k) + k, EE_GLTF_RT_BSDF_EPSILON);
}

float EE_GLTF_RT_GEOMETRY_SMITH(const vec3 normal, const vec3 view_direction, const vec3 light_direction,
                                const float alpha) {
  return EE_GLTF_RT_GEOMETRY_SCHLICK_GGX(max(dot(normal, view_direction), 0.0f), alpha) *
         EE_GLTF_RT_GEOMETRY_SCHLICK_GGX(max(dot(normal, light_direction), 0.0f), alpha);
}

vec3 EE_GLTF_RT_FRESNEL_SCHLICK(const float cos_theta, const vec3 f0) {
  return f0 + (vec3(1.0f) - f0) * pow(max(1.0f - cos_theta, 0.0f), 5.0f);
}

vec3 EE_GLTF_RT_WEIGHTED_SPECULAR_FRESNEL(const GltfRayTracingPbrMaterial material,
                                           const float cos_theta) {
  return clamp(material.specular, 0.0f, 1.0f) *
         EE_GLTF_RT_FRESNEL_SCHLICK(cos_theta, clamp(material.specular_f0, vec3(0.0f), vec3(1.0f)));
}

float EE_GLTF_RT_FRESNEL_COSINE_APPROXIMATION(const float v_dot_n, const float roughness) {
  return mix(v_dot_n, sqrt(0.5f + 0.5f * v_dot_n), sqrt(clamp(roughness, 0.0f, 1.0f)));
}

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

float EE_GLTF_RT_IOR_TO_F0(const float transmitted_ior, const float incident_ior) {
  const float ratio = (transmitted_ior - incident_ior) /
                      max(transmitted_ior + incident_ior, EE_GLTF_RT_BSDF_EPSILON);
  return ratio * ratio;
}

vec3 EE_GLTF_RT_IOR_TO_F0(const vec3 transmitted_ior, const float incident_ior) {
  const vec3 ratio = (transmitted_ior - incident_ior) /
                     max(transmitted_ior + incident_ior, vec3(EE_GLTF_RT_BSDF_EPSILON));
  return ratio * ratio;
}

vec3 EE_GLTF_RT_F0_TO_IOR(const vec3 f0) {
  const vec3 sqrt_f0 = sqrt(clamp(f0, vec3(0.0f), vec3(0.9999f)));
  return (vec3(1.0f) + sqrt_f0) / max(vec3(1.0f) - sqrt_f0,
                                      vec3(EE_GLTF_RT_BSDF_EPSILON));
}

vec3 EE_GLTF_RT_IRIDESCENCE_SENSITIVITY(const float optical_path_difference, const vec3 shift) {
  const float phase = 2.0f * EE_GLTF_RT_BSDF_PI * optical_path_difference * 1.0e-9f;
  const vec3 value = vec3(5.4856e-13f, 4.4201e-13f, 5.2481e-13f);
  const vec3 position = vec3(1.6810e+06f, 1.7953e+06f, 2.2084e+06f);
  const vec3 variance = vec3(4.3278e+09f, 9.3046e+09f, 6.6121e+09f);
  vec3 xyz = value * sqrt(2.0f * EE_GLTF_RT_BSDF_PI * variance) *
             cos(position * phase + shift) * exp(-(phase * phase) * variance);
  xyz.x += 9.7470e-14f * sqrt(2.0f * EE_GLTF_RT_BSDF_PI * 4.5282e+09f) *
           cos(2.2399e+06f * phase + shift.x) * exp(-4.5282e+09f * phase * phase);
  xyz /= 1.0685e-7f;
  const mat3 xyz_to_rec709 = mat3(3.2404542f, -0.9692660f, 0.0556434f,
                                   -1.5371385f, 1.8760108f, -0.2040259f,
                                   -0.4985314f, 0.0415560f, 1.0572252f);
  return xyz_to_rec709 * xyz;
}

vec3 EE_GLTF_RT_THIN_FILM_FACTOR(const float coating_thickness, const float coating_ior,
                                 const vec3 base_f0, const float incoming_ior, const float kh) {
  const float thickness = max(coating_thickness, 0.0f);
  const float film_ior = mix(max(incoming_ior, 1.0f), max(coating_ior, 1.0f),
                             smoothstep(0.0f, 0.03f, thickness));
  const float cos_theta_1 = clamp(kh, 0.0f, 1.0f);
  const float eta = max(incoming_ior, 1.0f) / film_ior;
  const float sin_theta_2_squared = eta * eta * (1.0f - cos_theta_1 * cos_theta_1);
  const float cos_theta_2_squared = 1.0f - sin_theta_2_squared;
  if (cos_theta_2_squared < 0.0f) {
    return vec3(1.0f);
  }

  const float cos_theta_2 = sqrt(cos_theta_2_squared);
  const float r0 = EE_GLTF_RT_IOR_TO_F0(film_ior, max(incoming_ior, 1.0f));
  const float r12 = EE_GLTF_RT_FRESNEL_SCHLICK(cos_theta_1, vec3(r0)).x;
  const float transmission_121 = 1.0f - r12;
  const float phi_12 = film_ior < incoming_ior ? EE_GLTF_RT_BSDF_PI : 0.0f;
  const float phi_21 = EE_GLTF_RT_BSDF_PI - phi_12;

  const vec3 base_ior = EE_GLTF_RT_F0_TO_IOR(base_f0);
  const vec3 r1 = EE_GLTF_RT_IOR_TO_F0(base_ior, film_ior);
  const vec3 r23 = EE_GLTF_RT_FRESNEL_SCHLICK(cos_theta_2, r1);
  const vec3 phi_23 = vec3(base_ior.x < film_ior ? EE_GLTF_RT_BSDF_PI : 0.0f,
                           base_ior.y < film_ior ? EE_GLTF_RT_BSDF_PI : 0.0f,
                           base_ior.z < film_ior ? EE_GLTF_RT_BSDF_PI : 0.0f);
  const float optical_path_difference = 2.0f * film_ior * thickness * cos_theta_2;
  const vec3 phase = vec3(phi_21) + phi_23;

  const vec3 r123 = clamp(r12 * r23, vec3(1.0e-5f), vec3(0.9999f));
  const vec3 root_r123 = sqrt(r123);
  const vec3 multiple_scatter = transmission_121 * transmission_121 * r23 /
                                max(vec3(1.0f) - r123, vec3(EE_GLTF_RT_BSDF_EPSILON));
  vec3 result = vec3(r12) + multiple_scatter;
  vec3 harmonic = multiple_scatter - vec3(transmission_121);
  for (int order = 1; order <= 2; ++order) {
    harmonic *= root_r123;
    result += harmonic * 2.0f *
              EE_GLTF_RT_IRIDESCENCE_SENSITIVITY(float(order) * optical_path_difference,
                                                  float(order) * phase);
  }
  return max(result, vec3(0.0f));
}

float EE_GLTF_RT_HVD_GGX_EVAL(const vec2 inv_roughness, const vec3 h) {
  const float x = h.x * inv_roughness.x;
  const float y = h.y * inv_roughness.y;
  const float aniso = x * x + y * y;
  const float f = aniso + h.z * h.z;
  return (1.0f / EE_GLTF_RT_BSDF_PI) * inv_roughness.x * inv_roughness.y * h.z /
         max(f * f, EE_GLTF_RT_BSDF_EPSILON);
}

vec3 EE_GLTF_RT_HVD_GGX_SAMPLE_VNDF(const vec3 k, const vec2 roughness, const vec2 xi) {
  const vec3 v = EE_GLTF_RT_SAFE_NORMALIZE(vec3(k.x * roughness.x, k.y * roughness.y, k.z),
                                           vec3(0.0f, 0.0f, 1.0f));
  const vec3 t1 = v.z < 0.99999f ? EE_GLTF_RT_SAFE_NORMALIZE(cross(v, vec3(0.0f, 0.0f, 1.0f)),
                                                             vec3(1.0f, 0.0f, 0.0f))
                                 : vec3(1.0f, 0.0f, 0.0f);
  const vec3 t2 = cross(t1, v);
  const float a = 1.0f / max(1.0f + v.z, EE_GLTF_RT_BSDF_EPSILON);
  const float r = sqrt(clamp(xi.x, 0.0f, 1.0f));
  const float phi = xi.y < a ? xi.y / a * EE_GLTF_RT_BSDF_PI
                             : EE_GLTF_RT_BSDF_PI + (xi.y - a) / max(1.0f - a, EE_GLTF_RT_BSDF_EPSILON) *
                                                           EE_GLTF_RT_BSDF_PI;
  const float p1 = r * cos(phi);
  const float p2 = r * sin(phi) * (xi.y < a ? 1.0f : v.z);
  vec3 h = p1 * t1 + p2 * t2 + sqrt(max(0.0f, 1.0f - p1 * p1 - p2 * p2)) * v;
  h.x *= roughness.x;
  h.y *= roughness.y;
  h.z = max(0.0f, h.z);
  return EE_GLTF_RT_SAFE_NORMALIZE(h, vec3(0.0f, 0.0f, 1.0f));
}

float EE_GLTF_RT_SMITH_SHADOW_OR_MASK(const vec3 k, const vec2 roughness) {
  const float kz2 = k.z * k.z;
  if (kz2 <= EE_GLTF_RT_BSDF_EPSILON) {
    return 0.0f;
  }
  const float ax = k.x * roughness.x;
  const float ay = k.y * roughness.y;
  const float inv_a2 = (ax * ax + ay * ay) / kz2;
  return 2.0f / (1.0f + sqrt(1.0f + inv_a2));
}

float EE_GLTF_RT_GGX_SMITH_SHADOW_MASK(out float g1, out float g2, const vec3 k1, const vec3 k2,
                                       const vec2 roughness) {
  g1 = EE_GLTF_RT_SMITH_SHADOW_OR_MASK(k1, roughness);
  g2 = EE_GLTF_RT_SMITH_SHADOW_OR_MASK(k2, roughness);
  return g1 * g2;
}

float EE_GLTF_RT_HVD_SHEEN_EVAL(const float inv_roughness, const float n_dot_h) {
  const float sin_theta = sqrt(max(0.0f, 1.0f - n_dot_h * n_dot_h));
  return (inv_roughness + 2.0f) * pow(sin_theta, inv_roughness) * 0.5f *
         (1.0f / EE_GLTF_RT_BSDF_PI) * n_dot_h;
}

vec3 EE_GLTF_RT_HVD_SHEEN_SAMPLE(const vec2 xi, const float inv_roughness) {
  const float phi = 2.0f * EE_GLTF_RT_BSDF_PI * xi.x;
  const float sin_theta = pow(max(1.0f - xi.y, 0.0f), 1.0f / (inv_roughness + 2.0f));
  const float cos_theta = sqrt(max(0.0f, 1.0f - sin_theta * sin_theta));
  return EE_GLTF_RT_SAFE_NORMALIZE(vec3(cos(phi) * sin_theta, sin(phi) * sin_theta, cos_theta),
                                   vec3(0.0f, 0.0f, 1.0f));
}

float EE_GLTF_RT_VCAVITIES_MASK(const float n_dot_h, const float k_dot_h, const float n_dot_k) {
  return min(2.0f * n_dot_h * n_dot_k / max(k_dot_h, EE_GLTF_RT_BSDF_EPSILON), 1.0f);
}

float EE_GLTF_RT_VCAVITIES_SHADOW_MASK(out float g1, out float g2, const float n_dot_h,
                                       const vec3 k1, const float k1_dot_h, const vec3 k2,
                                       const float k2_dot_h) {
  g1 = EE_GLTF_RT_VCAVITIES_MASK(n_dot_h, k1_dot_h, k1.z);
  g2 = EE_GLTF_RT_VCAVITIES_MASK(n_dot_h, k2_dot_h, k2.z);
  return min(g1, g2);
}

vec3 EE_GLTF_RT_FLIP_SHEEN_HALF_VECTOR(const vec3 h, const vec3 k, const float xi) {
  const float a = h.z * k.z;
  const float b = h.x * k.x + h.y * k.y;
  const float kh = max(0.0f, a + b);
  const float kh_flipped = max(0.0f, a - b);
  const float flip_probability = kh_flipped / max(kh + kh_flipped, EE_GLTF_RT_BSDF_EPSILON);
  return xi < flip_probability ? vec3(-h.x, -h.y, h.z) : h;
}

float EE_GLTF_RT_RERANDOMIZE(const float value) {
  uint word = floatBitsToUint(value);
  word = ((word >> ((word >> 28u) + 4u)) ^ word) * 277803737u;
  word = (word >> 22u) ^ word;
  return float(word) / uintBitsToFloat(0x4f800000u);
}

float EE_GLTF_RT_COMPUTE_DISPERSED_IOR(const float base_ior, const float dispersion,
                                       const float wavelength_nm) {
  const float abbe_number = 20.0f / max(dispersion, EE_GLTF_RT_BSDF_EPSILON);
  return max(base_ior + (base_ior - 1.0f) *
                            (523655.0f / max(wavelength_nm * wavelength_nm, EE_GLTF_RT_BSDF_EPSILON) -
                             1.5168f) /
                            abbe_number,
             1.0f);
}

vec3 EE_GLTF_RT_WAVELENGTH_TO_RGB(const float wavelength) {
  vec3 rgb = vec3(0.0f);
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
  return max(rgb, vec3(0.0f));
}

vec2 EE_GLTF_RT_TRANSMISSION_IOR(const GltfRayTracingPbrMaterial material, const float xi,
                                 inout vec3 tint) {
  const float wavelength_min = 399.43862850585765f;
  const float wavelength_max = 668.6617899434457f;
  vec2 ior = vec2(max(material.ior1, 1.0f), max(material.ior2, 1.0f));
  if (material.dispersion > 0.0f) {
    const float wavelength = mix(wavelength_min, wavelength_max, EE_GLTF_RT_RERANDOMIZE(xi));
    if (ior.x > ior.y) {
      ior.x = EE_GLTF_RT_COMPUTE_DISPERSED_IOR(ior.x, material.dispersion, wavelength);
    } else {
      ior.y = EE_GLTF_RT_COMPUTE_DISPERSED_IOR(ior.y, material.dispersion, wavelength);
    }
    tint *= (wavelength_max - wavelength_min) * EE_GLTF_RT_WAVELENGTH_TO_RGB(wavelength);
  }
  return ior;
}

bool EE_GLTF_RT_IS_TIR(const vec2 ior, const float k_dot_h) {
  const float eta = ior.x / max(ior.y, EE_GLTF_RT_BSDF_EPSILON);
  return 1.0f < eta * eta * (1.0f - k_dot_h * k_dot_h);
}

vec3 EE_GLTF_RT_COMPUTE_HALF_VECTOR(const vec3 k1, const vec3 k2, const vec3 normal,
                                    const vec2 ior, const float n_dot_k2, const bool transmission,
                                    const bool thin_walled) {
  vec3 half_vector;
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

vec3 EE_GLTF_RT_REFRACT(const vec3 k, const vec3 normal, const float eta,
                        const float n_dot_k, out bool tir) {
  const float refraction = eta * eta * (1.0f - n_dot_k * n_dot_k);
  tir = 1.0f <= refraction;
  return tir ? EE_GLTF_RT_SAFE_NORMALIZE(normal * (n_dot_k + n_dot_k) - k, normal)
             : EE_GLTF_RT_SAFE_NORMALIZE(-k * eta + normal * (eta * n_dot_k - sqrt(1.0f - refraction)),
                                          -normal);
}

void EE_GLTF_RT_EVALUATE_GGX_TRANSMISSION(inout GltfRayTracingBsdfEvaluateData data,
                                          const GltfRayTracingPbrMaterial material, vec3 tint) {
  const bool thin_walled = material.thickness <= EE_GLTF_RT_BSDF_EPSILON;
  const vec2 roughness = max(material.roughness, vec2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS));
  vec2 ior = EE_GLTF_RT_TRANSMISSION_IOR(material, data.xi.z, tint);

  const float n_dot_k1 = abs(dot(data.k1, material.normal));
  float n_dot_k2 = dot(data.k2, material.normal);
  const bool backside = n_dot_k2 < 0.0f;
  n_dot_k2 = abs(n_dot_k2);

  const vec3 half_vector =
      EE_GLTF_RT_COMPUTE_HALF_VECTOR(data.k1, data.k2, material.normal, ior, n_dot_k2, backside,
                                     thin_walled);
  const float n_dot_h = dot(material.normal, half_vector);
  const float k1_dot_h = dot(data.k1, half_vector);
  const float k2_dot_h = dot(data.k2, half_vector) * (backside ? -1.0f : 1.0f);
  if (n_dot_k1 <= EE_GLTF_RT_BSDF_EPSILON || n_dot_h <= EE_GLTF_RT_BSDF_EPSILON ||
      k1_dot_h < 0.0f || k2_dot_h < 0.0f) {
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

  const vec3 local_half =
      vec3(dot(material.tangent, half_vector), dot(material.bitangent, half_vector), n_dot_h);
  float scattering_pdf = EE_GLTF_RT_HVD_GGX_EVAL(vec2(1.0f) / roughness, local_half);

  float g1;
  float g2;
  const float g12 = EE_GLTF_RT_GGX_SMITH_SHADOW_MASK(
      g1, g2, vec3(dot(material.tangent, data.k1), dot(material.bitangent, data.k1), n_dot_k1),
      vec3(dot(material.tangent, data.k2), dot(material.bitangent, data.k2), n_dot_k2), roughness);

  if (!thin_walled && backside) {
    const float tmp = k1_dot_h * ior.x - k2_dot_h * ior.y;
    if (tmp <= EE_GLTF_RT_BSDF_EPSILON) {
      return;
    }
    scattering_pdf *= k1_dot_h * k2_dot_h /
                      max(n_dot_k1 * n_dot_h * tmp * tmp, EE_GLTF_RT_BSDF_EPSILON);
  } else {
    scattering_pdf *= 0.25f / max(n_dot_k1 * n_dot_h, EE_GLTF_RT_BSDF_EPSILON);
  }

  const float probability = backside ? 1.0f - fresnel : fresnel;
  if (probability <= EE_GLTF_RT_BSDF_EPSILON) {
    return;
  }
  data.pdf = scattering_pdf * probability * g1;
  data.bsdf_glossy = vec3(probability * g12 * scattering_pdf) * tint;
  data.event_type =
      backside ? EE_GLTF_RT_BSDF_EVENT_GLOSSY_TRANSMISSION : EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION;
}

void EE_GLTF_RT_SAMPLE_GGX_TRANSMISSION(inout GltfRayTracingBsdfSampleData data,
                                        const GltfRayTracingPbrMaterial material, vec3 tint) {
  const bool thin_walled = material.thickness <= EE_GLTF_RT_BSDF_EPSILON;
  const vec2 roughness = max(material.roughness, vec2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS));
  vec2 ior = EE_GLTF_RT_TRANSMISSION_IOR(material, data.xi.z, tint);

  const float n_dot_k1 = abs(dot(data.k1, material.normal));
  if (n_dot_k1 <= EE_GLTF_RT_BSDF_EPSILON) {
    return;
  }
  const vec3 local_k1 = vec3(dot(data.k1, material.tangent), dot(data.k1, material.bitangent), n_dot_k1);
  const vec3 local_half = EE_GLTF_RT_HVD_GGX_SAMPLE_VNDF(local_k1, roughness, data.xi.xy);
  if (abs(local_half.z) <= EE_GLTF_RT_BSDF_EPSILON) {
    return;
  }
  const vec3 half_vector =
      EE_GLTF_RT_SAFE_NORMALIZE(local_half.x * material.tangent + local_half.y * material.bitangent +
                                    local_half.z * material.normal,
                                material.normal);
  const float k_dot_h = dot(data.k1, half_vector);
  if (k_dot_h <= EE_GLTF_RT_BSDF_EPSILON) {
    return;
  }

  bool tir = false;
  if (thin_walled) {
    data.k2 = (2.0f * k_dot_h) * half_vector - data.k1;
    data.k2 = EE_GLTF_RT_SAFE_NORMALIZE(data.k2 - 2.0f * material.normal * dot(data.k2, material.normal),
                                        -material.normal);
  } else {
    data.k2 = EE_GLTF_RT_REFRACT(data.k1, half_vector, ior.x / max(ior.y, EE_GLTF_RT_BSDF_EPSILON),
                                 k_dot_h, tir);
  }

  data.bsdf_over_pdf = vec3(1.0f);
  data.event_type = tir ? EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION
                        : EE_GLTF_RT_BSDF_EVENT_GLOSSY_TRANSMISSION;

  const float n_dot_k2 = dot(data.k2, material.normal) * (tir ? 1.0f : -1.0f);
  if (n_dot_k2 <= EE_GLTF_RT_BSDF_EPSILON || isnan(data.k2.x)) {
    data.k2 = vec3(0.0f);
    data.bsdf_over_pdf = vec3(0.0f);
    data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    return;
  }

  const float k2_dot_h = abs(dot(data.k2, half_vector));
  float g1;
  float g2;
  const float g12 = EE_GLTF_RT_GGX_SMITH_SHADOW_MASK(
      g1, g2, local_k1, vec3(dot(data.k2, material.tangent), dot(data.k2, material.bitangent), n_dot_k2),
      roughness);
  if (g12 <= EE_GLTF_RT_BSDF_EPSILON) {
    data.k2 = vec3(0.0f);
    data.bsdf_over_pdf = vec3(0.0f);
    data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    return;
  }

  data.bsdf_over_pdf *= g2;
  data.pdf = EE_GLTF_RT_HVD_GGX_EVAL(vec2(1.0f) / roughness, local_half) * g1;
  if (!thin_walled && data.event_type == EE_GLTF_RT_BSDF_EVENT_GLOSSY_TRANSMISSION) {
    const float tmp = k_dot_h * ior.x - k2_dot_h * ior.y;
    if (tmp <= EE_GLTF_RT_BSDF_EPSILON) {
      data.k2 = vec3(0.0f);
      data.pdf = 0.0f;
      data.bsdf_over_pdf = vec3(0.0f);
      data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
      return;
    }
    data.pdf *= k_dot_h * k2_dot_h /
                max(n_dot_k1 * local_half.z * tmp * tmp, EE_GLTF_RT_BSDF_EPSILON);
  } else {
    data.pdf *= 0.25f / max(n_dot_k1 * local_half.z, EE_GLTF_RT_BSDF_EPSILON);
  }
  data.bsdf_over_pdf *= tint;
}

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
  weights.specular_tint = vec3(1.0f);

  float coat_fresnel = 0.0f;
  if (material.clearcoat > 0.0f) {
    const float coat_cosine =
        EE_GLTF_RT_FRESNEL_COSINE_APPROXIMATION(v_dot_n, material.clearcoat_roughness);
    coat_fresnel = clamp(material.clearcoat * EE_GLTF_RT_IOR_FRESNEL(1.5f / max(material.ior1,
                                                                                  EE_GLTF_RT_BSDF_EPSILON),
                                                                       coat_cosine),
                         0.0f, 1.0f);
  }

  vec3 dielectric_fresnel = vec3(0.0f);
  if (material.specular > 0.0f) {
    const float dielectric_cosine = EE_GLTF_RT_FRESNEL_COSINE_APPROXIMATION(
        v_dot_n, (material.roughness.x + material.roughness.y) * 0.5f);
    dielectric_fresnel = EE_GLTF_RT_WEIGHTED_SPECULAR_FRESNEL(material, dielectric_cosine);
  }

  if (material.iridescence > 0.0f && material.iridescence_thickness > 0.0f) {
    const vec3 iridescence_fresnel = EE_GLTF_RT_THIN_FILM_FACTOR(
        material.iridescence_thickness, material.iridescence_ior,
        material.specular * material.specular_f0,
        material.ior1, v_dot_n);
    dielectric_fresnel = mix(dielectric_fresnel, iridescence_fresnel,
                              clamp(material.iridescence, 0.0f, 1.0f));
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
                              : vec3(0.0f);
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

float EE_GLTF_RT_LOBE_WEIGHT_SUM(const GltfRayTracingBsdfLobeWeights weights) {
  return weights.diffuse_reflection + weights.specular_transmission + weights.specular_reflection +
         weights.metal_reflection + weights.sheen_reflection + weights.clearcoat_reflection +
         weights.diffuse_transmission;
}

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

GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec4 vertex_color, vec3 normal, vec3 tangent,
    vec3 bitangent, vec3 geometric_normal, bool is_inside, vec2 tex_grad) {
  const GltfShadeMaterial material = EE_GLTF_MATERIALS[material_index];
  const GltfRasterMaterial surface =
      EE_EVALUATE_GLTF_RASTER_SURFACE(material_index, tex_coord_0, tex_coord_1, vertex_color, tex_grad);

  GltfRayTracingPbrMaterial pbr;
  pbr.base_color = max(surface.base_color.rgb, vec3(0.0f));
  pbr.opacity = clamp(surface.base_color.a, 0.0f, 1.0f);
  const float roughness = max(surface.roughness, EE_GLTF_RT_BSDF_MIN_ROUGHNESS);
  pbr.roughness = vec2(roughness * roughness);
  pbr.metallic = clamp(surface.metallic, 0.0f, 1.0f);
  pbr.emissive = max(surface.emissive, vec3(0.0f));
  pbr.occlusion = surface.occlusion;

  pbr.normal = EE_GLTF_RT_SAFE_NORMALIZE(normal, geometric_normal);
  pbr.geometric_normal = EE_GLTF_RT_SAFE_NORMALIZE(geometric_normal, pbr.normal);
  pbr.tangent = EE_GLTF_RT_SAFE_NORMALIZE(tangent, vec3(1.0f, 0.0f, 0.0f));
  pbr.bitangent = EE_GLTF_RT_SAFE_NORMALIZE(bitangent, cross(pbr.normal, pbr.tangent));
  const float basis_handedness =
      dot(cross(pbr.normal, pbr.tangent), pbr.bitangent) < 0.0f ? -1.0f : 1.0f;
  bool needs_tangent_update = false;

  if (EE_GLTF_HAS_TEXTURE(material.normal_texture)) {
    vec3 normal_vector =
        EE_GLTF_SAMPLE_TEXTURE(material.normal_texture, tex_coord_0, tex_coord_1,
                               vec4(0.5f, 0.5f, 1.0f, 1.0f), tex_grad).xyz;
    normal_vector = normal_vector * 2.0f - 1.0f;
    normal_vector.xy *= material.normal_texture_scale;
    pbr.normal = EE_GLTF_RT_SAFE_NORMALIZE(mat3(pbr.tangent, pbr.bitangent, pbr.normal) * normal_vector, pbr.normal);
    needs_tangent_update = true;
  }

  pbr.ior1 = 1.0f;
  pbr.ior2 = 1.5f;
  pbr.dispersion = 0.0f;
  pbr.specular = 1.0f;
  pbr.specular_color = vec3(1.0f);
  pbr.specular_f0 = surface.specular_f0;
  pbr.transmission = 0.0f;
  pbr.attenuation_color = vec3(1.0f);
  pbr.attenuation_distance = 1.0f;
  pbr.thickness = 0.0f;
  pbr.scatter_coefficient = vec3(0.0f);
  pbr.scatter_anisotropy = 0.0f;
  pbr.clearcoat = 0.0f;
  pbr.clearcoat_roughness = EE_GLTF_RT_BSDF_MIN_ROUGHNESS;
  pbr.clearcoat_normal = pbr.normal;
  pbr.iridescence = 0.0f;
  pbr.iridescence_ior = 1.3f;
  pbr.iridescence_thickness = 400.0f;
  pbr.sheen_color = vec3(0.0f);
  pbr.sheen_roughness = 0.0f;
  pbr.diffuse_transmission_factor = 0.0f;
  pbr.diffuse_transmission_color = vec3(1.0f);
  pbr.retroreflection = 0.0f;

#if MAT_EXT_VOLUME
  pbr.attenuation_color = material.attenuation_color;
  pbr.attenuation_distance = material.attenuation_distance;
  pbr.thickness = material.thickness_factor;
#endif

  float dielectric_ior = 1.5f;
#if MAT_EXT_IOR
  dielectric_ior = material.ior == 0.0f ? 0.0f : max(material.ior, 1.0f);
  pbr.ior2 = material.ior == 0.0f ? EE_GLTF_RT_IOR_COMPATIBILITY_INFINITY : max(material.ior, 1.0f);
#endif
#if MAT_EXT_SPECULAR_GLOSSINESS
  if (material.pbr_model != EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS)
#endif
  {
    const float dielectric_f0 = pow((dielectric_ior - pbr.ior1) / max(dielectric_ior + pbr.ior1,
                                                                      EE_GLTF_RT_BSDF_EPSILON),
                                    2.0f);
    pbr.specular_f0 = vec3(dielectric_f0);
  }
  if (is_inside && pbr.thickness > 0.0f) {
    pbr.ior1 = pbr.ior2;
    pbr.ior2 = 1.0f;
  }

#if MAT_EXT_SPECULAR
  pbr.specular_color = material.specular_color_factor;
  if (EE_GLTF_HAS_TEXTURE(material.specular_color_texture)) {
    pbr.specular_color *=
        EE_GLTF_SAMPLE_TEXTURE(material.specular_color_texture, tex_coord_0, tex_coord_1, vec4(1.0f),
                               tex_grad).rgb;
  }
  pbr.specular = clamp(material.specular_factor, 0.0f, 1.0f);
  if (EE_GLTF_HAS_TEXTURE(material.specular_texture)) {
    pbr.specular *=
        EE_GLTF_SAMPLE_TEXTURE(material.specular_texture, tex_coord_0, tex_coord_1, vec4(1.0f), tex_grad).a;
  }
#if MAT_EXT_SPECULAR_GLOSSINESS
  if (material.pbr_model != EE_GLTF_PBR_MODEL_SPECULAR_GLOSSINESS)
#endif
  {
    pbr.specular_f0 =
        clamp(pbr.specular_f0 * max(pbr.specular_color, vec3(0.0f)), vec3(0.0f), vec3(1.0f));
  }
#endif

#if MAT_EXT_TRANSMISSION
  pbr.transmission = material.transmission_factor;
  if (EE_GLTF_HAS_TEXTURE(material.transmission_texture)) {
    pbr.transmission *=
        EE_GLTF_SAMPLE_TEXTURE(material.transmission_texture, tex_coord_0, tex_coord_1, vec4(1.0f), tex_grad).r;
  }
#endif

#if MAT_EXT_CLEARCOAT
  pbr.clearcoat = material.clearcoat_factor;
  if (EE_GLTF_HAS_TEXTURE(material.clearcoat_texture)) {
    pbr.clearcoat *=
        EE_GLTF_SAMPLE_TEXTURE(material.clearcoat_texture, tex_coord_0, tex_coord_1, vec4(1.0f), tex_grad).r;
  }
  pbr.clearcoat_roughness = max(material.clearcoat_roughness, EE_GLTF_RT_BSDF_MIN_ROUGHNESS);
  if (EE_GLTF_HAS_TEXTURE(material.clearcoat_roughness_texture)) {
    pbr.clearcoat_roughness *=
        EE_GLTF_SAMPLE_TEXTURE(material.clearcoat_roughness_texture, tex_coord_0, tex_coord_1, vec4(1.0f),
                               tex_grad).g;
  }
  if (EE_GLTF_HAS_TEXTURE(material.clearcoat_normal_texture)) {
    vec3 normal_vector =
        EE_GLTF_SAMPLE_TEXTURE(material.clearcoat_normal_texture, tex_coord_0, tex_coord_1,
                               vec4(0.5f, 0.5f, 1.0f, 1.0f), tex_grad).xyz;
    normal_vector = normal_vector * 2.0f - 1.0f;
    pbr.clearcoat_normal =
        EE_GLTF_RT_SAFE_NORMALIZE(mat3(pbr.tangent, pbr.bitangent, pbr.normal) * normal_vector, pbr.normal);
  }
#endif

#if MAT_EXT_IRIDESCENCE
  pbr.iridescence = material.iridescence_factor;
  pbr.iridescence_ior = max(material.iridescence_ior, 1.0f);
  pbr.iridescence_thickness = material.iridescence_thickness_maximum;
  if (EE_GLTF_HAS_TEXTURE(material.iridescence_texture)) {
    pbr.iridescence *=
        EE_GLTF_SAMPLE_TEXTURE(material.iridescence_texture, tex_coord_0, tex_coord_1, vec4(1.0f), tex_grad).r;
  }
  if (EE_GLTF_HAS_TEXTURE(material.iridescence_thickness_texture)) {
    const float thickness_mix =
        EE_GLTF_SAMPLE_TEXTURE(material.iridescence_thickness_texture, tex_coord_0, tex_coord_1, vec4(1.0f),
                               tex_grad).g;
    pbr.iridescence_thickness =
        mix(material.iridescence_thickness_minimum, material.iridescence_thickness_maximum, thickness_mix);
  }
#endif

#if MAT_EXT_ANISOTROPY
  float anisotropy_strength = material.anisotropy_strength;
  if (anisotropy_strength > 0.0f) {
    vec2 anisotropy_direction = vec2(1.0f, 0.0f);
    if (EE_GLTF_HAS_TEXTURE(material.anisotropy_texture)) {
      const vec4 anisotropy =
          EE_GLTF_SAMPLE_TEXTURE(material.anisotropy_texture, tex_coord_0, tex_coord_1,
                                 vec4(0.5f, 0.5f, 1.0f, 1.0f), tex_grad);
      anisotropy_direction = EE_GLTF_RT_SAFE_NORMALIZE(vec3(anisotropy.xy * 2.0f - 1.0f, 0.0f), vec3(1.0f, 0.0f, 0.0f)).xy;
      anisotropy_strength *= anisotropy.z;
    }
    pbr.roughness.x = mix(pbr.roughness.y, 1.0f, anisotropy_strength * anisotropy_strength);
    const float c = material.anisotropy_rotation.x;
    const float s = material.anisotropy_rotation.y;
    anisotropy_direction =
        vec2(c * anisotropy_direction.x - s * anisotropy_direction.y,
             s * anisotropy_direction.x + c * anisotropy_direction.y);
    pbr.tangent =
        EE_GLTF_RT_SAFE_NORMALIZE(pbr.tangent * anisotropy_direction.x + pbr.bitangent * anisotropy_direction.y,
                                  pbr.tangent);
    needs_tangent_update = true;
  }
#endif

  if (needs_tangent_update) {
    pbr.bitangent =
        EE_GLTF_RT_SAFE_NORMALIZE(cross(pbr.normal, pbr.tangent), pbr.bitangent) * basis_handedness;
    pbr.tangent =
        EE_GLTF_RT_SAFE_NORMALIZE(cross(pbr.bitangent, pbr.normal) * basis_handedness, pbr.tangent);
  }

#if MAT_EXT_SHEEN
  pbr.sheen_color = material.sheen_color_factor;
  if (EE_GLTF_HAS_TEXTURE(material.sheen_color_texture)) {
    pbr.sheen_color *=
        EE_GLTF_SAMPLE_TEXTURE(material.sheen_color_texture, tex_coord_0, tex_coord_1, vec4(1.0f), tex_grad).rgb;
  }
  pbr.sheen_roughness = max(material.sheen_roughness_factor, EE_GLTF_RT_BSDF_MIN_ROUGHNESS);
  if (EE_GLTF_HAS_TEXTURE(material.sheen_roughness_texture)) {
    pbr.sheen_roughness *=
        EE_GLTF_SAMPLE_TEXTURE(material.sheen_roughness_texture, tex_coord_0, tex_coord_1, vec4(1.0f),
                               tex_grad).a;
  }
#endif

#if MAT_EXT_DISPERSION
#if MAT_EXT_IOR
  if (material.ior != 0.0f)
#endif
  pbr.dispersion = max(material.dispersion, 0.0f);
#endif

#if MAT_EXT_DIFFUSE_TRANSMISSION
  pbr.diffuse_transmission_factor = material.diffuse_transmission_factor;
  if (EE_GLTF_HAS_TEXTURE(material.diffuse_transmission_texture)) {
    pbr.diffuse_transmission_factor *=
        EE_GLTF_SAMPLE_TEXTURE(material.diffuse_transmission_texture, tex_coord_0, tex_coord_1, vec4(1.0f),
                               tex_grad).a;
  }
  pbr.diffuse_transmission_color = material.diffuse_transmission_color;
  if (EE_GLTF_HAS_TEXTURE(material.diffuse_transmission_color_texture)) {
    pbr.diffuse_transmission_color *=
        EE_GLTF_SAMPLE_TEXTURE(material.diffuse_transmission_color_texture, tex_coord_0, tex_coord_1, vec4(1.0f),
                               tex_grad).rgb;
  }
#endif

#if MAT_EXT_RETROREFLECTION
  pbr.retroreflection = clamp(material.retroreflection_factor, 0.0f, 1.0f);
  if (EE_GLTF_HAS_TEXTURE(material.retroreflection_texture)) {
    pbr.retroreflection *= EE_GLTF_SAMPLE_TEXTURE(
        material.retroreflection_texture, tex_coord_0, tex_coord_1, vec4(1.0f), tex_grad).r;
  }
#endif

#if MAT_EXT_VOLUME_SCATTER
  pbr.scatter_anisotropy = material.scatter_anisotropy;
  if (max(max(material.multiscatter_color_factor.x, material.multiscatter_color_factor.y),
          material.multiscatter_color_factor.z) > 0.0f) {
    const vec3 single_scatter_albedo = EE_GLTF_RT_MULTI_TO_SINGLE_SCATTER_ALBEDO(material.multiscatter_color_factor);
    const vec3 attenuation_coefficient = EE_GLTF_RT_VOLUME_EXTINCTION_COEFFICIENT(pbr);
    pbr.scatter_coefficient = attenuation_coefficient * single_scatter_albedo;
  }
#endif

  return pbr;
}

GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec4 vertex_color, vec3 normal, vec3 tangent,
    vec3 bitangent, vec3 geometric_normal, bool is_inside, float tex_grad) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(material_index, tex_coord_0, tex_coord_1, vertex_color, normal,
                                                   tangent, bitangent, geometric_normal, is_inside, vec2(tex_grad));
}

GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec3 normal, vec3 tangent, vec3 bitangent,
    vec3 geometric_normal, bool is_inside, vec2 tex_grad) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(material_index, tex_coord_0, tex_coord_1, vec4(1.0), normal,
                                                   tangent, bitangent, geometric_normal, is_inside, tex_grad);
}

GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec3 normal, vec3 tangent, vec3 bitangent,
    vec3 geometric_normal, bool is_inside, float tex_grad) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(material_index, tex_coord_0, tex_coord_1, vec4(1.0), normal,
                                                   tangent, bitangent, geometric_normal, is_inside, vec2(tex_grad));
}

GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec4 vertex_color, vec3 normal, vec3 tangent,
    vec3 bitangent, vec3 geometric_normal, bool is_inside) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(material_index, tex_coord_0, tex_coord_1, vertex_color, normal,
                                                   tangent, bitangent, geometric_normal, is_inside, vec2(0.0f));
}

GltfRayTracingPbrMaterial EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(
    uint material_index, vec2 tex_coord_0, vec2 tex_coord_1, vec3 normal, vec3 tangent, vec3 bitangent,
    vec3 geometric_normal, bool is_inside) {
  return EE_EVALUATE_GLTF_RAY_TRACING_PBR_MATERIAL(material_index, tex_coord_0, tex_coord_1, vec4(1.0), normal,
                                                   tangent, bitangent, geometric_normal, is_inside, vec2(0.0f));
}

void EE_GLTF_RT_EVALUATE_DIFFUSE_REFLECTION(inout GltfRayTracingBsdfEvaluateData data,
                                            const GltfRayTracingPbrMaterial material, const vec3 tint) {
  const float n_dot_l = dot(data.k2, material.normal);
  if (n_dot_l <= 0.0f) {
    return;
  }
  data.pdf = n_dot_l / EE_GLTF_RT_BSDF_PI;
  data.bsdf_diffuse = tint * data.pdf;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_DIFFUSE_REFLECTION;
}

void EE_GLTF_RT_EVALUATE_DIFFUSE_TRANSMISSION(inout GltfRayTracingBsdfEvaluateData data,
                                              const GltfRayTracingPbrMaterial material, const vec3 tint) {
  const float n_dot_l = dot(data.k2, material.normal);
  if (-n_dot_l <= 0.0f) {
    return;
  }
  data.pdf = max(0.0f, -n_dot_l / EE_GLTF_RT_BSDF_PI);
  data.bsdf_diffuse = tint * data.pdf;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_DIFFUSE_TRANSMISSION;
}

void EE_GLTF_RT_EVALUATE_GGX_REFLECTION_LOBE(inout GltfRayTracingBsdfEvaluateData data,
                                              const GltfRayTracingPbrMaterial material, const int lobe,
                                              vec3 tint, const vec3 normal, const vec3 tangent,
                                              const vec3 bitangent, const vec2 roughness,
                                              const float dielectric_fresnel_weight) {
  const float n_dot_v = abs(dot(data.k1, normal));
  const float n_dot_l = abs(dot(data.k2, normal));
  const vec3 half_vector = EE_GLTF_RT_SAFE_NORMALIZE(data.k1 + data.k2, normal);
  const float n_dot_h = dot(normal, half_vector);
  const float v_dot_h = dot(data.k1, half_vector);
  const float l_dot_h = dot(data.k2, half_vector);
  if (n_dot_v <= EE_GLTF_RT_BSDF_EPSILON || n_dot_h <= EE_GLTF_RT_BSDF_EPSILON ||
      v_dot_h < 0.0f || l_dot_h < 0.0f) {
    return;
  }

  const vec3 local_half = vec3(dot(tangent, half_vector), dot(bitangent, half_vector), n_dot_h);
  data.pdf = EE_GLTF_RT_HVD_GGX_EVAL(vec2(1.0f) / max(roughness, vec2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS)),
                                     local_half);

  float g1;
  float g2;
  const float g12 = EE_GLTF_RT_GGX_SMITH_SHADOW_MASK(
      g1, g2, vec3(dot(tangent, data.k1), dot(bitangent, data.k1), n_dot_v),
      vec3(dot(tangent, data.k2), dot(bitangent, data.k2), n_dot_l),
      max(roughness, vec2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS)));
  data.pdf *= 0.25f / max(n_dot_v * n_dot_h, EE_GLTF_RT_BSDF_EPSILON);
  const float scattering_pdf = data.pdf;

  if (lobe == EE_GLTF_RT_BSDF_LOBE_SPECULAR_REFLECTION) {
    vec3 fresnel = EE_GLTF_RT_WEIGHTED_SPECULAR_FRESNEL(material, v_dot_h);
    if (material.iridescence > 0.0f && material.iridescence_thickness > 0.0f) {
      const vec3 film = EE_GLTF_RT_THIN_FILM_FACTOR(
          material.iridescence_thickness, material.iridescence_ior,
          material.specular * material.specular_f0,
          material.ior1, v_dot_h);
      fresnel = mix(fresnel, film, clamp(material.iridescence, 0.0f, 1.0f));
    }
    tint = fresnel / max(dielectric_fresnel_weight, EE_GLTF_RT_BSDF_EPSILON);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_METAL_REFLECTION) {
    const vec3 base_fresnel = EE_GLTF_RT_FRESNEL_SCHLICK(
        v_dot_h, clamp(material.base_color, vec3(0.0f), vec3(1.0f)));
    tint = base_fresnel;
    if (material.iridescence > 0.0f && material.iridescence_thickness > 0.0f) {
      const vec3 film = EE_GLTF_RT_THIN_FILM_FACTOR(
          material.iridescence_thickness, material.iridescence_ior, material.base_color,
          material.ior1, v_dot_h);
      tint = mix(base_fresnel, film, clamp(material.iridescence, 0.0f, 1.0f));
    }
  }

  data.pdf = scattering_pdf * g1;
  data.bsdf_glossy = vec3(g12 * scattering_pdf) * tint;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION;
}

void EE_GLTF_RT_EVALUATE_SHEEN_REFLECTION(inout GltfRayTracingBsdfEvaluateData data,
                                          const GltfRayTracingPbrMaterial material) {
  const float n_dot_l = dot(data.k2, material.normal);
  if (n_dot_l <= 0.0f) {
    return;
  }
  const float n_dot_v = abs(dot(data.k1, material.normal));
  const vec3 half_vector = EE_GLTF_RT_SAFE_NORMALIZE(data.k1 + data.k2, material.normal);
  const float n_dot_h = dot(material.normal, half_vector);
  const float v_dot_h = dot(data.k1, half_vector);
  const float l_dot_h = dot(data.k2, half_vector);
  if (n_dot_v <= EE_GLTF_RT_BSDF_EPSILON || n_dot_h <= EE_GLTF_RT_BSDF_EPSILON ||
      v_dot_h < 0.0f || l_dot_h < 0.0f) {
    return;
  }
  const float inv_roughness = 1.0f / max(material.sheen_roughness * material.sheen_roughness,
                                         EE_GLTF_RT_BSDF_EPSILON);
  data.pdf = EE_GLTF_RT_HVD_SHEEN_EVAL(inv_roughness, n_dot_h);

  float g1;
  float g2;
  const float g12 = EE_GLTF_RT_VCAVITIES_SHADOW_MASK(
      g1, g2, n_dot_h, vec3(dot(material.tangent, data.k1), dot(material.bitangent, data.k1), n_dot_v),
      v_dot_h, vec3(dot(material.tangent, data.k2), dot(material.bitangent, data.k2), n_dot_l), l_dot_h);
  data.pdf *= 0.25f / max(n_dot_v * n_dot_h, EE_GLTF_RT_BSDF_EPSILON);
  const float scattering_pdf = data.pdf;
  data.pdf = scattering_pdf * g1;
  data.bsdf_glossy = vec3(g12 * scattering_pdf) * material.sheen_color;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION;
}

void EE_GLTF_RT_SAMPLE_DIFFUSE_REFLECTION(inout GltfRayTracingBsdfSampleData data,
                                           const GltfRayTracingPbrMaterial material, const vec3 tint) {
  const vec3 sampled_dir = EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(data.xi.xy);
  data.k2 = EE_GLTF_RT_SAFE_NORMALIZE(material.tangent * sampled_dir.x + material.bitangent * sampled_dir.y +
                                          material.normal * sampled_dir.z,
                                      material.normal);
  data.pdf = dot(data.k2, material.normal) / EE_GLTF_RT_BSDF_PI;
  data.bsdf_over_pdf = tint;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_DIFFUSE_REFLECTION;
}

void EE_GLTF_RT_SAMPLE_DIFFUSE_TRANSMISSION(inout GltfRayTracingBsdfSampleData data,
                                             const GltfRayTracingPbrMaterial material, const vec3 tint) {
  const vec3 sampled_dir = EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(data.xi.xy);
  data.k2 = EE_GLTF_RT_SAFE_NORMALIZE(material.tangent * sampled_dir.x + material.bitangent * sampled_dir.y -
                                          material.normal * sampled_dir.z,
                                      -material.normal);
  data.pdf = dot(data.k2, -material.normal) / EE_GLTF_RT_BSDF_PI;
  data.bsdf_over_pdf = tint;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_DIFFUSE_TRANSMISSION;
}

void EE_GLTF_RT_SAMPLE_GGX_REFLECTION_LOBE(inout GltfRayTracingBsdfSampleData data,
                                            const GltfRayTracingPbrMaterial material, const int lobe,
                                            vec3 tint, const vec3 normal, const vec3 tangent,
                                            const vec3 bitangent, const vec2 roughness,
                                            const float dielectric_fresnel_weight) {
  const float n_dot_v = dot(data.k1, normal);
  if (n_dot_v <= 0.0f) {
    return;
  }
  const vec3 local_view = vec3(dot(data.k1, tangent), dot(data.k1, bitangent), n_dot_v);
  const vec3 local_half =
      EE_GLTF_RT_HVD_GGX_SAMPLE_VNDF(local_view, max(roughness, vec2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS)), data.xi.xy);
  if (local_half.z <= 0.0f) {
    return;
  }

  const vec3 half_vector =
      EE_GLTF_RT_SAFE_NORMALIZE(local_half.x * tangent + local_half.y * bitangent + local_half.z * normal,
                                normal);
  const float v_dot_h = dot(data.k1, half_vector);
  if (v_dot_h <= 0.0f) {
    return;
  }
  data.k2 = EE_GLTF_RT_SAFE_NORMALIZE(2.0f * v_dot_h * half_vector - data.k1,
                                      reflect(-data.k1, normal));
  const float n_dot_l = dot(data.k2, normal);
  if (n_dot_l <= 0.0f) {
    return;
  }

  float g1;
  float g2;
  const float g12 = EE_GLTF_RT_GGX_SMITH_SHADOW_MASK(
      g1, g2, local_view, vec3(dot(data.k2, tangent), dot(data.k2, bitangent), n_dot_l),
      max(roughness, vec2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS)));
  if (g12 <= 0.0f) {
    return;
  }

  data.pdf = EE_GLTF_RT_HVD_GGX_EVAL(vec2(1.0f) / max(roughness, vec2(EE_GLTF_RT_BSDF_MIN_ROUGHNESS)),
                                     local_half) *
             g1 * 0.25f / max(n_dot_v * local_half.z, EE_GLTF_RT_BSDF_EPSILON);
  if (lobe == EE_GLTF_RT_BSDF_LOBE_SPECULAR_REFLECTION) {
    vec3 fresnel = EE_GLTF_RT_WEIGHTED_SPECULAR_FRESNEL(material, v_dot_h);
    if (material.iridescence > 0.0f && material.iridescence_thickness > 0.0f) {
      const vec3 film = EE_GLTF_RT_THIN_FILM_FACTOR(
          material.iridescence_thickness, material.iridescence_ior,
          material.specular * material.specular_f0,
          material.ior1, v_dot_h);
      fresnel = mix(fresnel, film, clamp(material.iridescence, 0.0f, 1.0f));
    }
    tint = fresnel / max(dielectric_fresnel_weight, EE_GLTF_RT_BSDF_EPSILON);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_METAL_REFLECTION) {
    const vec3 base_fresnel = EE_GLTF_RT_FRESNEL_SCHLICK(
        v_dot_h, clamp(material.base_color, vec3(0.0f), vec3(1.0f)));
    tint = base_fresnel;
    if (material.iridescence > 0.0f && material.iridescence_thickness > 0.0f) {
      const vec3 film = EE_GLTF_RT_THIN_FILM_FACTOR(
          material.iridescence_thickness, material.iridescence_ior, material.base_color,
          material.ior1, v_dot_h);
      tint = mix(base_fresnel, film, clamp(material.iridescence, 0.0f, 1.0f));
    }
  }
  data.bsdf_over_pdf = vec3(g2) * tint;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION;
}

void EE_GLTF_RT_SAMPLE_SHEEN_REFLECTION(inout GltfRayTracingBsdfSampleData data,
                                        const GltfRayTracingPbrMaterial material) {
  const float n_dot_v = dot(data.k1, material.normal);
  if (n_dot_v <= 0.0f) {
    return;
  }
  const float inv_roughness = 1.0f / max(material.sheen_roughness * material.sheen_roughness,
                                         EE_GLTF_RT_BSDF_EPSILON);
  const vec3 local_view = vec3(dot(data.k1, material.tangent), dot(data.k1, material.bitangent), n_dot_v);
  vec3 local_half = EE_GLTF_RT_HVD_SHEEN_SAMPLE(data.xi.xy, inv_roughness);
  local_half = EE_GLTF_RT_FLIP_SHEEN_HALF_VECTOR(local_half, local_view, data.xi.z);
  if (local_half.z <= 0.0f) {
    return;
  }

  const vec3 half_vector =
      EE_GLTF_RT_SAFE_NORMALIZE(local_half.x * material.tangent + local_half.y * material.bitangent +
                                    local_half.z * material.normal,
                                material.normal);
  const float v_dot_h = dot(data.k1, half_vector);
  if (v_dot_h <= 0.0f) {
    return;
  }
  data.k2 = EE_GLTF_RT_SAFE_NORMALIZE(2.0f * v_dot_h * half_vector - data.k1,
                                      reflect(-data.k1, material.normal));
  const float n_dot_l = dot(data.k2, material.normal);
  if (n_dot_l <= 0.0f) {
    return;
  }

  const float l_dot_h = abs(dot(data.k2, half_vector));
  float g1;
  float g2;
  const float g12 = EE_GLTF_RT_VCAVITIES_SHADOW_MASK(
      g1, g2, local_half.z, local_view, v_dot_h,
      vec3(dot(data.k2, material.tangent), dot(data.k2, material.bitangent), n_dot_l), l_dot_h);
  if (g12 <= 0.0f) {
    return;
  }
  data.pdf = EE_GLTF_RT_HVD_SHEEN_EVAL(inv_roughness, local_half.z) * g1 * 0.25f /
             max(n_dot_v * local_half.z, EE_GLTF_RT_BSDF_EPSILON);
  data.bsdf_over_pdf = vec3(g12 / max(g1, EE_GLTF_RT_BSDF_EPSILON)) * material.sheen_color;
  data.event_type = EE_GLTF_RT_BSDF_EVENT_GLOSSY_REFLECTION;
}

bool EE_GLTF_RT_IS_REFLECTION_LOBE(const int lobe) {
  return lobe != EE_GLTF_RT_BSDF_LOBE_SPECULAR_TRANSMISSION &&
         lobe != EE_GLTF_RT_BSDF_LOBE_DIFFUSE_TRANSMISSION;
}

void EE_GLTF_RT_BSDF_EVALUATE_LOBE(inout GltfRayTracingBsdfEvaluateData data,
                                    const GltfRayTracingPbrMaterial material, const int lobe,
                                    const GltfRayTracingBsdfLobeWeights weights) {
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
    const vec3 clearcoat_tangent =
        EE_GLTF_RT_SAFE_NORMALIZE(material.tangent - material.clearcoat_normal *
                                      dot(material.tangent, material.clearcoat_normal),
                                  material.tangent);
    const vec3 clearcoat_bitangent = EE_GLTF_RT_SAFE_NORMALIZE(cross(material.clearcoat_normal, clearcoat_tangent),
                                                               material.bitangent);
    EE_GLTF_RT_EVALUATE_GGX_REFLECTION_LOBE(
        data, material, lobe, vec3(1.0f), material.clearcoat_normal, clearcoat_tangent, clearcoat_bitangent,
        vec2(max(material.clearcoat_roughness * material.clearcoat_roughness, EE_GLTF_RT_BSDF_MIN_ROUGHNESS)),
        1.0f);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_SHEEN_REFLECTION) {
    EE_GLTF_RT_EVALUATE_SHEEN_REFLECTION(data, material);
  }
}

void EE_GLTF_RT_FINALIZE_BSDF_EVALUATION(inout GltfRayTracingBsdfEvaluateData data,
                                         const GltfRayTracingPbrMaterial material) {
  data.bsdf_diffuse = EE_GLTF_RT_SANITIZE(data.bsdf_diffuse);
  data.bsdf_glossy = EE_GLTF_RT_SANITIZE(data.bsdf_glossy);
  data.bsdf_diffuse *= max(material.occlusion, 0.0f);
  data.bsdf_glossy *= max(material.occlusion, 0.0f);
}

void EE_GLTF_RT_BSDF_EVALUATE(inout GltfRayTracingBsdfEvaluateData data,
                              const GltfRayTracingPbrMaterial material) {
  data.k1 = EE_GLTF_RT_SAFE_NORMALIZE(data.k1, material.normal);
  data.k2 = EE_GLTF_RT_SAFE_NORMALIZE(data.k2, material.normal);
  data.bsdf_diffuse = vec3(0.0f);
  data.bsdf_glossy = vec3(0.0f);
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
    retro_data.bsdf_diffuse = vec3(0.0f);
    retro_data.bsdf_glossy = vec3(0.0f);
    retro_data.pdf = 0.0f;
    retro_data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    const GltfRayTracingBsdfLobeWeights retro_weights =
        EE_GLTF_RT_COMPUTE_LOBE_WEIGHTS(material, dot(material.normal, retro_data.k1));
    EE_GLTF_RT_BSDF_EVALUATE_LOBE(retro_data, material, lobe, retro_weights);
    EE_GLTF_RT_FINALIZE_BSDF_EVALUATION(retro_data, material);
    data.bsdf_diffuse = mix(data.bsdf_diffuse, retro_data.bsdf_diffuse, retroreflection);
    data.bsdf_glossy = mix(data.bsdf_glossy, retro_data.bsdf_glossy, retroreflection);
    data.pdf = mix(data.pdf, retro_data.pdf, retroreflection);
  }
  data.bsdf_diffuse = EE_GLTF_RT_SANITIZE(data.bsdf_diffuse);
  data.bsdf_glossy = EE_GLTF_RT_SANITIZE(data.bsdf_glossy);
}

void EE_GLTF_RT_BSDF_SAMPLE_LOBE(inout GltfRayTracingBsdfSampleData data,
                                  const GltfRayTracingPbrMaterial material, const int lobe,
                                  const GltfRayTracingBsdfLobeWeights weights) {
  data.k2 = vec3(0.0f);
  data.pdf = 0.0f;
  data.bsdf_over_pdf = vec3(0.0f);
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
    EE_GLTF_RT_SAMPLE_GGX_REFLECTION_LOBE(data, material, lobe, material.base_color, material.normal,
                                          material.tangent, material.bitangent, material.roughness, 1.0f);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_CLEARCOAT_REFLECTION) {
    const vec3 clearcoat_tangent =
        EE_GLTF_RT_SAFE_NORMALIZE(material.tangent - material.clearcoat_normal *
                                      dot(material.tangent, material.clearcoat_normal),
                                  material.tangent);
    const vec3 clearcoat_bitangent = EE_GLTF_RT_SAFE_NORMALIZE(cross(material.clearcoat_normal, clearcoat_tangent),
                                                               material.bitangent);
    EE_GLTF_RT_SAMPLE_GGX_REFLECTION_LOBE(
        data, material, lobe, vec3(1.0f), material.clearcoat_normal, clearcoat_tangent, clearcoat_bitangent,
        vec2(max(material.clearcoat_roughness * material.clearcoat_roughness, EE_GLTF_RT_BSDF_MIN_ROUGHNESS)),
        1.0f);
  } else if (lobe == EE_GLTF_RT_BSDF_LOBE_SHEEN_REFLECTION) {
    EE_GLTF_RT_SAMPLE_SHEEN_REFLECTION(data, material);
  }

}

void EE_GLTF_RT_BSDF_SAMPLE(inout GltfRayTracingBsdfSampleData data,
                            const GltfRayTracingPbrMaterial material) {
  const vec3 forward_k1 = EE_GLTF_RT_SAFE_NORMALIZE(data.k1, material.normal);
  data.k1 = forward_k1;
  const GltfRayTracingBsdfLobeWeights weights =
      EE_GLTF_RT_COMPUTE_LOBE_WEIGHTS(material, dot(material.normal, data.k1));
  if (EE_GLTF_RT_LOBE_WEIGHT_SUM(weights) <= EE_GLTF_RT_BSDF_EPSILON) {
    data.k2 = vec3(0.0f);
    data.pdf = 0.0f;
    data.bsdf_over_pdf = vec3(0.0f);
    data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    return;
  }

  const int lobe = EE_GLTF_RT_FIND_BSDF_LOBE(weights, data.xi.z);
  const float retroreflection = clamp(material.retroreflection, 0.0f, 1.0f);
  const bool has_retroreflection =
      retroreflection > 0.0f && EE_GLTF_RT_IS_REFLECTION_LOBE(lobe);
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
  if (has_retroreflection && data.event_type != EE_GLTF_RT_BSDF_EVENT_ABSORB &&
      data.pdf != EE_GLTF_RT_BSDF_DIRAC_PDF) {
    GltfRayTracingBsdfEvaluateData forward_data;
    forward_data.k1 = forward_k1;
    forward_data.k2 = data.k2;
    forward_data.xi = data.xi;
    forward_data.bsdf_diffuse = vec3(0.0f);
    forward_data.bsdf_glossy = vec3(0.0f);
    forward_data.pdf = 0.0f;
    forward_data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    EE_GLTF_RT_BSDF_EVALUATE_LOBE(forward_data, material, lobe, weights);

    GltfRayTracingBsdfEvaluateData retro_data;
    retro_data.k1 = EE_GLTF_RT_SAFE_NORMALIZE(reflect(-forward_k1, material.normal), material.normal);
    retro_data.k2 = data.k2;
    retro_data.xi = data.xi;
    retro_data.bsdf_diffuse = vec3(0.0f);
    retro_data.bsdf_glossy = vec3(0.0f);
    retro_data.pdf = 0.0f;
    retro_data.event_type = EE_GLTF_RT_BSDF_EVENT_ABSORB;
    const GltfRayTracingBsdfLobeWeights retro_weights =
        EE_GLTF_RT_COMPUTE_LOBE_WEIGHTS(material, dot(material.normal, retro_data.k1));
    EE_GLTF_RT_BSDF_EVALUATE_LOBE(retro_data, material, lobe, retro_weights);
    data.pdf = mix(forward_data.pdf, retro_data.pdf, retroreflection);
    const vec3 mixture_bsdf = mix(forward_data.bsdf_diffuse + forward_data.bsdf_glossy,
                                  retro_data.bsdf_diffuse + retro_data.bsdf_glossy,
                                  retroreflection);
    data.bsdf_over_pdf = data.pdf > EE_GLTF_RT_BSDF_EPSILON ? mixture_bsdf / data.pdf : vec3(0.0f);
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

#endif
