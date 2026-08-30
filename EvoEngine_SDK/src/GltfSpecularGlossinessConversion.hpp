#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include <glm/glm.hpp>

namespace evo_engine::gltf_import {

struct MetallicRoughnessSample {
  glm::vec4 base_color = glm::vec4(1.0f);
  float metallic = 1.0f;
  float roughness = 1.0f;
};

inline float PerceivedBrightness(const glm::vec3& color) {
  return std::sqrt(0.299f * color.r * color.r + 0.587f * color.g * color.g + 0.114f * color.b * color.b);
}

inline MetallicRoughnessSample ConvertSpecularGlossiness(const glm::vec4& diffuse, const glm::vec3& specular,
                                                         const float glossiness) {
  constexpr float dielectric_specular = 0.04f;
  constexpr float epsilon = 1e-6f;
  const glm::vec3 clamped_specular = glm::clamp(specular, glm::vec3(0.0f), glm::vec3(1.0f));
  const float one_minus_specular_strength =
      1.0f - std::max({clamped_specular.r, clamped_specular.g, clamped_specular.b});
  const float diffuse_brightness = PerceivedBrightness(glm::vec3(diffuse));
  const float specular_brightness = PerceivedBrightness(clamped_specular);

  float metallic = 0.0f;
  if (specular_brightness >= dielectric_specular) {
    const float a = dielectric_specular;
    const float b = diffuse_brightness * one_minus_specular_strength / (1.0f - dielectric_specular) +
                    specular_brightness - 2.0f * dielectric_specular;
    const float c = dielectric_specular - specular_brightness;
    const float discriminant = std::max(b * b - 4.0f * a * c, 0.0f);
    metallic = std::clamp((-b + std::sqrt(discriminant)) / (2.0f * a), 0.0f, 1.0f);
  }

  const glm::vec3 base_color_from_diffuse =
      glm::vec3(diffuse) *
      (one_minus_specular_strength / (1.0f - dielectric_specular) / std::max(1.0f - metallic, epsilon));
  const glm::vec3 base_color_from_specular =
      (clamped_specular - glm::vec3(dielectric_specular * (1.0f - metallic))) / std::max(metallic, epsilon);
  const glm::vec3 base_color =
      glm::clamp(glm::mix(base_color_from_diffuse, base_color_from_specular, metallic * metallic), glm::vec3(0.0f),
                 glm::vec3(1.0f));

  return {glm::vec4(base_color, std::clamp(diffuse.a, 0.0f, 1.0f)), metallic,
          1.0f - std::clamp(glossiness, 0.0f, 1.0f)};
}

inline float DecodeSrgbChannel(const float value) {
  const float clamped = std::clamp(value, 0.0f, 1.0f);
  return clamped <= 0.04045f ? clamped / 12.92f : std::pow((clamped + 0.055f) / 1.055f, 2.4f);
}

inline float EncodeSrgbChannel(const float value) {
  const float clamped = std::clamp(value, 0.0f, 1.0f);
  return clamped <= 0.0031308f ? clamped * 12.92f : 1.055f * std::pow(clamped, 1.0f / 2.4f) - 0.055f;
}

inline glm::vec3 DecodeSrgb(const glm::vec3& color) {
  return {DecodeSrgbChannel(color.r), DecodeSrgbChannel(color.g), DecodeSrgbChannel(color.b)};
}

inline glm::vec3 EncodeSrgb(const glm::vec3& color) {
  return {EncodeSrgbChannel(color.r), EncodeSrgbChannel(color.g), EncodeSrgbChannel(color.b)};
}

inline std::pair<glm::vec4, glm::vec4> ConvertSpecularGlossinessTexel(const glm::vec4& diffuse_srgb,
                                                                      const glm::vec4& specular_glossiness_srgb,
                                                                      const glm::vec4& diffuse_factor,
                                                                      const glm::vec3& specular_factor,
                                                                      const float glossiness_factor) {
  const glm::vec4 diffuse(DecodeSrgb(glm::vec3(diffuse_srgb)) * glm::vec3(diffuse_factor),
                          diffuse_srgb.a * diffuse_factor.a);
  const glm::vec3 specular = DecodeSrgb(glm::vec3(specular_glossiness_srgb)) * specular_factor;
  const auto converted = ConvertSpecularGlossiness(diffuse, specular, specular_glossiness_srgb.a * glossiness_factor);
  return {glm::vec4(EncodeSrgb(glm::vec3(converted.base_color)), converted.base_color.a),
          glm::vec4(1.0f, converted.roughness, converted.metallic, 1.0f)};
}

}  // namespace evo_engine::gltf_import
