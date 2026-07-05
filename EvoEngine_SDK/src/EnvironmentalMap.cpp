#include "EnvironmentalMap.hpp"
#include "AssetManager.hpp"
#include "Texture2D.hpp"
using namespace evo_engine;

namespace {
constexpr float kPi = 3.14159265358979323846f;

float EnvironmentLuminance(const glm::vec3& value) {
  return glm::max(glm::dot(glm::max(value, glm::vec3(0.0f)), glm::vec3(0.2126f, 0.7152f, 0.0722f)), 0.0f);
}

std::shared_ptr<Texture2D> BuildEnvironmentPdfTexture(const std::shared_ptr<Texture2D>& texture) {
  if (!texture)
    return {};

  const auto resolution = texture->GetResolution();
  const auto& pixels = texture->GetLocalData();
  if (resolution.x == 0 || resolution.y == 0 || pixels.size() != static_cast<size_t>(resolution.x) * resolution.y)
    return {};

  const size_t pixel_count = pixels.size();
  std::vector<float> weights(pixel_count, 0.0f);
  std::vector<float> row_weights(resolution.y, 0.0f);
  const float d_azimuth = 2.0f * kPi / static_cast<float>(resolution.x);
  const float d_elevation = kPi / static_cast<float>(resolution.y);
  double total_weight = 0.0;

  for (uint32_t y = 0; y < resolution.y; ++y) {
    const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(resolution.y);
    const float elevation = (v - 0.5f) * kPi;
    const float solid_angle = d_azimuth * d_elevation * glm::max(glm::cos(elevation), 0.0f);
    for (uint32_t x = 0; x < resolution.x; ++x) {
      const size_t index = static_cast<size_t>(y) * resolution.x + x;
      const float weight = EnvironmentLuminance(glm::vec3(pixels[index])) * solid_angle;
      weights[index] = weight;
      row_weights[y] += weight;
      total_weight += weight;
    }
  }

  std::vector<glm::vec4> cdf_pixels(pixel_count, glm::vec4(0.0f));
  const bool has_distribution = total_weight > 0.0;
  double marginal_cdf = 0.0;
  for (uint32_t y = 0; y < resolution.y; ++y) {
    const float row_weight = row_weights[y];
    double conditional_cdf = 0.0;
    marginal_cdf +=
        has_distribution ? static_cast<double>(row_weight) / total_weight : 1.0 / static_cast<double>(resolution.y);
    for (uint32_t x = 0; x < resolution.x; ++x) {
      const size_t index = static_cast<size_t>(y) * resolution.x + x;
      conditional_cdf += has_distribution && row_weight > 0.0f
                             ? static_cast<double>(weights[index]) / static_cast<double>(row_weight)
                             : 1.0 / static_cast<double>(resolution.x);
      const float pdf = has_distribution
                            ? EnvironmentLuminance(glm::vec3(pixels[index])) / static_cast<float>(total_weight)
                            : 1.0f / (4.0f * kPi);
      cdf_pixels[index] = glm::vec4(x + 1u == resolution.x ? 1.0f : static_cast<float>(conditional_cdf),
                                    y + 1u == resolution.y ? 1.0f : static_cast<float>(marginal_cdf), pdf, 1.0f);
    }
  }

  auto pdf_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  pdf_texture->SetRgbaChannelData(cdf_pixels, resolution);
  return pdf_texture;
}
}  // namespace

void EnvironmentalMap::BuildSkyIllumination(const SkyIllumination& sky_illumination, uint32_t resolution) {
  environment_pdf_texture.Clear();
  const auto cubemap = AssetManager::CreateTemporaryAsset<Cubemap>();
  cubemap->BuildSkyIllumination(sky_illumination, resolution);

  light_probe = AssetManager::CreateTemporaryAsset<LightProbe>();
  light_probe.Get<LightProbe>()->ConstructFromCubemap(cubemap);
  reflection_probe = AssetManager::CreateTemporaryAsset<ReflectionProbe>();
  reflection_probe.Get<ReflectionProbe>()->ConstructFromCubemap(cubemap);
}

void EnvironmentalMap::ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap) {
  environment_pdf_texture.Clear();
  light_probe = AssetManager::CreateTemporaryAsset<LightProbe>();
  light_probe.Get<LightProbe>()->ConstructFromCubemap(target_cubemap);
  reflection_probe = AssetManager::CreateTemporaryAsset<ReflectionProbe>();
  reflection_probe.Get<ReflectionProbe>()->ConstructFromCubemap(target_cubemap);
}

void EnvironmentalMap::ConstructFromTexture2D(const std::shared_ptr<Texture2D>& target_texture_2d) {
  environment_pdf_texture = BuildEnvironmentPdfTexture(target_texture_2d);
  const auto cubemap = AssetManager::CreateTemporaryAsset<Cubemap>();
  cubemap->ConvertFromEquirectangularTexture(target_texture_2d);
  light_probe = AssetManager::CreateTemporaryAsset<LightProbe>();
  light_probe.Get<LightProbe>()->ConstructFromCubemap(cubemap);
  reflection_probe = AssetManager::CreateTemporaryAsset<ReflectionProbe>();
  reflection_probe.Get<ReflectionProbe>()->ConstructFromCubemap(cubemap);
}

void EnvironmentalMap::ConstructFromRenderTexture(const std::shared_ptr<RenderTexture>& target_render_texture) {
  environment_pdf_texture.Clear();
}
