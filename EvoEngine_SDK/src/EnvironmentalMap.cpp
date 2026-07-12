#include "EnvironmentalMap.hpp"
#include "AssetManager.hpp"
#include "Texture2D.hpp"

#include <cmath>

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

  const auto cdf_pixels = EnvironmentalMap::BuildEnvironmentPdfData(pixels, resolution);
  if (cdf_pixels.empty())
    return {};

  auto pdf_texture = AssetManager::CreateTemporaryAsset<Texture2D>();
  pdf_texture->SetRgbaChannelData(cdf_pixels, resolution);
  return pdf_texture;
}
}  // namespace

std::vector<glm::vec4> EnvironmentalMap::BuildEnvironmentPdfData(const std::vector<glm::vec4>& pixels,
                                                                 const glm::uvec2& resolution) {
  if (resolution.x == 0 || resolution.y == 0 || pixels.size() != static_cast<size_t>(resolution.x) * resolution.y)
    return {};

  const size_t pixel_count = pixels.size();
  std::vector<double> weights(pixel_count, 0.0);
  std::vector<double> row_weights(resolution.y, 0.0);
  std::vector<double> row_solid_angles(resolution.y, 0.0);
  const double d_azimuth = 2.0 * static_cast<double>(kPi) / static_cast<double>(resolution.x);
  double total_weight = 0.0;

  for (uint32_t y = 0; y < resolution.y; ++y) {
    const double elevation_0 =
        (static_cast<double>(y) / static_cast<double>(resolution.y) - 0.5) * static_cast<double>(kPi);
    const double elevation_1 =
        (static_cast<double>(y + 1u) / static_cast<double>(resolution.y) - 0.5) * static_cast<double>(kPi);
    const double solid_angle = d_azimuth * (std::sin(elevation_1) - std::sin(elevation_0));
    row_solid_angles[y] = solid_angle;
    for (uint32_t x = 0; x < resolution.x; ++x) {
      const size_t index = static_cast<size_t>(y) * resolution.x + x;
      const double weight = static_cast<double>(EnvironmentLuminance(glm::vec3(pixels[index]))) * solid_angle;
      weights[index] = weight;
      row_weights[y] += weight;
      total_weight += weight;
    }
  }

  if (total_weight <= 0.0)
    return {};

  std::vector<glm::vec4> cdf_pixels(pixel_count, glm::vec4(0.0f));
  double marginal_cdf = 0.0;
  for (uint32_t y = 0; y < resolution.y; ++y) {
    const double row_weight = row_weights[y];
    double conditional_cdf = 0.0;
    marginal_cdf += row_weight / total_weight;
    for (uint32_t x = 0; x < resolution.x; ++x) {
      const size_t index = static_cast<size_t>(y) * resolution.x + x;
      conditional_cdf += row_weight > 0.0 ? weights[index] / row_weight : 1.0 / static_cast<double>(resolution.x);
      cdf_pixels[index] = glm::vec4(x + 1u == resolution.x ? 1.0f : static_cast<float>(conditional_cdf),
                                    y + 1u == resolution.y ? 1.0f : static_cast<float>(marginal_cdf), 0.0f, 1.0f);
    }
  }

  double previous_marginal_cdf = 0.0;
  for (uint32_t y = 0; y < resolution.y; ++y) {
    const double current_marginal_cdf = cdf_pixels[static_cast<size_t>(y + 1u) * resolution.x - 1u].g;
    const double marginal_probability = glm::max(current_marginal_cdf - previous_marginal_cdf, 0.0);
    double previous_conditional_cdf = 0.0;
    for (uint32_t x = 0; x < resolution.x; ++x) {
      const size_t index = static_cast<size_t>(y) * resolution.x + x;
      const double current_conditional_cdf = cdf_pixels[index].r;
      const double conditional_probability = glm::max(current_conditional_cdf - previous_conditional_cdf, 0.0);
      cdf_pixels[index].b = static_cast<float>(marginal_probability * conditional_probability / row_solid_angles[y]);
      previous_conditional_cdf = current_conditional_cdf;
    }
    previous_marginal_cdf = current_marginal_cdf;
  }
  return cdf_pixels;
}

void EnvironmentalMap::BuildSkyIllumination(const SkyIllumination& sky_illumination, uint32_t resolution) {
  environment_source.Clear();
  environment_source_type = SourceType::SkyIllumination;
  sky_illumination_source = sky_illumination;
  sky_illumination_resolution = resolution;
  environment_source_pdf_expected = false;
  environment_pdf_texture.Clear();
  const auto cubemap = AssetManager::CreateTemporaryAsset<Cubemap>();
  cubemap->BuildSkyIllumination(sky_illumination, resolution);
  environment_cubemap = cubemap;

  light_probe = AssetManager::CreateTemporaryAsset<LightProbe>();
  light_probe.Get<LightProbe>()->ConstructFromCubemap(cubemap);
  reflection_probe = AssetManager::CreateTemporaryAsset<ReflectionProbe>();
  reflection_probe.Get<ReflectionProbe>()->ConstructFromCubemap(cubemap);
}

void EnvironmentalMap::ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap) {
  environment_source = target_cubemap;
  environment_source_type = SourceType::Cubemap;
  environment_source_pdf_expected = false;
  environment_pdf_texture.Clear();
  if (!target_cubemap || target_cubemap->GetResolution() == 0 || !target_cubemap->GetImage()) {
    environment_cubemap.Clear();
    light_probe.Clear();
    reflection_probe.Clear();
    return;
  }
  environment_cubemap = target_cubemap;
  light_probe = AssetManager::CreateTemporaryAsset<LightProbe>();
  light_probe.Get<LightProbe>()->ConstructFromCubemap(target_cubemap);
  reflection_probe = AssetManager::CreateTemporaryAsset<ReflectionProbe>();
  reflection_probe.Get<ReflectionProbe>()->ConstructFromCubemap(target_cubemap);
}

void EnvironmentalMap::ConstructFromTexture2D(const std::shared_ptr<Texture2D>& target_texture_2d) {
  environment_source = target_texture_2d;
  environment_source_type = SourceType::Texture2D;
  environment_source_pdf_expected = true;
  if (!target_texture_2d || !target_texture_2d->GetImage()) {
    environment_pdf_texture.Clear();
    environment_cubemap.Clear();
    light_probe.Clear();
    reflection_probe.Clear();
    return;
  }
  const auto pdf_texture = BuildEnvironmentPdfTexture(target_texture_2d);
  environment_pdf_texture = pdf_texture;
  environment_source_pdf_expected = pdf_texture != nullptr;
  const auto cubemap = AssetManager::CreateTemporaryAsset<Cubemap>();
  cubemap->ConvertFromEquirectangularTexture(target_texture_2d);
  environment_cubemap = cubemap;
  light_probe = AssetManager::CreateTemporaryAsset<LightProbe>();
  light_probe.Get<LightProbe>()->ConstructFromCubemap(cubemap);
  reflection_probe = AssetManager::CreateTemporaryAsset<ReflectionProbe>();
  reflection_probe.Get<ReflectionProbe>()->ConstructFromCubemap(cubemap);
}

void EnvironmentalMap::ConstructFromRenderTexture(const std::shared_ptr<RenderTexture>& target_render_texture) {
  environment_source.Clear();
  environment_source_type = SourceType::None;
  environment_source_pdf_expected = false;
  environment_pdf_texture.Clear();
  environment_cubemap.Clear();
}

void EnvironmentalMap::CollectAssetRef(std::vector<AssetRef>& list) {
  switch (environment_source_type) {
    case SourceType::Texture2D:
    case SourceType::Cubemap:
      list.emplace_back(environment_source);
      break;
    case SourceType::SkyIllumination:
    case SourceType::None:
      break;
  }
}

void EnvironmentalMap::EnsureEnvironmentSource() {
  const bool pdf_ready = environment_source_type != SourceType::Texture2D || !environment_source_pdf_expected ||
                         environment_pdf_texture.Get<Texture2D>();
  const auto cubemap = environment_cubemap.Get<Cubemap>();
  const auto light = light_probe.Get<LightProbe>();
  const auto reflection = reflection_probe.Get<ReflectionProbe>();
  if (pdf_ready && cubemap && cubemap->GetImage() && light && light->GetCubemap() && light->GetCubemap()->GetImage() &&
      reflection && reflection->GetCubemap() && reflection->GetCubemap()->GetImage()) {
    return;
  }
  switch (environment_source_type) {
    case SourceType::Texture2D:
      if (const auto texture = environment_source.Get<Texture2D>()) {
        ConstructFromTexture2D(texture);
      }
      break;
    case SourceType::Cubemap:
      if (const auto source_cubemap = environment_source.Get<Cubemap>()) {
        ConstructFromCubemap(source_cubemap);
      }
      break;
    case SourceType::SkyIllumination:
      BuildSkyIllumination(sky_illumination_source, sky_illumination_resolution);
      break;
    case SourceType::None:
      break;
  }
}
