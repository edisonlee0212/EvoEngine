#pragma once
#include "VolumetricCloudSettings.hpp"

namespace evo_engine {
enum class CloudFormation { Scattered, BroadBank };

inline void ApplyCloudFormation(VolumetricCloudSettings& settings, CloudFormation formation) {
  const VolumetricCloudSettings defaults{};
  const bool broad = formation == CloudFormation::BroadBank;
  settings.base_noise_scale = broad ? 0.00045f : defaults.base_noise_scale;
  settings.detail_noise_scale = broad ? 0.0035f : defaults.detail_noise_scale;
  settings.coverage = broad ? 0.9f : defaults.coverage;
  settings.density = broad ? 2.5f : defaults.density;
  settings.extinction_scale = broad ? 0.12f : defaults.extinction_scale;
}
}  // namespace evo_engine
