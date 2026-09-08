#pragma once

#include "DdgiSettings.hpp"
#include "HddagiSettings.hpp"
#include "SdfgiSettings.hpp"

namespace evo_engine {
struct EVOENGINE_API GiSettings {
  GiProbeSettings gi_probe_settings;
  IndirectGiProvider indirect_gi_provider = IndirectGiProvider::AutomaticSdfgi;
  SdfgiSettings sdfgi_settings;
  DdgiSettings ddgi_settings;
  HddagiSettings hddagi_settings{};

  [[nodiscard]] bool operator==(const GiSettings& other) const;
  [[nodiscard]] std::string Validate(bool device_limits = true) const;
};
}  // namespace evo_engine
