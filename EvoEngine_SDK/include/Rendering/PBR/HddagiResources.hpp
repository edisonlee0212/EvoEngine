#pragma once

#include "GraphicsResources.hpp"
#include "HddagiSettings.hpp"

#include <map>
#include <memory>

namespace evo_engine {

struct HddagiImageRequirement {
  std::string name;
  VkFormat storage_format;
  VkFormat sampled_format;
  VkImageType type;
  VkExtent3D extent;
  uint32_t layers = 1;
  bool temporal = false;
  bool filtered = false;
  bool atomic = false;
};

struct EVOENGINE_API HddagiCapabilityReport {
  std::string device_name;
  std::string failure = "HDDAGI capabilities have not been queried";
  uint64_t temporal_bytes = 0;
  uint64_t image_bytes = 0;
  [[nodiscard]] bool Supported() const {
    return failure.empty();
  }
};

EVOENGINE_API std::vector<HddagiImageRequirement> GetHddagiImageRequirements(const GiProbeSettings& probes,
                                                                             const HddagiSettings& settings);
EVOENGINE_API HddagiCapabilityReport QueryHddagiCapabilities(const GiProbeSettings& probes,
                                                             const HddagiSettings& settings);
EVOENGINE_API uint64_t HddagiLogicalTemporalBytes(const GiProbeSettings& probes, const HddagiSettings& settings);

struct HddagiImage {
  HddagiImageRequirement requirement;
  std::shared_ptr<Image> image;
  std::shared_ptr<ImageView> storage;
  std::shared_ptr<ImageView> sampled;
};

class EVOENGINE_API HddagiResources {
 public:
  GiProbeSettings probes;
  HddagiSettings settings;
  std::map<std::string, HddagiImage> images;
  uint64_t allocation_bytes = 0;
  uint64_t temporal_bytes = 0;

  static std::shared_ptr<HddagiResources> TryCreate(const GiProbeSettings& probes, const HddagiSettings& settings,
                                                    std::string& failure, uint32_t fail_after_allocations = UINT32_MAX);
};

struct EVOENGINE_API HddagiRuntime {
  GiProbeSettings probes;
  HddagiSettings settings;
  GiProbeFrame frame;
  HddagiCapabilityReport capabilities;
  std::shared_ptr<HddagiResources> resources;
  bool allocation_attempted = false;
  bool published = false;
  std::string fallback_reason = "HDDAGI transport is not ready";
};

}  // namespace evo_engine
