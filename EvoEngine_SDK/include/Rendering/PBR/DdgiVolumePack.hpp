#pragma once

#include "DdgiSettings.hpp"
#include "IAsset.hpp"

#include <glm/glm.hpp>
#include <string>
#include <vector>

namespace evo_engine {

class DdgiVolumePack final : public IAsset {
 public:
  struct Volume {
    std::string name = "DDGI Volume";
    uint64_t stable_id = 0;
    glm::mat4 transform = glm::mat4(1.0f);
    glm::ivec3 probe_counts = glm::ivec3(10, 6, 16);
    glm::vec3 probe_spacing = glm::vec3(1.5f);
    glm::vec3 volume_origin = glm::vec3(0.0f, 3.0f, 3.0f);
    int artist_priority = 0;
    int movement_type = static_cast<int>(DdgiVolumeMovementType::Default);
    int emissive_mesh_sampling_mode = static_cast<int>(DdgiEmissiveMeshSamplingMode::Inherit);
    bool enabled = true;
    bool enable_probe_relocation = true;
    bool enable_probe_classification = false;
    bool enable_probe_variability = true;
    bool enable_probe_variability_gating = true;
    bool pause_probe_updates_after_convergence = true;
    float relocation_distance = 0.25f;
    float random_ray_backface_threshold = 0.1f;
    float fixed_ray_backface_threshold = 0.25f;
    float probe_variability_threshold = 0.03f;
    int probe_variability_min_samples = 128;
    int hysteresis_boost_trigger_conditions = DdgiVolumeTriggerConditionAll;
    int variability_reset_trigger_conditions =
        DdgiVolumeTriggerConditionLightingConditionChanged | DdgiVolumeTriggerConditionGeometryChanged;

    void ClampSettings();
    [[nodiscard]] uint32_t GetProbeAmount() const;
    [[nodiscard]] glm::vec3 GetLocalGridSize() const;
    [[nodiscard]] glm::vec3 GetProbeLocalPosition(const glm::ivec3& probe_index) const;
  };

  std::vector<Volume> volumes;

  [[nodiscard]] bool RepairStableIds();
};

void SerializeDdgiVolumePack(YAML::Emitter& out, const DdgiVolumePack& pack);
void DeserializeDdgiVolumePack(const YAML::Node& in, DdgiVolumePack& pack);

}  // namespace evo_engine
