#pragma once

#include "GlobalReflectionProbe.hpp"
#include "IAsset.hpp"

#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <vector>

namespace evo_engine {

class EVOENGINE_API ReflectionProbePack final : public IAsset {
 public:
  struct EVOENGINE_API Probe {
    std::string name = "Local Reflection Probe";
    uint64_t stable_id = 0;
    glm::mat4 transform = glm::mat4(1.0f);
    glm::vec3 box_projection_extents = glm::vec3(0.5f);
    float sphere_radius = 5.0f;
    float blend_distance = 0.05f;
    float reflection_intensity = 1.0f;
    int artist_priority = 0;
    int shape = 0;
    bool box_projection = true;
    bool enabled = true;
    bool debug_draw_bounds = false;
    std::shared_ptr<GlobalReflectionProbe> payload;

    [[nodiscard]] bool HasValidPayload() const;
    [[nodiscard]] std::shared_ptr<GlobalReflectionProbe> GetOrCreatePayload();
  };

  static constexpr uint32_t kSchemaVersion = 1;
  std::vector<Probe> probes;

  [[nodiscard]] bool RepairStableIds();
  [[nodiscard]] Probe* FindProbe(uint64_t stable_id);
  [[nodiscard]] const Probe* FindProbe(uint64_t stable_id) const;
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  static bool RegisterAssetIoHandlers(const std::string& owner_name = {},
                                      const std::string& type_name = "ReflectionProbePack");

 private:
  bool SaveInternal(const std::filesystem::path& path) const;
  bool LoadInternal(const std::filesystem::path& path);
  [[nodiscard]] std::shared_ptr<StagedAssetLoadPayload> LoadStagedPayloadInternal(
      const std::filesystem::path& path) const;
  bool ApplyStagedPayloadInternal(const std::filesystem::path& path,
                                  const std::shared_ptr<StagedAssetLoadPayload>& payload);
};

void SerializeReflectionProbePack(YAML::Emitter& out, const ReflectionProbePack& pack);
void DeserializeReflectionProbePack(const YAML::Node& in, ReflectionProbePack& pack);

}  // namespace evo_engine
