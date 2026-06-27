#pragma once
#include "IAsset.hpp"

namespace evo_engine {

class GaussianSplat final : public IAsset {
  glm::vec3 min_bound_ = glm::vec3(0.0f);
  glm::vec3 max_bound_ = glm::vec3(0.0f);

 protected:
  bool LoadInternal(const std::filesystem::path& path);
  [[nodiscard]] bool SupportsStagedLoading(const std::filesystem::path& path) const;
  [[nodiscard]] std::shared_ptr<StagedAssetLoadPayload> LoadStagedPayloadInternal(
      const std::filesystem::path& path) const;
  bool ApplyStagedPayloadInternal(const std::filesystem::path& path,
                                  const std::shared_ptr<StagedAssetLoadPayload>& payload);
  bool SaveInternal(const std::filesystem::path& path) const;

 public:
  static bool RegisterAssetIoHandlers(const std::string& owner_name = {},
                                      const std::string& type_name = "GaussianSplat");

  std::vector<glm::vec3> positions;
  std::vector<glm::vec3> scales;
  std::vector<glm::vec4> rotations;
  std::vector<float> opacities;
  std::vector<glm::vec3> colors;
  std::vector<float> spherical_harmonics_rest;
  uint32_t spherical_harmonics_rest_float_count = 0;

  void OnCreate() override;
  [[nodiscard]] size_t GetSplatCount() const;
  [[nodiscard]] bool Empty() const;
  [[nodiscard]] glm::vec3 GetMinBound() const;
  [[nodiscard]] glm::vec3 GetMaxBound() const;
  void SetBounds(const glm::vec3& min_bound, const glm::vec3& max_bound);
  void RecalculateBoundingBox();
  bool LoadPly(const std::filesystem::path& path);
};

}  // namespace evo_engine
