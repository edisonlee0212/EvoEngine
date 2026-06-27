#pragma once
#include "IAsset.hpp"

#include <unordered_map>

namespace evo_engine {

class Buffer;

struct GaussianSplatGpuData {
  glm::vec4 position_opacity = glm::vec4(0.0f);
  glm::vec4 scale_reserved = glm::vec4(0.0f);
  glm::vec4 rotation = glm::vec4(1.0f, 0.0f, 0.0f, 0.0f);
  glm::vec4 color_rest_offset = glm::vec4(1.0f, 1.0f, 1.0f, -1.0f);
};

struct GaussianSplatSortCache {
  std::vector<uint32_t> indices;
  std::vector<float> depths;
  std::shared_ptr<Buffer> index_buffer;
  std::shared_ptr<Buffer> depth_buffer;
  glm::mat4 model = glm::mat4(1.0f);
  glm::mat4 view = glm::mat4(1.0f);
  uint32_t generation = 0;
  bool valid = false;
};

class GaussianSplat final : public IAsset {
  glm::vec3 min_bound_ = glm::vec3(0.0f);
  glm::vec3 max_bound_ = glm::vec3(0.0f);
  mutable std::vector<GaussianSplatGpuData> gpu_data_;
  mutable std::shared_ptr<Buffer> gpu_data_buffer_;
  mutable std::unordered_map<Handle, GaussianSplatSortCache> sort_caches_;
  mutable bool gpu_data_dirty_ = true;
  mutable bool gpu_data_buffer_dirty_ = true;
  mutable uint32_t gpu_data_revision_ = 0;

  void BuildGpuData() const;

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
  void InvalidateGpuCaches();
  [[nodiscard]] const std::vector<GaussianSplatGpuData>& EnsureGpuData() const;
  [[nodiscard]] const std::shared_ptr<Buffer>& GetGpuDataBuffer() const;
  [[nodiscard]] uint32_t GetGpuDataRevision() const;
  [[nodiscard]] const GaussianSplatSortCache& EnsureSortedIndices(const Handle& camera_handle, const glm::mat4& model,
                                                                  const glm::mat4& view) const;
  [[nodiscard]] const GaussianSplatSortCache* FindSortCache(const Handle& camera_handle) const;
  bool LoadPly(const std::filesystem::path& path);
};

}  // namespace evo_engine
