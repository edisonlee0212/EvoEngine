#pragma once

#include "Cubemap.hpp"

namespace evo_engine {

class EVOENGINE_API GlobalReflectionProbe final : public IAsset {
 public:
  enum class SourceKind : uint32_t { Empty = 0, Imported = 1, Baked = 2 };

 private:
  std::shared_ptr<Cubemap> cubemap_;
  static std::shared_ptr<GraphicsPipeline> shared_prefilter_construct_pipeline_;
  std::vector<std::vector<std::shared_ptr<ImageView>>> mip_map_views_;
  SourceKind source_kind_ = SourceKind::Empty;
  mutable uint64_t payload_hash_ = 0;

  void RebuildMipMapViews();
  bool ConstructFilteredFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap, SourceKind source_kind,
                                    bool retain_cpu_payload);
  static std::shared_ptr<GraphicsPipeline> AcquirePrefilterPipeline(
      const std::shared_ptr<DescriptorSetLayout>& descriptor_set_layout);
  static void RecordPrefilter(const VkCommandBuffer command_buffer, const std::shared_ptr<Cubemap>& filtered,
                              const std::vector<std::vector<std::shared_ptr<ImageView>>>& filtered_mip_views,
                              const std::shared_ptr<Image>& depth_image, const std::shared_ptr<ImageView>& depth_view,
                              const std::shared_ptr<DescriptorSet>& descriptor_set,
                              const std::shared_ptr<GraphicsPipeline>& pipeline);
  static void RecordPrefilterFaces(VkCommandBuffer command_buffer, const std::shared_ptr<Cubemap>& filtered,
                                   const std::vector<std::vector<std::shared_ptr<ImageView>>>& filtered_mip_views,
                                   const std::shared_ptr<Image>& depth_image,
                                   const std::shared_ptr<ImageView>& depth_view,
                                   const std::shared_ptr<DescriptorSet>& descriptor_set,
                                   const std::shared_ptr<GraphicsPipeline>& pipeline, uint32_t first_face,
                                   uint32_t face_count);

  friend class RenderLayer;
  friend class Camera;

  bool SaveInternal(const std::filesystem::path& path) const;
  bool LoadInternal(const std::filesystem::path& path);
  [[nodiscard]] std::shared_ptr<StagedAssetLoadPayload> LoadStagedPayloadInternal(
      const std::filesystem::path& path) const;
  bool ApplyStagedPayloadInternal(const std::filesystem::path& path,
                                  const std::shared_ptr<StagedAssetLoadPayload>& payload);

 public:
  static constexpr uint32_t kSchemaVersion = 2;
  static constexpr uint32_t kResolution = 256;
  static constexpr uint32_t kMipLevels = 9;
  static constexpr VkFormat kCanonicalFormat = VK_FORMAT_R16G16B16A16_SFLOAT;
  static constexpr size_t kCanonicalTexelCount = 524286;
  static constexpr size_t kCanonicalPayloadByteSize = 4194288;
  static constexpr size_t kPackedRuntimeByteSize = 2097144;
  static constexpr float kBakeNearPlane = 0.1f;
  static constexpr float kBakeFarPlane = 1000.0f;
  static constexpr float kPackedMaxNormalizedRmsError = 0.002f;
  static constexpr float kPackedMaxRelativePeakError = 0.02f;
  static constexpr float kPackedMinMemorySaving = 0.49f;
  static constexpr float kPackedMinGpuTimeImprovement = 0.05f;

  GlobalReflectionProbe();

  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  void Initialize();
  void Serialize(YAML::Emitter& out) const;
  void Deserialize(const YAML::Node& in);
  [[nodiscard]] std::shared_ptr<Cubemap> GetCubemap() const;
  [[nodiscard]] const std::vector<uint16_t>& GetCanonicalPayload() const;
  [[nodiscard]] bool ReadCanonicalPayload(std::vector<uint16_t>& payload) const;
  [[nodiscard]] size_t GetCanonicalPayloadByteSize() const;
  [[nodiscard]] VkFormat GetRuntimeFormat() const;
  [[nodiscard]] SourceKind GetSourceKind() const;
  [[nodiscard]] uint64_t GetPayloadHash() const;
  [[nodiscard]] bool IsRuntimeReady() const;
  [[nodiscard]] bool PackedRuntimeFormatSupported() const;
  bool ConstructFromCubemap(const std::shared_ptr<Cubemap>& target_cubemap);
  void MarkBaked();
  bool SetCanonicalPayload(const std::vector<uint16_t>& payload);
  [[nodiscard]] static uint64_t CalculatePayloadHash(const std::vector<uint16_t>& payload);
  static bool ValidateCanonicalPayload(const std::vector<uint16_t>& payload, std::string& error);
  static bool EvaluatePackedRuntimeQuality(const std::vector<uint16_t>& payload, float& normalized_rms_error,
                                           float& relative_peak_error);
  static bool RegisterAssetIoHandlers(const std::string& owner_name = {},
                                      const std::string& type_name = "GlobalReflectionProbe");
};

}  // namespace evo_engine
