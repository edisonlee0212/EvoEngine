#pragma once
#include "AssetRef.hpp"
#include "GraphicsResources.hpp"
#include "IAsset.hpp"
#include "Jobs.hpp"
#include "TextureStorage.hpp"

#include <optional>
namespace evo_engine {
class EVOENGINE_API Texture2DStorage;
struct TextureStorageHandle;

enum class TextureColorType { Red = 1, Rg = 2, Rgb = 3, Rgba = 4 };

struct EVOENGINE_API Texture2DSamplerSettings {
  VkFilter mag_filter = VK_FILTER_LINEAR;
  VkFilter min_filter = VK_FILTER_LINEAR;
  VkSamplerMipmapMode mipmap_mode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  VkSamplerAddressMode address_mode_u = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  VkSamplerAddressMode address_mode_v = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  float min_lod = 0.0f;
  float max_lod = VK_LOD_CLAMP_NONE;

  [[nodiscard]] VkSamplerCreateInfo CreateInfo() const;
  [[nodiscard]] bool operator==(const Texture2DSamplerSettings& other) const {
    return mag_filter == other.mag_filter && min_filter == other.min_filter && mipmap_mode == other.mipmap_mode &&
           address_mode_u == other.address_mode_u && address_mode_v == other.address_mode_v &&
           min_lod == other.min_lod && max_lod == other.max_lod;
  }
};

class EVOENGINE_API Texture2D : public IAsset {
  friend class Resources;
  friend class Cubemap;
  friend class TextureStorage;
  friend class RenderLayer;

  std::shared_ptr<TextureStorageHandle> texture_storage_handle_;

  void SetData(const std::vector<glm::vec4>& data, const glm::uvec2& resolution, bool local_copy);
  void DownloadData();
  std::vector<glm::vec4> local_data_;

 protected:
  bool SaveInternal(const std::filesystem::path& path) const;
  bool LoadInternal(const std::filesystem::path& path);
  [[nodiscard]] bool SupportsStagedLoading() const;
  [[nodiscard]] std::shared_ptr<StagedAssetLoadPayload> LoadStagedPayloadInternal(
      const std::filesystem::path& path) const;
  bool ApplyStagedPayloadInternal(const std::filesystem::path& path,
                                  const std::shared_ptr<StagedAssetLoadPayload>& payload);

 public:
  void UnsafeUploadDataImmediately() const;

  bool red_channel = false;
  bool green_channel = false;
  bool blue_channel = false;
  bool alpha_channel = false;
  bool srgb = false;

  static void StoreToPng(const std::filesystem::path& path, const std::vector<float>& src_data, int src_x, int src_y,
                         int src_channel_size, int target_channel_size, unsigned compression_level = 8,
                         int resize_x = -1, int resize_y = -1);
  static auto StoreToJpg(const std::filesystem::path& path, const std::vector<float>& src_data, int src_x, int src_y,
                         int src_channel_size, int target_channel_size, unsigned quality = 100, int resize_x = -1,
                         int resize_y = -1) -> void;
  static void StoreToTga(const std::filesystem::path& path, const std::vector<float>& src_data, int src_x, int src_y,
                         int src_channel_size, int target_channel_size, int resize_x = -1, int resize_y = -1);
  static void StoreToHdr(const std::filesystem::path& path, const std::vector<float>& src_data, int src_x, int src_y,
                         int src_channel_size, int target_channel_size, int resize_x = -1, int resize_y = -1);

  void ApplyOpacityMap(const std::shared_ptr<Texture2D>& target);
  void SetResolution(const glm::uvec2& resolution, bool preserve_data = true);
  bool hdr = false;
  Texture2D();
  static bool RegisterAssetIoHandlers(const std::string& owner_name = {}, const std::string& type_name = "Texture2D");
  const Texture2DStorage& PeekTexture2DStorage() const;
  Texture2DStorage& RefTexture2DStorage() const;
  [[nodiscard]] VkImageLayout GetLayout() const;
  [[nodiscard]] VkImage GetVkImage() const;
  [[nodiscard]] VkImageView GetVkImageView() const;
  [[nodiscard]] VkSampler GetVkSampler() const;
  void SetSamplerSettings(const Texture2DSamplerSettings& settings);
  [[nodiscard]] const Texture2DSamplerSettings& GetSamplerSettings() const;
  void SetSrgbImportOverride(bool value);
  [[nodiscard]] bool ShareGpuImage(const Texture2D& source, bool srgb,
                                   const Texture2DSamplerSettings& sampler_settings);
  [[nodiscard]] bool SamplesLinearSrgb() const;
  [[nodiscard]] std::shared_ptr<Image> GetImage() const;
  ImTextureID GetImTextureId() const;
  [[nodiscard]] uint32_t GetTextureStorageIndex() const;
  ~Texture2D() override;
  [[nodiscard]] glm::uvec2 GetResolution() const;
  void StoreToPng(const std::filesystem::path& path, int resize_x = -1, int resize_y = -1,
                  unsigned compression_level = 8) const;
  void StoreToTga(const std::filesystem::path& path, int resize_x = -1, int resize_y = -1) const;
  void StoreToJpg(const std::filesystem::path& path, int resize_x = -1, int resize_y = -1,
                  unsigned quality = 100) const;
  void StoreToHdr(const std::filesystem::path& path, int resize_x = -1, int resize_y = -1) const;

  template <typename T>
  void GetData(std::vector<T>& dst);

  [[nodiscard]] const std::vector<glm::vec4>& PeekLocalData() const;
  const std::vector<glm::vec4>& GetLocalData();
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture();
  void GetRgbaChannelData(std::vector<glm::vec4>& dst, int resize_x = -1, int resize_y = -1) const;
  void GetRgbChannelData(std::vector<glm::vec3>& dst, int resize_x = -1, int resize_y = -1) const;
  void GetRgChannelData(std::vector<glm::vec2>& dst, int resize_x = -1, int resize_y = -1) const;
  void GetRedChannelData(std::vector<float>& dst, int resize_x = -1, int resize_y = -1) const;

  void SetRgbaChannelData(const std::vector<glm::vec4>& src, const glm::uvec2& resolution, bool local_copy = true);
  void SetRgbChannelData(const std::vector<glm::vec3>& src, const glm::uvec2& resolution, bool local_copy = true);
  void SetRgChannelData(const std::vector<glm::vec2>& src, const glm::uvec2& resolution, bool local_copy = true);
  void SetRedChannelData(const std::vector<float>& src, const glm::uvec2& resolution, bool local_copy = true);

  static void Resize(const std::vector<glm::vec4>& src, const glm::uvec2& src_resolution, std::vector<glm::vec4>& dst,
                     const glm::uvec2& dst_resolution);
  static void Resize(const std::vector<glm::vec3>& src, const glm::uvec2& src_resolution, std::vector<glm::vec3>& dst,
                     const glm::uvec2& dst_resolution);
  static void Resize(const std::vector<glm::vec2>& src, const glm::uvec2& src_resolution, std::vector<glm::vec2>& dst,
                     const glm::uvec2& dst_resolution);
  static void Resize(const std::vector<float>& src, const glm::uvec2& src_resolution, std::vector<float>& dst,
                     const glm::uvec2& dst_resolution);

 private:
  Texture2DSamplerSettings sampler_settings_;
  std::optional<bool> srgb_import_override_;
  bool srgb_fallback_linear_ = false;
};

template <typename T>
void Texture2D::GetData(std::vector<T>& dst) {
  DownloadData();
  dst.resize(local_data_.size());
  Jobs::RunParallelFor(local_data_.size(), [&](size_t i) {
    dst[i] = local_data_[i];
  });
}
}  // namespace evo_engine
