#pragma once
#include "GraphicsResources.hpp"
#include "IAsset.hpp"
#include "Jobs.hpp"
#include "TextureStorage.hpp"
namespace evo_engine {
class Texture2DStorage;
struct TextureStorageHandle;

enum class TextureColorType { Red = 1, Rg = 2, Rgb = 3, Rgba = 4 };

class Texture2D : public IAsset {
  friend class EditorLayer;
  friend class Resources;
  friend class Cubemap;
  friend class TextureStorage;
  friend class RenderLayer;

  std::shared_ptr<TextureStorageHandle> texture_storage_handle_;

  void SetData(const std::vector<glm::vec4>& data, const glm::uvec2& resolution, bool local_copy);
  std::vector<glm::vec4> local_data_;
  glm::uvec2 local_resolution_;

 protected:
  bool SaveInternal(const std::filesystem::path& path) const override;
  bool LoadInternal(const std::filesystem::path& path) override;

 public:
  void UnsafeUploadDataImmediately() const;

  bool red_channel = false;
  bool green_channel = false;
  bool blue_channel = false;
  bool alpha_channel = false;

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

  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  bool hdr = false;
  Texture2D();
  const Texture2DStorage& PeekTexture2DStorage() const;
  Texture2DStorage& RefTexture2DStorage() const;
  [[nodiscard]] VkImageLayout GetLayout() const;
  [[nodiscard]] VkImage GetVkImage() const;
  [[nodiscard]] VkImageView GetVkImageView() const;
  [[nodiscard]] VkSampler GetVkSampler() const;
  [[nodiscard]] std::shared_ptr<Image> GetImage() const;
  ImTextureID GetImTextureId() const;
  [[nodiscard]] uint32_t GetTextureStorageIndex() const;
  ~Texture2D() override;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  [[nodiscard]] glm::uvec2 GetResolution() const;
  void StoreToPng(const std::filesystem::path& path, int resize_x = -1, int resize_y = -1,
                  unsigned compression_level = 8) const;
  void StoreToTga(const std::filesystem::path& path, int resize_x = -1, int resize_y = -1) const;
  void StoreToJpg(const std::filesystem::path& path, int resize_x = -1, int resize_y = -1,
                  unsigned quality = 100) const;
  void StoreToHdr(const std::filesystem::path& path, int resize_x = -1, int resize_y = -1) const;

  template<typename T>
  void GetData(std::vector<T>& dst) const;

  void GetRgbaChannelData(std::vector<glm::vec4>& dst, int resize_x = -1, int resize_y = -1) const;
  void GetRgbChannelData(std::vector<glm::vec3>& dst, int resize_x = -1, int resize_y = -1) const;
  void GetRgChannelData(std::vector<glm::vec2>& dst, int resize_x = -1, int resize_y = -1) const;
  void GetRedChannelData(std::vector<float>& dst, int resize_x = -1, int resize_y = -1) const;

  void SetRgbaChannelData(const std::vector<glm::vec4>& src, const glm::uvec2& resolution, bool local_copy = false);
  void SetRgbChannelData(const std::vector<glm::vec3>& src, const glm::uvec2& resolution, bool local_copy = false);
  void SetRgChannelData(const std::vector<glm::vec2>& src, const glm::uvec2& resolution, bool local_copy = false);
  void SetRedChannelData(const std::vector<float>& src, const glm::uvec2& resolution, bool local_copy = false);

  static void Resize(std::vector<glm::vec4>& src, const glm::uvec2& src_resolution, std::vector<glm::vec4>& dst,
                     const glm::uvec2& dst_resolution);
  static void Resize(std::vector<glm::vec3>& src, const glm::uvec2& src_resolution, std::vector<glm::vec3>& dst,
                     const glm::uvec2& dst_resolution);
  static void Resize(std::vector<glm::vec2>& src, const glm::uvec2& src_resolution, std::vector<glm::vec2>& dst,
                     const glm::uvec2& dst_resolution);
  static void Resize(std::vector<float>& src, const glm::uvec2& src_resolution, std::vector<float>& dst,
                     const glm::uvec2& dst_resolution);
};

template <typename T>
void Texture2D::GetData(std::vector<T>& dst) const {
  const auto& texture_storage = PeekTexture2DStorage();
  const auto resolution = GetResolution();
  std::vector<glm::vec4> pixels;
  pixels.resize(resolution.x * resolution.y);
  Buffer image_buffer(sizeof(glm::vec4) * resolution.x * resolution.y);
  image_buffer.CopyFromImage(*texture_storage.image);
  image_buffer.DownloadVector(pixels, resolution.x * resolution.y);
  dst.resize(pixels.size());
  Jobs::RunParallelFor(pixels.size(), [&](unsigned i) {
    dst[i] = pixels[i];
  });
}
}  // namespace evo_engine
