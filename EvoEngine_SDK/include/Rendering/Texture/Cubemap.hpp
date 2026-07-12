#pragma once
#include "GraphicsResources.hpp"
#include "IAsset.hpp"
#include "Platform.hpp"
#include "SkyIllumination.hpp"
#include "Texture2D.hpp"

namespace evo_engine {

/**
 * Forward declaration of the CubemapStorage class.
 */
class CubemapStorage;

/**
 * Forward declaration of the TextureStorageHandle struct.
 */
struct TextureStorageHandle;

/**
 * @class Cubemap
 * @brief Represents a cubemap texture used for various rendering purposes.
 *
 * The Cubemap class provides functionality for managing cubemaps, including
 * converting equirectangular textures to cubemaps, building sky illumination,
 * and more. It inherits from the IAsset interface.
 */
class Cubemap final : public IAsset {
  friend class RenderLayer;
  friend class LightProbe;
  friend class ReflectionProbe;
  friend class TextureStorage;

  /**
   * @brief Handle to the texture storage for this cubemap.
   */
  std::shared_ptr<TextureStorageHandle> texture_storage_handle_;
  mutable std::shared_ptr<GraphicsPipeline> atmosphere_to_cubemap_pipeline_;
  mutable std::shared_ptr<GraphicsPipeline> equirectangular_to_cubemap_pipeline_;
  mutable uint32_t resolution_ = 0;
  mutable uint32_t mip_levels_ = 1;
  mutable std::vector<glm::vec4> local_data_;
  mutable bool local_data_dirty_ = false;
  mutable bool gpu_content_valid_ = false;

  void UploadLocalData() const;
  void BeginGpuWrite() const;
  void MarkGpuContentValid() const;

 public:
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  /**
   * @struct EquirectangularToCubemapConstant
   * @brief Represents constants used for converting equirectangular textures to cubemaps.
   */
  struct EquirectangularToCubemapConstant {
    /**
     * @brief The projection and view matrix.
     */
    glm::mat4 projection_view = {};

    /**
     * @brief Equirectangular environment luminance-to-PDF scale.
     */
    float environment_pdf_scale = 0.0f;
  };

  /**
   * @brief Constructs a new Cubemap object.
   */
  Cubemap();

  /**
   * @brief Returns a constant reference to the cubemap storage.
   * @return A constant reference to the CubemapStorage instance.
   */
  const CubemapStorage& PeekStorage() const;

  /**
   * @brief Returns a mutable reference to the cubemap storage.
   * @return A reference to the CubemapStorage instance.
   */
  CubemapStorage& RefStorage() const;

  /**
   * @brief Destructor for the Cubemap class.
   */
  ~Cubemap() override;

  /**
   * @brief Initializes the cubemap with the specified resolution and mip levels.
   * @param resolution The resolution of the cubemap.
   * @param mip_levels The number of mip levels (default is 1).
   */
  void Initialize(uint32_t resolution, uint32_t mip_levels = 1) const;

  /**
   * @brief Stores all cubemap texels in Vulkan face order (+X, -X, +Y, -Y, +Z, -Z), then mip-major order per face.
   * @return False when the dimensions or payload size are invalid.
   */
  bool SetRgbaChannelData(const std::vector<glm::vec4>& pixels, uint32_t resolution, uint32_t mip_levels = 1);

  /** @brief Restores the canonical empty state while retaining a valid GPU placeholder. */
  void Reset();

  /** @brief Reads texels in the same face/mip order, optionally bypassing the valid CPU cache. */
  void GetRgbaChannelData(std::vector<glm::vec4>& pixels, bool force_gpu_readback = false) const;

  [[nodiscard]] static size_t CalculatePixelCount(uint32_t resolution, uint32_t mip_levels);

  [[nodiscard]] const std::vector<glm::vec4>& PeekLocalData() const;
  [[nodiscard]] uint32_t GetResolution() const;
  [[nodiscard]] uint32_t GetMipLevels() const;

  /**
   * @brief Gets the texture storage index associated with this cubemap.
   * @return The texture storage index.
   */
  [[nodiscard]] uint32_t GetTextureStorageIndex() const;

  /**
   * @brief Converts an equirectangular texture to a cubemap.
   * @param target_texture A shared pointer to the target equirectangular texture.
   */
  void ConvertFromEquirectangularTexture(const std::shared_ptr<Texture2D>& target_texture) const;

  /**
   * @brief Builds sky illumination using the specified SkyIllumination.
   * @param sky_illumination The SkyIllumination data.
   * @param resolution The resolution for the sky illumination texture (default is 1024).
   */
  void BuildSkyIllumination(const SkyIllumination& sky_illumination, uint32_t resolution = 1024) const;

  /**
   * @brief Gets the associated image of the cubemap.
   * @return A shared pointer to the Image instance.
   */
  [[nodiscard]] const std::shared_ptr<Image>& GetImage() const;

  /**
   * @brief Gets the associated image view of the cubemap.
   * @return A shared pointer to the ImageView instance.
   */
  [[nodiscard]] const std::shared_ptr<ImageView>& GetImageView() const;

  /**
   * @brief Gets the sampler assigned to the cubemap.
   * @return A shared pointer to the Sampler instance.
   */
  [[nodiscard]] const std::shared_ptr<Sampler>& GetSampler() const;

  /**
   * @brief Gets the image views of each face of the cubemap.
   * @return A vector of shared pointers to ImageView instances for each face.
   */
  [[nodiscard]] const std::vector<std::shared_ptr<ImageView>>& GetFaceViews() const;
};

}  // namespace evo_engine
