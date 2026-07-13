
#pragma once
#include "GpuService.hpp"
#include "Vertex.hpp"
#include "shaderc/shaderc.h"

namespace evo_engine {

/**
 * @class RenderInstanceStorage
 * @brief Forward declaration for RenderInstanceStorage class.
 */
class RenderInstanceStorage;
class RangeDescriptor;

struct FrameSubmissionState;

inline constexpr VkDeviceSize kStaticBlasBuildBudgetBytes = 512ull * 1024ull * 1024ull;

struct StaticBlasBuildSize {
  VkDeviceSize destination_size = 0;
  VkDeviceSize scratch_size = 0;
};

struct StaticBlasBuildPassPlan {
  size_t begin = 0;
  size_t count = 0;
  VkDeviceSize destination_size = 0;
  VkDeviceSize scratch_size = 0;
  uint32_t scratch_wave_count = 0;
  bool oversized_singleton = false;
};

[[nodiscard]] std::vector<StaticBlasBuildPassPlan> PlanStaticBlasBuildPasses(
    const std::vector<StaticBlasBuildSize>& build_sizes, VkDeviceSize scratch_alignment,
    VkDeviceSize budget = kStaticBlasBuildBudgetBytes);

struct StaticBlasBuildTelemetry {
  uint64_t fixed_hint_bytes = kStaticBlasBuildBudgetBytes;
  uint64_t pending_count = 0;
  uint64_t total_blas_count = 0;
  uint64_t static_eligible_count = 0;
  uint64_t updateable_count = 0;
  uint64_t shared_input_count = 0;
  uint64_t private_input_count = 0;
  uint64_t private_input_bytes = 0;
  uint64_t cumulative_built_static_count = 0;
  uint64_t cumulative_uncompacted_bytes = 0;
  uint64_t cumulative_compacted_bytes = 0;
  uint64_t pass_count = 0;
  uint64_t scratch_wave_count = 0;
  uint64_t scratch_peak_bytes = 0;
  uint64_t eligible_static_uncompacted_bytes = 0;
  uint64_t eligible_static_compacted_bytes = 0;
  uint64_t final_compacted_storage_bytes = 0;
  uint64_t transient_peak_bytes = 0;
  double wall_milliseconds = 0.0;
  bool complete = true;
  std::vector<StaticBlasBuildPassPlan> passes;
};

/**
 * @class CommandBuffer
 * @brief Forward declaration for CommandBuffer class.
 */
class CommandBuffer;

/**
 * @class IGraphicsResource
 * @brief Base class for graphics resources in the engine.
 */
class IGraphicsResource {
 protected:
  /**
   * @brief Default protected constructor for IGraphicsResource.
   */
  IGraphicsResource() = default;

 public:
  /**
   * @brief Prevents assignment.
   */
  IGraphicsResource& operator=(IGraphicsResource&) = delete;

  /**
   * @brief Prevents constant assignment.
   */
  IGraphicsResource& operator=(const IGraphicsResource&) = delete;

  /**
   * @brief Virtual destructor for IGraphicsResource.
   */
  virtual ~IGraphicsResource() = default;

  /**
   * @brief A utility static method to apply data to a vector.
   * @tparam T Type of vector elements.
   * @param target The destination std::vector.
   * @param size The size of the data.
   * @param data Pointer to the data to be copied.
   */
  template <typename T>
  static void ApplyVector(std::vector<T>& target, uint32_t size, const T* data);
};

template <typename T>
void IGraphicsResource::ApplyVector(std::vector<T>& target, uint32_t size, const T* data) {
  if (size == 0 || data == nullptr)
    return;
  target.resize(size);
  memcpy(target.data(), data, sizeof(T) * size);
}

/**
 * @class Fence
 * @brief Represents a Vulkan fence resource.
 */
class Fence final : public IGraphicsResource {
  VkFence vk_fence_ = VK_NULL_HANDLE; /**< Vulkan fence handle. */
  VkFenceCreateFlags flags_ = {};     /**< Vulkan fence creation flags. */

 public:
  /**
   * @brief Constructs a Fence with the given Vulkan fence creation info.
   * @param vk_fence_create_info Information for creating the Vulkan fence.
   */
  explicit Fence(const VkFenceCreateInfo& vk_fence_create_info);

  /**
   * @brief Destructor for Fence.
   */
  ~Fence() override;

  /**
   * @brief Retrieves the Vulkan fence handle.
   * @return The Vulkan fence handle.
   */
  [[nodiscard]] const VkFence& GetVkFence() const;
};

/**
 * @class Semaphore
 * @brief Represents a Vulkan semaphore resource.
 */
class Semaphore final : public IGraphicsResource {
  VkSemaphore vk_semaphore_ = VK_NULL_HANDLE; /**< Vulkan semaphore handle. */
  VkSemaphoreCreateFlags flags_ = {};         /**< Vulkan semaphore creation flags. */

 public:
  /**
   * @brief Constructs a Semaphore with the specified semaphore creation info.
   * @param semaphore_create_info Vulkan semaphore creation information.
   */
  explicit Semaphore(const VkSemaphoreCreateInfo& semaphore_create_info);

  /**
   * @brief Destructor for Semaphore.
   */
  ~Semaphore() override;

  /**
   * @brief Retrieves the Vulkan Semaphore handle.
   * @return The Vulkan semaphore handle.
   */
  [[nodiscard]] const VkSemaphore& GetVkSemaphore() const;

#ifdef _WIN64
  /**
   * @brief Get a semaphore handle for Windows platform.
   * @param external_semaphore_handle_type Vulkan external handle type.
   * @return Handle to the Vulkan semaphore.
   */
  void* GetVkSemaphoreHandle(VkExternalSemaphoreHandleTypeFlagBitsKHR external_semaphore_handle_type) const;
#else
  /**
   * @brief Get a semaphore handle for non-Windows platform.
   * @param external_semaphore_handle_type Vulkan external handle type.
   * @return Handle to the Vulkan semaphore.
   */
  int GetVkSemaphoreHandle(VkExternalSemaphoreHandleTypeFlagBitsKHR external_semaphore_handle_type) const;
#endif
};

/**
 * @class Image
 * @brief Represents a Vulkan image resource.
 */
class Image final : public IGraphicsResource {
  VkImage vk_image_ = VK_NULL_HANDLE;             /**< Vulkan image handle. */
  VmaAllocation vma_allocation_ = VK_NULL_HANDLE; /**< VMA allocation handle for the image. */
  VmaAllocationInfo vma_allocation_info_ = {};    /**< VMA allocation information. */

  VkImageCreateFlags flags_;                   /**< Vulkan image creation flags. */
  VkImageType image_type_;                     /**< Vulkan image type. */
  VkFormat format_;                            /**< Image format. */
  VkExtent3D extent_;                          /**< Image extent (width, height, depth). */
  uint32_t mip_levels_;                        /**< Mipmap levels. */
  uint32_t array_layers_;                      /**< Array layers count. */
  VkSampleCountFlagBits samples_;              /**< Multisample count. */
  VkImageTiling tiling_;                       /**< Image tiling. */
  VkImageUsageFlags usage_;                    /**< Image usage flags. */
  VkSharingMode sharing_mode_;                 /**< Sharing mode. */
  std::vector<uint32_t> queue_family_indices_; /**< List of queue family indices. */
  VkImageLayout initial_layout_;               /**< Initial image layout. */

  VkImageLayout layout_; /**< The image's layout. */

 public:
  // Method declarations with Doxygen comments not completed yet
  [[nodiscard]] uint32_t GetMipLevels() const;
  explicit Image(VkImageCreateInfo image_create_info);
  /**
   * @brief Constructs an Image with Vulkan image creation info and VMA allocation info.
   * @param image_create_info Vulkan image creation information.
   * @param vma_allocation_create_info VMA allocation creation information.
   */
  Image(VkImageCreateInfo image_create_info, const VmaAllocationCreateInfo& vma_allocation_create_info);

  /**
   * @brief Checks whether the format includes a stencil component.
   * @return True if the format includes a stencil component, otherwise false.
   */
  bool HasStencilComponent() const;

  /**
   * @brief Destructor for Image.
   */
  ~Image() override;

  /**
   * @brief Transitions the image to a new layout.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param new_layout The new image layout.
   */
  void TransitImageLayout(VkCommandBuffer vk_command_buffer, VkImageLayout new_layout);
  void TransitImageLayout(VkCommandBuffer vk_command_buffer, VkImageLayout old_layout, VkImageLayout new_layout,
                          uint32_t src_queue_family_index, uint32_t dst_queue_family_index, bool update_tracked_layout);

  /**
   * @brief Copies data from a buffer to the image.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param src_buffer The source Vulkan buffer.
   * @param src_offset Offset in the buffer from which to start copying. Defaults to 0.
   */
  void CopyFromBuffer(VkCommandBuffer vk_command_buffer, const VkBuffer& src_buffer, VkDeviceSize src_offset = 0) const;
  void CopyFromBuffer(VkCommandBuffer vk_command_buffer, const VkBuffer& src_buffer,
                      const std::vector<VkBufferImageCopy>& regions) const;

  /**
   * @brief Generates mipmaps for the image.
   * @param vk_command_buffer The Vulkan command buffer.
   */
  void GenerateMipmaps(VkCommandBuffer vk_command_buffer);

  /**
   * @brief Retrieves the Vulkan handle for the image.
   * @return Vulkan image handle.
   */
  [[nodiscard]] VkImage GetVkImage() const;

  /**
   * @brief Retrieves the format of the image.
   * @return The Vulkan format of the image.
   */
  [[nodiscard]] VkFormat GetFormat() const;

  /**
   * @brief Retrieves the VMA allocation handle for the image.
   * @return VMA allocation handle.
   */
  [[nodiscard]] VmaAllocation GetVmaAllocation() const;

  /**
   * @brief Retrieves the extent of the image (width, height, depth).
   * @return Extent of the image.
   */
  [[nodiscard]] VkExtent3D GetExtent() const;

  /**
   * @brief Retrieves the current layout of the image.
   * @return Image layout.
   */
  [[nodiscard]] VkImageLayout GetLayout() const;

  /**
   * @brief Retrieves the VMA allocation information.
   * @return VMA allocation information.
   */
  [[nodiscard]] const VmaAllocationInfo& GetVmaAllocationInfo() const;

  /**
   * @brief Retrieves Vulkan memory requirements through the SDK binary.
   * @return Vulkan memory requirements for this image.
   */
  [[nodiscard]] VkMemoryRequirements GetMemoryRequirements() const;

#ifdef _WIN64
  /**
   * @brief Gets the Vulkan image memory handle for Windows platform.
   * @param external_memory_handle_type Vulkan external memory handle type.
   * @return Handle to the Vulkan image memory.
   */
  void* GetVkImageMemHandle(VkExternalMemoryHandleTypeFlagsKHR external_memory_handle_type) const;
#else
  /**
   * @brief Gets the Vulkan image memory handle for non-Windows platform.
   * @param external_memory_handle_type Vulkan external memory handle type.
   * @return Handle to the Vulkan image memory.
   */
  int GetVkImageMemHandle(VkExternalMemoryHandleTypeFlagsKHR external_memory_handle_type) const;
#endif
};

/**
 * @class ImageView
 * @brief Represents a Vulkan image view resource.
 */
class ImageView final : public IGraphicsResource {
  VkImageView vk_image_view_ = VK_NULL_HANDLE; /**< Vulkan image view handle. */

  VkImageViewCreateFlags flags_;              /**< Vulkan image view creation flags. */
  std::shared_ptr<Image> image_;              /**< Associated image object. */
  VkImageViewType view_type_;                 /**< Image view type. */
  VkFormat format_;                           /**< Image view format. */
  VkComponentMapping components_;             /**< Component mapping for the image view. */
  VkImageSubresourceRange subresource_range_; /**< Subresource range for the image view. */
  friend class Swapchain;
  friend class Platform;

 public:
  /**
   * @brief Constructs an ImageView with Vulkan image view creation info.
   * @param image_view_create_info Vulkan image view creation information.
   */
  explicit ImageView(const VkImageViewCreateInfo& image_view_create_info);

  /**
   * @brief Constructs an ImageView with Vulkan image view creation info and an associated image.
   * @param image_view_create_info Vulkan image view creation information.
   * @param image Associated image object.
   */
  explicit ImageView(const VkImageViewCreateInfo& image_view_create_info, const std::shared_ptr<Image>& image);

  /**
   * @brief Destructor for ImageView.
   */
  ~ImageView() override;

  /**
   * @brief Retrieves the Vulkan image view handle.
   * @return Vulkan image view handle.
   */
  [[nodiscard]] VkImageView GetVkImageView() const;

  /**
   * @brief Retrieves the associated image.
   * @return Shared pointer to the associated image.
   */
  [[nodiscard]] const std::shared_ptr<Image>& GetImage() const;
};

/**
 * @class Swapchain
 * @brief Represents a Vulkan swapchain resource.
 */
class Swapchain final : public IGraphicsResource {
  VkSwapchainKHR vk_swapchain_ = VK_NULL_HANDLE; /**< Vulkan swapchain handle. */
  std::vector<VkImage> vk_images_;               /**< List of Vulkan images in the swapchain. */

  VkSwapchainCreateFlagsKHR flags_;             /**< Vulkan swapchain creation flags. */
  VkSurfaceKHR surface_;                        /**< Vulkan surface associated with the swapchain. */
  uint32_t min_image_count_;                    /**< Minimum number of images in the swapchain. */
  VkFormat image_format_;                       /**< Format of images in the swapchain. */
  VkColorSpaceKHR image_color_space_;           /**< Color space of images in the swapchain. */
  VkExtent2D image_extent_;                     /**< Extent (width and height) of the images. */
  uint32_t image_array_layers_;                 /**< Number of array layers for the images. */
  VkImageUsageFlags image_usage_;               /**< Usage flags for the images. */
  VkSharingMode image_sharing_mode_;            /**< Sharing mode for the images. */
  std::vector<uint32_t> queue_family_indices_;  /**< Queue family indices for sharing. */
  VkSurfaceTransformFlagBitsKHR pre_transform_; /**< Pre-transform applied to the images. */
  VkCompositeAlphaFlagBitsKHR composite_alpha_; /**< Composite alpha mode. */
  VkPresentModeKHR present_mode_;               /**< Presentation mode for the swapchain. */
  VkBool32 clipped_;                            /**< Clipping status. */

  std::vector<std::shared_ptr<ImageView>> vk_image_views_; /**< List of image views for the swapchain. */

 public:
  /**
   * @brief Constructs a Swapchain with Vulkan swapchain creation info.
   * @param swapchain_create_info Vulkan swapchain creation information.
   */
  explicit Swapchain(const VkSwapchainCreateInfoKHR& swapchain_create_info);

  /**
   * @brief Destructor for Swapchain.
   */
  ~Swapchain() override;

  /**
   * @brief Retrieves the Vulkan swapchain handle.
   * @return Vulkan swapchain handle.
   */
  [[nodiscard]] VkSwapchainKHR GetVkSwapchain() const;

  /**
   * @brief Retrieves all Vulkan images in the swapchain.
   * @return A vector containing Vulkan image handles.
   */
  [[nodiscard]] const std::vector<VkImage>& GetAllVkImages() const;

  /**
   * @brief Retrieves a single Vulkan image in the swapchain.
   * @return Vulkan image handle.
   */
  [[nodiscard]] const VkImage& GetVkImage() const;

  /**
   * @brief Retrieves a Vulkan image view in the swapchain.
   * @return Vulkan image view handle.
   */
  [[nodiscard]] const VkImageView& GetVkImageView() const;

  /**
   * @brief Retrieves all image views in the swapchain.
   * @return A vector containing shared pointers to image views.
   */
  [[nodiscard]] const std::vector<std::shared_ptr<ImageView>>& GetAllImageViews() const;

  /**
   * @brief Retrieves the format of images in the swapchain.
   * @return Vulkan image format.
   */
  [[nodiscard]] VkFormat GetImageFormat() const;

  /**
   * @brief Retrieves the extent (width, height) of images in the swapchain.
   * @return Vulkan image extent.
   */
  [[nodiscard]] VkExtent2D GetImageExtent() const;
};

/**
 * @class ShaderModule
 * @brief Represents a Vulkan shader module resource.
 */
class ShaderModule final : public IGraphicsResource {
  VkShaderModule vk_shader_module_ = VK_NULL_HANDLE; /**< Vulkan shader module handle. */

 public:
  /**
   * @brief Constructs a ShaderModule from Vulkan shader module creation info.
   * @param create_info Vulkan shader module creation information.
   */
  ShaderModule(const VkShaderModuleCreateInfo& create_info);

  /**
   * @brief Destructor for ShaderModule.
   */
  ~ShaderModule() override;

  /**
   * @brief Retrieves the Vulkan shader module handle.
   * @return Vulkan shader module handle.
   */
  [[nodiscard]] VkShaderModule GetVkShaderModule() const;
};

/**
 * @class PipelineLayout
 * @brief Represents a Vulkan pipeline layout resource.
 */
class PipelineLayout final : public IGraphicsResource {
  VkPipelineLayout vk_pipeline_layout_ = VK_NULL_HANDLE; /**< Vulkan pipeline layout handle. */

  VkPipelineLayoutCreateFlags flags_;                     /**< Vulkan pipeline layout creation flags. */
  std::vector<VkDescriptorSetLayout> set_layouts_;        /**< Descriptor set layouts for the pipeline layout. */
  std::vector<VkPushConstantRange> push_constant_ranges_; /**< Push constant ranges for the pipeline layout. */

 public:
  /**
   * @brief Constructs a PipelineLayout from Vulkan pipeline layout creation info.
   * @param pipeline_layout_create_info Vulkan pipeline layout creation information.
   */
  PipelineLayout(const VkPipelineLayoutCreateInfo& pipeline_layout_create_info);

  /**
   * @brief Destructor for PipelineLayout.
   */
  ~PipelineLayout() override;

  /**
   * @brief Retrieves the Vulkan pipeline layout handle.
   * @return Vulkan pipeline layout handle.
   */
  [[nodiscard]] VkPipelineLayout GetVkPipelineLayout() const;
};

/**
 * @class CommandPool
 * @brief Represents a Vulkan command pool resource.
 */
class CommandPool final : public IGraphicsResource {
  VkCommandPool vk_command_pool_ = VK_NULL_HANDLE; /**< Vulkan command pool handle. */

 public:
  /**
   * @brief Constructs a CommandPool with Vulkan command pool creation info.
   * @param command_pool_create_info Vulkan command pool creation information.
   */
  explicit CommandPool(const VkCommandPoolCreateInfo& command_pool_create_info);

  /**
   * @brief Destructor for CommandPool.
   */
  ~CommandPool() override;

  /**
   * @brief Retrieves the Vulkan command pool handle.
   * @return Vulkan command pool handle.
   */
  [[nodiscard]] VkCommandPool GetVkCommandPool() const;
};

/**
 * @class Buffer
 * @brief Represents a Vulkan buffer resource.
 */
class Buffer final : public IGraphicsResource {
  struct GpuState;
  std::shared_ptr<GpuState> gpu_state_;

  /**
   * @brief Allocates memory for the buffer with the specified creation info and allocation info.
   * @param buffer_create_info Vulkan buffer creation information.
   * @param vma_allocation_create_info VMA allocation creation information.
   */
  void Allocate(VkBufferCreateInfo buffer_create_info, const VmaAllocationCreateInfo& vma_allocation_create_info);
  static void AllocateOnGpuThread(const std::shared_ptr<GpuState>& state, VkBufferCreateInfo buffer_create_info,
                                  const VmaAllocationCreateInfo& vma_allocation_create_info);
  static void ResizeOnGpuThread(const std::shared_ptr<GpuState>& state, VkDeviceSize new_size);
  static void DestroyOnGpuThread(const std::shared_ptr<GpuState>& state);
  static void UploadDataOnGpuThread(const std::shared_ptr<GpuState>& state, size_t size, const void* src,
                                    VkDeviceSize dst_offset);
  static void DownloadDataOnGpuThread(const std::shared_ptr<GpuState>& state, size_t size, void* dst);
  static void CopyFromBufferOnGpuThread(const std::shared_ptr<GpuState>& state,
                                        const std::shared_ptr<GpuState>& src_state, VkDeviceSize size,
                                        VkDeviceSize src_offset, VkDeviceSize dst_offset);
  static void CopyFromBufferOnGpuThread(const std::shared_ptr<GpuState>& state, VkBuffer src_buffer, VkDeviceSize size,
                                        VkDeviceSize src_offset, VkDeviceSize dst_offset);
  static void CopyFromImageOnGpuThread(const std::shared_ptr<GpuState>& state, Image& src_image,
                                       const VkBufferImageCopy& image_copy_info);
  void TrackPendingGpuWork(const GpuWorkHandle& handle) const;
  void WaitForPendingGpuWork() const;

 public:
  /**
   * @brief Constructs a staging Buffer with the specified size and access mode.
   * @param staging_buffer_size Size for the staging buffer.
   * @param random_access Whether random access is required.
   */
  explicit Buffer(size_t staging_buffer_size, bool random_access = false);

  /**
   * @brief Constructs a Buffer with Vulkan buffer creation info.
   * @param buffer_create_info Vulkan buffer creation information.
   */
  explicit Buffer(const VkBufferCreateInfo& buffer_create_info);
  /**
   * @brief Destructor for Buffer.
   */
  ~Buffer() override;
  /**
   * @brief Constructs a Buffer with Vulkan buffer creation info and VMA allocation info.
   * @param buffer_create_info Vulkan buffer creation information.
   * @param vma_allocation_create_info VMA allocation creation information.
   */
  Buffer(const VkBufferCreateInfo& buffer_create_info, const VmaAllocationCreateInfo& vma_allocation_create_info);

  /**
   * @brief Uploads data to the buffer.
   * @param size Size of the data to upload.
   * @param src Pointer to the data source.
   */
  void UploadData(size_t size, const void* src);

  /**
   * @brief Uploads data into an existing buffer subrange.
   * @param size Size of the data to upload.
   * @param src Pointer to the data source.
   * @param dst_offset Destination byte offset.
   */
  void UploadSubData(size_t size, const void* src, VkDeviceSize dst_offset);

  /**
   * @brief Enqueues an asynchronous upload to the buffer.
   * @param size Size of the data to upload.
   * @param src Pointer to the data source. The data is copied before this function returns.
   */
  [[nodiscard]] GpuWorkHandle UploadDataAsync(size_t size, const void* src);

  /**
   * @brief Enqueues an asynchronous upload into an existing buffer subrange.
   * @param size Size of the data to upload.
   * @param src Pointer to the data source. The data is copied before this function returns.
   * @param dst_offset Destination byte offset.
   */
  [[nodiscard]] GpuWorkHandle UploadSubDataAsync(size_t size, const void* src, VkDeviceSize dst_offset);

  /**
   * @brief Downloads data from the buffer.
   * @param size Size of the data to download.
   * @param dst Pointer to the destination buffer.
   */
  void DownloadData(size_t size, void* dst);

  /**
   * @brief Enqueues an asynchronous readback from the buffer.
   * @param size Size of the data to download.
   * @return A future containing the downloaded bytes.
   */
  [[nodiscard]] std::shared_future<std::vector<std::byte>> DownloadDataAsync(size_t size);

  /**
   * @brief Resizes the buffer to the specified size.
   * @param new_size New size of the buffer.
   */
  void Resize(VkDeviceSize new_size);

  /**
   * @brief Uploads a vector of data to the buffer.
   * @tparam T Type of vector elements.
   * @param data The data vector to upload.
   */
  template <typename T>
  void UploadVector(const std::vector<T>& data);

  /**
   * @brief Uploads a single object of data to the buffer.
   * @tparam T Type of object.
   * @param data The data to upload.
   */
  template <typename T>
  void Upload(const T& data);

  /**
   * @brief Downloads a vector of data from the buffer.
   * @tparam T Type of vector elements.
   * @param data The destination data vector.
   * @param element_size Number of elements to download.
   */
  template <typename T>
  void DownloadVector(std::vector<T>& data, size_t element_size);

  /**
   * @brief Downloads a single object of data from the buffer.
   * @tparam T Type of object.
   * @param data The destination data object.
   */
  template <typename T>
  void Download(T& data);

  /**
   * @brief Copies data from another buffer.
   * @param src_buffer Source buffer.
   * @param size Size of the data to copy.
   * @param src_offset Offset in the source buffer.
   * @param dst_offset Offset in the destination buffer.
   */
  void CopyFromBuffer(const Buffer& src_buffer, VkDeviceSize size, VkDeviceSize src_offset = 0,
                      VkDeviceSize dst_offset = 0);

  /**
   * @brief Copies data from an image to the buffer.
   * @param src_image Source image.
   * @param image_copy_info Vulkan buffer image copy information.
   */
  void CopyFromImage(Image& src_image, const VkBufferImageCopy& image_copy_info) const;

  /**
   * @brief Copies data from an image to the buffer with a specified pixel size.
   * @param src_image Source image.
   * @param pixel_size Pixel size of the data.
   */
  void CopyFromImage(Image& src_image, VkDeviceSize pixel_size = 16);

  /**
   * @brief Copies depth data from an image to the buffer with a specified pixel size.
   * @param src_image Source image.
   * @param pixel_size Pixel size of the depth data.
   */
  void CopyFromDepth(Image& src_image, VkDeviceSize pixel_size = 4);

  /**
   * @brief Records a buffer fill through the SDK binary.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param offset Offset into the buffer.
   * @param size Number of bytes to fill.
   * @param data Fill value.
   */
  void Fill(VkCommandBuffer vk_command_buffer, VkDeviceSize offset, VkDeviceSize size, uint32_t data) const;

  /**
   * @brief Binds this buffer as a vertex buffer through the SDK binary.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param first_binding First vertex buffer binding slot.
   * @param offset Offset into the buffer.
   */
  void BindVertex(VkCommandBuffer vk_command_buffer, uint32_t first_binding = 0, VkDeviceSize offset = 0) const;

  /**
   * @brief Binds this buffer as an index buffer through the SDK binary.
   * @param vk_command_buffer The Vulkan command buffer.
   * @param offset Offset into the buffer.
   * @param index_type Vulkan index type.
   */
  void BindIndex(VkCommandBuffer vk_command_buffer, VkDeviceSize offset = 0,
                 VkIndexType index_type = VK_INDEX_TYPE_UINT32) const;

  /**
   * @brief Retrieves the Vulkan buffer handle.
   * @return Vulkan buffer handle.
   */
  [[nodiscard]] const VkBuffer& GetVkBuffer() const;

  /**
   * @brief Retrieves the current allocated buffer size in bytes.
   * @return Current buffer size in bytes.
   */
  [[nodiscard]] VkDeviceSize GetSize() const;

  /**
   * @brief Retrieves the VMA allocation handle for the buffer.
   * @return VMA allocation handle.
   */
  [[nodiscard]] VmaAllocation GetVmaAllocation() const;

  /**
   * @brief Retrieves the device address of the buffer.
   * @return Device address of the buffer.
   */
  [[nodiscard]] VkDeviceAddress GetDeviceAddress() const;

  /**
   * @brief Retrieves the VMA allocation information for the buffer.
   * @return VMA allocation information.
   */
  [[nodiscard]] const VmaAllocationInfo& GetVmaAllocationInfo() const;

  void SetDebugName(const std::string& name) const;
};
template <typename T>
void Buffer::UploadVector(const std::vector<T>& data) {
  if (data.empty())
    return;
  const T* address = data.data();
  UploadData(data.size() * sizeof(T), static_cast<const void*>(address));
}

template <typename T>
void Buffer::Upload(const T& data) {
  UploadData(sizeof(T), static_cast<const void*>(&data));
}

template <typename T>
void Buffer::DownloadVector(std::vector<T>& data, size_t element_size) {
  data.resize(element_size);
  T* address = data.data();
  DownloadData(data.size() * sizeof(T), address);
}

template <typename T>
void Buffer::Download(T& data) {
  DownloadData(sizeof(T), static_cast<void*>(&data));
}

/**
 * @class Sampler
 * @brief Represents a Vulkan sampler resource.
 */
class Sampler final : public IGraphicsResource {
  VkSampler vk_sampler_; /**< Vulkan sampler handle. */

 public:
  /**
   * @brief Constructs a Sampler with Vulkan sampler creation info.
   * @param sampler_create_info Vulkan sampler creation information.
   */
  explicit Sampler(const VkSamplerCreateInfo& sampler_create_info);

  /**
   * @brief Destructor for Sampler.
   */
  ~Sampler() override;

  /**
   * @brief Retrieves the Vulkan sampler handle.
   * @return Vulkan sampler handle.
   */
  [[nodiscard]] VkSampler GetVkSampler() const;
};

/**
 * @struct DescriptorBinding
 * @brief Represents a binding in a descriptor set layout.
 */
struct DescriptorBinding {
  VkDescriptorSetLayoutBinding binding;   /**< Vulkan descriptor set layout binding. */
  VkDescriptorBindingFlags binding_flags; /**< Vulkan descriptor binding flags. */
};

/**
 * @class DescriptorSetLayout
 * @brief Represents a Vulkan descriptor set layout resource.
 */
class DescriptorSetLayout final : public IGraphicsResource {
  friend class DescriptorSet;

  std::unordered_map<uint32_t, DescriptorBinding> descriptor_set_layout_bindings_; /**< Descriptor bindings. */
  VkDescriptorSetLayout vk_descriptor_set_layout_ = VK_NULL_HANDLE; /**< Vulkan descriptor set layout handle. */

 public:
  /**
   * @brief Destructor for DescriptorSetLayout.
   */
  ~DescriptorSetLayout() override;

  /**
   * @brief Retrieves the Vulkan descriptor set layout handle.
   * @return Vulkan descriptor set layout handle.
   */
  [[nodiscard]] const VkDescriptorSetLayout& GetVkDescriptorSetLayout() const;

  /**
   * @brief Adds a descriptor binding to the layout.
   * @param binding_index Index of the binding.
   * @param type Vulkan descriptor type.
   * @param stage_flags Shader stage flags for the binding.
   * @param binding_flags Descriptor binding flags.
   * @param descriptor_count Number of descriptors in the binding. Defaults to 1.
   */
  void PushDescriptorBinding(uint32_t binding_index, VkDescriptorType type, VkShaderStageFlags stage_flags,
                             VkDescriptorBindingFlags binding_flags, uint32_t descriptor_count = 1);

  /**
   * @brief Initializes the descriptor set layout.
   */
  void Initialize();
};

/**
 * @class DescriptorPool
 * @brief Represents a Vulkan descriptor pool resource.
 */
class DescriptorPool final : public IGraphicsResource {
  VkDescriptorPool vk_descriptor_pool_ = VK_NULL_HANDLE; /**< Vulkan descriptor pool handle. */

 public:
  /**
   * @brief Constructs a DescriptorPool with Vulkan descriptor pool creation info.
   * @param descriptor_pool_create_info Vulkan descriptor pool creation information.
   */
  explicit DescriptorPool(const VkDescriptorPoolCreateInfo& descriptor_pool_create_info);

  /**
   * @brief Destructor for DescriptorPool.
   */
  ~DescriptorPool() override;

  /**
   * @brief Retrieves the Vulkan descriptor pool handle.
   * @return Vulkan descriptor pool handle.
   */
  [[nodiscard]] VkDescriptorPool GetVkDescriptorPool() const;
};

/**
 * @class ShaderExt
 * @brief Represents an extended Vulkan shader resource.
 */
class ShaderExt final : public IGraphicsResource {
  VkShaderEXT shader_ext_ = VK_NULL_HANDLE; /**< Vulkan extended shader handle. */

  VkShaderCreateFlagsEXT flags_;                            /**< Vulkan shader creation flags. */
  VkShaderStageFlagBits stage_;                             /**< Shader stage (e.g., vertex, fragment). */
  VkShaderStageFlags next_stage_;                           /**< Next shader stage for dependency. */
  VkShaderCodeTypeEXT code_type_;                           /**< Shader code type. */
  std::string name_;                                        /**< Shader name. */
  std::vector<VkDescriptorSetLayout> set_layouts_;          /**< Descriptor set layouts used by the shader. */
  std::vector<VkPushConstantRange> push_constant_ranges_;   /**< Push constant ranges used by the shader. */
  std::optional<VkSpecializationInfo> specialization_info_; /**< Shader specialization information. */

 public:
  /**
   * @brief Constructs a ShaderExt with extended Vulkan shader creation info.
   * @param shader_create_info_ext Vulkan extended shader creation information.
   */
  explicit ShaderExt(const VkShaderCreateInfoEXT& shader_create_info_ext);

  /**
   * @brief Destructor for ShaderExt.
   */
  ~ShaderExt() override;

  /**
   * @brief Retrieves the Vulkan extended shader handle.
   * @return Vulkan extended shader handle.
   */
  [[nodiscard]] const VkShaderEXT& GetVkShaderExt() const;
};

/**
 * @enum CommandBufferStatus
 * @brief Represents the status of a command buffer.
 */
enum class CommandBufferStatus { Ready, Recording, Recorded, Invalid };

/**
 * @class CommandBuffer
 * @brief Represents a Vulkan command buffer resource.
 */
class CommandBuffer final : public IGraphicsResource {
  friend class Platform;

  CommandBufferStatus status_ = CommandBufferStatus::Invalid; /**< Current status of the command buffer. */
  VkCommandBuffer vk_command_buffer_ = VK_NULL_HANDLE;        /**< Vulkan command buffer handle. */
  VkCommandPool vk_command_pool_ = VK_NULL_HANDLE;            /**< Vulkan command pool that owns this buffer. */

 public:
  /**
   * @brief Retrieves the current status of the command buffer.
   * @return Status of the command buffer.
   */
  CommandBufferStatus GetStatus() const;

  /**
   * @brief Constructs a CommandBuffer with the specified command buffer level.
   * @param buffer_level Vulkan command buffer level. Defaults to primary level.
   */
  CommandBuffer(const VkCommandBufferLevel& buffer_level = VK_COMMAND_BUFFER_LEVEL_PRIMARY);

  /**
   * @brief Constructs a CommandBuffer from a specific command pool.
   * @param command_pool Vulkan command pool to allocate from.
   * @param buffer_level Vulkan command buffer level.
   */
  explicit CommandBuffer(VkCommandPool command_pool,
                         const VkCommandBufferLevel& buffer_level = VK_COMMAND_BUFFER_LEVEL_PRIMARY);

  /**
   * @brief Destructor for CommandBuffer.
   */
  ~CommandBuffer() override;

  /**
   * @brief Retrieves the Vulkan command buffer handle.
   * @return Vulkan command buffer handle.
   */
  [[nodiscard]] const VkCommandBuffer& GetVkCommandBuffer() const;

  /**
   * @brief Begins recording commands in the command buffer.
   * @param usage Vulkan command buffer usage flags. Defaults to one-time submit mode.
   */
  void Begin(const VkCommandBufferUsageFlags& usage = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

  /**
   * @brief Ends recording commands in the command buffer.
   */
  void End();

  /**
   * @brief Records commands using the specified callback function.
   * @param commands Callback function to record Vulkan commands.
   * @return True if recording completed successfully.
   */
  [[nodiscard]] bool Record(const std::function<void(VkCommandBuffer vk_command_buffer)>& commands);

  /**
   * @brief Resets the command buffer to its initial state.
   */
  void Reset();
};

/**
 * @class CommandQueue
 * @brief Represents a Vulkan command queue resource.
 */
class CommandQueue final : public IGraphicsResource {
  friend class Platform;

  VkQueue vk_queue_ = VK_NULL_HANDLE; /**< Vulkan command queue handle. */

 public:
  /**
   * @brief Submits command buffers to the queue with synchronization primitives.
   * @param command_buffers List of shared command buffers to submit.
   * @param offset Offset for starting the command buffer range.
   * @param buffer_count Number of buffers to submit.
   * @param wait_semaphores List of semaphores to wait for, along with pipeline stage flags.
   * @param signal_semaphores List of semaphores to signal after submission.
   * @param fence Fence to signal when submission is completed.
   */
  void Submit(const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers, uint32_t offset,
              uint32_t buffer_count,
              const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
              const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores,
              const std::shared_ptr<Fence>& fence) const;

  /**
   * @brief Submits command buffers to the queue with synchronization primitives.
   * @param command_buffers List of shared command buffers to submit.
   * @param wait_semaphores List of semaphores to wait for, along with pipeline stage flags.
   * @param signal_semaphores List of semaphores to signal after submission.
   * @param fence Fence to signal when submission is completed.
   */
  void Submit(const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
              const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
              const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores,
              const std::shared_ptr<Fence>& fence) const;

  /**
   * @brief Submits command buffers to the queue without a fence.
   * @param command_buffers List of shared command buffers to submit.
   * @param wait_semaphores List of semaphores to wait for, along with pipeline stage flags.
   * @param signal_semaphores List of semaphores to signal after submission.
   */
  void Submit(const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
              const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
              const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores) const;

  /**
   * @brief Submits command buffers immediately to the queue with synchronization primitives.
   * @param command_buffers List of shared command buffers to submit.
   * @param wait_semaphores List of semaphores to wait for, along with pipeline stage flags.
   * @param signal_semaphores List of semaphores to signal after submission.
   */
  void ImmediateSubmit(const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
                       const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
                       const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores) const;

  /**
   * @brief Presents swapchain images to the queue.
   * @param wait_semaphores List of semaphores to wait for before presenting.
   * @param targets List of pairs of swapchain objects and image indices to present.
   */
  void Present(const std::vector<std::shared_ptr<Semaphore>>& wait_semaphores,
               const std::vector<std::pair<std::shared_ptr<Swapchain>, uint32_t>>& targets) const;

  /**
   * @brief Waits until all pending operations in the queue are completed.
   */
  void WaitIdle() const;

  /**
   * @brief Retrieves the Vulkan command queue handle.
   * @return Vulkan command queue handle.
   */
  VkQueue GetVkQueue() const;
};

/**
 * @class BottomLevelAccelerationStructure
 * @brief Represents a Vulkan bottom-level acceleration structure resource.
 */
class BottomLevelAccelerationStructure final : public IGraphicsResource {
  VkAccelerationStructureKHR vk_acceleration_structure_khr_ =
      VK_NULL_HANDLE;                                       /**< Vulkan bottom-level acceleration structure handle. */
  std::shared_ptr<Buffer> acceleration_structure_buffer_{}; /**< Buffer associated with the acceleration structure. */
  VkDeviceAddress device_address_{};                        /**< Device address of the structure. */

  std::shared_ptr<Buffer> vertex_buffer;    /**< Buffer for vertex data. */
  std::shared_ptr<Buffer> index_buffer;     /**< Buffer for index data. */
  std::shared_ptr<Buffer> transform_buffer; /**< Buffer for transform data. */
  std::shared_ptr<Buffer> scratch_buffer_{};
  std::vector<std::shared_ptr<Buffer>> vertex_staging_buffers_{};
  uint32_t vertex_count_ = 0;
  uint32_t primitive_count_ = 0;
  uint32_t content_version_ = 0;
  uint32_t pending_content_version_ = 0;
  bool allow_update_ = false;
  bool pending_update_ = false;
  bool telemetry_registered_ = false;
  bool telemetry_updateable_ = false;
  VkDeviceSize telemetry_uncompacted_bytes_ = 0;
  VkDeviceSize telemetry_compacted_bytes_ = 0;
  VkDeviceSize telemetry_private_input_bytes_ = 0;
  std::shared_ptr<FrameSubmissionState> pending_submission_state_{};

  void ResolvePendingUpdate();

  BottomLevelAccelerationStructure(uint32_t vertex_count, uint32_t primitive_count);

 public:
  /**
   * @brief Constructs a BottomLevelAccelerationStructure using vertex and triangle data.
   * @param vertices List of vertices for the structure.
   * @param triangles List of triangles for the structure.
   */
  explicit BottomLevelAccelerationStructure(const std::vector<Vertex>& vertices,
                                            const std::vector<glm::uvec3>& triangles, bool allow_update);

  [[nodiscard]] static std::shared_ptr<BottomLevelAccelerationStructure> CreateStatic(
      const std::shared_ptr<RangeDescriptor>& meshlet_range, const std::shared_ptr<RangeDescriptor>& triangle_range,
      const std::vector<Vertex>& vertices);
  static void ProcessStaticBuilds();
  static void WaitForActiveStaticBuild();
  static void WaitForStaticBuilds();
  [[nodiscard]] static bool HasPendingStaticBuilds();
  [[nodiscard]] static bool StaticBuildInProgress();
  [[nodiscard]] static StaticBlasBuildTelemetry GetStaticBuildTelemetry();

  /**
   * @brief Records an in-place vertex-only update for a dynamic BLAS.
   * @param vertices Packed vertices matching the topology used for the initial build.
   * @return Submission state used to commit or retry the update.
   */
  [[nodiscard]] std::shared_ptr<FrameSubmissionState> UpdateVertices(const std::vector<Vertex>& vertices);

  /**
   * @brief Retrieves the content version visible to work recorded for the current frame.
   * @return Committed or pending vertex-content version.
   */
  [[nodiscard]] uint32_t GetContentVersion();

  /**
   * @brief Destructor for BottomLevelAccelerationStructure.
   */
  ~BottomLevelAccelerationStructure() override;

  /**
   * @brief Retrieves the device address of the bottom-level acceleration structure.
   * @return Device address of the structure.
   */
  [[nodiscard]] VkDeviceAddress GetDeviceAddress() const;
  [[nodiscard]] bool IsReady() const;
};

/**
 * @class TopLevelAccelerationStructure
 * @brief Represents a Vulkan top-level acceleration structure resource.
 */
class TopLevelAccelerationStructure final : public IGraphicsResource {
 public:
  enum class UpdateMode { NoOp, Build, Update };

  struct InstanceUploadRange {
    uint32_t first_instance = 0;
    uint32_t instance_count = 0;
  };

  struct UploadTelemetry {
    uint64_t source_bytes = 0;
    uint64_t uploaded_bytes = 0;
    uint64_t range_count = 0;
    uint64_t operation_count = 0;
    uint64_t build_count = 0;
    uint64_t update_count = 0;
    uint64_t no_op_count = 0;
    uint64_t full_upload_count = 0;
    uint64_t zero_instance_upload_update_count = 0;

    UploadTelemetry& operator+=(const UploadTelemetry& other);
    [[nodiscard]] UploadTelemetry DeltaFrom(const UploadTelemetry& baseline) const;
  };

 private:
  VkAccelerationStructureKHR vk_acceleration_structure_khr_ =
      VK_NULL_HANDLE;                                       /**< Vulkan top-level acceleration structure handle. */
  std::shared_ptr<Buffer> acceleration_structure_buffer_{}; /**< Buffer associated with the acceleration structure. */
  VkDeviceAddress device_address_{};                        /**< Device address of the structure. */
  uint32_t instance_capacity_ = 0;
  std::shared_ptr<Buffer> instance_staging_buffer_{};
  std::shared_ptr<Buffer> instances_data_buffer_{};
  std::shared_ptr<Buffer> scratch_buffer_{};
  bool built_ = false;
  std::vector<VkAccelerationStructureInstanceKHR> previous_instances_{};
  std::vector<uint32_t> previous_blas_content_versions_{};
  std::vector<std::shared_ptr<BottomLevelAccelerationStructure>> committed_blas_references_{};
  bool pending_ = false;
  uint32_t pending_frame_index_ = 0;
  uint32_t pending_frame_count_ = 0;
  std::shared_ptr<FrameSubmissionState> pending_submission_state_{};
  std::vector<VkAccelerationStructureInstanceKHR> pending_instances_{};
  std::vector<uint32_t> pending_blas_content_versions_{};
  std::vector<std::shared_ptr<BottomLevelAccelerationStructure>> pending_final_blas_references_{};
  std::vector<std::shared_ptr<BottomLevelAccelerationStructure>> pending_retained_blas_references_{};
  std::vector<std::shared_ptr<Buffer>> pending_extra_staging_buffers_{};
  UploadTelemetry upload_telemetry_{};
  UploadTelemetry pending_upload_telemetry_{};

  void Allocate(uint32_t instance_capacity);
  void Destroy();
  void ResolvePendingUpdate();

 public:
  /**
   * @brief Constructs an initially empty top-level acceleration structure.
   */
  TopLevelAccelerationStructure() = default;

  /**
   * @brief Destructor for TopLevelAccelerationStructure.
   */
  ~TopLevelAccelerationStructure() override;

  /**
   * @brief Records a build or update for the current render instances on the main queue.
   * @param render_instance_storage Current frame-slot render instances.
   * @return The operation recorded, or NoOp when the instance input is unchanged.
   */
  UpdateMode Update(RenderInstanceStorage& render_instance_storage);

  /**
   * @brief Classifies an acceleration-structure input change.
   * @param built Whether the destination acceleration structure contains a completed build.
   * @param previous_instances Last completed or earlier same-frame instance input.
   * @param current_instances New instance input.
   * @return NoOp for identical input, Update for compatible input, or Build otherwise.
   */
  [[nodiscard]] static UpdateMode ClassifyUpdateMode(
      bool built, const std::vector<VkAccelerationStructureInstanceKHR>& previous_instances,
      const std::vector<VkAccelerationStructureInstanceKHR>& current_instances);

  [[nodiscard]] static UpdateMode ClassifyUpdateMode(
      bool built, const std::vector<VkAccelerationStructureInstanceKHR>& previous_instances,
      const std::vector<VkAccelerationStructureInstanceKHR>& current_instances,
      const std::vector<uint32_t>& previous_blas_content_versions,
      const std::vector<uint32_t>& current_blas_content_versions);

  [[nodiscard]] static std::vector<InstanceUploadRange> PlanInstanceUploadRanges(
      bool full_upload, const std::vector<VkAccelerationStructureInstanceKHR>& previous_instances,
      const std::vector<VkAccelerationStructureInstanceKHR>& current_instances);

  [[nodiscard]] UploadTelemetry GetUploadTelemetry();

  /**
   * @brief Retrieves the Vulkan handle for the top-level acceleration structure.
   * @return Vulkan top-level acceleration structure handle.
   */
  [[nodiscard]] VkAccelerationStructureKHR GetVkAccelerationStructure() const;

  /**
   * @brief Retrieves the device address of the top-level acceleration structure.
   * @return Device address of the structure.
   */
  [[nodiscard]] VkDeviceAddress GetDeviceAddress() const;
};

/**
 * @class DescriptorSet
 * @brief Represents a Vulkan descriptor set resource.
 */
class DescriptorSet final : public IGraphicsResource {
  std::shared_ptr<DescriptorSetLayout> descriptor_set_layout_; /**< Associated descriptor set layout. */
  VkDescriptorSet descriptor_set_ = VK_NULL_HANDLE;            /**< Vulkan descriptor set handle. */

 public:
  struct LifetimeStats {
    uint64_t live_count = 0;
    uint64_t peak_live_count = 0;
    uint64_t creation_count = 0;
  };

  [[nodiscard]] static LifetimeStats GetLifetimeStats();

  /**
   * @brief Retrieves the Vulkan descriptor set handle.
   * @return Vulkan descriptor set handle.
   */
  [[nodiscard]] const VkDescriptorSet& GetVkDescriptorSet() const;

  /**
   * @brief Destructor for DescriptorSet.
   */
  ~DescriptorSet() override;

  /**
   * @brief Constructs a DescriptorSet with the target layout.
   * @param target_layout Shared pointer to the target descriptor set layout.
   */
  DescriptorSet(const std::shared_ptr<DescriptorSetLayout>& target_layout);

  /**
   * @brief Updates an image binding in the descriptor set.
   * @param binding_index Target binding index.
   * @param image_info The Vulkan image info for the update.
   * @param array_element Array index for the binding. Defaults to 0.
   */
  void UpdateImageDescriptorBinding(uint32_t binding_index, const VkDescriptorImageInfo& image_info,
                                    uint32_t array_element = 0) const;

  /**
   * @brief Updates an acceleration structure binding in the descriptor set.
   * @param binding_index Target binding index.
   * @param acceleration_structure Vulkan acceleration structure handle.
   */
  void UpdateAccelerationStructureDescriptorBinding(uint32_t binding_index,
                                                    const VkAccelerationStructureKHR& acceleration_structure) const;

  /**
   * @brief Updates an acceleration structure binding in the descriptor set.
   * @param binding_index Target binding index.
   * @param acceleration_structure Shared pointer to the top-level acceleration structure.
   */
  void UpdateAccelerationStructureDescriptorBinding(
      uint32_t binding_index, const std::shared_ptr<TopLevelAccelerationStructure>& acceleration_structure) const;

  /**
   * @brief Updates a buffer binding in the descriptor set.
   * @param binding_index Target binding index.
   * @param buffer_info Vulkan buffer info for the update.
   * @param array_element Array index for the binding. Defaults to 0.
   */
  void UpdateBufferDescriptorBinding(uint32_t binding_index, const VkDescriptorBufferInfo& buffer_info,
                                     uint32_t array_element = 0) const;

  /**
   * @brief Updates a buffer binding in the descriptor set.
   * @param binding_index Target binding index.
   * @param buffer Shared pointer to the buffer resource.
   * @param array_element Array index for the binding. Defaults to 0.
   */
  void UpdateBufferDescriptorBinding(uint32_t binding_index, const std::shared_ptr<Buffer>& buffer,
                                     uint32_t array_element = 0) const;
};

}  // namespace evo_engine
