#pragma once
#include "shaderc/shaderc.h"
namespace evo_engine {
class CommandBuffer;

class IGraphicsResource {
 protected:
  IGraphicsResource() = default;

 public:
  template <typename T>
  static void ApplyVector(std::vector<T>& target, uint32_t size, const T* data);
  IGraphicsResource& operator=(IGraphicsResource&) = delete;
  IGraphicsResource& operator=(const IGraphicsResource&) = delete;
  virtual ~IGraphicsResource() = default;
};

template <typename T>
void IGraphicsResource::ApplyVector(std::vector<T>& target, uint32_t size, const T* data) {
  if (size == 0 || data == nullptr)
    return;
  target.resize(size);
  memcpy(target.data(), data, sizeof(T) * size);
}

class Fence final : public IGraphicsResource {
  VkFence vk_fence_ = VK_NULL_HANDLE;
  VkFenceCreateFlags flags_ = {};

 public:
  explicit Fence(const VkFenceCreateInfo& vk_fence_create_info);
  ~Fence() override;

  [[nodiscard]] const VkFence& GetVkFence() const;
};

class Semaphore final : public IGraphicsResource {
  VkSemaphore vk_semaphore_ = VK_NULL_HANDLE;
  VkSemaphoreCreateFlags flags_ = {};

 public:
  explicit Semaphore(const VkSemaphoreCreateInfo& semaphore_create_info);
  ~Semaphore() override;
  [[nodiscard]] const VkSemaphore& GetVkSemaphore() const;
#ifdef _WIN64
  void* GetVkSemaphoreHandle(VkExternalSemaphoreHandleTypeFlagBitsKHR external_semaphore_handle_type) const;
#else
  int GetVkSemaphoreHandle(VkExternalSemaphoreHandleTypeFlagBitsKHR external_semaphore_handle_type) const;
#endif
};

class Image final : public IGraphicsResource {
  VkImage vk_image_ = VK_NULL_HANDLE;
  VmaAllocation vma_allocation_ = VK_NULL_HANDLE;
  VmaAllocationInfo vma_allocation_info_ = {};

  VkImageCreateFlags flags_;
  VkImageType image_type_;
  VkFormat format_;
  VkExtent3D extent_;
  uint32_t mip_levels_;
  uint32_t array_layers_;
  VkSampleCountFlagBits samples_;
  VkImageTiling tiling_;
  VkImageUsageFlags usage_;
  VkSharingMode sharing_mode_;
  std::vector<uint32_t> queue_family_indices_;
  VkImageLayout initial_layout_;

  VkImageLayout layout_;
 public:
  [[nodiscard]] uint32_t GetMipLevels() const;
  explicit Image(VkImageCreateInfo image_create_info);
  Image(VkImageCreateInfo image_create_info, const VmaAllocationCreateInfo& vma_allocation_create_info);
  bool HasStencilComponent() const;
  ~Image() override;
  void TransitImageLayout(VkCommandBuffer vk_command_buffer, VkImageLayout new_layout);
  void CopyFromBuffer(VkCommandBuffer vk_command_buffer, const VkBuffer& src_buffer,
                      VkDeviceSize src_offset = 0) const;

  void GenerateMipmaps(VkCommandBuffer vk_command_buffer);

  [[nodiscard]] VkImage GetVkImage() const;
  [[nodiscard]] VkFormat GetFormat() const;
  [[nodiscard]] VmaAllocation GetVmaAllocation() const;
  [[nodiscard]] VkExtent3D GetExtent() const;
  [[nodiscard]] VkImageLayout GetLayout() const;
  [[nodiscard]] const VmaAllocationInfo& GetVmaAllocationInfo() const;

#ifdef _WIN64
  void* GetVkImageMemHandle(VkExternalMemoryHandleTypeFlagsKHR external_memory_handle_type) const;
#else
  int GetVkImageMemHandle(VkExternalMemoryHandleTypeFlagsKHR external_memory_handle_type) const;
#endif
};

class ImageView final : public IGraphicsResource {
  VkImageView vk_image_view_ = VK_NULL_HANDLE;

  VkImageViewCreateFlags flags_;
  std::shared_ptr<Image> image_;
  VkImageViewType view_type_;
  VkFormat format_;
  VkComponentMapping components_;
  VkImageSubresourceRange subresource_range_;
  friend class Swapchain;
  friend class Platform;

 public:
  explicit ImageView(const VkImageViewCreateInfo& image_view_create_info);
  explicit ImageView(const VkImageViewCreateInfo& image_view_create_info, const std::shared_ptr<Image>& image);
  ~ImageView() override;
  [[nodiscard]] VkImageView GetVkImageView() const;

  [[nodiscard]] const std::shared_ptr<Image>& GetImage() const;
};

class Swapchain final : public IGraphicsResource {
  VkSwapchainKHR vk_swapchain_ = VK_NULL_HANDLE;
  std::vector<VkImage> vk_images_;

  VkSwapchainCreateFlagsKHR flags_;
  VkSurfaceKHR surface_;
  uint32_t min_image_count_;
  VkFormat image_format_;
  VkColorSpaceKHR image_color_space_;
  VkExtent2D image_extent_;
  uint32_t image_array_layers_;
  VkImageUsageFlags image_usage_;
  VkSharingMode image_sharing_mode_;
  std::vector<uint32_t> queue_family_indices_;
  VkSurfaceTransformFlagBitsKHR pre_transform_;
  VkCompositeAlphaFlagBitsKHR composite_alpha_;
  VkPresentModeKHR present_mode_;
  VkBool32 clipped_;

  std::vector<std::shared_ptr<ImageView>> vk_image_views_;

 public:
  explicit Swapchain(const VkSwapchainCreateInfoKHR& swapchain_create_info);
  ~Swapchain() override;

  [[nodiscard]] VkSwapchainKHR GetVkSwapchain() const;

  [[nodiscard]] const std::vector<VkImage>& GetAllVkImages() const;
  [[nodiscard]] const VkImage& GetVkImage() const;
  [[nodiscard]] const VkImageView& GetVkImageView() const;
  [[nodiscard]] const std::vector<std::shared_ptr<ImageView>>& GetAllImageViews() const;

  [[nodiscard]] VkFormat GetImageFormat() const;

  [[nodiscard]] VkExtent2D GetImageExtent() const;
};

class ShaderModule final : public IGraphicsResource {
  VkShaderModule vk_shader_module_ = VK_NULL_HANDLE;

 public:
  ~ShaderModule() override;

  ShaderModule(const VkShaderModuleCreateInfo& create_info);

  [[nodiscard]] VkShaderModule GetVkShaderModule() const;
};

class PipelineLayout final : public IGraphicsResource {
  VkPipelineLayout vk_pipeline_layout_ = VK_NULL_HANDLE;

  VkPipelineLayoutCreateFlags flags_;
  std::vector<VkDescriptorSetLayout> set_layouts_;
  std::vector<VkPushConstantRange> push_constant_ranges_;

 public:
  PipelineLayout(const VkPipelineLayoutCreateInfo& pipeline_layout_create_info);
  ~PipelineLayout() override;

  [[nodiscard]] VkPipelineLayout GetVkPipelineLayout() const;
};

class CommandPool final : public IGraphicsResource {
  VkCommandPool vk_command_pool_ = VK_NULL_HANDLE;

 public:
  explicit CommandPool(const VkCommandPoolCreateInfo& command_pool_create_info);

  ~CommandPool() override;

  [[nodiscard]] VkCommandPool GetVkCommandPool() const;
};

class Buffer final : public IGraphicsResource {
  VkBuffer vk_buffer_ = VK_NULL_HANDLE;
  VmaAllocation vma_allocation_ = VK_NULL_HANDLE;
  VmaAllocationInfo vma_allocation_info_ = {};

  VkBufferCreateFlags flags_ = {};
  VkDeviceSize size_ = {};
  VkBufferUsageFlags usage_ = {};
  VkSharingMode sharing_mode_ = {};
  std::vector<uint32_t> queue_family_indices_ = {};
  VmaAllocationCreateInfo vma_allocation_create_info_ = {};

  void UploadData(size_t size, const void* src);
  void DownloadData(size_t size, void* dst);
  void Allocate(VkBufferCreateInfo buffer_create_info, const VmaAllocationCreateInfo& vma_allocation_create_info);

 public:
  explicit Buffer(size_t staging_buffer_size, bool random_access = false);
  explicit Buffer(const VkBufferCreateInfo& buffer_create_info);
  ~Buffer() override;
  Buffer(const VkBufferCreateInfo& buffer_create_info, const VmaAllocationCreateInfo& vma_allocation_create_info);
  void Resize(VkDeviceSize new_size);
  template <typename T>
  void UploadVector(const std::vector<T>& data);
  template <typename T>
  void Upload(const T& data);
  template <typename T>
  void DownloadVector(std::vector<T>& data, size_t element_size);
  template <typename T>
  void Download(T& data);
  void CopyFromBuffer(const Buffer& src_buffer, VkDeviceSize size, VkDeviceSize src_offset = 0,
                      VkDeviceSize dst_offset = 0);
  void CopyFromImage(Image& src_image, const VkBufferImageCopy& image_copy_info) const;
  void CopyFromImage(Image& src_image);
  [[nodiscard]] const VkBuffer& GetVkBuffer() const;

  [[nodiscard]] VmaAllocation GetVmaAllocation() const;

  [[nodiscard]] const VmaAllocationInfo& GetVmaAllocationInfo() const;
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

class Sampler final : public IGraphicsResource {
  VkSampler vk_sampler_;

 public:
  explicit Sampler(const VkSamplerCreateInfo& sampler_create_info);
  ~Sampler() override;
  [[nodiscard]] VkSampler GetVkSampler() const;
};

struct DescriptorBinding {
  VkDescriptorSetLayoutBinding binding;
  VkDescriptorBindingFlags binding_flags;
};
class DescriptorSetLayout final : public IGraphicsResource {
  friend class DescriptorSet;
  std::unordered_map<uint32_t, DescriptorBinding> descriptor_set_layout_bindings_;
  VkDescriptorSetLayout vk_descriptor_set_layout_ = VK_NULL_HANDLE;

 public:
  ~DescriptorSetLayout() override;
  [[nodiscard]] const VkDescriptorSetLayout& GetVkDescriptorSetLayout() const;

  void PushDescriptorBinding(uint32_t binding_index, VkDescriptorType type, VkShaderStageFlags stage_flags,
                             VkDescriptorBindingFlags binding_flags, uint32_t descriptor_count = 1);
  void Initialize();
};

class DescriptorSet final : public IGraphicsResource {
  std::shared_ptr<DescriptorSetLayout> descriptor_set_layout_;
  VkDescriptorSet descriptor_set_ = VK_NULL_HANDLE;

 public:
  [[nodiscard]] const VkDescriptorSet& GetVkDescriptorSet() const;
  ~DescriptorSet() override;
  DescriptorSet(const std::shared_ptr<DescriptorSetLayout>& target_layout);
  /**
   * \brief UpdateImageDescriptorBinding
   * \param binding_index Target binding
   * \param image_info
   * \param array_element
   * \param imageInfos The image info for update. Make sure the size is max frame size.
   */
  void UpdateImageDescriptorBinding(uint32_t binding_index, const VkDescriptorImageInfo& image_info,
                                    uint32_t array_element = 0) const;
  void UpdateBufferDescriptorBinding(uint32_t binding_index, const VkDescriptorBufferInfo& buffer_info,
                                     uint32_t array_element = 0) const;

  void UpdateBufferDescriptorBinding(uint32_t binding_index, const std::shared_ptr<Buffer>& buffer,
                                     uint32_t array_element = 0) const;
};

class DescriptorPool final : public IGraphicsResource {
  VkDescriptorPool vk_descriptor_pool_ = VK_NULL_HANDLE;

 public:
  explicit DescriptorPool(const VkDescriptorPoolCreateInfo& descriptor_pool_create_info);
  ~DescriptorPool() override;
  [[nodiscard]] VkDescriptorPool GetVkDescriptorPool() const;
};

class ShaderExt final : public IGraphicsResource {
  VkShaderEXT shader_ext_ = VK_NULL_HANDLE;

  VkShaderCreateFlagsEXT flags_;
  VkShaderStageFlagBits stage_;
  VkShaderStageFlags next_stage_;
  VkShaderCodeTypeEXT code_type_;
  std::string name_;
  std::vector<VkDescriptorSetLayout> set_layouts_;
  std::vector<VkPushConstantRange> push_constant_ranges_;
  std::optional<VkSpecializationInfo> specialization_info_;

 public:
  explicit ShaderExt(const VkShaderCreateInfoEXT& shader_create_info_ext);
  ~ShaderExt() override;
  [[nodiscard]] const VkShaderEXT& GetVkShaderExt() const;
};

enum class CommandBufferStatus { Ready, Recording, Recorded, Invalid };
class CommandBuffer final : public IGraphicsResource {
  friend class Platform;
  CommandBufferStatus status_ = CommandBufferStatus::Invalid;
  VkCommandBuffer vk_command_buffer_ = VK_NULL_HANDLE;

 public:
  CommandBufferStatus GetStatus() const;
  CommandBuffer(const VkCommandBufferLevel& buffer_level = VK_COMMAND_BUFFER_LEVEL_PRIMARY);
  ~CommandBuffer() override;
  [[nodiscard]] const VkCommandBuffer& GetVkCommandBuffer() const;
  /**
   * Begins the recording state for this command buffer.
   * @param usage How this command buffer will be used.
   */
  void Begin(const VkCommandBufferUsageFlags& usage = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT);

  /**
   * Ends the recording state for this command buffer.
   */
  void End();

  void Record(const std::function<void(VkCommandBuffer vk_command_buffer)>& commands);

  void Reset();
};

class CommandQueue final : public IGraphicsResource {
  friend class Platform;
  VkQueue vk_queue_ = VK_NULL_HANDLE;

public:
  void Submit(const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
             const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
              const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores, const std::shared_ptr<Fence>& fence) const;

  void Submit(const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
              const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
              const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores) const;

  void ImmediateSubmit(const std::vector<std::shared_ptr<CommandBuffer>>& command_buffers,
                       const std::vector<std::pair<std::shared_ptr<Semaphore>, VkPipelineStageFlags>>& wait_semaphores,
                       const std::vector<std::shared_ptr<Semaphore>>& signal_semaphores) const;

  void Present(const std::vector<std::shared_ptr<Semaphore>>& wait_semaphores,
               const std::vector<std::pair<std::shared_ptr<Swapchain>, uint32_t>>& targets) const;

  void WaitIdle() const;
};

}  // namespace evo_engine
