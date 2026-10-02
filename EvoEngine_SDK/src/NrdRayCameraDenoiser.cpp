#include "NrdRayCameraDenoiser.hpp"

#ifdef EVOENGINE_ENABLE_NRD

#  include "Camera.hpp"
#  include "GraphicsResources.hpp"
#  include "Platform.hpp"

#  include <NRD.h>
#  include <NRI.h>

#  include <Extensions/NRIHelper.h>
#  include <Extensions/NRIRayTracing.h>
#  include <Extensions/NRIWrapperVK.h>

#  include <NRDIntegration.hpp>

#  include <cstring>
#  include <limits>

using namespace evo_engine;

namespace {
constexpr nrd::Identifier kDenoiser = 0u;

nrd::Resource NrdResource(const std::shared_ptr<Image>& image) {
  nrd::Resource resource{};
  resource.vk.image = reinterpret_cast<uint64_t>(image->GetVkImage());
  resource.vk.format = static_cast<VKEnum>(image->GetFormat());
  resource.state = {nri::AccessBits::SHADER_RESOURCE_STORAGE, nri::Layout::GENERAL, nri::StageBits::COMPUTE_SHADER};
  return resource;
}
}  // namespace

struct NrdRayCameraDenoiser::Impl {
  nrd::Integration integration;
  uint32_t width = 0u;
  uint32_t height = 0u;
  uint32_t frame_index = 0u;
  bool initialized = false;
};

NrdRayCameraDenoiser::NrdRayCameraDenoiser() : impl_(std::make_unique<Impl>()) {
}
NrdRayCameraDenoiser::~NrdRayCameraDenoiser() = default;

bool NrdRayCameraDenoiser::Initialize(const uint32_t width, const uint32_t height) {
  if (width == 0u || height == 0u || width > std::numeric_limits<uint16_t>::max() ||
      height > std::numeric_limits<uint16_t>::max())
    return false;
  const auto* library = nrd::GetLibraryDesc();
  if (library->normalEncoding != nrd::NormalEncoding::R10_G10_B10_A2_UNORM ||
      library->roughnessEncoding != nrd::RoughnessEncoding::LINEAR)
    return false;

  nri::QueueFamilyVKDesc queue_family{};
  queue_family.queueNum = 1u;
  queue_family.queueType = nri::QueueType::GRAPHICS;
  queue_family.familyIndex = Platform::GetGraphicsAndComputeQueueFamilyIndex();
  nri::DeviceCreationVKDesc device{};
  device.vkInstance = Platform::GetVkInstance();
  device.vkPhysicalDevice = Platform::GetSelectedPhysicalDevice()->vk_physical_device;
  device.vkDevice = Platform::GetVkDevice();
  device.queueFamilies = &queue_family;
  device.queueFamilyNum = 1u;
  device.minorVersion =
      static_cast<uint8_t>(VK_API_VERSION_MINOR(Platform::GetSelectedPhysicalDevice()->properties.apiVersion));

  const nrd::DenoiserDesc denoiser{kDenoiser, nrd::Denoiser::RELAX_DIFFUSE_SPECULAR};
  nrd::InstanceCreationDesc instance{};
  instance.denoisers = &denoiser;
  instance.denoisersNum = 1u;
  nrd::IntegrationCreationDesc integration{};
  std::memcpy(integration.name, "EvoEngine", sizeof("EvoEngine"));
  integration.resourceWidth = static_cast<uint16_t>(width);
  integration.resourceHeight = static_cast<uint16_t>(height);
  integration.queuedFrameNum = static_cast<uint8_t>(Platform::GetMaxFramesInFlight());
  integration.autoWaitForIdle = true;
  if (impl_->integration.RecreateVK(integration, instance, device) != nrd::Result::SUCCESS)
    return false;
  nrd::RelaxSettings settings{};
  settings.hitDistanceReconstructionMode = nrd::HitDistanceReconstructionMode::AREA_3X3;
  settings.diffusePrepassBlurRadius = 2.0f;
  settings.specularPrepassBlurRadius = 8.0f;
  settings.atrousIterationNum = 2u;
  if (impl_->integration.SetDenoiserSettings(kDenoiser, &settings) != nrd::Result::SUCCESS)
    return false;
  impl_->width = width;
  impl_->height = height;
  impl_->frame_index = 0u;
  impl_->initialized = true;
  return true;
}

bool NrdRayCameraDenoiser::Denoise(const VkCommandBuffer command_buffer, const CameraInfoBlock& camera,
                                   const RayCameraHistoryResources& history, const bool reset_history) {
  if (!impl_->initialized || !history.restir_nrd_motion_image || !history.restir_nrd_normal_roughness_image ||
      !history.restir_nrd_view_z_image || !history.restir_nrd_diffuse_signal_image ||
      !history.restir_nrd_specular_signal_image || !history.restir_nrd_denoised_diffuse_image ||
      !history.restir_nrd_denoised_specular_image)
    return false;

  impl_->integration.NewFrame();
  nrd::CommonSettings common{};
  std::memcpy(common.viewToClipMatrix, &camera.projection[0][0], sizeof(common.viewToClipMatrix));
  std::memcpy(common.worldToViewMatrix, &camera.view[0][0], sizeof(common.worldToViewMatrix));
  const auto previous_projection = glm::inverse(camera.previous_inverse_projection);
  const auto previous_view = glm::inverse(camera.previous_inverse_view);
  std::memcpy(common.viewToClipMatrixPrev, &previous_projection[0][0], sizeof(common.viewToClipMatrixPrev));
  std::memcpy(common.worldToViewMatrixPrev, &previous_view[0][0], sizeof(common.worldToViewMatrixPrev));
  common.resourceSize[0] = common.resourceSizePrev[0] = common.rectSize[0] = common.rectSizePrev[0] =
      static_cast<uint16_t>(impl_->width);
  common.resourceSize[1] = common.resourceSizePrev[1] = common.rectSize[1] = common.rectSizePrev[1] =
      static_cast<uint16_t>(impl_->height);
  common.motionVectorScale[2] = 1.0f;
  common.frameIndex = impl_->frame_index++;
  common.accumulationMode = reset_history ? nrd::AccumulationMode::CLEAR_AND_RESTART : nrd::AccumulationMode::CONTINUE;
  if (impl_->integration.SetCommonSettings(common) != nrd::Result::SUCCESS)
    return false;
  nrd::ResourceSnapshot resources{};
  resources.restoreInitialState = true;
  resources.SetResource(nrd::ResourceType::IN_MV, NrdResource(history.restir_nrd_motion_image));
  resources.SetResource(nrd::ResourceType::IN_NORMAL_ROUGHNESS, NrdResource(history.restir_nrd_normal_roughness_image));
  resources.SetResource(nrd::ResourceType::IN_VIEWZ, NrdResource(history.restir_nrd_view_z_image));
  resources.SetResource(nrd::ResourceType::IN_DIFF_RADIANCE_HITDIST,
                        NrdResource(history.restir_nrd_diffuse_signal_image));
  resources.SetResource(nrd::ResourceType::IN_SPEC_RADIANCE_HITDIST,
                        NrdResource(history.restir_nrd_specular_signal_image));
  resources.SetResource(nrd::ResourceType::OUT_DIFF_RADIANCE_HITDIST,
                        NrdResource(history.restir_nrd_denoised_diffuse_image));
  resources.SetResource(nrd::ResourceType::OUT_SPEC_RADIANCE_HITDIST,
                        NrdResource(history.restir_nrd_denoised_specular_image));
  nri::CommandBufferVKDesc command{};
  command.vkCommandBuffer = command_buffer;
  command.queueType = nri::QueueType::GRAPHICS;
  impl_->integration.DenoiseVK(&kDenoiser, 1u, command, resources);
  return true;
}

#else

struct evo_engine::NrdRayCameraDenoiser::Impl {};
evo_engine::NrdRayCameraDenoiser::NrdRayCameraDenoiser() : impl_(std::make_unique<Impl>()) {
}
evo_engine::NrdRayCameraDenoiser::~NrdRayCameraDenoiser() = default;
bool evo_engine::NrdRayCameraDenoiser::Initialize(uint32_t, uint32_t) {
  return false;
}
bool evo_engine::NrdRayCameraDenoiser::Denoise(VkCommandBuffer, const CameraInfoBlock&,
                                               const RayCameraHistoryResources&, bool) {
  return false;
}

#endif
