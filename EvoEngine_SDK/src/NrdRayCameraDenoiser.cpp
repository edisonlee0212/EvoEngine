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

#  include <algorithm>
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
  bool reblur = false;
  bool initialized = false;
};

NrdRayCameraDenoiser::NrdRayCameraDenoiser() : impl_(std::make_unique<Impl>()) {
}
NrdRayCameraDenoiser::~NrdRayCameraDenoiser() = default;

bool NrdRayCameraDenoiser::Initialize(const uint32_t width, const uint32_t height, const bool reblur) {
  if (width == 0u || height == 0u || width > std::numeric_limits<uint16_t>::max() ||
      height > std::numeric_limits<uint16_t>::max())
    return false;
  if (!Platform::GetSelectedPhysicalDevice()->CheckExtensionSupport(VK_KHR_PUSH_DESCRIPTOR_EXTENSION_NAME))
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
  const char* extensions[] = {VK_KHR_PUSH_DESCRIPTOR_EXTENSION_NAME};
  device.vkExtensions.deviceExtensions = extensions;
  device.vkExtensions.deviceExtensionNum = 1u;
  // Platform enables features through Vulkan 1.3; physical support alone does not enable 1.4 features.
  device.minorVersion = static_cast<uint8_t>(VK_API_VERSION_MINOR(
      std::min(VK_API_VERSION_1_3,
               std::min(volkGetInstanceVersion(), Platform::GetSelectedPhysicalDevice()->properties.apiVersion))));

  const nrd::DenoiserDesc denoiser{
      kDenoiser, reblur ? nrd::Denoiser::REBLUR_DIFFUSE_SPECULAR : nrd::Denoiser::RELAX_DIFFUSE_SPECULAR};
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
  impl_->width = width;
  impl_->height = height;
  impl_->frame_index = 0u;
  impl_->reblur = reblur;
  impl_->initialized = true;
  return true;
}

bool NrdRayCameraDenoiser::UsesReblur() const {
  return impl_->reblur;
}

bool NrdRayCameraDenoiser::Denoise(const VkCommandBuffer command_buffer, const CameraInfoBlock& camera,
                                   const RayCameraHistoryResources& history, const bool reset_history,
                                   const CameraSettings& camera_settings) {
  if (!impl_->initialized || !history.restir_nrd_motion_image || !history.restir_nrd_normal_roughness_image ||
      !history.restir_nrd_view_z_image || !history.restir_nrd_diffuse_signal_image ||
      !history.restir_nrd_specular_signal_image || !history.restir_nrd_denoised_diffuse_image ||
      !history.restir_nrd_denoised_specular_image)
    return false;

  impl_->integration.NewFrame();
  const bool accumulated_input =
      camera_settings.accumulate_samples && history.technique != RayCameraHistoryTechnique::RestirPt;
  if (impl_->reblur) {
    nrd::ReblurSettings settings{};
    settings.hitDistanceReconstructionMode = nrd::HitDistanceReconstructionMode::OFF;
    settings.minMaterialForDiffuse = 0.0f;
    settings.minMaterialForSpecular = 0.0f;
    settings.maxBlurRadius = std::max(camera_settings.nrd_reblur_max_blur_radius, 0.1f);
    settings.minBlurRadius = std::clamp(camera_settings.nrd_reblur_min_blur_radius, 0.0f, settings.maxBlurRadius);
    settings.diffusePrepassBlurRadius = std::max(camera_settings.nrd_reblur_diffuse_prepass_radius, 0.0f);
    settings.specularPrepassBlurRadius = std::max(camera_settings.nrd_reblur_specular_prepass_radius, 0.0f);
    settings.maxAccumulatedFrameNum =
        accumulated_input ? 0u
                          : static_cast<uint32_t>(std::clamp(camera_settings.nrd_reblur_history_frames, 0,
                                                             static_cast<int>(nrd::REBLUR_MAX_HISTORY_FRAME_NUM)));
    settings.maxFastAccumulatedFrameNum =
        std::min(settings.maxFastAccumulatedFrameNum, settings.maxAccumulatedFrameNum);
    settings.historyFixFrameNum =
        std::min(settings.historyFixFrameNum,
                 settings.maxFastAccumulatedFrameNum > 0u ? settings.maxFastAccumulatedFrameNum - 1u : 0u);
    settings.lobeAngleFraction = std::clamp(camera_settings.nrd_reblur_lobe_angle_fraction, 0.001f, 1.0f);
    settings.roughnessFraction = std::clamp(camera_settings.nrd_reblur_roughness_fraction, 0.001f, 1.0f);
    if (impl_->integration.SetDenoiserSettings(kDenoiser, &settings) != nrd::Result::SUCCESS)
      return false;
  } else {
    nrd::RelaxSettings settings{};
    settings.hitDistanceReconstructionMode = nrd::HitDistanceReconstructionMode::OFF;
    settings.minMaterialForDiffuse = 0.0f;
    settings.minMaterialForSpecular = 0.0f;
    settings.diffusePrepassBlurRadius = std::max(camera_settings.nrd_relax_diffuse_prepass_radius, 0.0f);
    settings.specularPrepassBlurRadius = std::max(camera_settings.nrd_relax_specular_prepass_radius, 0.0f);
    settings.diffuseMaxAccumulatedFrameNum =
        accumulated_input ? 0u
                          : static_cast<uint32_t>(std::clamp(camera_settings.nrd_relax_diffuse_history_frames, 0,
                                                             static_cast<int>(nrd::RELAX_MAX_HISTORY_FRAME_NUM)));
    settings.specularMaxAccumulatedFrameNum =
        accumulated_input ? 0u
                          : static_cast<uint32_t>(std::clamp(camera_settings.nrd_relax_specular_history_frames, 0,
                                                             static_cast<int>(nrd::RELAX_MAX_HISTORY_FRAME_NUM)));
    settings.diffuseMaxFastAccumulatedFrameNum =
        std::min(settings.diffuseMaxFastAccumulatedFrameNum, settings.diffuseMaxAccumulatedFrameNum);
    settings.specularMaxFastAccumulatedFrameNum =
        std::min(settings.specularMaxFastAccumulatedFrameNum, settings.specularMaxAccumulatedFrameNum);
    const auto fast_frames =
        std::min(settings.diffuseMaxFastAccumulatedFrameNum, settings.specularMaxFastAccumulatedFrameNum);
    settings.historyFixFrameNum = std::min(settings.historyFixFrameNum, fast_frames > 0u ? fast_frames - 1u : 0u);
    settings.atrousIterationNum = static_cast<uint32_t>(std::clamp(camera_settings.nrd_relax_atrous_iterations, 2, 8));
    settings.lobeAngleFraction = std::clamp(camera_settings.nrd_relax_lobe_angle_fraction, 0.001f, 1.0f);
    settings.roughnessFraction = std::clamp(camera_settings.nrd_relax_roughness_fraction, 0.001f, 1.0f);
    settings.diffusePhiLuminance = std::max(camera_settings.nrd_relax_diffuse_luminance_sensitivity, 0.001f);
    settings.specularPhiLuminance = std::max(camera_settings.nrd_relax_specular_luminance_sensitivity, 0.001f);
    settings.depthThreshold = std::clamp(camera_settings.nrd_relax_depth_threshold, 0.000001f, 1.0f);
    settings.enableAntiFirefly = camera_settings.nrd_relax_anti_firefly;
    settings.enableRoughnessEdgeStopping = camera_settings.nrd_relax_roughness_edge_stopping;
    if (impl_->integration.SetDenoiserSettings(kDenoiser, &settings) != nrd::Result::SUCCESS)
      return false;
  }
  nrd::CommonSettings common{};
  // Ray images use bottom-left UVs; NRD reconstructs positions with top-left UVs.
  const auto image_orientation = glm::scale(glm::mat4(1.0f), glm::vec3(1.0f, -1.0f, 1.0f));
  const auto projection = image_orientation * camera.projection;
  std::memcpy(common.viewToClipMatrix, &projection[0][0], sizeof(common.viewToClipMatrix));
  std::memcpy(common.worldToViewMatrix, &camera.view[0][0], sizeof(common.worldToViewMatrix));
  const auto previous_projection = image_orientation * glm::inverse(camera.previous_inverse_projection);
  const auto previous_view = glm::inverse(camera.previous_inverse_view);
  std::memcpy(common.viewToClipMatrixPrev, &previous_projection[0][0], sizeof(common.viewToClipMatrixPrev));
  std::memcpy(common.worldToViewMatrixPrev, &previous_view[0][0], sizeof(common.worldToViewMatrixPrev));
  common.resourceSize[0] = common.resourceSizePrev[0] = common.rectSize[0] = common.rectSizePrev[0] =
      static_cast<uint16_t>(impl_->width);
  common.resourceSize[1] = common.resourceSizePrev[1] = common.rectSize[1] = common.rectSizePrev[1] =
      static_cast<uint16_t>(impl_->height);
  common.motionVectorScale[2] = 1.0f;
  common.frameIndex = impl_->frame_index++;
  common.accumulationMode = reset_history       ? nrd::AccumulationMode::CLEAR_AND_RESTART
                            : accumulated_input ? nrd::AccumulationMode::RESTART
                                                : nrd::AccumulationMode::CONTINUE;
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
bool evo_engine::NrdRayCameraDenoiser::Initialize(uint32_t, uint32_t, bool) {
  return false;
}
bool evo_engine::NrdRayCameraDenoiser::UsesReblur() const {
  return false;
}
bool evo_engine::NrdRayCameraDenoiser::Denoise(VkCommandBuffer, const CameraInfoBlock&,
                                               const RayCameraHistoryResources&, bool, const CameraSettings&) {
  return false;
}

#endif
