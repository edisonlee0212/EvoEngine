#include "Application.hpp"
#include "PostProcessingStack.hpp"
#include "RenderLayer.hpp"
#include "Resources.hpp"
#include "Shader.hpp"
#include "Times.hpp"

#include <algorithm>
#include <array>

using namespace evo_engine;

namespace {
constexpr uint32_t kExposureHistogramSize = 256;

std::shared_ptr<Buffer> CreateToneMappingStorageBuffer(const VkDeviceSize size) {
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = size;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;

  VmaAllocationCreateInfo allocation_create_info{};
  allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  return std::make_shared<Buffer>(buffer_create_info, allocation_create_info);
}

int32_t ToneMapMethodToInt(const ToneMapping::ToneMapMethod method) {
  return static_cast<int32_t>(method);
}
}  // namespace

void ToneMapping::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "method" << YAML::Value << ToneMapMethodToInt(method);
  out << YAML::Key << "exposure" << YAML::Value << exposure;
  out << YAML::Key << "brightness" << YAML::Value << brightness;
  out << YAML::Key << "contrast" << YAML::Value << contrast;
  out << YAML::Key << "saturation" << YAML::Value << saturation;
  out << YAML::Key << "vignette" << YAML::Value << vignette;
  out << YAML::Key << "auto_exposure" << YAML::Value << auto_exposure;
  out << YAML::Key << "auto_exposure_speed" << YAML::Value << auto_exposure_speed;
  out << YAML::Key << "ev_min_value" << YAML::Value << ev_min_value;
  out << YAML::Key << "ev_max_value" << YAML::Value << ev_max_value;
  out << YAML::Key << "enable_center_metering" << YAML::Value << enable_center_metering;
  out << YAML::Key << "center_metering_size" << YAML::Value << center_metering_size;
  out << YAML::Key << "average_mode" << YAML::Value << average_mode;
  out << YAML::Key << "dither" << YAML::Value << dither;
}

void ToneMapping::Deserialize(const YAML::Node& in) {
  if (in["method"])
    method = static_cast<ToneMapMethod>(in["method"].as<int32_t>());
  if (in["exposure"])
    exposure = in["exposure"].as<float>();
  if (in["brightness"])
    brightness = in["brightness"].as<float>();
  else if (in["gamma"])
    brightness = in["gamma"].as<float>();
  if (in["contrast"])
    contrast = in["contrast"].as<float>();
  if (in["saturation"])
    saturation = in["saturation"].as<float>();
  if (in["vignette"])
    vignette = in["vignette"].as<float>();
  if (in["auto_exposure"])
    auto_exposure = in["auto_exposure"].as<bool>();
  if (in["auto_exposure_speed"])
    auto_exposure_speed = in["auto_exposure_speed"].as<float>();
  if (in["ev_min_value"])
    ev_min_value = in["ev_min_value"].as<float>();
  if (in["ev_max_value"])
    ev_max_value = in["ev_max_value"].as<float>();
  if (in["enable_center_metering"])
    enable_center_metering = in["enable_center_metering"].as<bool>();
  if (in["center_metering_size"])
    center_metering_size = in["center_metering_size"].as<float>();
  if (in["average_mode"])
    average_mode = in["average_mode"].as<int>();
  if (in["dither"])
    dither = in["dither"].as<bool>();
}

void ToneMapping::Process(const PostProcessingStack& post_processing_stack,
                          const std::shared_ptr<Camera>& target_camera) {
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  const auto resolution = target_camera->GetSize();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);

    PushConstant push_constant;
    push_constant.camera_index =
        render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
    push_constant.method = ToneMapMethodToInt(method);
    push_constant.is_active = 1;
    push_constant.auto_exposure = auto_exposure ? 1 : 0;
    push_constant.enable_center_metering = enable_center_metering ? 1 : 0;
    push_constant.average_mode = average_mode;
    push_constant.dither = dither ? 1 : 0;
    push_constant.exposure = exposure;
    push_constant.brightness = brightness;
    push_constant.contrast = contrast;
    push_constant.saturation = saturation;
    push_constant.vignette = vignette;
    push_constant.ev_min_value = ev_min_value;
    push_constant.ev_max_value = ev_max_value;
    push_constant.center_metering_size = center_metering_size;
    if (auto_exposure) {
      double delta_seconds = auto_exposure_delta_time_override;
      if (delta_seconds < 0.0) {
        const double now = ApplicationContext::Get().GetTimes().Now();
        delta_seconds = auto_exposure_time_initialized_ ? std::max(now - last_auto_exposure_time_, 0.0)
                                                        : ApplicationContext::Get().GetTimes().DeltaTime();
        last_auto_exposure_time_ = now;
        auto_exposure_time_initialized_ = true;
      }
      push_constant.auto_exposure_speed = auto_exposure_speed * static_cast<float>(delta_seconds);

      histogram_pipeline->Bind(vk_command_buffer);
      histogram_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                            render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
      histogram_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, target_camera->GetRenderTexture()->GetStorageDescriptorSet()->GetVkDescriptorSet());
      histogram_pipeline->BindDescriptorSet(vk_command_buffer, 2, auto_exposure_descriptor_set->GetVkDescriptorSet());
      histogram_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      histogram_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(resolution.x, 16),
                                   Platform::DivUp(resolution.y, 16));
      Platform::BufferMemoryBarrier(vk_command_buffer, *histogram_buffer);

      auto_exposure_pipeline->Bind(vk_command_buffer);
      auto_exposure_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
      auto_exposure_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, target_camera->GetRenderTexture()->GetStorageDescriptorSet()->GetVkDescriptorSet());
      auto_exposure_pipeline->BindDescriptorSet(vk_command_buffer, 2,
                                                auto_exposure_descriptor_set->GetVkDescriptorSet());
      auto_exposure_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      auto_exposure_pipeline->Dispatch(vk_command_buffer, 1, 1, 1);
      Platform::BufferMemoryBarrier(vk_command_buffer, *luminance_buffer);
    } else {
      auto_exposure_time_initialized_ = false;
    }

    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(vk_command_buffer, 0, render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                target_camera->GetRenderTexture()->GetStorageDescriptorSet()->GetVkDescriptorSet());
    pipeline->BindDescriptorSet(vk_command_buffer, 2, auto_exposure_descriptor_set->GetVkDescriptorSet());
    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    pipeline->Dispatch(vk_command_buffer, Platform::DivUp(resolution.x * resolution.y, work_group_invocations));
    /**
     * Remember, many of vulkan commands are executed without ordering. So we have this Platform::EverythingBarrier() to
     * make sure that the above commands finishes before moving on. This is syncronization on GPU, not between GPU and
     * CPU.
     */
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void ToneMapping::BuildPipelines(const bool force_rebuild) {
  if (force_rebuild || !auto_exposure_layout) {
    auto_exposure_layout = std::make_shared<DescriptorSetLayout>();
    auto_exposure_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    auto_exposure_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    auto_exposure_layout->Initialize();
  }
  if (force_rebuild || !histogram_buffer) {
    histogram_buffer = CreateToneMappingStorageBuffer(sizeof(uint32_t) * kExposureHistogramSize);
    std::array<uint32_t, kExposureHistogramSize> zero_histogram{};
    histogram_buffer->UploadData(sizeof(zero_histogram), zero_histogram.data());
  }
  if (force_rebuild || !luminance_buffer) {
    luminance_buffer = CreateToneMappingStorageBuffer(sizeof(float));
    const float initial_luminance = 0.18f;
    luminance_buffer->Upload(initial_luminance);
  }
  if (force_rebuild || !auto_exposure_descriptor_set) {
    auto_exposure_descriptor_set = std::make_shared<DescriptorSet>(auto_exposure_layout);
    auto_exposure_descriptor_set->UpdateBufferDescriptorBinding(0, histogram_buffer);
    auto_exposure_descriptor_set->UpdateBufferDescriptorBinding(1, luminance_buffer);
  }
  if (force_rebuild || !histogram_pipeline) {
    histogram_pipeline = std::make_shared<ComputePipeline>();
    histogram_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/ToneMappingHistogram.comp");
    histogram_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    histogram_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTextureStorageDescriptorSetLayout());
    histogram_pipeline->descriptor_set_layouts.emplace_back(auto_exposure_layout);
    auto& push_constant_range = histogram_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    histogram_pipeline->Initialize();
  }
  if (force_rebuild || !auto_exposure_pipeline) {
    auto_exposure_pipeline = std::make_shared<ComputePipeline>();
    auto_exposure_pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/ToneMappingAutoExposure.comp");
    auto_exposure_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    auto_exposure_pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTextureStorageDescriptorSetLayout());
    auto_exposure_pipeline->descriptor_set_layouts.emplace_back(auto_exposure_layout);
    auto& push_constant_range = auto_exposure_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    auto_exposure_pipeline->Initialize();
  }
  if (force_rebuild || !pipeline) {
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        Resources::GetDefaultResourcesPath() / "Shaders/Compute/PostProcessing/ToneMapping.comp");

    pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTextureStorageDescriptorSetLayout());
    pipeline->descriptor_set_layouts.emplace_back(auto_exposure_layout);
    auto& downsampling_push_constant_range = pipeline->push_constant_ranges.emplace_back();
    downsampling_push_constant_range.size = sizeof(PushConstant);
    downsampling_push_constant_range.offset = 0;
    downsampling_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    pipeline->Initialize();
  }
}
