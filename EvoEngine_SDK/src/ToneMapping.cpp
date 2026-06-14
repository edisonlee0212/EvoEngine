#include "Application.hpp"
#include "PostProcessingStack.hpp"
#include "RenderLayer.hpp"
#include "Shader.hpp"
using namespace evo_engine;

void ToneMapping::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "exposure" << YAML::Value << exposure;
  out << YAML::Key << "gamma" << YAML::Value << gamma;
}

void ToneMapping::Deserialize(const YAML::Node& in) {
  if (in["exposure"])
    exposure = in["exposure"].as<float>();
  if (in["gamma"])
    gamma = in["gamma"].as<float>();
}

void ToneMapping::Process(const PostProcessingStack& post_processing_stack,
                          const std::shared_ptr<Camera>& target_camera) {
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto render_layer = ApplicationContext::Get().GetLayer<RenderLayer>();
  const auto resolution = target_camera->GetSize();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    target_camera->GetRenderTexture()->GetColorImage()->TransitImageLayout(vk_command_buffer, VK_IMAGE_LAYOUT_GENERAL);
    pipeline->Bind(vk_command_buffer);

    pipeline->BindDescriptorSet(vk_command_buffer, 0, render_layer->GetPerFrameDescriptorSet()->GetVkDescriptorSet());
    pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                target_camera->GetRenderTexture()->GetStorageDescriptorSet()->GetVkDescriptorSet());

    PushConstant push_constant;
    push_constant.camera_index =
        render_layer->GetCurrentRenderInstanceStorage()->GetCameraIndex(target_camera->GetHandle());
    push_constant.exposure = exposure;
    push_constant.gamma = gamma;
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
  if (force_rebuild || !pipeline) {
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = Shader::CreateTemporary(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./DefaultResources") / "Shaders/Compute/PostProcessing/ToneMapping.comp");

    pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetPerFrameDescriptorSetLayout());
    pipeline->descriptor_set_layouts.emplace_back(
        ApplicationContext::Get().GetLayer<RenderLayer>()->GetRenderTextureStorageDescriptorSetLayout());
    auto& downsampling_push_constant_range = pipeline->push_constant_ranges.emplace_back();
    downsampling_push_constant_range.size = sizeof(PushConstant);
    downsampling_push_constant_range.offset = 0;
    downsampling_push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;
    pipeline->Initialize();
  }
}
