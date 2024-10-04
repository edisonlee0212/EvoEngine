#include "DynamicStrands.hpp"
#include "Shader.hpp"
using namespace eco_sys_lab_plugin;

void DynamicStrands::Physics(const StepParameters::PhysicsParameters& physics_parameters) const {
  const auto current_frame_index = current_left ? 0 : 1;
  static std::shared_ptr<ComputePipeline> precompute_pipeline{};
  static std::vector<std::shared_ptr<DescriptorSet>> strands_descriptor_sets{};

  struct PrecomputePushConstant {
    uint32_t strand_size = 0;
    uint32_t strand_segment_size = 0;
  };

  if (!precompute_pipeline) {
    const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
    strands_descriptor_sets.resize(max_frame_in_flight);
    for (auto& i : strands_descriptor_sets) {
      i = std::make_shared<DescriptorSet>(strands_layout);
    }
    static std::shared_ptr<Shader> precompute_shader{};
    precompute_shader = std::make_shared<Shader>();
    precompute_shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrandsPrecompute.comp");

    precompute_pipeline = std::make_shared<ComputePipeline>();
    precompute_pipeline->compute_shader = precompute_shader;

    auto per_frame_layout = Platform::GetDescriptorSetLayout("PER_FRAME_LAYOUT");
    precompute_pipeline->descriptor_set_layouts.emplace_back(strands_layout);

    auto& push_constant_range = precompute_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PrecomputePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_ALL;

    precompute_pipeline->Initialize();
  }
  if (ref_strand_segments.empty())
    return;




  BindStrandsDescriptorSet(strands_descriptor_sets[current_frame_index]);
  PrecomputePushConstant push_constant{};
  push_constant.strand_segment_size = ref_strand_segments.size();
  push_constant.strand_size = ref_strands.size();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    precompute_pipeline->Bind(vk_command_buffer);
    precompute_pipeline->BindDescriptorSet(vk_command_buffer, 0, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    precompute_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    /**
     * Dispatch!
     */
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(ref_strand_segments.size(), task_work_group_invocations), 1, 1);
    /**
     * Remember, many of vulkan commands are executed without ordering. So we have this Platform::EverythingBarrier() to
     * make sure that the above commands finishes before moving on. This is syncronization on GPU, not between GPU and
     * CPU.
     */
    Platform::EverythingBarrier(vk_command_buffer);
  });
}
