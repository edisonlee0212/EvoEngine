#include "DynamicStrandsPhysics.hpp"
#include "Shader.hpp"
#include "VoxelGrid.hpp"
using namespace eco_sys_lab_plugin;

void DynamicStrands::Physics(const PhysicsParameters& physics_parameters, const std::function<void()>& operators_action,
                             const std::function<void()>& pre_constraint_action) const {
  if (pre_step)
    pre_step->Execute(physics_parameters, *this);
  operators_action();
  for (int sub_step_index = 0; sub_step_index < physics_parameters.sub_step; sub_step_index++) {
    if (prediction)
      prediction->Execute(physics_parameters, *this);
    pre_constraint_action();
    for (const auto& c : constraints) {
      if (c->enabled)
        for (int iteration_i = 0; iteration_i < physics_parameters.constraint_iteration; iteration_i++) {
          c->Project(physics_parameters, *this);
        }
    }
  }
}

DynamicStrandsPreStep::DynamicStrandsPreStep() {
  if (!particle_pre_step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/ParticlePreStep.comp");

    particle_pre_step_pipeline = std::make_shared<ComputePipeline>();
    particle_pre_step_pipeline->compute_shader = shader;
    particle_pre_step_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = particle_pre_step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(ParticlePreStepPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    particle_pre_step_pipeline->Initialize();
  }
  if (!segment_pre_step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/SegmentPreStep.comp");
    segment_pre_step_pipeline = std::make_shared<ComputePipeline>();
    segment_pre_step_pipeline->compute_shader = shader;
    segment_pre_step_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_pre_step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPreStepPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_pre_step_pipeline->Initialize();
  }

  if (!connection_pre_step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/ConnectionPreStep.comp");
    connection_pre_step_pipeline = std::make_shared<ComputePipeline>();
    connection_pre_step_pipeline->compute_shader = shader;
    connection_pre_step_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = connection_pre_step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(ConnectionPreStepPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    connection_pre_step_pipeline->Initialize();
  }
}

void DynamicStrandsPreStep::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                                    const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  ParticlePreStepPushConstant particle_push_constant;
  particle_push_constant.particle_size = target_dynamic_strands.particles.size();
  particle_push_constant.time_step = physics_parameters.time_step;
  particle_push_constant.inv_time_step = 1.f / particle_push_constant.time_step;
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  SegmentPreStepPushConstant segment_push_constant;
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.time_step = physics_parameters.time_step;
  segment_push_constant.inv_time_step = 1.f / segment_push_constant.time_step;

  ConnectionPreStepPushConstant connection_push_constant;
  connection_push_constant.connection_size = target_dynamic_strands.connections.size();
  connection_push_constant.time_step = physics_parameters.time_step;
  connection_push_constant.inv_time_step = 1.f / connection_push_constant.time_step;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    particle_pre_step_pipeline->Bind(vk_command_buffer);
    particle_pre_step_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    particle_pre_step_pipeline->PushConstant(vk_command_buffer, 0, particle_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(particle_push_constant.particle_size, task_work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    segment_pre_step_pipeline->Bind(vk_command_buffer);
    segment_pre_step_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_pre_step_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, task_work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    connection_pre_step_pipeline->Bind(vk_command_buffer);
    connection_pre_step_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    connection_pre_step_pipeline->PushConstant(vk_command_buffer, 0, connection_push_constant);
    vkCmdDispatch(vk_command_buffer,
                  Platform::DivUp(connection_push_constant.connection_size, task_work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

DynamicStrandsPrediction::DynamicStrandsPrediction() {
  if (!particle_prediction_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/ParticlePrediction.comp");

    particle_prediction_pipeline = std::make_shared<ComputePipeline>();
    particle_prediction_pipeline->compute_shader = shader;
    particle_prediction_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = particle_prediction_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(ParticlePredictionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    particle_prediction_pipeline->Initialize();
  }
  if (!segment_prediction_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/SegmentPrediction.comp");
    segment_prediction_pipeline = std::make_shared<ComputePipeline>();
    segment_prediction_pipeline->compute_shader = shader;
    segment_prediction_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_prediction_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPredictionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_prediction_pipeline->Initialize();
  }
  if (!connection_prediction_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/ConnectionPrediction.comp");
    connection_prediction_pipeline = std::make_shared<ComputePipeline>();
    connection_prediction_pipeline->compute_shader = shader;
    connection_prediction_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = connection_prediction_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(ConnectionPredictionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    connection_prediction_pipeline->Initialize();
  }
}

void DynamicStrandsPrediction::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                                       const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  ParticlePredictionPushConstant particle_push_constant;
  particle_push_constant.particle_size = target_dynamic_strands.particles.size();
  particle_push_constant.time_step = physics_parameters.time_step / physics_parameters.sub_step;
  particle_push_constant.inv_time_step = 1.f / particle_push_constant.time_step;
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  SegmentPredictionPushConstant segment_push_constant;
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.time_step = physics_parameters.time_step;
  segment_push_constant.inv_time_step = 1.f / segment_push_constant.time_step;

  ConnectionPredictionPushConstant connection_push_constant;
  connection_push_constant.connection_size = target_dynamic_strands.connections.size();
  connection_push_constant.allow_breaking = physics_parameters.allow_breaking ? 1 : 0;
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    particle_prediction_pipeline->Bind(vk_command_buffer);
    particle_prediction_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    particle_prediction_pipeline->PushConstant(vk_command_buffer, 0, particle_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(particle_push_constant.particle_size, task_work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    segment_prediction_pipeline->Bind(vk_command_buffer);
    segment_prediction_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_prediction_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, task_work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    connection_prediction_pipeline->Bind(vk_command_buffer);
    connection_prediction_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    connection_prediction_pipeline->PushConstant(vk_command_buffer, 0, connection_push_constant);
    vkCmdDispatch(vk_command_buffer,
                  Platform::DivUp(connection_push_constant.connection_size, task_work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void DsGroundPlane::Project(const DynamicStrands::PhysicsParameters& physics_parameters,
                            const DynamicStrands& target_dynamic_strands) {
  GroundPlanePushConstant push_constant;
  push_constant.ground_height = ground_height;
  push_constant.particle_size = target_dynamic_strands.particles.size();
  push_constant.ground_softness = ground_softness;
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.particle_size, task_work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

bool DsGroundPlane::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("Ground Plane")) {
    if (ImGui::Checkbox("Enable", &enabled))
      changed = true;
    if (ImGui::DragFloat("Ground height", &ground_height, 0.01f, -100.0f, 100.0f))
      changed = true;
    if (ImGui::DragFloat("Ground softness", &ground_softness, 0.01f, 0.f, 1.f))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

DsGroundPlane::DsGroundPlane() {
  if (!pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/GroundPlane.comp");
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroundPlanePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    pipeline->Initialize();
  }
}

DsStiffRod::DsStiffRod() {
  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
  }
  VkBufferCreateInfo storage_buffer_create_info{};
  storage_buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  storage_buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  storage_buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  storage_buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
  per_strand_data_list_buffer = std::make_shared<Buffer>(storage_buffer_create_info, buffer_vma_allocation_create_info);
#ifdef USE_XPBD
  std::string xpbd_prefix = "\n#define USE_XPBD\n";
#else
  std::string xpbd_prefix = "";
#endif
  if (!bilateral_stretch_shear_constraint_pipeline) {
    static std::shared_ptr<Shader> stretch_shear_shader{};
    stretch_shear_shader = std::make_shared<Shader>();
    stretch_shear_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines + xpbd_prefix,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Compute/DynamicStrands/Constraints/StiffRodStretchShearBilateral.comp");
    bilateral_stretch_shear_constraint_pipeline = std::make_shared<ComputePipeline>();
    bilateral_stretch_shear_constraint_pipeline->compute_shader = stretch_shear_shader;

    bilateral_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    bilateral_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range =
        bilateral_stretch_shear_constraint_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(StretchShearConstraintConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    bilateral_stretch_shear_constraint_pipeline->Initialize();

    static std::shared_ptr<Shader> bend_twist_constraint_shader{};
    bend_twist_constraint_shader = std::make_shared<Shader>();
    bend_twist_constraint_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines + xpbd_prefix,
                                      std::filesystem::path("./EcoSysLabResources") /
                                          "Shaders/Compute/DynamicStrands/Constraints/StiffRodBendTwistBilateral.comp");

    bilateral_bend_twist_constraint_pipeline = std::make_shared<ComputePipeline>();
    bilateral_bend_twist_constraint_pipeline->compute_shader = bend_twist_constraint_shader;

    bilateral_bend_twist_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    bilateral_bend_twist_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& bend_twist_push_constant_range =
        bilateral_bend_twist_constraint_pipeline->push_constant_ranges.emplace_back();
    bend_twist_push_constant_range.size = sizeof(BendTwistConstraintConstant);
    bend_twist_push_constant_range.offset = 0;
    bend_twist_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    bilateral_bend_twist_constraint_pipeline->Initialize();
  }
  if (!forward_stretch_shear_constraint_pipeline) {
    static std::shared_ptr<Shader> stretch_shear_shader{};
    stretch_shear_shader = std::make_shared<Shader>();

    stretch_shear_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines + xpbd_prefix,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Compute/DynamicStrands/Constraints/StiffRodStretchShearForward.comp");
    forward_stretch_shear_constraint_pipeline = std::make_shared<ComputePipeline>();
    forward_stretch_shear_constraint_pipeline->compute_shader = stretch_shear_shader;

    forward_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    forward_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range =
        forward_stretch_shear_constraint_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(StretchShearConstraintConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    forward_stretch_shear_constraint_pipeline->Initialize();

    static std::shared_ptr<Shader> bend_twist_constraint_shader{};
    bend_twist_constraint_shader = std::make_shared<Shader>();
    bend_twist_constraint_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines + xpbd_prefix,
                                      std::filesystem::path("./EcoSysLabResources") /
                                          "Shaders/Compute/DynamicStrands/Constraints/StiffRodBendTwistForward.comp");

    forward_bend_twist_constraint_pipeline = std::make_shared<ComputePipeline>();
    forward_bend_twist_constraint_pipeline->compute_shader = bend_twist_constraint_shader;

    forward_bend_twist_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    forward_bend_twist_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& bend_twist_push_constant_range = forward_bend_twist_constraint_pipeline->push_constant_ranges.emplace_back();
    bend_twist_push_constant_range.size = sizeof(BendTwistConstraintConstant);
    bend_twist_push_constant_range.offset = 0;
    bend_twist_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    forward_bend_twist_constraint_pipeline->Initialize();
  }
  if (!backward_stretch_shear_constraint_pipeline) {
    static std::shared_ptr<Shader> stretch_shear_shader{};
    stretch_shear_shader = std::make_shared<Shader>();
    stretch_shear_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines + xpbd_prefix,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Compute/DynamicStrands/Constraints/StiffRodStretchShearBackward.comp");
    backward_stretch_shear_constraint_pipeline = std::make_shared<ComputePipeline>();
    backward_stretch_shear_constraint_pipeline->compute_shader = stretch_shear_shader;

    backward_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    backward_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range =
        backward_stretch_shear_constraint_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(StretchShearConstraintConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    backward_stretch_shear_constraint_pipeline->Initialize();

    static std::shared_ptr<Shader> bend_twist_constraint_shader{};
    bend_twist_constraint_shader = std::make_shared<Shader>();
    bend_twist_constraint_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines + xpbd_prefix,
                                      std::filesystem::path("./EcoSysLabResources") /
                                          "Shaders/Compute/DynamicStrands/Constraints/StiffRodBendTwistBackward.comp");

    backward_bend_twist_constraint_pipeline = std::make_shared<ComputePipeline>();
    backward_bend_twist_constraint_pipeline->compute_shader = bend_twist_constraint_shader;

    backward_bend_twist_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    backward_bend_twist_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& bend_twist_push_constant_range = backward_bend_twist_constraint_pipeline->push_constant_ranges.emplace_back();
    bend_twist_push_constant_range.size = sizeof(BendTwistConstraintConstant);
    bend_twist_push_constant_range.offset = 0;
    bend_twist_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    backward_bend_twist_constraint_pipeline->Initialize();
  }
  if (strands_physics_descriptor_sets.empty()) {
    strands_physics_descriptor_sets.resize(max_frame_in_flight);
    for (auto& i : strands_physics_descriptor_sets) {
      i = std::make_shared<DescriptorSet>(layout);
    }
  }
}

bool DsStiffRod::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("StiffRod")) {
    if (ImGui::Checkbox("Enable", &enabled))
      changed = true;
    if (enabled) {
      if (ImGui::Checkbox("Bend/twist", &bend_twist))
        changed = true;
      if (ImGui::Checkbox("Stretch/shear", &stretch_shear))
        changed = true;
      if (ImGui::DragInt("Sub iteration", &sub_iteration, 1, 1, 100))
        changed = true;
      if (ImGui::Combo("Project Mode", {"Forward", "Backward", "Bilateral"}, project_mode))
        changed = true;
    }
    ImGui::TreePop();
  }
  return changed;
}

void DsStiffRod::InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                                const StrandModelSkeleton& strand_model_skeleton,
                                const DtsStrandGroup& subdivided_strand_group,
                                const DynamicStrands& target_dynamic_strands) {
  per_strand_data_list.resize(target_dynamic_strands.strands.size());
  for (uint32_t strand_index = 0; strand_index < target_dynamic_strands.strands.size(); strand_index++) {
    const auto& gpu_strand = target_dynamic_strands.strands[strand_index];
    auto& stiff_rod_per_strand_data = per_strand_data_list[strand_index];
    if (gpu_strand.begin_segment_handle == -1) {
      stiff_rod_per_strand_data.front_propagate_begin_segment_handle = -1;
      stiff_rod_per_strand_data.back_propagate_begin_segment_handle = -1;
      stiff_rod_per_strand_data.front_propagate_begin_connection_handle = -1;
      stiff_rod_per_strand_data.back_propagate_begin_connection_handle = -1;
      continue;
    }
    stiff_rod_per_strand_data.front_propagate_begin_segment_handle = gpu_strand.begin_segment_handle;
    if (gpu_strand.begin_segment_handle == gpu_strand.end_segment_handle) {
      stiff_rod_per_strand_data.back_propagate_begin_segment_handle = -1;
      stiff_rod_per_strand_data.front_propagate_begin_connection_handle = -1;
      stiff_rod_per_strand_data.back_propagate_begin_connection_handle = -1;
      continue;
    }
    stiff_rod_per_strand_data.front_propagate_begin_connection_handle = gpu_strand.begin_connection_handle;
    int connection_size = 0;
    int connection_handle = stiff_rod_per_strand_data.front_propagate_begin_connection_handle;
    while (connection_handle != -1) {
      connection_size++;
      connection_handle = target_dynamic_strands.connections[connection_handle].next_handle;
    }

    stiff_rod_per_strand_data.back_propagate_begin_connection_handle =
        connection_size % 2 == 0 ? gpu_strand.end_connection_handle
                                 : target_dynamic_strands.connections[gpu_strand.end_connection_handle].prev_handle;

    stiff_rod_per_strand_data.back_propagate_begin_segment_handle =
        connection_size % 2 == 1 ? gpu_strand.end_segment_handle
                                 : target_dynamic_strands.segments[gpu_strand.end_segment_handle].prev_handle;
  }
}

void DsStiffRod::Project(const DynamicStrands::PhysicsParameters& physics_parameters,
                         const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  StretchShearConstraintConstant stretch_shear_constraint_constant;
  stretch_shear_constraint_constant.strand_size = per_strand_data_list.size();
  stretch_shear_constraint_constant.inv_time_step =
      1.f / (physics_parameters.time_step / physics_parameters.sub_step);
  BendTwistConstraintConstant bend_twist_constraint_constant;
  bend_twist_constraint_constant.strand_size = per_strand_data_list.size();
  bend_twist_constraint_constant.inv_time_step =
      1.f / (physics_parameters.time_step / physics_parameters.sub_step);
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (int sub_iteration_index = 0; sub_iteration_index < sub_iteration; sub_iteration_index++) {
      switch (static_cast<ProjectMode>(project_mode)) {
        case ProjectMode::Bilateral:
          if (stretch_shear) {
            bilateral_stretch_shear_constraint_pipeline->Bind(vk_command_buffer);
            bilateral_stretch_shear_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 0,
                target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            bilateral_stretch_shear_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

            bilateral_stretch_shear_constraint_pipeline->PushConstant(vk_command_buffer, 0,
                                                                      stretch_shear_constraint_constant);
            vkCmdDispatch(vk_command_buffer,
                          Platform::DivUp(stretch_shear_constraint_constant.strand_size, task_work_group_invocations),
                          1, 1);
            Platform::EverythingBarrier(vk_command_buffer);
          }
          if (bend_twist) {
            bilateral_bend_twist_constraint_pipeline->Bind(vk_command_buffer);
            bilateral_bend_twist_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 0,
                target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            bilateral_bend_twist_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

            bilateral_bend_twist_constraint_pipeline->PushConstant(vk_command_buffer, 0,
                                                                   bend_twist_constraint_constant);
            vkCmdDispatch(vk_command_buffer,
                          Platform::DivUp(bend_twist_constraint_constant.strand_size, task_work_group_invocations), 1,
                          1);
            Platform::EverythingBarrier(vk_command_buffer);
          }
          break;
        case ProjectMode::Forward:
          if (stretch_shear) {
            forward_stretch_shear_constraint_pipeline->Bind(vk_command_buffer);
            forward_stretch_shear_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 0,
                target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            forward_stretch_shear_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

            forward_stretch_shear_constraint_pipeline->PushConstant(vk_command_buffer, 0,
                                                                    stretch_shear_constraint_constant);
            vkCmdDispatch(vk_command_buffer,
                          Platform::DivUp(stretch_shear_constraint_constant.strand_size, task_work_group_invocations),
                          1, 1);
            Platform::EverythingBarrier(vk_command_buffer);
          }
          if (bend_twist) {
            forward_bend_twist_constraint_pipeline->Bind(vk_command_buffer);
            forward_bend_twist_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 0,
                target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            forward_bend_twist_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

            forward_bend_twist_constraint_pipeline->PushConstant(vk_command_buffer, 0, bend_twist_constraint_constant);
            vkCmdDispatch(vk_command_buffer,
                          Platform::DivUp(bend_twist_constraint_constant.strand_size, task_work_group_invocations), 1,
                          1);
            Platform::EverythingBarrier(vk_command_buffer);
          }
          break;
        case ProjectMode::Backward:
          if (stretch_shear) {
            backward_stretch_shear_constraint_pipeline->Bind(vk_command_buffer);
            backward_stretch_shear_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 0,
                target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            backward_stretch_shear_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

            backward_stretch_shear_constraint_pipeline->PushConstant(vk_command_buffer, 0,
                                                                     stretch_shear_constraint_constant);
            vkCmdDispatch(vk_command_buffer,
                          Platform::DivUp(stretch_shear_constraint_constant.strand_size, task_work_group_invocations),
                          1, 1);
            Platform::EverythingBarrier(vk_command_buffer);
          }
          if (bend_twist) {
            backward_bend_twist_constraint_pipeline->Bind(vk_command_buffer);
            backward_bend_twist_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 0,
                target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
            backward_bend_twist_constraint_pipeline->BindDescriptorSet(
                vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

            backward_bend_twist_constraint_pipeline->PushConstant(vk_command_buffer, 0, bend_twist_constraint_constant);
            vkCmdDispatch(vk_command_buffer,
                          Platform::DivUp(bend_twist_constraint_constant.strand_size, task_work_group_invocations), 1,
                          1);
            Platform::EverythingBarrier(vk_command_buffer);
          }
          break;
      }
    }
  });
}

void DsStiffRod::DownloadData() {
  per_strand_data_list_buffer->DownloadVector(per_strand_data_list, per_strand_data_list.size());
}

void DsStiffRod::UploadData() {
  per_strand_data_list_buffer->UploadVector(per_strand_data_list);
}

void DsStiffRod::UpdateBindings() {
  strands_physics_descriptor_sets[Platform::GetCurrentFrameIndex()]->UpdateBufferDescriptorBinding(
      0, per_strand_data_list_buffer);
}

glm::vec3 DsStiffRod::ComputeDarbouxVector(const glm::quat& q0, const glm::quat& q1,
                                           const float average_segment_length) {
  const auto relative_rotation = glm::conjugate(q0) * q1;
  return 2.f / average_segment_length * glm::vec3(relative_rotation.x, relative_rotation.y, relative_rotation.z);
}

DsRandomBundle::DsRandomBundle() {
  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
  }

  VkBufferCreateInfo storage_buffer_create_info{};
  storage_buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  storage_buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  storage_buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  storage_buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  pairs_buffer = std::make_shared<Buffer>(storage_buffer_create_info, buffer_vma_allocation_create_info);
  segment_data_list_buffer = std::make_shared<Buffer>(storage_buffer_create_info, buffer_vma_allocation_create_info);
  if (!bundle_update_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/RandomBundleUpdate.comp");
    bundle_update_pipeline = std::make_shared<ComputePipeline>();
    bundle_update_pipeline->compute_shader = shader;
    bundle_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    bundle_update_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range = bundle_update_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(RandomBundleUpdateConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bundle_update_pipeline->Initialize();
  }

  if (!bundle_offset_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/RandomBundleOffset.comp");
    bundle_offset_pipeline = std::make_shared<ComputePipeline>();
    bundle_offset_pipeline->compute_shader = shader;
    bundle_offset_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    bundle_offset_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range = bundle_offset_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(RandomBundleConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bundle_offset_pipeline->Initialize();
  }

  if (!bundle_apply_segments_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/RandomBundleApplySegments.comp");

    bundle_apply_segments_pipeline = std::make_shared<ComputePipeline>();
    bundle_apply_segments_pipeline->compute_shader = shader;

    bundle_apply_segments_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    bundle_apply_segments_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range = bundle_apply_segments_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(RandomBundleApplySegmentsConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    bundle_apply_segments_pipeline->Initialize();
  }

  if (!bundle_apply_connections_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/RandomBundleApplyConnections.comp");

    bundle_apply_connections_pipeline = std::make_shared<ComputePipeline>();
    bundle_apply_connections_pipeline->compute_shader = shader;
    bundle_apply_connections_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    auto& stretch_shear_push_constant_range = bundle_apply_connections_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(RandomBundleApplyConnectionsConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bundle_apply_connections_pipeline->Initialize();
  }

  if (bundle_descriptor_sets.empty()) {
    bundle_descriptor_sets.resize(max_frame_in_flight);
    for (auto& i : bundle_descriptor_sets) {
      i = std::make_shared<DescriptorSet>(layout);
    }
  }
}

void DsRandomBundle::Project(const DynamicStrands::PhysicsParameters& physics_parameters,
                             const DynamicStrands& target_dynamic_strands) {
  if (segment_pairs.empty())
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  RandomBundleUpdateConstant update_constant;
  update_constant.pair_size = static_cast<uint32_t>(segment_pairs.size());
  update_constant.allow_breaking = physics_parameters.allow_breaking ? 1 : 0;
  RandomBundleConstant constraint_constant;
  constraint_constant.segment_size = static_cast<uint32_t>(segment_data_list.size());
  constraint_constant.inv_time_step =
      1.f / (physics_parameters.time_step / physics_parameters.sub_step);

  RandomBundleApplySegmentsConstant constraint_apply_segments_constant;
  constraint_apply_segments_constant.segment_size = static_cast<uint32_t>(segment_data_list.size());
  constraint_apply_segments_constant.inv_time_step =
      1.f / (physics_parameters.time_step / physics_parameters.sub_step);

  RandomBundleApplyConnectionsConstant constraint_apply_connections_constant;
  constraint_apply_connections_constant.connection_size = static_cast<uint32_t>(target_dynamic_strands.connections.size());
  constraint_apply_connections_constant.inv_time_step =
      1.f / (physics_parameters.time_step / physics_parameters.sub_step);

  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (int sub_iteration_index = 0; sub_iteration_index < sub_iteration; sub_iteration_index++) {
      bundle_update_pipeline->Bind(vk_command_buffer);
      bundle_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      bundle_update_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                bundle_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      bundle_update_pipeline->PushConstant(vk_command_buffer, 0, update_constant);
      vkCmdDispatch(vk_command_buffer, Platform::DivUp(update_constant.pair_size, task_work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);

      bundle_offset_pipeline->Bind(vk_command_buffer);
      bundle_offset_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      bundle_offset_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                bundle_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      bundle_offset_pipeline->PushConstant(vk_command_buffer, 0, constraint_constant);
      vkCmdDispatch(vk_command_buffer, Platform::DivUp(constraint_constant.segment_size, task_work_group_invocations),
                    1, 1);
      Platform::EverythingBarrier(vk_command_buffer);

      bundle_apply_segments_pipeline->Bind(vk_command_buffer);
      bundle_apply_segments_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      bundle_apply_segments_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                               bundle_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      bundle_apply_segments_pipeline->PushConstant(vk_command_buffer, 0, constraint_apply_segments_constant);
      vkCmdDispatch(vk_command_buffer,
                    Platform::DivUp(constraint_apply_segments_constant.segment_size, task_work_group_invocations),
                    1, 1);
      Platform::EverythingBarrier(vk_command_buffer);

      bundle_apply_connections_pipeline->Bind(vk_command_buffer);
      bundle_apply_connections_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      bundle_apply_connections_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, bundle_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      bundle_apply_connections_pipeline->PushConstant(vk_command_buffer, 0, constraint_apply_connections_constant);
      vkCmdDispatch(vk_command_buffer,
                    Platform::DivUp(constraint_apply_connections_constant.connection_size, task_work_group_invocations),
                    1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    }
  });
}

void DsRandomBundle::InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                                    const StrandModelSkeleton& strand_model_skeleton,
                                    const DtsStrandGroup& subdivided_strand_group,
                                    const DynamicStrands& target_dynamic_strands) {
  segment_data_list.resize(target_dynamic_strands.segments.size());

  std::vector<glm::vec3> max_bounds(Jobs::GetWorkerSize());
  std::vector<glm::vec3> min_bounds(Jobs::GetWorkerSize());
  for (auto& i : max_bounds)
    i = glm::vec3(-FLT_MAX);
  for (auto& i : min_bounds)
    i = glm::vec3(FLT_MAX);
  Jobs::RunParallelFor(segment_data_list.size(), [&](const auto i, const auto worker_i) {
    const auto& segment = target_dynamic_strands.segments[i];
    const auto& particle0 = target_dynamic_strands.particles[segment.particle0_handle];
    const auto& particle1 = target_dynamic_strands.particles[segment.particle1_handle];

    max_bounds[worker_i] = glm::max(max_bounds[worker_i], particle0.x0);
    min_bounds[worker_i] = glm::min(min_bounds[worker_i], particle0.x0);
    max_bounds[worker_i] = glm::max(max_bounds[worker_i], particle1.x0);
    min_bounds[worker_i] = glm::min(min_bounds[worker_i], particle1.x0);

    for (int j = 0; j < BUNDLE_MAX_CONNECTION; j++) {
      segment_data_list[i].pair_handles[j] = -1;
    }
  });
  auto max_bound = glm::vec3(-FLT_MAX);
  auto min_bound = glm::vec3(FLT_MAX);
  for (auto& i : max_bounds)
    max_bound = glm::max(i, max_bound);
  for (auto& i : min_bounds)
    min_bound = glm::min(i, min_bound);
  struct SegmentInfo {
    glm::vec3 p0;
    glm::vec3 p1;
    glm::vec3 center_position;
    int node_handle;
    int strand_handle;
    int segment_handle;
  };
  VoxelGrid<std::vector<SegmentInfo>> voxel_grid;
  constexpr auto grid_size = 0.1f;
  voxel_grid.Initialize(grid_size, min_bound - glm::vec3(grid_size) * 2.f, max_bound + glm::vec3(grid_size) * 2.f, {});
  for (int segment_index = 0; segment_index < target_dynamic_strands.segments.size(); segment_index++) {
    const auto& segment = target_dynamic_strands.segments[segment_index];
    const auto& particle0 = target_dynamic_strands.particles[segment.particle0_handle];
    const auto& particle1 = target_dynamic_strands.particles[segment.particle1_handle];
    SegmentInfo s_d;
    s_d.p0 = particle0.x0;
    s_d.p1 = particle1.x0;
    s_d.center_position = (particle0.x0 + particle1.x0) * .5f;
    s_d.node_handle = particle0.node_handle;
    s_d.strand_handle = segment.strand_handle;
    s_d.segment_handle = segment_index;
    voxel_grid.Ref(s_d.center_position).emplace_back(s_d);
  }
  std::multimap<float, std::set<std::pair<int, int>>> candidates;

  for (const auto& connection : target_dynamic_strands.connections) {
    const auto& segment0_handle = connection.segment0_handle;
    const auto& segment1_handle = connection.segment1_handle;
    const auto pair =
        std::make_pair(glm::min(segment0_handle, segment1_handle), glm::max(segment0_handle, segment1_handle));
    if (const auto search = candidates.find(0.0f); search != candidates.end()) {
      search->second.emplace(pair);
    } else {
      candidates.insert({0.0f, {pair}});
    }
  }

  for (int segment_handle = 0; segment_handle < target_dynamic_strands.segments.size(); segment_handle++) {
    const auto& segment = target_dynamic_strands.segments[segment_handle];
    const auto& particle0 = target_dynamic_strands.particles[segment.particle0_handle];
    const auto& particle1 = target_dynamic_strands.particles[segment.particle1_handle];
    const auto segment_center_position = (particle0.x0 + particle1.x0) * .5f;
    voxel_grid.ForEach(segment_center_position, segment.rest_length * 3.0f, [&](const std::vector<SegmentInfo>& list) {
      for (const auto& info : list) {
        if (info.segment_handle == segment_handle)
          continue;
        if (info.strand_handle == segment.strand_handle)
          continue;
        //  Function to check if a point is inside a cylinder
        const auto cylinder_check = [](const glm::vec3& p0, const glm::vec3& p1, const float radius,
                                       const glm::vec3& point, bool& check) {
          // Calculate the direction vector of the cylinder's axis
          const glm::vec3 d_v = p1 - p0;
          const float height = glm::length(d_v);
          const glm::vec3 direction = glm::normalize(d_v);

          // Vector from p0 to point
          const glm::vec3 p0_p = point - p0;

          // Projection scalar
          const float t = glm::dot(p0_p, direction);

          // Check if projection is within the cylinder's height
          if (t < 0.0f || t > height) {
            check = false;
            return 0.0f;  // Outside the cylinder height
          }

          // Closest point on the cylinder's axis
          const glm::vec3 closest_point = p0 + t * direction;

          // Distance from point to the axis
          const float distance = glm::length(point - closest_point);
          check = distance <= radius;
          // Check if the distance is within the radius
          return distance;
        };
        bool check1, check2;
        const auto distance1 = cylinder_check(particle0.x0, particle1.x0,
                                              segment.radius * initialize_parameters.neighbor_range, info.p0, check1);
        const auto distance2 = cylinder_check(particle0.x0, particle1.x0,
                                              segment.radius * initialize_parameters.neighbor_range, info.p1, check2);
        if (!check1 && !check2)
          continue;

        bool node_check = false;
        if (info.node_handle == particle0.node_handle)
          node_check = true;
        if (!node_check) {
          if (auto& node = strand_model_skeleton.PeekNode(particle0.node_handle);
              info.node_handle == node.GetParentHandle()) {
            node_check = true;
          } else {
            for (const auto& child_handle : node.PeekChildHandles()) {
              if (info.node_handle == child_handle) {
                node_check = true;
                break;
              }
            }
          }
        }
        if (!node_check)
          continue;
        const auto distance = glm::min(distance1, distance2);
        const auto pair = segment_handle <= info.segment_handle ? std::make_pair(segment_handle, info.segment_handle)
                                                                : std::make_pair(info.segment_handle, segment_handle);
        if (const auto search = candidates.find(distance); search != candidates.end()) {
          search->second.emplace(pair);
        } else {
          candidates.insert({distance, {pair}});
        }
      }
    });
  }
  segment_pairs.clear();
  std::vector<uint32_t> counters(target_dynamic_strands.segments.size(), 0);

  for (const auto& candidate_set : candidates) {
    for (const auto& pair : candidate_set.second) {
      auto& first = counters[pair.first];
      auto& second = counters[pair.second];
      if (first >= BUNDLE_MAX_CONNECTION || second >= BUNDLE_MAX_CONNECTION)
        continue;
      const auto pair_handle = static_cast<int>(segment_pairs.size());
      segment_pairs.emplace_back();
      auto& new_pair = segment_pairs.back();
      new_pair.segment0_handle = pair.first;
      new_pair.segment1_handle = pair.second;

      const auto& segment0 = target_dynamic_strands.segments[pair.first];
      const auto& segment1 = target_dynamic_strands.segments[pair.second];
      const float segment_length = 1.f;//(segment0.rest_length + segment1.rest_length) * .5f;
      const float segment_radius = 1.f;//((segment0.radius + segment1.radius) * .5f;

      const float area = glm::pi<float>() * segment_radius * segment_radius;
      const float youngs_modulus = initialize_parameters.youngs_modulus.GetValue() * 1e9f;
      const float shear_modulus = initialize_parameters.shear_modulus.GetValue() * 1e9f;


      const float torsion_modulus = initialize_parameters.torsion_modulus.GetValue() * 1e9f;
      const float bending_modulus = initialize_parameters.bending_modulus.GetValue() * 1e9f;

      const float stretching_alpha = 1.f / (youngs_modulus * area / segment_length);
      const float shearing_alpha = 1.f / (shear_modulus * area / segment_length);

      const auto second_moment_of_area = glm::pi<float>() * std::pow(segment_radius, 4.f) * 0.25f;
      const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(segment_radius, 4.f) * 0.5f;

      const float bending_alpha = 1.f / (bending_modulus * second_moment_of_area / glm::pow(segment_length, 3.f));
      const float torsion_alpha = 1.f / (torsion_modulus * polar_moment_of_inertia / segment_length);

      new_pair.alphas = glm::vec4(bending_alpha, torsion_alpha, shearing_alpha, stretching_alpha);

      new_pair.max_strain =
          glm::max(initialize_parameters.min_neighbor_strain, initialize_parameters.max_neighbor_strain.GetValue());
      new_pair.valid = 1;
      segment_data_list[pair.first].pair_handles[first] = pair_handle;
      segment_data_list[pair.second].pair_handles[second] = pair_handle;
      first++;
      second++;
    }
  }


  Jobs::RunParallelFor(segment_pairs.size(), [&](const auto pair_index) {
    auto& pair = segment_pairs[pair_index];
    auto& segment0 = target_dynamic_strands.segments[pair.segment0_handle];
    auto& segment1 = target_dynamic_strands.segments[pair.segment1_handle];
    auto& segment0_particle0 = target_dynamic_strands.particles[segment0.particle0_handle];
    auto& segment0_particle1 = target_dynamic_strands.particles[segment0.particle1_handle];

    auto& segment1_particle0 = target_dynamic_strands.particles[segment1.particle0_handle];
    auto& segment1_particle1 = target_dynamic_strands.particles[segment1.particle1_handle];

    const auto segment0_center_position = (segment0_particle0.x0 + segment0_particle1.x0) * .5f;
    const auto segment1_center_position = (segment1_particle0.x0 + segment1_particle1.x0) * .5f;

    pair.segment0_particle0_offset =
        glm::vec4(glm::inverse(segment1.q0) * (segment0_particle0.x0 - segment1_center_position), 0.0f);
    pair.segment0_particle1_offset =
        glm::vec4(glm::inverse(segment1.q0) * (segment0_particle1.x0 - segment1_center_position), 0.0f);

    pair.segment1_particle0_offset =
        glm::vec4(glm::inverse(segment0.q0) * (segment1_particle0.x0 - segment0_center_position), 0.0f);
    pair.segment1_particle1_offset =
        glm::vec4(glm::inverse(segment0.q0) * (segment1_particle1.x0 - segment0_center_position), 0.0f);

    pair.rest_darboux_vector = glm::conjugate(segment0.q0) * segment1.q0;
  });
}

void DsRandomBundle::UploadData() {
  pairs_buffer->UploadVector(segment_pairs);
  segment_data_list_buffer->UploadVector(segment_data_list);
}

void DsRandomBundle::UpdateBindings() {
  bundle_descriptor_sets[Platform::GetCurrentFrameIndex()]->UpdateBufferDescriptorBinding(0, pairs_buffer, 0);
  bundle_descriptor_sets[Platform::GetCurrentFrameIndex()]->UpdateBufferDescriptorBinding(1, segment_data_list_buffer,
                                                                                          0);
}

bool DsRandomBundle::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("Random Bundle")) {
    if (ImGui::Checkbox("Enable", &enabled))
      changed = true;
    if (ImGui::DragInt("Sub iteration", &sub_iteration, 1, 1, 100))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

DsUniformBundle::DsUniformBundle() {
  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
  }

  VkBufferCreateInfo storage_buffer_create_info{};
  storage_buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  storage_buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  storage_buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  storage_buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  segment_pairs_buffer = std::make_shared<Buffer>(storage_buffer_create_info, buffer_vma_allocation_create_info);
  front_profiles_buffer = std::make_shared<Buffer>(storage_buffer_create_info, buffer_vma_allocation_create_info);
  back_profiles_buffer = std::make_shared<Buffer>(storage_buffer_create_info, buffer_vma_allocation_create_info);
  if (descriptor_sets.empty()) {
    descriptor_sets.resize(max_frame_in_flight);
    for (auto& i : descriptor_sets) {
      i = std::make_shared<DescriptorSet>(layout);
    }
  }

  if (!uniform_bundle_update_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/UniformBundleUpdate.comp");
    uniform_bundle_update_pipeline = std::make_shared<ComputePipeline>();
    uniform_bundle_update_pipeline->compute_shader = shader;
    uniform_bundle_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    uniform_bundle_update_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range = uniform_bundle_update_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(UniformBundleUpdateConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    uniform_bundle_update_pipeline->Initialize();
  }

  if (!uniform_bundle_front_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/UniformBundleFront.comp");
    uniform_bundle_front_pipeline = std::make_shared<ComputePipeline>();
    uniform_bundle_front_pipeline->compute_shader = shader;
    uniform_bundle_front_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    uniform_bundle_front_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range = uniform_bundle_front_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(UniformBundleConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    uniform_bundle_front_pipeline->Initialize();
  }
  if (!uniform_bundle_back_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/UniformBundleBack.comp");
    uniform_bundle_back_pipeline = std::make_shared<ComputePipeline>();
    uniform_bundle_back_pipeline->compute_shader = shader;
    uniform_bundle_back_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    uniform_bundle_back_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range = uniform_bundle_back_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(UniformBundleConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    uniform_bundle_back_pipeline->Initialize();
  }
}

void DsUniformBundle::Project(const DynamicStrands::PhysicsParameters& physics_parameters,
                              const DynamicStrands& target_dynamic_strands) {
  if (front_profiles.empty() && back_profiles.empty())
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  UniformBundleUpdateConstant update_constant;
  update_constant.pair_size = static_cast<uint32_t>(segment_pairs.size());
  update_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);
  update_constant.allow_breaking = physics_parameters.allow_breaking ? 1 : 0;

  UniformBundleConstant front_constraint_constant;
  front_constraint_constant.profile_size = static_cast<uint32_t>(front_profiles.size());
  front_constraint_constant.inv_time_step =
      1.f / (physics_parameters.time_step / physics_parameters.sub_step);
  
  UniformBundleConstant back_constraint_constant;
  back_constraint_constant.profile_size = static_cast<uint32_t>(back_profiles.size());
  back_constraint_constant.inv_time_step =
      1.f / (physics_parameters.time_step / physics_parameters.sub_step);

  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (int sub_iteration_index = 0; sub_iteration_index < sub_iteration; sub_iteration_index++) {
      uniform_bundle_update_pipeline->Bind(vk_command_buffer);
      uniform_bundle_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      uniform_bundle_update_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                        descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      uniform_bundle_update_pipeline->PushConstant(vk_command_buffer, 0, update_constant);
      vkCmdDispatch(vk_command_buffer, Platform::DivUp(update_constant.pair_size, task_work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);

      uniform_bundle_front_pipeline->Bind(vk_command_buffer);
      uniform_bundle_front_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      uniform_bundle_front_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                       descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      uniform_bundle_front_pipeline->PushConstant(vk_command_buffer, 0, front_constraint_constant);
      vkCmdDispatch(vk_command_buffer, front_constraint_constant.profile_size, 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);

      uniform_bundle_back_pipeline->Bind(vk_command_buffer);
      uniform_bundle_back_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      uniform_bundle_back_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                      descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      uniform_bundle_back_pipeline->PushConstant(vk_command_buffer, 0, back_constraint_constant);
      vkCmdDispatch(vk_command_buffer, front_constraint_constant.profile_size, 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    }
  });
}

void DsUniformBundle::InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                                     const StrandModelSkeleton& strand_model_skeleton,
                                     const DtsStrandGroup& subdivided_strand_group,
                                     const DynamicStrands& target_dynamic_strands) {
  struct SegmentProfile {
    std::vector<int> segment_handles;
    std::vector<glm::vec2> projected_positions;

    std::vector<std::pair<int, int>> edges;
    int segment_index;
    int node_handle;

    void CalculateEdges(const float avg_segment_radius) {
      std::vector<float> coords(segment_handles.size() * 2);
      memcpy(coords.data(), projected_positions.data(),
             sizeof(glm::vec2) * projected_positions.size());
      const Delaunator::Delaunator2D d(coords);
      std::set<std::pair<int, int>> edge_set;
      const float removal_length = avg_segment_radius * 6;
      for (std::size_t i = 0; i < d.triangles.size(); i += 3) {
        const auto& v0 = d.triangles[i];
        const auto& v1 = d.triangles[i + 1];
        const auto& v2 = d.triangles[i + 2];
        const auto& positions = projected_positions;
        if (glm::distance(positions[v0], positions[v1]) > removal_length ||
            glm::distance(positions[v1], positions[v2]) > removal_length ||
            glm::distance(positions[v0], positions[v2]) > removal_length)
          continue;
        edge_set.emplace(std::make_pair(glm::min(v0, v1), glm::max(v0, v1)));
        edge_set.emplace(std::make_pair(glm::min(v1, v2), glm::max(v1, v2)));
        edge_set.emplace(std::make_pair(glm::min(v0, v2), glm::max(v0, v2)));
      }
      edges.resize(edge_set.size());
      int index = 0;
      for (const auto i : edge_set) {
        edges[index] = i;
        index++;
      }
    }
  };

  std::vector<std::vector<SegmentProfile>> segment_profiles_list;
  std::vector<int> sub_segment_index_offsets;
  const auto& sorted_node_list = strand_model_skeleton.PeekSortedNodeList();
  segment_profiles_list.resize(sorted_node_list.size());
  sub_segment_index_offsets.resize(sorted_node_list.size());
  sub_segment_index_offsets[0] = 0;
  for (int node_handle : sorted_node_list) {
    auto& sub_segment_offset = sub_segment_index_offsets[node_handle];
    if (node_handle != 0) {
      const auto parent_handle = strand_model_skeleton.PeekNode(node_handle).GetParentHandle();
      sub_segment_offset = sub_segment_index_offsets[parent_handle] + initialize_parameters.sub_segment;
    }
    segment_profiles_list[node_handle].resize(initialize_parameters.sub_segment);
    auto& segment_profiles = segment_profiles_list[node_handle];
    for (int distance = 0; distance < initialize_parameters.sub_segment; distance++) {
      auto& segment_profile = segment_profiles[distance];
      segment_profile.node_handle = node_handle;
      segment_profile.segment_index = sub_segment_offset + distance;
    }
  }
  for (int segment_handle = 0; segment_handle < target_dynamic_strands.segments.size(); segment_handle++) {
    const auto& strand_segment_data = subdivided_strand_group.PeekStrandSegmentData(segment_handle);
    const auto& root_distance_offset = sub_segment_index_offsets[strand_segment_data.node_handle];
    segment_profiles_list[strand_segment_data.node_handle][strand_segment_data.segment_index - root_distance_offset]
        .segment_handles.emplace_back(segment_handle);
  }
  std::vector<SegmentProfile> valid_profiles;
  for (const auto& segment_profiles : segment_profiles_list) {
    for (const auto& profile : segment_profiles) {
      if (profile.segment_handles.size() > 1) {
        valid_profiles.emplace_back(profile);
      }
    }
  }
  segment_pairs.clear();
  for (auto& valid_profile : valid_profiles) {
    const auto& node = strand_model_skeleton.PeekNode(valid_profile.node_handle);
    const auto node_position = initialize_parameters.root_transform.TransformPoint(node.info.global_position);
    const auto node_rotation = initialize_parameters.root_transform.GetRotation() * node.info.global_rotation;
    const auto up = node_rotation * glm::vec3(0, 1, 0);
    const auto right = node_rotation * glm::vec3(1, 0, 0);
    valid_profile.projected_positions.resize(valid_profile.segment_handles.size());
    auto position_sum = glm::vec2(0.0f);
    float segment_radius_sum = 0.0f;
    for (int local_segment_index = 0; local_segment_index < valid_profile.segment_handles.size();
         local_segment_index++) {
      const auto& segment = target_dynamic_strands.segments[valid_profile.segment_handles[local_segment_index]];
      segment_radius_sum += segment.radius;
      const auto& particle1 = target_dynamic_strands.particles[segment.particle1_handle];
      auto& projected_position = valid_profile.projected_positions[local_segment_index];
      projected_position = Plane::ProjectPointToPlane(particle1.x0, node_position, up, right);
      position_sum += projected_position;
    }
    position_sum /= valid_profile.segment_handles.size();
    
    Jobs::RunParallelFor(valid_profile.segment_handles.size(), [&](const auto local_segment_index) {
      auto& projected_position = valid_profile.projected_positions[local_segment_index];
      projected_position -= position_sum;
    });
    PerProfileData profile;
    profile.start_pair_handle = static_cast<int>(segment_pairs.size());

    if (valid_profile.segment_handles.size() > 2) {
      valid_profile.CalculateEdges(segment_radius_sum / valid_profile.segment_handles.size());

      std::multimap<float, SegmentPair> current_profile_pairs;
      for (const auto& edge : valid_profile.edges) {
        const auto& segment_handle0 = valid_profile.segment_handles[edge.first];
        const auto& segment_handle1 = valid_profile.segment_handles[edge.second];

        float edge_center_distance = glm::length(valid_profile.projected_positions[edge.first]) +
                                     glm::length(valid_profile.projected_positions[edge.second]);

        SegmentPair new_pair;
        new_pair.segment0_handle = glm::min(segment_handle0, segment_handle1);
        new_pair.segment1_handle = glm::max(segment_handle0, segment_handle1);
        new_pair.valid = 1;
        new_pair.stiffness = glm::vec4(initialize_parameters.bending_modulus.GetValue() * 1e9f,
                                       initialize_parameters.bending_modulus.GetValue() * 1e9f,
                                       initialize_parameters.bending_modulus.GetValue() * 1e9f,
                                       initialize_parameters.youngs_modulus.GetValue() * 1e9f);
        new_pair.max_strain = glm::max(0.0f, initialize_parameters.max_neighbor_strain.GetValue());

        current_profile_pairs.emplace(edge_center_distance, new_pair);
      }
      for (const auto& i : current_profile_pairs) {
        segment_pairs.emplace_back(i.second);
      }
    } else {
      SegmentPair new_pair;
      const auto& segment_handle0 = valid_profile.segment_handles[0];
      const auto& segment_handle1 = valid_profile.segment_handles[1];
      new_pair.segment0_handle = glm::min(segment_handle0, segment_handle1);
      new_pair.segment1_handle = glm::max(segment_handle0, segment_handle1);
      new_pair.valid = 1;
      new_pair.stiffness = glm::vec4(initialize_parameters.bending_modulus.GetValue() * 1e9f,
                                     initialize_parameters.bending_modulus.GetValue() * 1e9f,
                                     initialize_parameters.bending_modulus.GetValue() * 1e9f,
                                     initialize_parameters.youngs_modulus.GetValue() * 1e9f);
      new_pair.max_strain = glm::max(0.0f, initialize_parameters.max_neighbor_strain.GetValue());
      segment_pairs.emplace_back(new_pair);
    }

    std::shuffle(segment_pairs.begin(), segment_pairs.end(), std::default_random_engine());

    profile.end_pair_handle = static_cast<int>(segment_pairs.size()) - 1;

    if (valid_profile.segment_index % 2 == 0)
      front_profiles.emplace_back(profile);
    else
      back_profiles.emplace_back(profile);
  }

  Jobs::RunParallelFor(segment_pairs.size(), [&](const auto pair_index) {
    auto& pair = segment_pairs[pair_index];
    auto& segment0 = target_dynamic_strands.segments[pair.segment0_handle];
    auto& segment1 = target_dynamic_strands.segments[pair.segment1_handle];
    auto& segment0_particle0 = target_dynamic_strands.particles[segment0.particle0_handle];
    auto& segment0_particle1 = target_dynamic_strands.particles[segment0.particle1_handle];

    auto& segment1_particle0 = target_dynamic_strands.particles[segment1.particle0_handle];
    auto& segment1_particle1 = target_dynamic_strands.particles[segment1.particle1_handle];

    const auto segment0_center_position = (segment0_particle0.x0 + segment0_particle1.x0) * .5f;
    const auto segment1_center_position = (segment1_particle0.x0 + segment1_particle1.x0) * .5f;

    pair.segment0_particle0_offset =
        glm::vec4(glm::inverse(segment1.q0) * (segment0_particle0.x0 - segment1_center_position), 0.0f);
    pair.segment0_particle1_offset =
        glm::vec4(glm::inverse(segment1.q0) * (segment0_particle1.x0 - segment1_center_position), 0.0f);

    pair.segment1_particle0_offset =
        glm::vec4(glm::inverse(segment0.q0) * (segment1_particle0.x0 - segment0_center_position), 0.0f);
    pair.segment1_particle1_offset =
        glm::vec4(glm::inverse(segment0.q0) * (segment1_particle1.x0 - segment0_center_position), 0.0f);

    pair.rest_darboux_vector = glm::conjugate(segment0.q0) * segment1.q0;
  });
}

void DsUniformBundle::UploadData() {
  segment_pairs_buffer->UploadVector(segment_pairs);
  front_profiles_buffer->UploadVector(front_profiles);
  back_profiles_buffer->UploadVector(back_profiles);
}

void DsUniformBundle::UpdateBindings() {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, segment_pairs_buffer, 0);
  descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(1, front_profiles_buffer, 0);
  descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(1, back_profiles_buffer, 0);
}

bool DsUniformBundle::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("Uniform Bundle")) {
    if (ImGui::Checkbox("Enable", &enabled))
      changed = true;
    if (ImGui::DragInt("Sub iteration", &sub_iteration, 1, 1, 100))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}
