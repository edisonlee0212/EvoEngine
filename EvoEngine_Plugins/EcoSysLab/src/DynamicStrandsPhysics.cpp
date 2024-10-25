#include "DynamicStrandsPhysics.hpp"
#include "Shader.hpp"
#include "VoxelGrid.hpp"
using namespace eco_sys_lab_plugin;

void DynamicStrands::Physics(const PhysicsParameters& physics_parameters) const {
  if (pre_step)
    pre_step->Execute(physics_parameters, *this);

  for (const auto& op : operators) {
    if (op->enabled)
      op->Execute(physics_parameters, *this);
  }
  if (prediction)
    prediction->Execute(physics_parameters, *this);

  for (const auto& c : constraints) {
    if (c->enabled)
      for (int iteration_i = 0; iteration_i < physics_parameters.constraint_iteration; iteration_i++) {
        c->Project(physics_parameters, *this);
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
}

void DynamicStrandsPrediction::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                                       const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  ParticlePredictionPushConstant particle_push_constant;
  particle_push_constant.particle_size = target_dynamic_strands.particles.size();
  particle_push_constant.time_step = physics_parameters.time_step;
  particle_push_constant.inv_time_step = 1.f / particle_push_constant.time_step;
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  SegmentPredictionPushConstant segment_push_constant;
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.time_step = physics_parameters.time_step;
  segment_push_constant.inv_time_step = 1.f / segment_push_constant.time_step;

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
  });
}

DsPositionUpdate::DsPositionUpdate() {
  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
  }
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  commands_buffer.resize(max_frame_in_flight);

  for (int frame_index = 0; frame_index < max_frame_in_flight; frame_index++) {
    commands_buffer[frame_index] = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  }

  if (!position_update_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/PositionUpdate.comp");

    position_update_pipeline = std::make_shared<ComputePipeline>();
    position_update_pipeline->compute_shader = shader;

    position_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    position_update_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& push_constant_range = position_update_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PositionUpdatePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    position_update_pipeline->Initialize();
  }
  commands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : commands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(layout);
  }
}

void DsPositionUpdate::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                               const DynamicStrands& target_dynamic_strands) {
  if (commands.empty())
    return;

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  commands_buffer[current_frame_index]->UploadVector(commands);
  commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, commands_buffer[current_frame_index]);

  PositionUpdatePushConstant push_constant;
  push_constant.commands_size = commands.size();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    position_update_pipeline->Bind(vk_command_buffer);
    position_update_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    position_update_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    position_update_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.commands_size, task_work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

DsRotationUpdate::DsRotationUpdate() {
  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
  }
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  commands_buffer.resize(max_frame_in_flight);

  for (int frame_index = 0; frame_index < max_frame_in_flight; frame_index++) {
    commands_buffer[frame_index] = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  }

  if (!rotation_update_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/RotationUpdate.comp");

    rotation_update_pipeline = std::make_shared<ComputePipeline>();
    rotation_update_pipeline->compute_shader = shader;

    rotation_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    rotation_update_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& push_constant_range = rotation_update_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RotationUpdatePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    rotation_update_pipeline->Initialize();
  }

  commands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : commands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(layout);
  }
}

void DsRotationUpdate::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                               const DynamicStrands& target_dynamic_strands) {
  if (commands.empty())
    return;

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  commands_buffer[current_frame_index]->UploadVector(commands);
  commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, commands_buffer[current_frame_index]);

  RotationUpdatePushConstant push_constant;
  push_constant.commands_size = commands.size();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    rotation_update_pipeline->Bind(vk_command_buffer);
    rotation_update_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    rotation_update_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    rotation_update_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.commands_size, task_work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

void DsGravityForce::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                             const DynamicStrands& target_dynamic_strands) {
  GravityForcePushConstant push_constant;
  push_constant.acceleration = gravity;
  push_constant.particle_size = target_dynamic_strands.particles.size();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    gravity_force_pipeline->Bind(vk_command_buffer);
    gravity_force_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    gravity_force_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.particle_size, task_work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

bool DsGravityForce::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("ExternalForce")) {
    if (ImGui::Checkbox("Enable", &enabled))
      changed = true;
    if (ImGui::DragFloat3("Gravity", &gravity.x, 0.01f, -100.0f, 100.0f))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

DsGravityForce::DsGravityForce() {
  if (!gravity_force_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Operators/ConstantAcceleration.comp");
    gravity_force_pipeline = std::make_shared<ComputePipeline>();
    gravity_force_pipeline->compute_shader = shader;
    gravity_force_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = gravity_force_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GravityForcePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    gravity_force_pipeline->Initialize();
  }
}

DsExternalForce::DsExternalForce() {
  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);

    layout->Initialize();
  }

  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  commands_buffer.resize(max_frame_in_flight);

  for (int frame_index = 0; frame_index < max_frame_in_flight; frame_index++) {
    commands_buffer[frame_index] = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  }

  if (!external_force_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/ExternalForce.comp");
    external_force_pipeline = std::make_shared<ComputePipeline>();
    external_force_pipeline->compute_shader = shader;

    external_force_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    external_force_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& push_constant_range = external_force_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(ExternalForcePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    external_force_pipeline->Initialize();
  }
  commands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : commands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(layout);
  }
}

void DsExternalForce::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                              const DynamicStrands& target_dynamic_strands) {
  if (commands.empty())
    return;

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  commands_buffer[current_frame_index]->UploadVector(commands);

  commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, commands_buffer[current_frame_index]);

  ExternalForcePushConstant push_constant;
  push_constant.commands_size = commands.size();
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    external_force_pipeline->Bind(vk_command_buffer);
    external_force_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    external_force_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                               commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    external_force_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.commands_size, task_work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
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
  if (!bilateral_stretch_shear_constraint_pipeline) {
    static std::shared_ptr<Shader> stretch_shear_shader{};
    stretch_shear_shader = std::make_shared<Shader>();
    stretch_shear_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
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
    bend_twist_constraint_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                                      std::filesystem::path("./EcoSysLabResources") /
                                          "Shaders/Compute/DynamicStrands/Constraints/StiffRodBendTwistBilateral.comp");

    bilateral_bend_twist_constraint_pipeline = std::make_shared<ComputePipeline>();
    bilateral_bend_twist_constraint_pipeline->compute_shader = bend_twist_constraint_shader;

    bilateral_bend_twist_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    bilateral_bend_twist_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& bend_twist_push_constant_range = bilateral_bend_twist_constraint_pipeline->push_constant_ranges.emplace_back();
    bend_twist_push_constant_range.size = sizeof(BendTwistConstraintConstant);
    bend_twist_push_constant_range.offset = 0;
    bend_twist_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    bilateral_bend_twist_constraint_pipeline->Initialize();
  }
  if (!forward_stretch_shear_constraint_pipeline) {
    static std::shared_ptr<Shader> stretch_shear_shader{};
    stretch_shear_shader = std::make_shared<Shader>();
    stretch_shear_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Compute/DynamicStrands/Constraints/StiffRodStretchShearForward.comp");
    forward_stretch_shear_constraint_pipeline = std::make_shared<ComputePipeline>();
    forward_stretch_shear_constraint_pipeline->compute_shader = stretch_shear_shader;

    forward_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    forward_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range = forward_stretch_shear_constraint_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(StretchShearConstraintConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    forward_stretch_shear_constraint_pipeline->Initialize();

    static std::shared_ptr<Shader> bend_twist_constraint_shader{};
    bend_twist_constraint_shader = std::make_shared<Shader>();
    bend_twist_constraint_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
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
    stretch_shear_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
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
    bend_twist_constraint_shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
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
    if (ImGui::DragInt("Sub iteration", &sub_iteration, 1, 1, 100))
      changed = true;
    if (ImGui::Combo("Project Mode", {"Bilateral", "Forward", "Backward", "Balanced"}, project_mode))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

void DsStiffRod::InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                                const StrandModelSkeleton& strand_model_skeleton,
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

  UploadData();

  for (int i = 0; i < Platform::GetMaxFramesInFlight(); i++) {
    strands_physics_descriptor_sets[i]->UpdateBufferDescriptorBinding(0, per_strand_data_list_buffer);
  }
}

void DsStiffRod::Project(const DynamicStrands::PhysicsParameters& physics_parameters,
                         const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  StretchShearConstraintConstant stretch_shear_constraint_constant;
  stretch_shear_constraint_constant.strand_size = per_strand_data_list.size();

  BendTwistConstraintConstant bend_twist_constraint_constant;
  bend_twist_constraint_constant.strand_size = per_strand_data_list.size();

  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (int sub_iteration_index = 0; sub_iteration_index < sub_iteration; sub_iteration_index++) {
      switch (static_cast<ProjectMode>(project_mode)) {
        case ProjectMode::Bilateral:
          bilateral_stretch_shear_constraint_pipeline->Bind(vk_command_buffer);
          bilateral_stretch_shear_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 0,
              target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
          bilateral_stretch_shear_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

          bilateral_stretch_shear_constraint_pipeline->PushConstant(vk_command_buffer, 0,
                                                                  stretch_shear_constraint_constant);
          vkCmdDispatch(vk_command_buffer,
                        Platform::DivUp(stretch_shear_constraint_constant.strand_size, task_work_group_invocations), 1,
                        1);
          Platform::EverythingBarrier(vk_command_buffer);

          bilateral_bend_twist_constraint_pipeline->Bind(vk_command_buffer);
          bilateral_bend_twist_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 0,
              target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
          bilateral_bend_twist_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

          bilateral_bend_twist_constraint_pipeline->PushConstant(vk_command_buffer, 0, bend_twist_constraint_constant);
          vkCmdDispatch(vk_command_buffer,
                        Platform::DivUp(bend_twist_constraint_constant.strand_size, task_work_group_invocations), 1, 1);
          Platform::EverythingBarrier(vk_command_buffer);
          break;
        case ProjectMode::Forward:
          forward_stretch_shear_constraint_pipeline->Bind(vk_command_buffer);
          forward_stretch_shear_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 0,
              target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
          forward_stretch_shear_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

          forward_stretch_shear_constraint_pipeline->PushConstant(vk_command_buffer, 0,
                                                                  stretch_shear_constraint_constant);
          vkCmdDispatch(vk_command_buffer,
                        Platform::DivUp(stretch_shear_constraint_constant.strand_size, task_work_group_invocations), 1,
                        1);
          Platform::EverythingBarrier(vk_command_buffer);

          forward_bend_twist_constraint_pipeline->Bind(vk_command_buffer);
          forward_bend_twist_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 0,
              target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
          forward_bend_twist_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

          forward_bend_twist_constraint_pipeline->PushConstant(vk_command_buffer, 0, bend_twist_constraint_constant);
          vkCmdDispatch(vk_command_buffer,
                        Platform::DivUp(bend_twist_constraint_constant.strand_size, task_work_group_invocations), 1, 1);
          Platform::EverythingBarrier(vk_command_buffer);
          break;
        case ProjectMode::Backward:
          backward_stretch_shear_constraint_pipeline->Bind(vk_command_buffer);
          backward_stretch_shear_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 0,
              target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
          backward_stretch_shear_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

          backward_stretch_shear_constraint_pipeline->PushConstant(vk_command_buffer, 0,
                                                                  stretch_shear_constraint_constant);
          vkCmdDispatch(vk_command_buffer,
                        Platform::DivUp(stretch_shear_constraint_constant.strand_size, task_work_group_invocations), 1,
                        1);
          Platform::EverythingBarrier(vk_command_buffer);

          backward_bend_twist_constraint_pipeline->Bind(vk_command_buffer);
          backward_bend_twist_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 0,
              target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
          backward_bend_twist_constraint_pipeline->BindDescriptorSet(
              vk_command_buffer, 1, strands_physics_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

          backward_bend_twist_constraint_pipeline->PushConstant(vk_command_buffer, 0, bend_twist_constraint_constant);
          vkCmdDispatch(vk_command_buffer,
                        Platform::DivUp(bend_twist_constraint_constant.strand_size, task_work_group_invocations), 1, 1);
          Platform::EverythingBarrier(vk_command_buffer);
          break;
        case ProjectMode::Balanced:
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

glm::vec3 DsStiffRod::ComputeDarbouxVector(const glm::quat& q0, const glm::quat& q1,
                                           const float average_segment_length) {
  const auto relative_rotation = glm::conjugate(q0) * q1;
  return 2.f / average_segment_length * glm::vec3(relative_rotation.x, relative_rotation.y, relative_rotation.z);
}

DsParticleNeighbor::DsParticleNeighbor() {
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

  particle_neighbors_buffer = std::make_shared<Buffer>(storage_buffer_create_info, buffer_vma_allocation_create_info);
  if (!particle_neighbor_offset_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/NeighborConstraintOffset.comp");

    particle_neighbor_offset_pipeline = std::make_shared<ComputePipeline>();
    particle_neighbor_offset_pipeline->compute_shader = shader;

    particle_neighbor_offset_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    particle_neighbor_offset_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range = particle_neighbor_offset_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(ParticleNeighborConstraintConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    particle_neighbor_offset_pipeline->Initialize();
  }
  if (!particle_neighbor_apply_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/NeighborConstraintApply.comp");

    particle_neighbor_apply_pipeline = std::make_shared<ComputePipeline>();
    particle_neighbor_apply_pipeline->compute_shader = shader;

    particle_neighbor_apply_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    particle_neighbor_apply_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range = particle_neighbor_apply_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(ParticleNeighborConstraintConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    particle_neighbor_apply_pipeline->Initialize();
  }

  if (particle_neighbors_descriptor_sets.empty()) {
    particle_neighbors_descriptor_sets.resize(max_frame_in_flight);
    for (auto& i : particle_neighbors_descriptor_sets) {
      i = std::make_shared<DescriptorSet>(layout);
    }
  }
}

void DsParticleNeighbor::InitializeData(const DynamicStrands::InitializeParameters& initialize_parameters,
                                        const StrandModelSkeleton& strand_model_skeleton,
                                        const DynamicStrands& target_dynamic_strands) {
  particle_neighbors.resize(target_dynamic_strands.particles.size());

  std::vector<glm::vec3> max_bounds(Jobs::GetWorkerSize());
  std::vector<glm::vec3> min_bounds(Jobs::GetWorkerSize());
  for (auto& i : max_bounds)
    i = glm::vec3(-FLT_MAX);
  for (auto& i : min_bounds)
    i = glm::vec3(FLT_MAX);
  Jobs::RunParallelFor(particle_neighbors.size(), [&](const auto i, const auto worker_i) {
    const auto& particle = target_dynamic_strands.particles[i];
    max_bounds[worker_i] = glm::max(max_bounds[worker_i], particle.x0);
    min_bounds[worker_i] = glm::min(min_bounds[worker_i], particle.x0);
  });
  auto max_bound = glm::vec3(-FLT_MAX);
  auto min_bound = glm::vec3(FLT_MAX);
  for (auto& i : max_bounds)
    max_bound = glm::max(i, max_bound);
  for (auto& i : min_bounds)
    min_bound = glm::min(i, min_bound);
  struct ParticleInfo {
    glm::vec3 position;
    int node_handle;
    int strand_handle;
    int particle_handle;
  };
  VoxelGrid<std::vector<ParticleInfo>> voxel_grid;
  voxel_grid.Initialize(glm::max(0.05f, initialize_parameters.neighbor_range), min_bound - glm::vec3(0.1f),
                        max_bound + glm::vec3(0.1f), {});
  for (int particle_index = 0; particle_index < target_dynamic_strands.particles.size(); particle_index++) {
    const auto& particle = target_dynamic_strands.particles[particle_index];
    ParticleInfo s_d;
    s_d.position = particle.x0;
    s_d.node_handle = particle.node_handle;
    s_d.strand_handle = particle.strand_handle;
    s_d.particle_handle = particle_index;
    voxel_grid.Ref(particle.x0).emplace_back(s_d);
  }

  Jobs::RunParallelFor(particle_neighbors.size(), [&](const auto i) {
    auto& connectivity = particle_neighbors[i];
    const DynamicStrands::GpuParticle& particle = target_dynamic_strands.particles[i];
    std::multimap<float, int> candidates;
    voxel_grid.ForEach(particle.x0, initialize_parameters.neighbor_range, [&](const std::vector<ParticleInfo>& list) {
      for (const auto& info : list) {
        if (info.strand_handle == particle.strand_handle)
          continue;
        bool node_check = false;
        if (info.node_handle == particle.node_handle)
          node_check = true;
        if (!node_check) {
          if (auto& node = strand_model_skeleton.PeekNode(particle.node_handle);
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
        const auto distance = glm::distance(info.position, particle.x0);
        candidates.emplace(distance, info.particle_handle);
      }
    });

    int neighbor_index = 0;
    for (const auto& candidate : candidates) {
      connectivity.distances[neighbor_index] = candidate.first;
      connectivity.neighbors[neighbor_index] = candidate.second;
      neighbor_index++;
      if (neighbor_index >= 8)
        break;
    }
    while (neighbor_index < 8) {
      connectivity.neighbors[neighbor_index] = -1;
      neighbor_index++;
    }
  });



  UploadData();
  for (int i = 0; i < Platform::GetMaxFramesInFlight(); i++) {
    particle_neighbors_descriptor_sets[i]->UpdateBufferDescriptorBinding(0, particle_neighbors_buffer);
  }
}

void DsParticleNeighbor::Project(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) {
  if (particle_neighbors.empty())
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  ParticleNeighborConstraintConstant push_constant;
  push_constant.particle_size = static_cast<uint32_t>(particle_neighbors.size());
  const uint32_t task_work_group_invocations =
      Platform::GetSelectedPhysicalDevice()->mesh_shader_properties_ext.maxPreferredTaskWorkGroupInvocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (int sub_iteration_index = 0; sub_iteration_index < sub_iteration; sub_iteration_index++) {
      particle_neighbor_offset_pipeline->Bind(vk_command_buffer);
      particle_neighbor_offset_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      particle_neighbor_offset_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, particle_neighbors_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

      particle_neighbor_offset_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

      vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.particle_size, task_work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);

      particle_neighbor_apply_pipeline->Bind(vk_command_buffer);
      particle_neighbor_apply_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      particle_neighbor_apply_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, particle_neighbors_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

      particle_neighbor_apply_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

      vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.particle_size, task_work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    }
  });
}

bool DsParticleNeighbor::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("ParticleNeighbor")) {
    if (ImGui::Checkbox("Enable", &enabled))
      changed = true;
    if (ImGui::DragInt("Sub iteration", &sub_iteration, 1, 1, 100))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}

void DsParticleNeighbor::UploadData() {
  particle_neighbors_buffer->UploadVector(particle_neighbors);
}
