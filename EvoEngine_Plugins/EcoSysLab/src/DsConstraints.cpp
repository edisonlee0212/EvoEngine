#include "DsConstraints.hpp"
#include "Shader.hpp"
#include "VoxelGrid.hpp"
using namespace eco_sys_lab_plugin;

DsPivotPoint::DsPivotPoint() {
  VkBufferCreateInfo buffer_create_info{};
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
  }

  segment_update_commands_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  if (!segment_update_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/PivotPoint.comp");

    segment_update_pipeline = std::make_shared<ComputePipeline>();
    segment_update_pipeline->compute_shader = shader;
    segment_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    segment_update_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& push_constant_range = segment_update_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentUpdatePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_update_pipeline->Initialize();
  }

  segment_commands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : segment_commands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(layout);
  }
}

void DsPivotPoint::Initialize(const GlobalTransform& target_base_global_transform,
                              const std::shared_ptr<DynamicStrands>& target_dynamic_strands,
                              const std::vector<std::pair<uint32_t, bool>>& segment_list) {
  base_global_transform = target_base_global_transform;
  inverse_base_global_transform.value = glm::inverse(base_global_transform.value);
  commands.resize(segment_list.size());
  const glm::vec3 pivot_position = target_base_global_transform.GetPosition();
  Jobs::RunParallelFor(segment_list.size(), [&](const size_t i) {
    const auto& segment_info = segment_list[i];
    const auto& segment = target_dynamic_strands->segments[segment_info.first];
    auto& command = commands[i];

    command.segment_index = segment_info.first;
    const glm::vec3 segment_center_position = (segment.particle0.x0 + segment.particle1.x0) * 0.5f;
    command.point_distance = glm::distance(pivot_position, segment_center_position);
  });
  segment_update_commands_buffer->UploadVector(commands);
}

void DsPivotPoint::Update(const GlobalTransform& new_global_transform) {
  push_constant.pivot_position = new_global_transform.GetPosition();
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0,
                                                                                       segment_update_commands_buffer);
}

void DsPivotPoint::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                             const DynamicStrands& target_dynamic_strands) {
  if (!commands.empty()) {
    const auto current_frame_index = Platform::GetCurrentFrameIndex();
    push_constant.commands_size = commands.size();
    const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      segment_update_pipeline->Bind(vk_command_buffer);
      segment_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      segment_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, segment_commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      segment_update_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      segment_update_pipeline->Dispatch(vk_command_buffer,
                                        Platform::DivUp(push_constant.commands_size, work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
  }
}

DsPivotAxis::DsPivotAxis() {
  VkBufferCreateInfo buffer_create_info{};
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
  }

  segment_update_commands_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  if (!segment_update_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/PivotAxis.comp");

    segment_update_pipeline = std::make_shared<ComputePipeline>();
    segment_update_pipeline->compute_shader = shader;
    segment_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    segment_update_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& push_constant_range = segment_update_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentUpdatePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_update_pipeline->Initialize();
  }

  segment_commands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : segment_commands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(layout);
  }
}

void DsPivotAxis::Initialize(const GlobalTransform& target_base_global_transform,
                             const std::shared_ptr<DynamicStrands>& target_dynamic_strands,
                             const std::vector<std::pair<uint32_t, bool>>& segment_list) {
  base_global_transform = target_base_global_transform;
  inverse_base_global_transform.value = glm::inverse(base_global_transform.value);
  commands.resize(segment_list.size());

  const glm::vec3 pivot_position = target_base_global_transform.GetPosition();
  const glm::vec3 axis = target_base_global_transform.GetRotation() * glm::vec3(0, 0, -1);

  Jobs::RunParallelFor(segment_list.size(), [&](const size_t i) {
    const auto& segment_info = segment_list[i];
    const auto& segment = target_dynamic_strands->segments[segment_info.first];
    auto& command = commands[i];

    command.segment_index = segment_info.first;
    const glm::vec3 segment_center_position = (segment.particle0.x0 + segment.particle1.x0) * 0.5f;
    const glm::vec3 line_projection = glm::dot(segment_center_position - pivot_position, axis) * axis;
    command.axis_distance = glm::distance(pivot_position + line_projection, segment_center_position);
    command.axis_offset =
        glm::dot(line_projection, axis) > 0.f ? glm::length(line_projection) : -glm::length(line_projection);
    command.particle0_closer = segment_info.second;
  });
  segment_update_commands_buffer->UploadVector(commands);
}

void DsPivotAxis::Update(const GlobalTransform& new_global_transform) {
  push_constant.pivot_position = new_global_transform.GetPosition();
  push_constant.axis = new_global_transform.GetRotation() * glm::vec3(0, 0, -1);
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0,
                                                                                       segment_update_commands_buffer);
}

void DsPivotAxis::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                            const DynamicStrands& target_dynamic_strands) {
  if (!commands.empty()) {
    const auto current_frame_index = Platform::GetCurrentFrameIndex();
    push_constant.commands_size = commands.size();
    const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      segment_update_pipeline->Bind(vk_command_buffer);
      segment_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      segment_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, segment_commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      segment_update_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      segment_update_pipeline->Dispatch(vk_command_buffer,
                                        Platform::DivUp(push_constant.commands_size, work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
  }
}

DsPivotTransform::DsPivotTransform() {
  VkBufferCreateInfo buffer_create_info{};
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
  }

  segment_update_commands_buffer.resize(max_frame_in_flight);

  for (int frame_index = 0; frame_index < max_frame_in_flight; frame_index++) {
    segment_update_commands_buffer[frame_index] =
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  }

  if (!segment_update_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/PivotTransform.comp");

    segment_update_pipeline = std::make_shared<ComputePipeline>();
    segment_update_pipeline->compute_shader = shader;
    segment_update_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    segment_update_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& push_constant_range = segment_update_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentUpdatePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_update_pipeline->Initialize();
  }

  segment_commands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : segment_commands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(layout);
  }
}

void DsPivotTransform::Initialize(const GlobalTransform& target_base_global_transform,
                                  const std::shared_ptr<DynamicStrands>& target_dynamic_strands,
                                  const std::vector<std::pair<uint32_t, std::pair<bool, bool>>>& segment_list) {
  base_global_transform = target_base_global_transform;
  inverse_base_global_transform.value = glm::inverse(base_global_transform.value);
  commands.resize(segment_list.size());

  Jobs::RunParallelFor(segment_list.size(), [&](const size_t i) {
    const auto& segment_info = segment_list[i];
    const auto& segment = target_dynamic_strands->segments[segment_info.first];
    auto& command = commands[i];

    command.segment_index = segment_info.first;
    if (command.segment_index != UINT32_MAX) {
      command.new_rotation = segment.q0;
      command.new_particle0_position = segment.particle0.x0;
      command.fix_particle0 = segment_info.second.first ? 1 : 0;
      command.new_particle1_position = segment.particle1.x0;
      command.fix_particle1 = segment_info.second.second ? 1 : 0;
    }
  });
}

void DsPivotTransform::Update(const GlobalTransform& new_global_transform,
                              const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  const glm::quat rotation = new_global_transform.GetRotation() * inverse_base_global_transform.GetRotation();
  Jobs::RunParallelFor(commands.size(), [&](const size_t i) {
    auto& command = commands[i];
    if (command.segment_index != UINT32_MAX) {
      command.new_particle0_position = new_global_transform.TransformPoint(inverse_base_global_transform.TransformPoint(
          target_dynamic_strands->segments[command.segment_index].particle0.x0));
      command.new_particle1_position = new_global_transform.TransformPoint(inverse_base_global_transform.TransformPoint(
          target_dynamic_strands->segments[command.segment_index].particle1.x0));

      command.new_rotation = rotation * target_dynamic_strands->segments[command.segment_index].q0;
    }
  });

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  if (!commands.empty()) {
    segment_update_commands_buffer[current_frame_index]->UploadVector(commands);
    segment_commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
        0, segment_update_commands_buffer[current_frame_index]);
  }
}

void DsPivotTransform::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                                 const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  if (!commands.empty()) {
    SegmentUpdatePushConstant push_constant;
    push_constant.commands_size = commands.size();
    push_constant.ring_radius = physics_parameters.pivot_ring_radius;
    push_constant.HC_threshold = physics_parameters.HC_threshold;
    push_constant.HL_threshold = physics_parameters.HL_threshold;
    const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;

    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      segment_update_pipeline->Bind(vk_command_buffer);
      segment_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      segment_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, segment_commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      segment_update_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      segment_update_pipeline->Dispatch(vk_command_buffer,
                                        Platform::DivUp(push_constant.commands_size, work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
  }
}

DsStiffRod::DsStiffRod() {
  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
  }

  if (!pipeline) {
    static std::shared_ptr<Shader> stretch_shear_shader{};
    stretch_shear_shader = std::make_shared<Shader>();
    stretch_shear_shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                     std::filesystem::path("./EcoSysLabResources") /
                                         "Shaders/Compute/DynamicStrands/Constraints/Position/StiffRod.comp");
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = stretch_shear_shader;

    pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range = pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(ShearStretchConstraintConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    pipeline->Initialize();
  }
}

bool DsStiffRod::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("StiffRod")) {
    if (ImGui::Checkbox("Enable", &enabled))
      changed = true;
    if (enabled) {
      if (ImGui::DragInt("Sub iteration", &sub_iteration, 1, 1, 100))
        changed = true;
    }
    ImGui::TreePop();
  }
  return changed;
}

void DsStiffRod::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                           const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  ShearStretchConstraintConstant stretch_shear_constraint_constant;
  stretch_shear_constraint_constant.strand_size = target_dynamic_strands.strands.size();
  stretch_shear_constraint_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);
  stretch_shear_constraint_constant.frame_index = target_dynamic_strands.GetFrameIndex();
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (int sub_iteration_index = 0; sub_iteration_index < sub_iteration; sub_iteration_index++) {
      pipeline->Bind(vk_command_buffer);
      pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      pipeline->PushConstant(vk_command_buffer, 0, stretch_shear_constraint_constant);
      pipeline->Dispatch(vk_command_buffer,
                         Platform::DivUp(stretch_shear_constraint_constant.strand_size, work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    }
  });
}

glm::vec3 DsStiffRod::ComputeDarbouxVector(const glm::quat& q0, const glm::quat& q1,
                                           const float average_segment_length) {
  const auto relative_rotation = glm::conjugate(q0) * q1;
  return 2.f / average_segment_length * glm::vec3(relative_rotation.x, relative_rotation.y, relative_rotation.z);
}

DsBundle::DsBundle() {
  if (!stretch_shear_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") /
            "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateShearStretchCorrections.comp");
    stretch_shear_pipeline = std::make_shared<ComputePipeline>();
    stretch_shear_pipeline->compute_shader = shader;
    stretch_shear_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = stretch_shear_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RandomBundleShearStretchConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    stretch_shear_pipeline->Initialize();
  }

  if (!bend_twist_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") /
            "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBendTwistCorrections.comp");
    bend_twist_pipeline = std::make_shared<ComputePipeline>();
    bend_twist_pipeline->compute_shader = shader;
    bend_twist_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = bend_twist_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RandomBundleBendTwistConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bend_twist_pipeline->Initialize();
  }

  if (!bundle_position_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") /
            "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBundlePositionCorrections.comp");
    bundle_position_pipeline = std::make_shared<ComputePipeline>();
    bundle_position_pipeline->compute_shader = shader;
    bundle_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = bundle_position_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RandomBundleConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bundle_position_pipeline->Initialize();
  }

  if (!bundle_rotation_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") /
            "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBundleRotationCorrections.comp");
    bundle_rotation_pipeline = std::make_shared<ComputePipeline>();
    bundle_rotation_pipeline->compute_shader = shader;
    bundle_rotation_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = bundle_rotation_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RandomBundleConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bundle_rotation_pipeline->Initialize();
  }

  if (!apply_rotation_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyRotationCorrections.comp");

    apply_rotation_pipeline = std::make_shared<ComputePipeline>();
    apply_rotation_pipeline->compute_shader = shader;

    apply_rotation_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = apply_rotation_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RandomBundleApplySegmentsConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    apply_rotation_pipeline->Initialize();
  }

  if (!apply_position_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyPositionCorrections.comp");

    apply_position_pipeline = std::make_shared<ComputePipeline>();
    apply_position_pipeline->compute_shader = shader;

    apply_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = apply_position_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RandomBundleApplySegmentsConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    apply_position_pipeline->Initialize();
  }

  if (!apply_position_rotation_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyCorrections.comp");

    apply_position_rotation_pipeline = std::make_shared<ComputePipeline>();
    apply_position_rotation_pipeline->compute_shader = shader;

    apply_position_rotation_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = apply_position_rotation_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RandomBundleApplySegmentsConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    apply_position_rotation_pipeline->Initialize();
  }

  if (!connections_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ConnectionCorrections.comp");

    connections_pipeline = std::make_shared<ComputePipeline>();
    connections_pipeline->compute_shader = shader;
    connections_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    auto& stretch_shear_push_constant_range = connections_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(RandomBundleApplyConnectionsConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    connections_pipeline->Initialize();
  }
}

void DsBundle::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                         const DynamicStrands& target_dynamic_strands) {
  if (target_dynamic_strands.segment_pairs.empty())
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  RandomBundleConstant constraint_constant;
  constraint_constant.skip_size = skip_size;
  constraint_constant.segment_size = static_cast<uint32_t>(target_dynamic_strands.segment_data_list.size());
  constraint_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);
  constraint_constant.over_relaxation = over_relaxation;
  constraint_constant.crack_bd_shrinkage_offset = physics_parameters.crack_bd_shrinkage_offset;
  constraint_constant.crack_R_scale = physics_parameters.crack_R_scale;
  constraint_constant.crack_T_scale = physics_parameters.crack_T_scale;
  constraint_constant.treespace = physics_parameters.treespace;
  constraint_constant.internal_pattern = physics_parameters.internal_pattern;

  RandomBundleBendTwistConstant bend_twist_constraint_constant;
  bend_twist_constraint_constant.skip_size = skip_size;
  bend_twist_constraint_constant.segment_size = static_cast<uint32_t>(target_dynamic_strands.segment_data_list.size());
  bend_twist_constraint_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);
  bend_twist_constraint_constant.over_relaxation = bend_twist_over_relaxation;

  RandomBundleShearStretchConstant stretch_shear_constraint_constant;
  stretch_shear_constraint_constant.skip_size = skip_size;
  stretch_shear_constraint_constant.segment_size =
      static_cast<uint32_t>(target_dynamic_strands.segment_data_list.size());
  stretch_shear_constraint_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);

  RandomBundleApplySegmentsConstant constraint_apply_segments_constant;
  constraint_apply_segments_constant.skip_size = skip_size;
  constraint_apply_segments_constant.segment_size =
      static_cast<uint32_t>(target_dynamic_strands.segment_data_list.size());
  constraint_apply_segments_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);

  RandomBundleApplyConnectionsConstant constraint_apply_connections_constant;
  constraint_apply_connections_constant.segment_pair_size =
      static_cast<uint32_t>(target_dynamic_strands.connection_segment_pair_size);
  constraint_apply_connections_constant.inv_time_step =
      1.f / (physics_parameters.time_step / physics_parameters.sub_step);

  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (int sub_iteration_index = 0; sub_iteration_index < sub_iteration; sub_iteration_index++) {
      const auto apply_rotations = [&](const uint32_t skip_index) {
        apply_rotation_pipeline->Bind(vk_command_buffer);
        apply_rotation_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        constraint_apply_segments_constant.skip_index = skip_index;
        apply_rotation_pipeline->PushConstant(vk_command_buffer, 0, constraint_apply_segments_constant);
        apply_rotation_pipeline->Dispatch(
            vk_command_buffer,
            Platform::DivUp(Platform::DivUp(constraint_apply_segments_constant.segment_size, skip_size),
                            work_group_invocations),
            1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      };

      const auto apply_positions = [&](const uint32_t skip_index) {
        apply_position_pipeline->Bind(vk_command_buffer);
        apply_position_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        constraint_apply_segments_constant.skip_index = skip_index;
        apply_position_pipeline->PushConstant(vk_command_buffer, 0, constraint_apply_segments_constant);
        apply_position_pipeline->Dispatch(
            vk_command_buffer,
            Platform::DivUp(Platform::DivUp(constraint_apply_segments_constant.segment_size, skip_size),
                            work_group_invocations),
            1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      };

      const auto apply_both = [&](const uint32_t skip_index) {
        apply_position_rotation_pipeline->Bind(vk_command_buffer);
        apply_position_rotation_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        constraint_apply_segments_constant.skip_index = skip_index;
        apply_position_rotation_pipeline->PushConstant(vk_command_buffer, 0, constraint_apply_segments_constant);
        apply_position_rotation_pipeline->Dispatch(
            vk_command_buffer,
            Platform::DivUp(Platform::DivUp(constraint_apply_segments_constant.segment_size, skip_size),
                            work_group_invocations),
            1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      };

      const auto correct_connections = [&]() {
        connections_pipeline->Bind(vk_command_buffer);
        connections_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        connections_pipeline->PushConstant(vk_command_buffer, 0, constraint_apply_connections_constant);
        connections_pipeline->Dispatch(
            vk_command_buffer,
            Platform::DivUp(constraint_apply_connections_constant.segment_pair_size, work_group_invocations), 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      };
      const auto calculate_stretch_shear_offset = [&](const uint32_t skip_index) {
        stretch_shear_pipeline->Bind(vk_command_buffer);
        stretch_shear_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        stretch_shear_constraint_constant.skip_index = skip_index;
        stretch_shear_pipeline->PushConstant(vk_command_buffer, 0, stretch_shear_constraint_constant);
        stretch_shear_pipeline->Dispatch(
            vk_command_buffer,
            Platform::DivUp(Platform::DivUp(stretch_shear_constraint_constant.segment_size, skip_size),
                            work_group_invocations),
            1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      };
      const auto calculate_bend_twist_offset = [&](const uint32_t skip_index) {
        bend_twist_pipeline->Bind(vk_command_buffer);
        bend_twist_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

        bend_twist_constraint_constant.skip_index = skip_index;
        bend_twist_pipeline->PushConstant(vk_command_buffer, 0, bend_twist_constraint_constant);
        bend_twist_pipeline->Dispatch(
            vk_command_buffer,
            Platform::DivUp(Platform::DivUp(bend_twist_constraint_constant.segment_size, skip_size),
                            work_group_invocations),
            1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      };

      const auto bundle_position = [&](const uint32_t skip_index) {
        bundle_position_pipeline->Bind(vk_command_buffer);
        bundle_position_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        constraint_constant.skip_index = skip_index;
        bundle_position_pipeline->PushConstant(vk_command_buffer, 0, constraint_constant);
        bundle_position_pipeline->Dispatch(
            vk_command_buffer,
            Platform::DivUp(Platform::DivUp(constraint_constant.segment_size, skip_size), work_group_invocations), 1,
            1);

        Platform::EverythingBarrier(vk_command_buffer);
      };

      const auto bundle_rotation = [&](const uint32_t skip_index) {
        bundle_rotation_pipeline->Bind(vk_command_buffer);
        bundle_rotation_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        constraint_constant.skip_index = skip_index;
        bundle_rotation_pipeline->PushConstant(vk_command_buffer, 0, constraint_constant);
        bundle_rotation_pipeline->Dispatch(
            vk_command_buffer,
            Platform::DivUp(Platform::DivUp(constraint_constant.segment_size, skip_size), work_group_invocations), 1,
            1);

        Platform::EverythingBarrier(vk_command_buffer);
      };

      for (uint32_t skip_index = 0; skip_index < skip_size; skip_index++) {
        if (enable_bundle_rotation && bundle_rotation_pipeline && bundle_rotation_pipeline->Initialized()) {
          bundle_rotation(skip_index);
          apply_rotations(skip_index);
        }
        if (enable_bundle_position && bundle_position_pipeline && bundle_position_pipeline->Initialized()) {
          bundle_position(skip_index);
          apply_positions(skip_index);
        }
        if (enable_bend_twist && bend_twist_pipeline && bend_twist_pipeline->Initialized()) {
          calculate_bend_twist_offset(skip_index);
          apply_rotations(skip_index);
        }
        if (enable_stretch_shear && stretch_shear_pipeline && stretch_shear_pipeline->Initialized()) {
          calculate_stretch_shear_offset(skip_index);
          apply_both(skip_index);
        }
      }
      if (enable_connections && connections_pipeline && connections_pipeline->Initialized())
        correct_connections();
    }
  });
}

bool DsBundle::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("Random Bundle")) {
    if (ImGui::Checkbox("Enable", &enabled))
      changed = true;
    if (enabled) {
      if (ImGui::Checkbox("Enable bundle position", &enable_bundle_position))
        changed = true;
      if (ImGui::Checkbox("Enable bundle rotation", &enable_bundle_rotation))
        changed = true;
      if (ImGui::Checkbox("Enable bend twist", &enable_bend_twist))
        changed = true;
      if (ImGui::Checkbox("Enable stretch shear", &enable_stretch_shear))
        changed = true;
      if (ImGui::Checkbox("Enable connections", &enable_connections))
        changed = true;
    }
    if (ImGui::DragInt("Sub iteration", &sub_iteration, 1, 1, 100))
      changed = true;
    if (ImGui::DragInt("Skip size", &skip_size, 1, 1, 100))
      changed = true;
    if (ImGui::DragFloat("Over relaxation", &over_relaxation, 0.01f, 1, 10.f))
      changed = true;
    if (ImGui::DragFloat("Bend Twist over relaxation", &bend_twist_over_relaxation, 0.01f, 1, 10.f))
      changed = true;

    if (ImGui::Button("Recompile")) {
      {
        static std::shared_ptr<Shader> shader{};
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateShearStretchCorrections.comp");
        stretch_shear_pipeline = std::make_shared<ComputePipeline>();
        stretch_shear_pipeline->compute_shader = shader;
        stretch_shear_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = stretch_shear_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(RandomBundleShearStretchConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        stretch_shear_pipeline->Initialize();
      }

      {
        static std::shared_ptr<Shader> shader{};
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBendTwistCorrections.comp");
        bend_twist_pipeline = std::make_shared<ComputePipeline>();
        bend_twist_pipeline->compute_shader = shader;
        bend_twist_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = bend_twist_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(RandomBundleBendTwistConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        bend_twist_pipeline->Initialize();
      }

      {
        static std::shared_ptr<Shader> shader{};
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBundlePositionCorrections.comp");
        bundle_position_pipeline = std::make_shared<ComputePipeline>();
        bundle_position_pipeline->compute_shader = shader;
        bundle_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = bundle_position_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(RandomBundleConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        bundle_position_pipeline->Initialize();
      }

      {
        static std::shared_ptr<Shader> shader{};
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBundleRotationCorrections.comp");
        bundle_rotation_pipeline = std::make_shared<ComputePipeline>();
        bundle_rotation_pipeline->compute_shader = shader;
        bundle_rotation_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = bundle_rotation_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(RandomBundleConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        bundle_rotation_pipeline->Initialize();
      }

      {
        static std::shared_ptr<Shader> shader{};
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyRotationCorrections.comp");

        apply_rotation_pipeline = std::make_shared<ComputePipeline>();
        apply_rotation_pipeline->compute_shader = shader;

        apply_rotation_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = apply_rotation_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(RandomBundleApplySegmentsConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

        apply_rotation_pipeline->Initialize();
      }

      {
        static std::shared_ptr<Shader> shader{};
        shader = std::make_shared<Shader>();
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyPositionCorrections.comp");

        apply_position_pipeline = std::make_shared<ComputePipeline>();
        apply_position_pipeline->compute_shader = shader;

        apply_position_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = apply_position_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(RandomBundleApplySegmentsConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

        apply_position_pipeline->Initialize();
      }

      {
        static std::shared_ptr<Shader> shader{};
        shader = std::make_shared<Shader>();
        shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                           std::filesystem::path("./EcoSysLabResources") /
                               "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyCorrections.comp");

        apply_position_rotation_pipeline = std::make_shared<ComputePipeline>();
        apply_position_rotation_pipeline->compute_shader = shader;

        apply_position_rotation_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

        auto& push_constant_range = apply_position_rotation_pipeline->push_constant_ranges.emplace_back();
        push_constant_range.size = sizeof(RandomBundleApplySegmentsConstant);
        push_constant_range.offset = 0;
        push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

        apply_position_rotation_pipeline->Initialize();
      }

      {
        static std::shared_ptr<Shader> shader{};
        shader = std::make_shared<Shader>();
        shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                           std::filesystem::path("./EcoSysLabResources") /
                               "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ConnectionCorrections.comp");

        connections_pipeline = std::make_shared<ComputePipeline>();
        connections_pipeline->compute_shader = shader;
        connections_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
        auto& stretch_shear_push_constant_range = connections_pipeline->push_constant_ranges.emplace_back();
        stretch_shear_push_constant_range.size = sizeof(RandomBundleApplyConnectionsConstant);
        stretch_shear_push_constant_range.offset = 0;
        stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
        connections_pipeline->Initialize();
      }
    }
    ImGui::TreePop();
  }
  return changed;
}

DsLeafAttachment::DsLeafAttachment() {
  static std::shared_ptr<Shader> shader{};
  shader = std::make_shared<Shader>();
  shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                     std::filesystem::path("./EcoSysLabResources") /
                         "Shaders/Compute/DynamicStrands/Constraints/Position/LeafAttachment.comp");

  pipeline = std::make_shared<ComputePipeline>();
  pipeline->compute_shader = shader;
  pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  auto& stretch_shear_push_constant_range = pipeline->push_constant_ranges.emplace_back();
  stretch_shear_push_constant_range.size = sizeof(LeafPredictionPushConstant);
  stretch_shear_push_constant_range.offset = 0;
  stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  pipeline->Initialize();
}

void DsLeafAttachment::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                                 const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  LeafPredictionPushConstant push_constant;
  push_constant.leaf_size = target_dynamic_strands.foliage.size();
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->PushConstant(vk_command_buffer, 0, push_constant);
    pipeline->Dispatch(vk_command_buffer, Platform::DivUp(push_constant.leaf_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}