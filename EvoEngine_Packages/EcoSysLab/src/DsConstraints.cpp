#include "DsConstraints.hpp"
#include "DynamicStrandsProfiler.hpp"
#include "GpuProfiler.hpp"
#include "Shader.hpp"
#include "VoxelGrid.hpp"
using namespace eco_sys_lab_package;

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
                           "Shaders/Compute/DynamicStrands/Constraints/Position/PivotPoint.slang");

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
                           "Shaders/Compute/DynamicStrands/Constraints/Position/PivotAxis.slang");

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
                           "Shaders/Compute/DynamicStrands/Constraints/Position/PivotTransform.slang");

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
                                         "Shaders/Compute/DynamicStrands/Constraints/Position/StiffRod.slang");
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

bool DsStiffRod::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
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
  if (!coupled_layout) {
    coupled_layout = std::make_shared<DescriptorSetLayout>();
    for (uint32_t binding = 0; binding < 12; ++binding)
      coupled_layout->PushDescriptorBinding(binding, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    coupled_layout->Initialize();
  }
  const auto initialize_coupled_pipeline = [](std::shared_ptr<ComputePipeline>& pipeline, const char* shader_name) {
    if (pipeline)
      return;
    const auto shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources/Shaders/Compute/DynamicStrands/Constraints/Position/Bundle") /
            shader_name);
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts = {DynamicStrands::strands_layout, coupled_layout};
    auto& range = pipeline->push_constant_ranges.emplace_back();
    range.size = sizeof(CoupledPairConstant);
    range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    pipeline->Initialize();
  };
  initialize_coupled_pipeline(coupled_pair_pipeline, "SolveCoupledPairs.slang");
  initialize_coupled_pipeline(coupled_gather_pipeline, "GatherCoupledPairs.slang");
  initialize_coupled_pipeline(slice_key_pipeline, "BuildSliceMembers.slang");
  initialize_coupled_pipeline(slice_sort_pipeline, "SortSliceMembers.slang");
  initialize_coupled_pipeline(slice_range_pipeline, "BuildSliceRanges.slang");
  initialize_coupled_pipeline(slice_fit_pipeline, "FitSlices.slang");
  initialize_coupled_pipeline(slice_apply_pipeline, "ApplySlices.slang");
  initialize_coupled_pipeline(coarse_key_pipeline, "BuildCoarseEdges.slang");
  initialize_coupled_pipeline(coarse_sort_pipeline, "SortCoarseEdges.slang");
  initialize_coupled_pipeline(coarse_reduce_pipeline, "ReduceCoarseEdges.slang");
  initialize_coupled_pipeline(coarse_solve_pipeline, "SolveCoarseEdges.slang");

  if (!stretch_shear_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") /
            "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateShearStretchCorrections.slang");
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
            "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBendTwistCorrections.slang");
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
            "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBundlePositionCorrections.slang");
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
            "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBundleRotationCorrections.slang");
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
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyRotationCorrections.slang");

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
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyPositionCorrections.slang");

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
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyCorrections.slang");

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
                           "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ConnectionCorrections.slang");

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

void DsBundle::InitializeData(const DynamicStrandsInitializeParameters& initialize_parameters,
                              const StrandModelSkeleton& strand_model_skeleton,
                              const DtsStrandGroup& subdivided_strand_group,
                              const DynamicStrands& target_dynamic_strands) {
  solver_settings = initialize_parameters.bundle_solver;
  sub_iteration = solver_settings.legacy_iterations;
  if (solver_settings.mode == BundleSolverMode::Legacy)
    return;
  VkBufferCreateInfo buffer_info{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
  buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_info.size = 1;
  VmaAllocationCreateInfo allocation_info{};
  allocation_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  coupled_pair_state_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  coupled_pair_correction_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  coupled_pair_state_buffer->UploadVector(std::vector<CoupledPairState>(target_dynamic_strands.segment_pairs.size()));
  coupled_pair_correction_buffer->UploadVector(
      std::vector<CoupledPairCorrection>(target_dynamic_strands.segment_pairs.size()));
  float average_length = 0.f;
  for (const auto& segment : target_dynamic_strands.segments)
    average_length += segment.rest_length;
  average_length /= glm::max(static_cast<float>(target_dynamic_strands.segments.size()), 1.f);
  const float spacing = glm::max(average_length * solver_settings.slice_spacing_factor, 1e-6f);
  std::map<std::pair<int32_t, int32_t>, uint32_t> base_slice_indices;
  std::vector<uint32_t> base_slices(target_dynamic_strands.segments.size());
  for (size_t segment_index = 0; segment_index < target_dynamic_strands.segments.size(); ++segment_index) {
    const auto& segment = target_dynamic_strands.segments[segment_index];
    const float root_distance = .5f * (segment.particle0.root_distance + segment.particle1.root_distance);
    const auto key = std::make_pair(segment.node_handle, static_cast<int32_t>(glm::floor(root_distance / spacing)));
    const auto [iterator, inserted] =
        base_slice_indices.try_emplace(key, static_cast<uint32_t>(base_slice_indices.size()));
    base_slices[segment_index] = iterator->second;
  }
  slice_padded_count = 1;
  while (slice_padded_count < target_dynamic_strands.segments.size())
    slice_padded_count *= 2;
  coarse_padded_count = 1;
  while (coarse_padded_count < target_dynamic_strands.connection_segment_pair_size)
    coarse_padded_count *= 2;
  buffer_info.usage |= VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT;
  base_slice_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  slice_member_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  slice_range_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  segment_slice_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  slice_transform_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  slice_count_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  slice_dispatch_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  coarse_candidate_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  coarse_edge_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  coarse_edge_count_buffer = std::make_shared<Buffer>(buffer_info, allocation_info);
  base_slice_buffer->UploadVector(base_slices);
  slice_member_buffer->UploadVector(std::vector<SliceMember>(slice_padded_count));
  slice_range_buffer->UploadVector(std::vector<SliceRange>(target_dynamic_strands.segments.size()));
  segment_slice_buffer->UploadVector(std::vector<uint32_t>(target_dynamic_strands.segments.size()));
  slice_transform_buffer->UploadVector(std::vector<SliceTransform>(target_dynamic_strands.segments.size()));
  slice_count_buffer->UploadVector(std::vector<uint32_t>(1));
  slice_dispatch_buffer->UploadVector(std::vector<VkDispatchIndirectCommand>(1, {0, 1, 1}));
  coarse_candidate_buffer->UploadVector(std::vector<CoarseEdgeCandidate>(coarse_padded_count));
  coarse_edge_buffer->UploadVector(std::vector<CoarseEdge>(target_dynamic_strands.connection_segment_pair_size));
  coarse_edge_count_buffer->UploadVector(std::vector<uint32_t>(1));
  coupled_descriptor_sets.resize(Platform::GetMaxFramesInFlight());
  for (auto& descriptor_set : coupled_descriptor_sets) {
    descriptor_set = std::make_shared<DescriptorSet>(coupled_layout);
    descriptor_set->UpdateBufferDescriptorBinding(0, coupled_pair_state_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(1, coupled_pair_correction_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(2, base_slice_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(3, slice_member_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(4, slice_range_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(5, segment_slice_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(6, slice_transform_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(7, slice_count_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(8, slice_dispatch_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(9, coarse_candidate_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(10, coarse_edge_buffer);
    descriptor_set->UpdateBufferDescriptorBinding(11, coarse_edge_count_buffer);
  }
}

void DsBundle::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                         const DynamicStrands& target_dynamic_strands) {
  if (target_dynamic_strands.segment_pairs.empty())
    return;
  if (solver_settings.mode != BundleSolverMode::Legacy) {
    const auto frame = Platform::GetCurrentFrameIndex();
    const auto work_group_size = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
    if (solver_settings.mode == BundleSolverMode::Hybrid) {
      SliceConstant slice_constant;
      slice_constant.segment_count = static_cast<uint32_t>(target_dynamic_strands.segments.size());
      slice_constant.padded_count = slice_padded_count;
      slice_constant.minimum_members = static_cast<uint32_t>(solver_settings.minimum_slice_members);
      slice_constant.shape_matching_strength = solver_settings.shape_matching_strength;
      if (last_slice_frame != target_dynamic_strands.GetFrameIndex()) {
        last_slice_frame = target_dynamic_strands.GetFrameIndex();
        const RecordedGpuProfilerScope topology_scope(dynamic_strands_profiler::GetItems().bundle_topology_rebuild);
        Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command_buffer) {
          slice_key_pipeline->Bind(command_buffer);
          slice_key_pipeline->BindDescriptorSet(
              command_buffer, 0, target_dynamic_strands.strands_descriptor_sets[frame]->GetVkDescriptorSet());
          slice_key_pipeline->BindDescriptorSet(command_buffer, 1,
                                                coupled_descriptor_sets[frame]->GetVkDescriptorSet());
          slice_key_pipeline->PushConstant(command_buffer, 0, slice_constant);
          slice_key_pipeline->Dispatch(command_buffer, Platform::DivUp(slice_padded_count, work_group_size), 1, 1);
          Platform::EverythingBarrier(command_buffer);
          for (uint32_t stage = 2; stage <= slice_padded_count; stage *= 2) {
            slice_constant.sort_stage = stage;
            for (uint32_t pass = stage / 2; pass > 0; pass /= 2) {
              slice_constant.sort_pass = pass;
              slice_sort_pipeline->Bind(command_buffer);
              slice_sort_pipeline->BindDescriptorSet(command_buffer, 1,
                                                     coupled_descriptor_sets[frame]->GetVkDescriptorSet());
              slice_sort_pipeline->PushConstant(command_buffer, 0, slice_constant);
              slice_sort_pipeline->Dispatch(command_buffer, Platform::DivUp(slice_padded_count, work_group_size), 1, 1);
              Platform::EverythingBarrier(command_buffer);
            }
          }
          slice_range_pipeline->Bind(command_buffer);
          slice_range_pipeline->BindDescriptorSet(command_buffer, 1,
                                                  coupled_descriptor_sets[frame]->GetVkDescriptorSet());
          slice_range_pipeline->PushConstant(command_buffer, 0, slice_constant);
          slice_range_pipeline->Dispatch(command_buffer, 1, 1, 1);
          Platform::EverythingBarrier(command_buffer);
          CoarseConstant coarse_constant;
          coarse_constant.direct_pair_count = target_dynamic_strands.connection_segment_pair_size;
          coarse_constant.padded_count = coarse_padded_count;
          coarse_key_pipeline->Bind(command_buffer);
          coarse_key_pipeline->BindDescriptorSet(
              command_buffer, 0, target_dynamic_strands.strands_descriptor_sets[frame]->GetVkDescriptorSet());
          coarse_key_pipeline->BindDescriptorSet(command_buffer, 1,
                                                 coupled_descriptor_sets[frame]->GetVkDescriptorSet());
          coarse_key_pipeline->PushConstant(command_buffer, 0, coarse_constant);
          coarse_key_pipeline->Dispatch(command_buffer, Platform::DivUp(coarse_padded_count, work_group_size), 1, 1);
          Platform::EverythingBarrier(command_buffer);
          for (uint32_t stage = 2; stage <= coarse_padded_count; stage *= 2) {
            coarse_constant.sort_stage = stage;
            for (uint32_t pass = stage / 2; pass > 0; pass /= 2) {
              coarse_constant.sort_pass = pass;
              coarse_sort_pipeline->Bind(command_buffer);
              coarse_sort_pipeline->BindDescriptorSet(command_buffer, 1,
                                                      coupled_descriptor_sets[frame]->GetVkDescriptorSet());
              coarse_sort_pipeline->PushConstant(command_buffer, 0, coarse_constant);
              coarse_sort_pipeline->Dispatch(command_buffer, Platform::DivUp(coarse_padded_count, work_group_size), 1,
                                             1);
              Platform::EverythingBarrier(command_buffer);
            }
          }
          coarse_reduce_pipeline->Bind(command_buffer);
          coarse_reduce_pipeline->BindDescriptorSet(
              command_buffer, 0, target_dynamic_strands.strands_descriptor_sets[frame]->GetVkDescriptorSet());
          coarse_reduce_pipeline->BindDescriptorSet(command_buffer, 1,
                                                    coupled_descriptor_sets[frame]->GetVkDescriptorSet());
          coarse_reduce_pipeline->PushConstant(command_buffer, 0, coarse_constant);
          coarse_reduce_pipeline->Dispatch(command_buffer, 1, 1, 1);
          Platform::EverythingBarrier(command_buffer);
        });
      }
      const RecordedGpuProfilerScope fit_scope(dynamic_strands_profiler::GetItems().bundle_slice_fit_apply);
      Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command_buffer) {
        slice_fit_pipeline->Bind(command_buffer);
        slice_fit_pipeline->BindDescriptorSet(
            command_buffer, 0, target_dynamic_strands.strands_descriptor_sets[frame]->GetVkDescriptorSet());
        slice_fit_pipeline->BindDescriptorSet(command_buffer, 1, coupled_descriptor_sets[frame]->GetVkDescriptorSet());
        slice_fit_pipeline->PushConstant(command_buffer, 0, slice_constant);
        slice_fit_pipeline->DispatchIndirect(command_buffer, *slice_dispatch_buffer);
        Platform::EverythingBarrier(command_buffer);
        CoarseConstant coarse_constant;
        coarse_constant.direct_pair_count = target_dynamic_strands.connection_segment_pair_size;
        coarse_constant.padded_count = coarse_padded_count;
        coarse_constant.inverse_time_step_squared = 1.f / physics_parameters.time_step / physics_parameters.time_step;
        coarse_constant.position_compliance_scale = solver_settings.position_compliance_scale;
        coarse_constant.bending_compliance_scale = solver_settings.bending_compliance_scale;
        coarse_constant.torsion_compliance_scale = solver_settings.torsion_compliance_scale;
        {
          const RecordedGpuProfilerScope coarse_scope(dynamic_strands_profiler::GetItems().bundle_coarse_edge_solve);
          coarse_solve_pipeline->Bind(command_buffer);
          coarse_solve_pipeline->BindDescriptorSet(command_buffer, 1,
                                                   coupled_descriptor_sets[frame]->GetVkDescriptorSet());
          coarse_solve_pipeline->PushConstant(command_buffer, 0, coarse_constant);
          for (int iteration = 0; iteration < solver_settings.coarse_iterations; ++iteration) {
            coarse_solve_pipeline->Dispatch(command_buffer, 1, 1, 1);
            Platform::EverythingBarrier(command_buffer);
          }
        }
        slice_apply_pipeline->Bind(command_buffer);
        slice_apply_pipeline->BindDescriptorSet(
            command_buffer, 0, target_dynamic_strands.strands_descriptor_sets[frame]->GetVkDescriptorSet());
        slice_apply_pipeline->BindDescriptorSet(command_buffer, 1,
                                                coupled_descriptor_sets[frame]->GetVkDescriptorSet());
        slice_apply_pipeline->PushConstant(command_buffer, 0, slice_constant);
        slice_apply_pipeline->Dispatch(command_buffer, Platform::DivUp(slice_constant.segment_count, work_group_size),
                                       1, 1);
        Platform::EverythingBarrier(command_buffer);
      });
    }
    CoupledPairConstant constant;
    constant.pair_begin = target_dynamic_strands.connection_segment_pair_size;
    constant.pair_count = static_cast<uint32_t>(target_dynamic_strands.segment_pairs.size()) - constant.pair_begin;
    constant.segment_count = static_cast<uint32_t>(target_dynamic_strands.segments.size());
    constant.inverse_time_step_squared = glm::pow(physics_parameters.sub_step / physics_parameters.time_step, 2.f);
    constant.position_compliance_scale = solver_settings.position_compliance_scale;
    constant.bending_compliance_scale = solver_settings.bending_compliance_scale;
    constant.torsion_compliance_scale = solver_settings.torsion_compliance_scale;
    const RecordedGpuProfilerScope gpu_scope(dynamic_strands_profiler::GetItems().bundle_pair_solve);
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer command_buffer) {
      for (int iteration = 0; constant.pair_count > 0 && iteration < solver_settings.pair_iterations; ++iteration) {
        constant.reset_lambdas = iteration == 0;
        coupled_pair_pipeline->Bind(command_buffer);
        coupled_pair_pipeline->BindDescriptorSet(
            command_buffer, 0, target_dynamic_strands.strands_descriptor_sets[frame]->GetVkDescriptorSet());
        coupled_pair_pipeline->BindDescriptorSet(command_buffer, 1,
                                                 coupled_descriptor_sets[frame]->GetVkDescriptorSet());
        coupled_pair_pipeline->PushConstant(command_buffer, 0, constant);
        coupled_pair_pipeline->Dispatch(command_buffer, Platform::DivUp(constant.pair_count, work_group_size), 1, 1);
        Platform::EverythingBarrier(command_buffer);
        coupled_gather_pipeline->Bind(command_buffer);
        coupled_gather_pipeline->BindDescriptorSet(
            command_buffer, 0, target_dynamic_strands.strands_descriptor_sets[frame]->GetVkDescriptorSet());
        coupled_gather_pipeline->BindDescriptorSet(command_buffer, 1,
                                                   coupled_descriptor_sets[frame]->GetVkDescriptorSet());
        coupled_gather_pipeline->PushConstant(command_buffer, 0, constant);
        coupled_gather_pipeline->Dispatch(command_buffer, Platform::DivUp(constant.segment_count, work_group_size), 1,
                                          1);
        Platform::EverythingBarrier(command_buffer);
      }
      if (enable_connections && connections_pipeline && connections_pipeline->Initialized()) {
        RandomBundleApplyConnectionsConstant connection_constant;
        connection_constant.segment_pair_size = target_dynamic_strands.connection_segment_pair_size;
        connection_constant.inv_time_step = physics_parameters.sub_step / physics_parameters.time_step;
        connections_pipeline->Bind(command_buffer);
        connections_pipeline->BindDescriptorSet(
            command_buffer, 0, target_dynamic_strands.strands_descriptor_sets[frame]->GetVkDescriptorSet());
        connections_pipeline->PushConstant(command_buffer, 0, connection_constant);
        connections_pipeline->Dispatch(command_buffer,
                                       Platform::DivUp(connection_constant.segment_pair_size, work_group_size), 1, 1);
        Platform::EverythingBarrier(command_buffer);
      }
    });
    return;
  }
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

  const RecordedGpuProfilerScope gpu_scope(dynamic_strands_profiler::GetItems().bundle_legacy);
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

bool DsBundle::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("Random Bundle")) {
    constexpr const char* mode_names[] = {"Legacy", "Coupled XPBD", "Hybrid"};
    ImGui::Text("Solver mode: %s", mode_names[static_cast<int>(solver_settings.mode)]);
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
    if (ImGui::DragInt("Sub iteration", &sub_iteration, 1, 1, 100)) {
      solver_settings.legacy_iterations = sub_iteration;
      changed = true;
    }
    if (ImGui::DragInt("Pair iterations", &solver_settings.pair_iterations, 1, 1, 100))
      changed = true;
    if (ImGui::DragInt("Coarse iterations", &solver_settings.coarse_iterations, 1, 1, 100))
      changed = true;
    if (ImGui::DragFloat("Position compliance scale", &solver_settings.position_compliance_scale, 0.01f, 0.f, 100.f))
      changed = true;
    if (ImGui::DragFloat("Bending compliance scale", &solver_settings.bending_compliance_scale, 0.01f, 0.f, 100.f))
      changed = true;
    if (ImGui::DragFloat("Torsion compliance scale", &solver_settings.torsion_compliance_scale, 0.01f, 0.f, 100.f))
      changed = true;
    if (ImGui::SliderFloat("Shape matching strength", &solver_settings.shape_matching_strength, 0.f, 1.f))
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
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateShearStretchCorrections.slang");
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
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBendTwistCorrections.slang");
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
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBundlePositionCorrections.slang");
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
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/CalculateBundleRotationCorrections.slang");
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
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyRotationCorrections.slang");

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
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyPositionCorrections.slang");

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
                               "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ApplyCorrections.slang");

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
        shader->TryCompile(
            ShaderType::Compute, Platform::GetShaderGlobalDefines(),
            std::filesystem::path("./EcoSysLabResources") /
                "Shaders/Compute/DynamicStrands/Constraints/Position/Bundle/ConnectionCorrections.slang");

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
                         "Shaders/Compute/DynamicStrands/Constraints/Position/LeafAttachment.slang");

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
