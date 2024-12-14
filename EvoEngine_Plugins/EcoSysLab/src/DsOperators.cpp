#include "DsOperators.hpp"
#include "Shader.hpp"
using namespace eco_sys_lab_plugin;

DsTransform::DsTransform() {
  if (!position_layout) {
    position_layout = std::make_shared<DescriptorSetLayout>();
    position_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    position_layout->Initialize();
  }
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  position_commands_buffer.resize(max_frame_in_flight);

  for (int frame_index = 0; frame_index < max_frame_in_flight; frame_index++) {
    position_commands_buffer[frame_index] =
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
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
    position_update_pipeline->descriptor_set_layouts.emplace_back(position_layout);

    auto& push_constant_range = position_update_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PositionUpdatePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    position_update_pipeline->Initialize();
  }
  position_commands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : position_commands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(position_layout);
  }

  if (!rotation_layout) {
    rotation_layout = std::make_shared<DescriptorSetLayout>();
    rotation_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    rotation_layout->Initialize();
  }
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  rotation_commands_buffer.resize(max_frame_in_flight);

  for (int frame_index = 0; frame_index < max_frame_in_flight; frame_index++) {
    rotation_commands_buffer[frame_index] =
        std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
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
    rotation_update_pipeline->descriptor_set_layouts.emplace_back(rotation_layout);

    auto& push_constant_range = rotation_update_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RotationUpdatePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    rotation_update_pipeline->Initialize();
  }

  rotation_commands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : rotation_commands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(rotation_layout);
  }
}

void DsTransform::Initialize(const GlobalTransform& target_base_global_transform,
                             const std::shared_ptr<DynamicStrands>& target_dynamic_strands,
                             const std::vector<uint32_t>& segment_handles) {
  base_global_transform = target_base_global_transform;
  inverse_base_global_transform.value = glm::inverse(base_global_transform.value);
  position_commands.resize(segment_handles.size() * 2);
  rotation_commands.resize(segment_handles.size());

  Jobs::RunParallelFor(segment_handles.size(), [&](const size_t i) {
    const auto& segment_handle = segment_handles[i];
    const auto& segment = target_dynamic_strands->segments[segment_handle];
    position_commands[i * 2].particle_index = segment.particle0_handle;
    position_commands[i * 2].new_position = target_dynamic_strands->particles[segment.particle0_handle].x0;
    position_commands[i * 2 + 1].particle_index = segment.particle1_handle;
    position_commands[i * 2 + 1].new_position = target_dynamic_strands->particles[segment.particle1_handle].x0;

    rotation_commands[i].segment_index = segment_handle;
    rotation_commands[i].new_rotation = segment.q0;
  });
}

void DsTransform::Update(const GlobalTransform& new_global_transform,
                         const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  const glm::quat rotation = new_global_transform.GetRotation() * inverse_base_global_transform.GetRotation();
  Jobs::RunParallelFor(rotation_commands.size(), [&](const size_t i) {
    auto& position_command0 = position_commands[2 * i];
    auto& position_command1 = position_commands[2 * i + 1];
    auto& rotation_command = rotation_commands[i];
    position_command0.new_position = new_global_transform.TransformPoint(inverse_base_global_transform.TransformPoint(
        target_dynamic_strands->particles[position_command0.particle_index].x0));
    position_command1.new_position = new_global_transform.TransformPoint(inverse_base_global_transform.TransformPoint(
        target_dynamic_strands->particles[position_command1.particle_index].x0));
    rotation_command.new_rotation = rotation * target_dynamic_strands->segments[rotation_command.segment_index].q0;
  });
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  if (!position_commands.empty()) {
    position_commands_buffer[current_frame_index]->UploadVector(position_commands);
    position_commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
        0, position_commands_buffer[current_frame_index]);
  }
  if (!rotation_commands.empty()) {
    rotation_commands_buffer[current_frame_index]->UploadVector(rotation_commands);
    rotation_commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(
        0, rotation_commands_buffer[current_frame_index]);
  }
}

void DsTransform::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                          const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  if (!position_commands.empty()) {
    PositionUpdatePushConstant push_constant;
    push_constant.commands_size = position_commands.size();
    const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      position_update_pipeline->Bind(vk_command_buffer);
      position_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      position_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, position_commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      position_update_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.commands_size, work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
  }
  if (!rotation_commands.empty()) {
    RotationUpdatePushConstant push_constant;
    push_constant.commands_size = rotation_commands.size();
    const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      rotation_update_pipeline->Bind(vk_command_buffer);
      rotation_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      rotation_update_pipeline->BindDescriptorSet(
          vk_command_buffer, 1, rotation_commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      rotation_update_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.commands_size, work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
  }
}

void DsGravity::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                        const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  GravityPushConstant push_constant;
  push_constant.acceleration = gravity;
  push_constant.ground_height = ground_height;
  push_constant.particle_size = target_dynamic_strands->particles.size();
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    gravity_force_pipeline->Bind(vk_command_buffer);
    gravity_force_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    gravity_force_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.particle_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

bool DsGravity::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Enable", &enabled))
    changed = true;
  if (ImGui::DragFloat3("Gravity", &gravity.x, 0.01f, -100.0f, 100.0f))
    changed = true;

  if (ImGui::DragFloat("Ground height", &ground_height, 0.01f, -100.0f, 100.0f))
    changed = true;
  ImGui::TreePop();

  return changed;
}

DsGravity::DsGravity() {
  if (!gravity_force_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/Gravity.comp");
    gravity_force_pipeline = std::make_shared<ComputePipeline>();
    gravity_force_pipeline->compute_shader = shader;
    gravity_force_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = gravity_force_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GravityPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    gravity_force_pipeline->Initialize();
  }
}

DsAttraction::DsAttraction() {
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

  if (!drag_force_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/Attraction.comp");
    drag_force_pipeline = std::make_shared<ComputePipeline>();
    drag_force_pipeline->compute_shader = shader;

    drag_force_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    drag_force_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& push_constant_range = drag_force_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(AttractionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    drag_force_pipeline->Initialize();
  }
  commands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : commands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(layout);
  }
}

void DsAttraction::Initialize(const std::vector<int>& particle_handles) {
  commands = particle_handles;
}

void DsAttraction::Update(const glm::vec3& new_position) {
  target_position = new_position;
}

void DsAttraction::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                           const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  if (commands.empty())
    return;

  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  commands_buffer[current_frame_index]->UploadVector(commands);
  commands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, commands_buffer[current_frame_index]);
  AttractionPushConstant push_constant;
  push_constant.target_position = target_position;
  push_constant.distance_multiplier = distance_multiplier;
  push_constant.commands_size = commands.size();
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    drag_force_pipeline->Bind(vk_command_buffer);
    drag_force_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    drag_force_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    drag_force_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.commands_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

bool DsAttraction::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Multiplier", &distance_multiplier, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  return changed;
}

DsBoxSelection::DsBoxSelection() {
  if (!pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/BoxSelection.comp");
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(BoxSelectionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    pipeline->Initialize();
  }
}

void DsBoxSelection::Update(const glm::vec2& box_start, const glm::vec2& box_end, const glm::mat4& projection_view,
                            const uint32_t selection_mode) {
  push_constant.box_min = glm::min(box_start, box_end);
  push_constant.box_max = glm::max(box_start, box_end);

  push_constant.projection_view = projection_view;
  push_constant.selection_mode = selection_mode;
}

void DsBoxSelection::Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  push_constant.particle_size = target_dynamic_strands->particles.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(target_dynamic_strands->particles.size(), work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

DsDrag::DsDrag() {
  if (!pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/Drag.comp");
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(DragPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    pipeline->Initialize();
  }
}

void DsDrag::Update(const glm::vec3& acceleration) {
  target_acceleration = acceleration;
}

void DsDrag::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                     const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  DragPushConstant push_constant;
  push_constant.acceleration = target_acceleration;
  push_constant.particle_size = target_dynamic_strands->particles.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(target_dynamic_strands->particles.size(), work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}
