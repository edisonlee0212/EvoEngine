#include "DsOperators.hpp"
#include "Shader.hpp"
using namespace eco_sys_lab_plugin;

void DsGravity::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                        const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  SegmentGravityPushConstant segment_push_constant;
  segment_push_constant.acceleration = gravity;
  segment_push_constant.ground_height = ground_height;
  segment_push_constant.segment_size = target_dynamic_strands->segments.size();

  LeafGravityPushConstant leaf_push_constant;
  leaf_push_constant.acceleration = gravity;
  leaf_push_constant.ground_height = ground_height;
  leaf_push_constant.leaf_size = target_dynamic_strands->foliage.size();

  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_gravity_force_pipeline->Bind(vk_command_buffer);
    segment_gravity_force_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_gravity_force_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    leaf_gravity_force_pipeline->Bind(vk_command_buffer);
    leaf_gravity_force_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    leaf_gravity_force_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1, 1);
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

  return changed;
}

DsGravity::DsGravity() {
  if (!segment_gravity_force_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/SegmentGravity.comp");
    segment_gravity_force_pipeline = std::make_shared<ComputePipeline>();
    segment_gravity_force_pipeline->compute_shader = shader;
    segment_gravity_force_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_gravity_force_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentGravityPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_gravity_force_pipeline->Initialize();
  }

  if (!leaf_gravity_force_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/LeafGravity.comp");
    leaf_gravity_force_pipeline = std::make_shared<ComputePipeline>();
    leaf_gravity_force_pipeline->compute_shader = shader;
    leaf_gravity_force_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = leaf_gravity_force_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafGravityPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_gravity_force_pipeline->Initialize();
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
    shader->TryCompile(
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
    shader->TryCompile(
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
  push_constant.segment_size = target_dynamic_strands->segments.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
  enabled = false;
}

DsDrag::DsDrag() {
  if (!pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
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
  push_constant.segment_size = target_dynamic_strands->segments.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
  enabled = false;
}

DsLineCut::DsLineCut() {
  if (!pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/LineCut.comp");
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LineCutPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    pipeline->Initialize();
  }
}

void DsLineCut::Update(const glm::vec2& line_start, const glm::vec2& line_end, const glm::mat4& projection_view,
                       const unsigned cut_mode) {
  push_constant.line_start = line_start;
  push_constant.line_end = line_end;
  push_constant.projection_view = projection_view;
  push_constant.cut_mode = cut_mode;
}

void DsLineCut::Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  push_constant.segment_pair_size = target_dynamic_strands->segment_pairs.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer,
                  Platform::DivUp(target_dynamic_strands->segment_pairs.size(), work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
  enabled = false;
}

DsSaw::DsSaw() {
  if (!layout) {
    layout = std::make_shared<DescriptorSetLayout>();
    layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    layout->Initialize();
  }
  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();

  line_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : line_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(layout);
  }
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  line_buffer.resize(max_frame_in_flight);

  for (int frame_index = 0; frame_index < max_frame_in_flight; frame_index++) {
    line_buffer[frame_index] = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  }

  if (!pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/Saw.comp");
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    pipeline->descriptor_set_layouts.emplace_back(layout);
    auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SawPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    pipeline->Initialize();
  }
}

void DsSaw::Update(const std::vector<glm::vec2>& line, const glm::mat4& projection_view, const unsigned cut_mode) {
  if (line.size() < 2)
    return;
  push_constant.cut_mode = cut_mode;
  line_point_pairs.resize(line.size() - 1);
  for (uint32_t i = 0; i < line.size() - 1; i++) {
    line_point_pairs[i] = glm::vec4(line[i].x, line[i].y, line[i + 1].x, line[i + 1].y);
  }
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  line_buffer[current_frame_index]->UploadVector(line_point_pairs);
  line_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, line_buffer[current_frame_index]);
  push_constant.projection_view = projection_view;
}

void DsSaw::Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  enabled = false;
  if (line_point_pairs.empty())
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;
  push_constant.segment_pair_size = target_dynamic_strands->segment_pairs.size();
  push_constant.line_point_pair_size = line_point_pairs.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->BindDescriptorSet(vk_command_buffer, 1, line_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer,
                  Platform::DivUp(target_dynamic_strands->segment_pairs.size(), work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}
