#include "DsOperators.hpp"
#include "Shader.hpp"
using namespace eco_sys_lab_package;

void DsLeafDrop::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                         const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  LeafDropPushConstant leaf_push_constant;
  leaf_push_constant.ground_height = ground_height;
  leaf_push_constant.leaf_size = target_dynamic_strands->foliage.size();
  leaf_push_constant.air_resistance_strength = air_resistance_strength;
  leaf_push_constant.rotation_correction_strength = rotation_correction_strength;
  leaf_push_constant.disturbance_frequency = disturbance_frequency;
  leaf_push_constant.disturbance_strength = disturbance_strength;
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);

    pipeline->Dispatch(vk_command_buffer, Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

bool DsLeafDrop::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Enabled", &enabled)) {
    changed = true;
  }
  if (ImGui::DragFloat("Ground height", &ground_height, 0.01f, -100.0f, 100.0f))
    changed = true;

  if (ImGui::DragFloat("Rotation correction strength", &rotation_correction_strength, 0.001f, 0.01f, 1.0f))
    changed = true;

  if (ImGui::DragFloat("Air resistance strength", &air_resistance_strength, 0.01f, 0.01f, 1.0f))
    changed = true;

  if (ImGui::DragFloat("Disturbance strength", &disturbance_strength, 0.01f, 0.01f, 1.0f))
    changed = true;
  if (ImGui::DragFloat3("Disturbance frequency", &disturbance_frequency.x, 0.01f, 0.01f, 10.0f))
    changed = true;
  return changed;
}

DsLeafDrop::DsLeafDrop() {
  if (!pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/LeafDrop.comp");
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafDropPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    pipeline->Initialize();
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
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
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
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    drag_force_pipeline->Bind(vk_command_buffer);
    drag_force_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    drag_force_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                           commands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    drag_force_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    drag_force_pipeline->Dispatch(vk_command_buffer,
                                  Platform::DivUp(push_constant.commands_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

bool DsAttraction::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Multiplier", &distance_multiplier, 0.01f, 0.0f, 1.0f)) {
    changed = true;
  }
  return changed;
}

DsBoxSelection::DsBoxSelection() {
  if (!segment_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Operators/SegmentBoxSelection.comp");
    segment_pipeline = std::make_shared<ComputePipeline>();
    segment_pipeline->compute_shader = shader;
    segment_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    auto& push_constant_range = segment_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentBoxSelectionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_pipeline->Initialize();
  }
  if (!leaf_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Operators/LeafBoxSelection.comp");
    leaf_pipeline = std::make_shared<ComputePipeline>();
    leaf_pipeline->compute_shader = shader;
    leaf_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    auto& push_constant_range = leaf_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafBoxSelectionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_pipeline->Initialize();
  }
}

void DsBoxSelection::Update(const glm::vec2& box_start, const glm::vec2& box_end, const glm::mat4& projection_view,
                            const uint32_t selection_mode) {
  segment_push_constant.box_min = glm::min(box_start, box_end);
  segment_push_constant.box_max = glm::max(box_start, box_end);

  segment_push_constant.projection_view = projection_view;
  segment_push_constant.selection_mode = selection_mode;

  leaf_push_constant.box_min = glm::min(box_start, box_end);
  leaf_push_constant.box_max = glm::max(box_start, box_end);

  leaf_push_constant.projection_view = projection_view;
  leaf_push_constant.selection_mode = selection_mode;
}

void DsBoxSelection::Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  segment_push_constant.segment_size = target_dynamic_strands->segments.size();
  leaf_push_constant.leaf_size = target_dynamic_strands->foliage.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_pipeline->Bind(vk_command_buffer);
    segment_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    segment_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);

    segment_pipeline->Dispatch(vk_command_buffer,
                               Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    leaf_pipeline->Bind(vk_command_buffer);
    leaf_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    leaf_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);

    leaf_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1,
                            1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
  enabled = false;
}

DsDrag::DsDrag() {
  if (!pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
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
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
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

    pipeline->Dispatch(vk_command_buffer, Platform::DivUp(push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
  enabled = false;
}

DsLineCut::DsLineCut() {
  if (!pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
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
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  push_constant.segment_pair_size = target_dynamic_strands->segment_pairs.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    pipeline->Dispatch(vk_command_buffer,
                       Platform::DivUp(target_dynamic_strands->segment_pairs.size(), work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
  enabled = false;
}

DsPointCut::DsPointCut() {
  if (!pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/PointCut.comp");
    pipeline = std::make_shared<ComputePipeline>();
    pipeline->compute_shader = shader;
    pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    auto& push_constant_range = pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PointCutPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    pipeline->Initialize();
  }
}

void DsPointCut::Update(const glm::vec2& point, const glm::vec2& screen_size, const float point_size,
                        const glm::mat4& projection_view, const unsigned cut_mode) {
  EVOENGINE_LOG("Cut mode: " << cut_mode);
  push_constant.point = point;
  push_constant.screen_size = screen_size;
  push_constant.point_size = point_size;
  push_constant.projection_view = projection_view;
  push_constant.cut_mode = cut_mode;
}

void DsPointCut::Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  push_constant.segment_pair_size = target_dynamic_strands->segment_pairs.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    pipeline->Dispatch(vk_command_buffer,
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
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
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
  line_buffer[current_frame_index]->SetDebugName("Line buffer");
  line_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, line_buffer[current_frame_index]);
  push_constant.projection_view = projection_view;
}

void DsSaw::Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  enabled = false;
  if (line_point_pairs.empty())
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  push_constant.segment_pair_size = target_dynamic_strands->segment_pairs.size();
  push_constant.line_point_pair_size = line_point_pairs.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->BindDescriptorSet(vk_command_buffer, 1, line_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    pipeline->Dispatch(vk_command_buffer,
                       Platform::DivUp(target_dynamic_strands->segment_pairs.size(), work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

DsSnow::DsSnow() {
  if (!segment_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/SegmentSnow.comp");
    segment_pipeline = std::make_shared<ComputePipeline>();
    segment_pipeline->compute_shader = shader;
    segment_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_pipeline->Initialize();
  }

  if (!leaf_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/LeafSnow.comp");
    leaf_pipeline = std::make_shared<ComputePipeline>();
    leaf_pipeline->compute_shader = shader;
    leaf_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = leaf_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_pipeline->Initialize();
  }
}

void DsSnow::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                     const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  SegmentPushConstant segment_push_constant;
  segment_push_constant.segment_size = target_dynamic_strands->segments.size();
  segment_push_constant.snow_intensity = snow_intensity * physics_parameters.time_step;
  segment_push_constant.snow_retain_ratio = snow_retain_ratio;

  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_pipeline->Bind(vk_command_buffer);
    segment_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);

    segment_pipeline->Dispatch(vk_command_buffer,
                               Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
    /*
    LeafPushConstant leaf_push_constant;
    leaf_push_constant.leaf_size = target_dynamic_strands->foliage.size();
    leaf_push_constant.snow_intensity = snow_intensity * physics_parameters.time_step;
    leaf_push_constant.snow_retain_ratio = snow_retain_ratio;
    leaf_pipeline->Bind(vk_command_buffer);
    leaf_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    leaf_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);

    leaf_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1,
    1); Platform::EverythingBarrier(vk_command_buffer);
    */
  });
}

bool DsSnow::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Enabled", &enabled)) {
    changed = true;
  }
  if (ImGui::DragFloat("Snow intensity", &snow_intensity, 0.0001f, -0.002f, 0.002f, "%.4f")) {
    changed = true;
  }
  if (ImGui::SliderFloat("Snow retain ratio after break", &snow_retain_ratio, 0.0f, 1.0f)) {
    changed = true;
  }
  return changed;
}

DsWind::DsWind() {
  enabled = false;
  if (!segment_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/SegmentWind.comp");
    segment_pipeline = std::make_shared<ComputePipeline>();
    segment_pipeline->compute_shader = shader;
    segment_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_pipeline->Initialize();
  }

  if (!leaf_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/LeafWind.comp");
    leaf_pipeline = std::make_shared<ComputePipeline>();
    leaf_pipeline->compute_shader = shader;
    leaf_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = leaf_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_pipeline->Initialize();
  }
}

void DsWind::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                     const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  SegmentPushConstant segment_push_constant;
  segment_push_constant.segment_size = target_dynamic_strands->segments.size();
  segment_push_constant.main_force = main_force;
  segment_push_constant.turbulence_strength = turbulence_strength;
  segment_push_constant.turbulence_direction_frequency = turbulence_direction_frequency;
  segment_push_constant.turbulence_speed_frequency = turbulence_speed_frequency;
  segment_push_constant.simulated_time = target_dynamic_strands->GetSimulatedTime();
  LeafPushConstant leaf_push_constant;
  leaf_push_constant.leaf_size = target_dynamic_strands->foliage.size();
  leaf_push_constant.main_force = main_force;
  leaf_push_constant.turbulence_strength = turbulence_strength;
  leaf_push_constant.turbulence_direction_frequency = turbulence_direction_frequency;
  leaf_push_constant.turbulence_speed_frequency = turbulence_speed_frequency;

  leaf_push_constant.simulated_time = target_dynamic_strands->GetSimulatedTime();
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_pipeline->Bind(vk_command_buffer);
    segment_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);

    segment_pipeline->Dispatch(vk_command_buffer,
                               Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    leaf_pipeline->Bind(vk_command_buffer);
    leaf_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    leaf_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);

    leaf_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1,
                            1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

bool DsWind::DrawGui(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Enabled", &enabled)) {
    changed = true;
  }
  if (ImGui::DragFloat3("Main force", &main_force.x, 0.001f, -1.f, 1.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Turbulence strength", &turbulence_strength, 0.01f, 0.f, 2.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Turbulence directional frequency", &turbulence_direction_frequency, 0.01f, -100.f, 100.f)) {
    changed = true;
  }
  if (ImGui::DragFloat("Turbulence speed frequency", &turbulence_speed_frequency, 0.01f, -100.f, 100.f)) {
    changed = true;
  }
  return changed;
}

DsStopAll::DsStopAll() {
  enabled = false;
  if (!segment_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/SegmentStopAll.comp");
    segment_pipeline = std::make_shared<ComputePipeline>();
    segment_pipeline->compute_shader = shader;
    segment_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_pipeline->Initialize();
  }

  if (!leaf_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/LeafStopAll.comp");
    leaf_pipeline = std::make_shared<ComputePipeline>();
    leaf_pipeline->compute_shader = shader;
    leaf_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = leaf_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_pipeline->Initialize();
  }
}

void DsStopAll::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                        const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  enabled = false;
  SegmentPushConstant segment_push_constant;
  segment_push_constant.segment_size = target_dynamic_strands->segments.size();
  LeafPushConstant leaf_push_constant;
  leaf_push_constant.leaf_size = target_dynamic_strands->foliage.size();
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_pipeline->Bind(vk_command_buffer);
    segment_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);

    segment_pipeline->Dispatch(vk_command_buffer,
                               Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    leaf_pipeline->Bind(vk_command_buffer);
    leaf_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    leaf_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);

    leaf_pipeline->Dispatch(vk_command_buffer, Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1,
                            1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

DsFungusInjection::DsFungusInjection() {
  if (!min_distance_layout) {
    min_distance_layout = std::make_shared<DescriptorSetLayout>();
    min_distance_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    min_distance_layout->Initialize();
  }

  if (!min_dist_reset_pipeline) {
    const auto reset_shader = std::make_shared<Shader>();
    reset_shader->TryCompile(
        ShaderType::Compute, Platform::GetShaderGlobalDefines(),
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Operators/MinDistReset.comp");
    min_dist_reset_pipeline = std::make_shared<ComputePipeline>();
    min_dist_reset_pipeline->compute_shader = reset_shader;
    min_dist_reset_pipeline->descriptor_set_layouts.emplace_back(min_distance_layout);

    min_dist_reset_pipeline->Initialize();
  }

  if (!find_closest_pipeline) {
    const auto find_closest_shader = std::make_shared<Shader>();
    find_closest_shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                    std::filesystem::path("./EcoSysLabResources") /
                                        "Shaders/Compute/DynamicStrands/Operators/FungusFindClosest.comp");
    find_closest_pipeline = std::make_shared<ComputePipeline>();
    find_closest_pipeline->compute_shader = find_closest_shader;
    find_closest_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    find_closest_pipeline->descriptor_set_layouts.emplace_back(min_distance_layout);
    auto& push_constant_range = find_closest_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(FungusInjectionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    find_closest_pipeline->Initialize();
  }

  if (!inject_pipeline) {
    const auto injection_shader = std::make_shared<Shader>();
    injection_shader->TryCompile(ShaderType::Compute, Platform::GetShaderGlobalDefines(),
                                 std::filesystem::path("./EcoSysLabResources") /
                                     "Shaders/Compute/DynamicStrands/Operators/FungusInjection.comp");
    inject_pipeline = std::make_shared<ComputePipeline>();
    inject_pipeline->compute_shader = injection_shader;
    inject_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    inject_pipeline->descriptor_set_layouts.emplace_back(min_distance_layout);
    auto& push_constant_range = inject_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(FungusInjectionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    inject_pipeline->Initialize();
  }

  struct GpuMinDistance {
    int global_min_distance = 0;
    unsigned int global_index_of_min = 0;
    int padding0 = 0;
    int padding1 = 0;
  };

  if (!min_distance_buffer) {
    VkBufferCreateInfo buffer_create_info{};
    buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_create_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    buffer_create_info.size = sizeof(GpuMinDistance);
    VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
    buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    min_distance_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  }

  if (!min_distance_descriptor_set) {
    min_distance_descriptor_set = std::make_shared<DescriptorSet>(min_distance_layout);
  }

  min_distance_buffer->Resize(sizeof(GpuMinDistance));
  min_distance_descriptor_set->UpdateBufferDescriptorBinding(0, min_distance_buffer);
  min_distance_buffer->SetDebugName("Min distance buffer");
}

void DsFungusInjection::ReleaseStaticGpuResources() {
  min_distance_descriptor_set.reset();
  inject_pipeline.reset();
  find_closest_pipeline.reset();
  min_dist_reset_pipeline.reset();
  min_distance_buffer.reset();
  min_distance_layout.reset();
}

void DsFungusInjection::Update(const glm::vec2& point, const glm::vec2& screen_size, float point_size,
                               float injection_amount, bool white_rot, bool brown_rot,
                               const glm::mat4& projection_view) {
  push_constant.point = point;
  push_constant.screen_size = screen_size;
  push_constant.point_size = point_size;
  push_constant.injection_amount = injection_amount;
  push_constant.projection_view = projection_view;

  constexpr unsigned int WHITE_ROT = 1 << 0;
  constexpr unsigned int BROWN_ROT = 1 << 1;

  if (white_rot) {
    push_constant.fungus_type |= WHITE_ROT;
  } else {
    push_constant.fungus_type &= ~WHITE_ROT;
  }

  if (brown_rot) {
    push_constant.fungus_type |= BROWN_ROT;
  } else {
    push_constant.fungus_type &= ~BROWN_ROT;
  }
}

void DsFungusInjection::Execute(const std::shared_ptr<DynamicStrands>& target_dynamic_strands) {
  const uint32_t work_group_invocations = Platform::GetInstance().GetCapabilities().compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  // first call reset shader
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    min_dist_reset_pipeline->Bind(vk_command_buffer);
    min_dist_reset_pipeline->BindDescriptorSet(vk_command_buffer, 0, min_distance_descriptor_set->GetVkDescriptorSet());
    min_dist_reset_pipeline->Dispatch(vk_command_buffer, 1, 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });

  push_constant.segment_size = target_dynamic_strands->segments.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    find_closest_pipeline->Bind(vk_command_buffer);

    find_closest_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    find_closest_pipeline->BindDescriptorSet(vk_command_buffer, 1, min_distance_descriptor_set->GetVkDescriptorSet());
    find_closest_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    find_closest_pipeline->Dispatch(
        vk_command_buffer, Platform::DivUp(target_dynamic_strands->segment_pairs.size(), work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    inject_pipeline->Bind(vk_command_buffer);
    inject_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands->strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    inject_pipeline->BindDescriptorSet(vk_command_buffer, 1, min_distance_descriptor_set->GetVkDescriptorSet());
    inject_pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    inject_pipeline->Dispatch(
        vk_command_buffer, Platform::DivUp(target_dynamic_strands->segment_pairs.size(), work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
  enabled = false;
}
