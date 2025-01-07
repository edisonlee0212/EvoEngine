#include "DsPhysics.hpp"

#include "Shader.hpp"

using namespace eco_sys_lab_plugin;
DsPreStep::DsPreStep() {
  if (!segment_pre_step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/PreStep/Segment.comp");
    segment_pre_step_pipeline = std::make_shared<ComputePipeline>();
    segment_pre_step_pipeline->compute_shader = shader;
    segment_pre_step_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_pre_step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPreStepPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_pre_step_pipeline->Initialize();
  }

  if (!leaf_pre_step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/PreStep/Leaf.comp");
    leaf_pre_step_pipeline = std::make_shared<ComputePipeline>();
    leaf_pre_step_pipeline->compute_shader = shader;
    leaf_pre_step_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = leaf_pre_step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafPreStepPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_pre_step_pipeline->Initialize();
  }
}

void DsPreStep::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                        const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

  SegmentPreStepPushConstant segment_push_constant;
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.time_step = physics_parameters.time_step;
  segment_push_constant.inv_time_step = 1.f / segment_push_constant.time_step;

  LeafPreStepPushConstant leaf_push_constant;
  leaf_push_constant.leaf_size = target_dynamic_strands.foliage.size();
  leaf_push_constant.time_step = physics_parameters.time_step;
  leaf_push_constant.inv_time_step = 1.f / leaf_push_constant.time_step;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_pre_step_pipeline->Bind(vk_command_buffer);
    segment_pre_step_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_pre_step_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    leaf_pre_step_pipeline->Bind(vk_command_buffer);
    leaf_pre_step_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    leaf_pre_step_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

DsPrediction::DsPrediction() {
  if (!segment_prediction_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Prediction/Segment.comp");
    segment_prediction_pipeline = std::make_shared<ComputePipeline>();
    segment_prediction_pipeline->compute_shader = shader;
    segment_prediction_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_prediction_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPredictionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_prediction_pipeline->Initialize();
  }
  if (!leaf_prediction_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Prediction/Leaf.comp");
    leaf_prediction_pipeline = std::make_shared<ComputePipeline>();
    leaf_prediction_pipeline->compute_shader = shader;
    leaf_prediction_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = leaf_prediction_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafPredictionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_prediction_pipeline->Initialize();
  }

  if (!uniform_particle_prediction_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::Constants::shader_global_defines,
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/Prediction/UniformParticle.comp");

    uniform_particle_prediction_pipeline = std::make_shared<ComputePipeline>();
    uniform_particle_prediction_pipeline->compute_shader = shader;
    uniform_particle_prediction_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = uniform_particle_prediction_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(UniformParticlePredictionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    uniform_particle_prediction_pipeline->Initialize();
  }
}

void DsPrediction::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                           const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

  UniformParticlePredictionPushConstant uniform_particle_push_constant;
  uniform_particle_push_constant.uniform_particle_size = target_dynamic_strands.uniform_particles.size();

  SegmentPredictionPushConstant segment_push_constant;
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.time_step = physics_parameters.time_step / physics_parameters.sub_step;
  segment_push_constant.inv_time_step = 1.f / segment_push_constant.time_step;
  segment_push_constant.angular_velocity_damping = physics_parameters.segment_angular_velocity_damping;
  segment_push_constant.velocity_damping = physics_parameters.segment_velocity_damping;

  LeafPredictionPushConstant leaf_push_constant;
  leaf_push_constant.leaf_size = target_dynamic_strands.foliage.size();
  leaf_push_constant.time_step = physics_parameters.time_step / physics_parameters.sub_step;
  leaf_push_constant.inv_time_step = 1.f / leaf_push_constant.time_step;
  leaf_push_constant.angular_velocity_damping = physics_parameters.segment_angular_velocity_damping;
  leaf_push_constant.velocity_damping = physics_parameters.segment_velocity_damping;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_prediction_pipeline->Bind(vk_command_buffer);
    segment_prediction_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    segment_prediction_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);

    uniform_particle_prediction_pipeline->Bind(vk_command_buffer);
    uniform_particle_prediction_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    uniform_particle_prediction_pipeline->PushConstant(vk_command_buffer, 0, uniform_particle_push_constant);
    vkCmdDispatch(vk_command_buffer,
                  Platform::DivUp(uniform_particle_push_constant.uniform_particle_size, work_group_invocations), 1, 1);
    leaf_prediction_pipeline->Bind(vk_command_buffer);
    leaf_prediction_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    leaf_prediction_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

DsBreaking::DsBreaking() {
  if (!segment_pair_breaking_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Breaking/SegmentPair.comp");
    segment_pair_breaking_pipeline = std::make_shared<ComputePipeline>();
    segment_pair_breaking_pipeline->compute_shader = shader;
    segment_pair_breaking_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_pair_breaking_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPairBreakingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_pair_breaking_pipeline->Initialize();
  }
  if (!leaf_breaking_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Breaking/Leaf.comp");
    leaf_breaking_pipeline = std::make_shared<ComputePipeline>();
    leaf_breaking_pipeline->compute_shader = shader;
    leaf_breaking_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = leaf_breaking_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(LeafBreakingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    leaf_breaking_pipeline->Initialize();
  }
}

void DsBreaking::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                         const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

  SegmentPairBreakingPushConstant segment_pair_push_constant;
  segment_pair_push_constant.segment_pair_size = target_dynamic_strands.segment_pairs.size();
  segment_pair_push_constant.allow_breaking =
      physics_parameters.enable_segment_breaking
          ? target_dynamic_strands.GetFrameIndex() % physics_parameters.segment_breaking_detection_frame == 0 ? 1 : 0
          : 0;
  segment_pair_push_constant.allow_disconnection =
      physics_parameters.enable_segment_disconnection
          ? target_dynamic_strands.GetFrameIndex() % physics_parameters.segment_disconnection_detection_frame == 0 ? 1
                                                                                                                   : 0
          : 0;

  LeafBreakingPushConstant leaf_push_constant;
  leaf_push_constant.leaf_size = target_dynamic_strands.foliage.size();

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    if (physics_parameters.enable_segment_breaking || physics_parameters.enable_segment_disconnection) {
      segment_pair_breaking_pipeline->Bind(vk_command_buffer);
      segment_pair_breaking_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      segment_pair_breaking_pipeline->PushConstant(vk_command_buffer, 0, segment_pair_push_constant);
      vkCmdDispatch(vk_command_buffer,
                    Platform::DivUp(segment_pair_push_constant.segment_pair_size, work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    }
    if (physics_parameters.enable_foliage_detachment &&
        target_dynamic_strands.GetFrameIndex() % physics_parameters.foliage_detachment_detection_frame == 0) {
      leaf_breaking_pipeline->Bind(vk_command_buffer);
      leaf_breaking_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      leaf_breaking_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);
      vkCmdDispatch(vk_command_buffer, Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    }
  });
}

DsVelocityUpdate::DsVelocityUpdate() {
  if (!segment_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/VelocityUpdate/Segment.comp");
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
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/VelocityUpdate/Leaf.comp");
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

void DsVelocityUpdate::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                               const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;
  SegmentPushConstant segment_push_constant;
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.time_step = physics_parameters.time_step / physics_parameters.sub_step;
  segment_push_constant.inv_time_step = 1.f / segment_push_constant.time_step;
  LeafPushConstant leaf_push_constant;
  leaf_push_constant.leaf_size = target_dynamic_strands.foliage.size();
  leaf_push_constant.time_step = physics_parameters.time_step / physics_parameters.sub_step;
  leaf_push_constant.inv_time_step = 1.f / leaf_push_constant.time_step;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    segment_pipeline->Bind(vk_command_buffer);
    segment_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    leaf_pipeline->Bind(vk_command_buffer);
    leaf_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    leaf_pipeline->PushConstant(vk_command_buffer, 0, leaf_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(leaf_push_constant.leaf_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

DsDynamicHashedGrid::DsDynamicHashedGrid() {
  if (!partition_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::Constants::shader_global_defines,
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/DynamicHashedGrid/Partition.comp");

    partition_pipeline = std::make_shared<ComputePipeline>();
    partition_pipeline->compute_shader = shader;
    partition_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = partition_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PartitionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    partition_pipeline->Initialize();
  }

  if (!local_merge_sort_shader) {
    local_merge_sort_shader = std::make_shared<Shader>();
    local_merge_sort_shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") /
            "Shaders/Compute/DynamicStrands/DynamicHashedGrid/Sort/LocalMergeSort.comp");
  }

  if (!big_flip_shader) {
    big_flip_shader = std::make_shared<Shader>();
    big_flip_shader->TryCompile(ShaderType::Compute, Platform::Constants::shader_global_defines,
                                std::filesystem::path("./EcoSysLabResources") /
                                    "Shaders/Compute/DynamicStrands/DynamicHashedGrid/Sort/BigFlip.comp");
  }

  if (!local_disperse_shader) {
    local_disperse_shader = std::make_shared<Shader>();
    local_disperse_shader->TryCompile(ShaderType::Compute, Platform::Constants::shader_global_defines,
                                      std::filesystem::path("./EcoSysLabResources") /
                                          "Shaders/Compute/DynamicStrands/DynamicHashedGrid/Sort/LocalDisperse.comp");
  }

  if (!global_disperse_shader) {
    global_disperse_shader = std::make_shared<Shader>();
    global_disperse_shader->TryCompile(ShaderType::Compute, Platform::Constants::shader_global_defines,
                                       std::filesystem::path("./EcoSysLabResources") /
                                           "Shaders/Compute/DynamicStrands/DynamicHashedGrid/Sort/GlobalDisperse.comp");
  }

  if (!offset_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/DynamicHashedGrid/Offset.comp");

    offset_pipeline = std::make_shared<ComputePipeline>();
    offset_pipeline->compute_shader = shader;
    offset_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = offset_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(OffsetPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    offset_pipeline->Initialize();
  }
}

bool DsDynamicHashedGrid::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Collision Range", &grid_cell_size, 0.001f, 0.001f, 1.0f))
    changed = true;
  return changed;
}

void DsDynamicHashedGrid::BuildGrid(const DynamicStrands::PhysicsParameters& physics_parameters,
                                    const DynamicStrands& target_dynamic_strands) {
#pragma region Partition
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

  PartitionPushConstant partition_push_constant;
  partition_push_constant.segment_size = target_dynamic_strands.segments.size();
  partition_push_constant.grid_cell_size = grid_cell_size;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    partition_pipeline->Bind(vk_command_buffer);
    partition_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    partition_pipeline->PushConstant(vk_command_buffer, 0, partition_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(partition_push_constant.segment_size, work_group_invocations), 1,
                  1);
    Platform::EverythingBarrier(vk_command_buffer);
  });

#pragma endregion
#pragma region Sort
  const int32_t max_work_group_size = glm::min(Platform::Constants::max_compute_work_group_invocations,
                                               static_cast<uint32_t>(Platform::Constants::max_shared_memory_size /
                                                                     sizeof(DynamicStrands::GpuHashedGridElement)));
  uint32_t work_group_size;
  const uint32_t segment_size = target_dynamic_strands.segments.size();
  // Adjust workgroup_size_x to get as close to max_workgroup_size as possible.
  if (segment_size < max_work_group_size * 2) {
    work_group_size = segment_size / 2;
  } else {
    work_group_size = max_work_group_size;
  }
  uint32_t segment_group_size = work_group_size * 2;

  local_merge_sort_pipeline = std::make_unique<ComputePipeline>();
  local_merge_sort_pipeline->compute_shader = local_merge_sort_shader;
  local_merge_sort_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  local_merge_sort_pipeline->map_entries.emplace_back(work_group_size);
  auto& local_merge_sort_push_constant_range = local_merge_sort_pipeline->push_constant_ranges.emplace_back();
  local_merge_sort_push_constant_range.size = sizeof(SortPushConstant);
  local_merge_sort_push_constant_range.offset = 0;
  local_merge_sort_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  local_merge_sort_pipeline->Initialize();

  big_flip_pipeline = std::make_unique<ComputePipeline>();
  big_flip_pipeline->compute_shader = big_flip_shader;
  big_flip_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  big_flip_pipeline->map_entries.emplace_back(work_group_size);
  auto& big_flip_push_constant_range = big_flip_pipeline->push_constant_ranges.emplace_back();
  big_flip_push_constant_range.size = sizeof(SortPushConstant);
  big_flip_push_constant_range.offset = 0;
  big_flip_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  big_flip_pipeline->Initialize();

  local_disperse_pipeline = std::make_unique<ComputePipeline>();
  local_disperse_pipeline->compute_shader = local_disperse_shader;
  local_disperse_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  local_disperse_pipeline->map_entries.emplace_back(work_group_size);
  auto& local_disperse_push_constant_range = local_disperse_pipeline->push_constant_ranges.emplace_back();
  local_disperse_push_constant_range.size = sizeof(SortPushConstant);
  local_disperse_push_constant_range.offset = 0;
  local_disperse_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  local_disperse_pipeline->Initialize();

  global_disperse_pipeline = std::make_unique<ComputePipeline>();
  global_disperse_pipeline->compute_shader = global_disperse_shader;
  global_disperse_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
  global_disperse_pipeline->map_entries.emplace_back(work_group_size);
  auto& global_disperse_push_constant_range = global_disperse_pipeline->push_constant_ranges.emplace_back();
  global_disperse_push_constant_range.size = sizeof(SortPushConstant);
  global_disperse_push_constant_range.offset = 0;
  global_disperse_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  global_disperse_pipeline->Initialize();

  const uint32_t workgroup_count = (segment_size * 2 + segment_group_size - 1) / segment_group_size;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    const auto local_merge_sort = [&](const uint32_t& current_segment_group_size) {
      local_merge_sort_pipeline->Bind(vk_command_buffer);
      SortPushConstant lms_push_constant;
      lms_push_constant.segment_size = segment_size;
      lms_push_constant.segment_group_size = current_segment_group_size;
      local_merge_sort_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      local_merge_sort_pipeline->PushConstant(vk_command_buffer, 0, lms_push_constant);
      vkCmdDispatch(vk_command_buffer, workgroup_count, 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    };
    const auto big_flip = [&](const uint32_t& current_segment_group_size) {
      big_flip_pipeline->Bind(vk_command_buffer);
      SortPushConstant lms_push_constant;
      lms_push_constant.segment_size = segment_size;
      lms_push_constant.segment_group_size = current_segment_group_size;
      big_flip_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      big_flip_pipeline->PushConstant(vk_command_buffer, 0, lms_push_constant);
      vkCmdDispatch(vk_command_buffer, workgroup_count, 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    };
    const auto local_disperse = [&](const uint32_t& current_segment_group_size) {
      local_disperse_pipeline->Bind(vk_command_buffer);
      SortPushConstant lms_push_constant;
      lms_push_constant.segment_size = segment_size;
      lms_push_constant.segment_group_size = current_segment_group_size;
      local_disperse_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      local_disperse_pipeline->PushConstant(vk_command_buffer, 0, lms_push_constant);
      vkCmdDispatch(vk_command_buffer, workgroup_count, 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    };
    const auto global_disperse = [&](const uint32_t& current_segment_group_size) {
      global_disperse_pipeline->Bind(vk_command_buffer);
      SortPushConstant lms_push_constant;
      lms_push_constant.segment_size = segment_size;
      lms_push_constant.segment_group_size = current_segment_group_size;
      global_disperse_pipeline->BindDescriptorSet(
          vk_command_buffer, 0,
          target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      global_disperse_pipeline->PushConstant(vk_command_buffer, 0, lms_push_constant);
      vkCmdDispatch(vk_command_buffer, workgroup_count, 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    };
    local_merge_sort(segment_group_size);
    // we must now double h, as this happens before every flip
    segment_group_size *= 2;
    uint32_t current_segment_group_size = segment_group_size;
    for (; current_segment_group_size <= segment_size * 2; current_segment_group_size *= 2) {
      big_flip(current_segment_group_size);
      for (uint32_t hh = current_segment_group_size / 2; hh > 1; hh /= 2) {
        if (hh <= work_group_size * 2) {
          local_disperse(hh);
          break;
        }
        global_disperse(hh);
      }
    }
  });
#pragma endregion
#pragma region Offset
  OffsetPushConstant offset_push_constant;
  offset_push_constant.segment_size = target_dynamic_strands.segments.size();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    vkCmdFillBuffer(vk_command_buffer, target_dynamic_strands.device_hashed_grid_cell_starts_buffer->GetVkBuffer(), 0,
                    VK_WHOLE_SIZE, 0xFFFFFFFF);
    Platform::EverythingBarrier(vk_command_buffer);
    offset_pipeline->Bind(vk_command_buffer);
    offset_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    offset_pipeline->PushConstant(vk_command_buffer, 0, offset_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(offset_push_constant.segment_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
#pragma endregion
}

DsSegmentCollision::DsSegmentCollision() {
  if (!spherical_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(ShaderType::Compute, Platform::Constants::shader_global_defines,
                       std::filesystem::path("./EcoSysLabResources") /
                           "Shaders/Compute/DynamicStrands/SegmentCollision/Spherical.comp");

    spherical_pipeline = std::make_shared<ComputePipeline>();
    spherical_pipeline->compute_shader = shader;
    spherical_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = spherical_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SphericalPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    spherical_pipeline->Initialize();
  }
}

void DsSegmentCollision::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                                 const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

  SphericalPushConstant spherical_push_constant;
  spherical_push_constant.segment_size = target_dynamic_strands.segments.size();
  spherical_push_constant.grid_cell_size = target_dynamic_strands.dynamic_hashed_grid->grid_cell_size;
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    spherical_pipeline->Bind(vk_command_buffer);
    spherical_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    spherical_pipeline->PushConstant(vk_command_buffer, 0, spherical_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(spherical_push_constant.segment_size, work_group_invocations), 1,
                  1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}
