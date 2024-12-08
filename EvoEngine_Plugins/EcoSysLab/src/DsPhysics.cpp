#include "DsPhysics.hpp"

#include "Shader.hpp"

using namespace eco_sys_lab_plugin;
DsPreStep::DsPreStep() {
  if (!particle_pre_step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/PreStep/Particle.comp");

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

  if (!connection_pre_step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/PreStep/Connection.comp");
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

void DsPreStep::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                        const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  ParticlePreStepPushConstant particle_push_constant;
  particle_push_constant.particle_size = target_dynamic_strands.particles.size();
  particle_push_constant.time_step = physics_parameters.time_step;
  particle_push_constant.inv_time_step = 1.f / particle_push_constant.time_step;
  const uint32_t work_group_invocations =
      Platform::Constants::compute_work_group_invocations;

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
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(particle_push_constant.particle_size, work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    segment_pre_step_pipeline->Bind(vk_command_buffer);
    segment_pre_step_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_pre_step_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    connection_pre_step_pipeline->Bind(vk_command_buffer);
    connection_pre_step_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    connection_pre_step_pipeline->PushConstant(vk_command_buffer, 0, connection_push_constant);
    vkCmdDispatch(vk_command_buffer,
                  Platform::DivUp(connection_push_constant.connection_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

DsPrediction::DsPrediction() {
  if (!particle_prediction_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Prediction/Particle.comp");

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
  if (!segment_pair_prediction_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Prediction/SegmentPair.comp");
    segment_pair_prediction_pipeline = std::make_shared<ComputePipeline>();
    segment_pair_prediction_pipeline->compute_shader = shader;
    segment_pair_prediction_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = segment_pair_prediction_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(SegmentPairPredictionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    segment_pair_prediction_pipeline->Initialize();
  }
  if (!uniform_particle_prediction_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
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

  if (!connection_prediction_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Prediction/Connection.comp");
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

void DsPrediction::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                           const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  ParticlePredictionPushConstant particle_push_constant;
  particle_push_constant.particle_size = target_dynamic_strands.particles.size();
  particle_push_constant.time_step = physics_parameters.time_step / physics_parameters.sub_step;
  particle_push_constant.inv_time_step = 1.f / particle_push_constant.time_step;
  particle_push_constant.velocity_damping = physics_parameters.velocity_damping;
  const uint32_t work_group_invocations =
      Platform::Constants::compute_work_group_invocations;

  UniformParticlePredictionPushConstant uniform_particle_push_constant;
  uniform_particle_push_constant.uniform_particle_size = target_dynamic_strands.uniform_particles.size();

  SegmentPredictionPushConstant segment_push_constant;
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.time_step = physics_parameters.time_step / physics_parameters.sub_step;
  segment_push_constant.inv_time_step = 1.f / segment_push_constant.time_step;
  segment_push_constant.angular_velocity_damping = physics_parameters.angular_velocity_damping;
  ConnectionPredictionPushConstant connection_push_constant;
  connection_push_constant.connection_size = target_dynamic_strands.connections.size();
  connection_push_constant.allow_breaking = physics_parameters.allow_breaking ? 1 : 0;

  SegmentPairPredictionPushConstant segment_pair_push_constant;
  segment_pair_push_constant.segment_pair_size = target_dynamic_strands.segment_pairs.size();
  segment_pair_push_constant.allow_breaking = physics_parameters.allow_breaking ? 1 : 0;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    particle_prediction_pipeline->Bind(vk_command_buffer);
    particle_prediction_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    particle_prediction_pipeline->PushConstant(vk_command_buffer, 0, particle_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(particle_push_constant.particle_size, work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    segment_prediction_pipeline->Bind(vk_command_buffer);
    segment_prediction_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_prediction_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    connection_prediction_pipeline->Bind(vk_command_buffer);
    connection_prediction_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    connection_prediction_pipeline->PushConstant(vk_command_buffer, 0, connection_push_constant);
    vkCmdDispatch(vk_command_buffer,
                  Platform::DivUp(connection_push_constant.connection_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    uniform_particle_prediction_pipeline->Bind(vk_command_buffer);
    uniform_particle_prediction_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    uniform_particle_prediction_pipeline->PushConstant(vk_command_buffer, 0, uniform_particle_push_constant);
    vkCmdDispatch(vk_command_buffer,
                  Platform::DivUp(uniform_particle_push_constant.uniform_particle_size, work_group_invocations), 1,
                  1);
    Platform::EverythingBarrier(vk_command_buffer);

    segment_pair_prediction_pipeline->Bind(vk_command_buffer);
    segment_pair_prediction_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_pair_prediction_pipeline->PushConstant(vk_command_buffer, 0, segment_pair_push_constant);
    vkCmdDispatch(vk_command_buffer,
                  Platform::DivUp(segment_pair_push_constant.segment_pair_size, work_group_invocations), 1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

DsVelocityUpdate::DsVelocityUpdate() {
  if (!particle_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/VelocityUpdate/Particle.comp");

    particle_pipeline = std::make_shared<ComputePipeline>();
    particle_pipeline->compute_shader = shader;
    particle_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = particle_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(ParticlePushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    particle_pipeline->Initialize();
  }
  if (!segment_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
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
}

void DsVelocityUpdate::Execute(const DynamicStrands::PhysicsParameters& physics_parameters,
                               const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  ParticlePushConstant particle_push_constant;
  particle_push_constant.particle_size = target_dynamic_strands.particles.size();
  particle_push_constant.time_step = physics_parameters.time_step / physics_parameters.sub_step;
  particle_push_constant.inv_time_step = 1.f / particle_push_constant.time_step;
  const uint32_t work_group_invocations =
      Platform::Constants::compute_work_group_invocations;

  SegmentPushConstant segment_push_constant;
  segment_push_constant.segment_size = target_dynamic_strands.segments.size();
  segment_push_constant.time_step = physics_parameters.time_step / physics_parameters.sub_step;
  segment_push_constant.inv_time_step = 1.f / segment_push_constant.time_step;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    particle_pipeline->Bind(vk_command_buffer);
    particle_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    particle_pipeline->PushConstant(vk_command_buffer, 0, particle_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(particle_push_constant.particle_size, work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);

    segment_pipeline->Bind(vk_command_buffer);
    segment_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    segment_pipeline->PushConstant(vk_command_buffer, 0, segment_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(segment_push_constant.segment_size, work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

DsHashedGrid::DsHashedGrid() {
  if (!partition_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Collision/Partition.comp");

    partition_pipeline = std::make_shared<ComputePipeline>();
    partition_pipeline->compute_shader = shader;
    partition_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = partition_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(PartitionPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    partition_pipeline->Initialize();
  }
}

void DsHashedGrid::Initialize(const DynamicStrands::PhysicsParameters& physics_parameters,
    const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const uint32_t work_group_invocations =
      Platform::Constants::compute_work_group_invocations;

  PartitionPushConstant partition_push_constant;
  partition_push_constant.segment_size = target_dynamic_strands.segments.size();
  partition_push_constant.grid_size = grid_size;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    partition_pipeline->Bind(vk_command_buffer);
    partition_pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    partition_pipeline->PushConstant(vk_command_buffer, 0, partition_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(partition_push_constant.segment_size, work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}
