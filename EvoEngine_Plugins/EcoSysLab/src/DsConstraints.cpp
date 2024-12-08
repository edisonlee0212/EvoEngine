#include "DsConstraints.hpp"
#include "Shader.hpp"
#include "VoxelGrid.hpp"
using namespace eco_sys_lab_plugin;

void DsGroundPlane::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                              const DynamicStrands& target_dynamic_strands) {
  GroundPlanePushConstant push_constant;
  push_constant.ground_height = ground_height;
  push_constant.particle_size = target_dynamic_strands.particles.size();
  push_constant.ground_softness = ground_softness;
  push_constant.ground_friction = ground_friction;
  const uint32_t work_group_invocations =
      Platform::Constants::compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    pipeline->Bind(vk_command_buffer);
    pipeline->BindDescriptorSet(
        vk_command_buffer, 0,
        target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());

    pipeline->PushConstant(vk_command_buffer, 0, push_constant);

    vkCmdDispatch(vk_command_buffer, Platform::DivUp(push_constant.particle_size, work_group_invocations), 1, 1);
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
    if (ImGui::DragFloat("Ground friction", &ground_friction, 0.01f, 0.f, 1.f))
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
                                  "Shaders/Compute/DynamicStrands/Constraints/StiffRodShearStretchBilateral.comp");
    bilateral_stretch_shear_constraint_pipeline = std::make_shared<ComputePipeline>();
    bilateral_stretch_shear_constraint_pipeline->compute_shader = stretch_shear_shader;

    bilateral_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    bilateral_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range =
        bilateral_stretch_shear_constraint_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(ShearStretchConstraintConstant);
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
                                  "Shaders/Compute/DynamicStrands/Constraints/StiffRodShearStretchForward.comp");
    forward_stretch_shear_constraint_pipeline = std::make_shared<ComputePipeline>();
    forward_stretch_shear_constraint_pipeline->compute_shader = stretch_shear_shader;

    forward_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    forward_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range =
        forward_stretch_shear_constraint_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(ShearStretchConstraintConstant);
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
                                  "Shaders/Compute/DynamicStrands/Constraints/StiffRodShearStretchBackward.comp");
    backward_stretch_shear_constraint_pipeline = std::make_shared<ComputePipeline>();
    backward_stretch_shear_constraint_pipeline->compute_shader = stretch_shear_shader;

    backward_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    backward_stretch_shear_constraint_pipeline->descriptor_set_layouts.emplace_back(layout);

    auto& stretch_shear_push_constant_range =
        backward_stretch_shear_constraint_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(ShearStretchConstraintConstant);
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
      if (ImGui::Combo("Project Mode", {"Forward", "Backward", "Alternating Direction", "Bilateral"}, project_mode))
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
    stiff_rod_per_strand_data.front_propagate_begin_segment_handle = -1;
    stiff_rod_per_strand_data.back_propagate_begin_segment_handle = -1;
    stiff_rod_per_strand_data.front_propagate_begin_connection_handle = -1;
    stiff_rod_per_strand_data.back_propagate_begin_connection_handle = -1;
    stiff_rod_per_strand_data.alternative_front_propagate_begin_connection_handle = -1;
    stiff_rod_per_strand_data.alternative_back_propagate_begin_connection_handle = -1;
    stiff_rod_per_strand_data.alternative_front_propagate_begin_segment_handle = -1;
    stiff_rod_per_strand_data.alternative_back_propagate_begin_segment_handle = -1;
    if (gpu_strand.begin_segment_handle == -1) {
      continue;
    }
    stiff_rod_per_strand_data.front_propagate_begin_segment_handle = gpu_strand.begin_segment_handle;
    if (gpu_strand.begin_segment_handle == gpu_strand.end_segment_handle) {
      stiff_rod_per_strand_data.alternative_front_propagate_begin_segment_handle = gpu_strand.begin_segment_handle;
      continue;
    }
    stiff_rod_per_strand_data.alternative_front_propagate_begin_segment_handle =
        target_dynamic_strands.segments[gpu_strand.begin_segment_handle].next_handle;

    stiff_rod_per_strand_data.front_propagate_begin_connection_handle = gpu_strand.begin_connection_handle;
    if (gpu_strand.begin_connection_handle == gpu_strand.end_connection_handle) {
      stiff_rod_per_strand_data.alternative_front_propagate_begin_connection_handle =
          gpu_strand.begin_connection_handle;
      continue;
    }
    stiff_rod_per_strand_data.alternative_front_propagate_begin_connection_handle =
        target_dynamic_strands.connections[gpu_strand.begin_connection_handle].next_handle;

    int connection_size = 0;
    int connection_handle = stiff_rod_per_strand_data.front_propagate_begin_connection_handle;
    while (connection_handle != -1) {
      connection_size++;
      connection_handle = target_dynamic_strands.connections[connection_handle].next_handle;
    }

    stiff_rod_per_strand_data.back_propagate_begin_connection_handle =
        connection_size % 2 == 0 ? gpu_strand.end_connection_handle
                                 : target_dynamic_strands.connections[gpu_strand.end_connection_handle].prev_handle;

    stiff_rod_per_strand_data.alternative_back_propagate_begin_connection_handle =
        connection_size % 2 == 0 ? target_dynamic_strands.connections[gpu_strand.end_connection_handle].prev_handle
                                 : gpu_strand.end_connection_handle;

    stiff_rod_per_strand_data.back_propagate_begin_segment_handle =
        connection_size % 2 == 1 ? gpu_strand.end_segment_handle
                                 : target_dynamic_strands.segments[gpu_strand.end_segment_handle].prev_handle;

    stiff_rod_per_strand_data.alternative_back_propagate_begin_segment_handle =
        connection_size % 2 == 1 ? target_dynamic_strands.segments[gpu_strand.end_segment_handle].prev_handle
                                 : gpu_strand.end_segment_handle;
  }
}

void DsStiffRod::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                           const DynamicStrands& target_dynamic_strands) {
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  ShearStretchConstraintConstant stretch_shear_constraint_constant;
  stretch_shear_constraint_constant.strand_size = per_strand_data_list.size();
  stretch_shear_constraint_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);
  stretch_shear_constraint_constant.frame_index = physics_parameters.frame_index;
  BendTwistConstraintConstant bend_twist_constraint_constant;
  bend_twist_constraint_constant.strand_size = per_strand_data_list.size();
  bend_twist_constraint_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);
  bend_twist_constraint_constant.frame_index = physics_parameters.frame_index;
  const uint32_t work_group_invocations =
      Platform::Constants::compute_work_group_invocations;
  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (int sub_iteration_index = 0; sub_iteration_index < sub_iteration; sub_iteration_index++) {
      const auto forward_projection = [&]() {
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
                        Platform::DivUp(stretch_shear_constraint_constant.strand_size, work_group_invocations), 1,
                        1);
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
                        Platform::DivUp(bend_twist_constraint_constant.strand_size, work_group_invocations), 1, 1);
          Platform::EverythingBarrier(vk_command_buffer);
        }
      };
      const auto& backward_projection = [&]() {
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
                        Platform::DivUp(stretch_shear_constraint_constant.strand_size, work_group_invocations), 1,
                        1);
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
                        Platform::DivUp(bend_twist_constraint_constant.strand_size, work_group_invocations), 1, 1);
          Platform::EverythingBarrier(vk_command_buffer);
        }
      };
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
                          Platform::DivUp(stretch_shear_constraint_constant.strand_size, work_group_invocations),
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
                          Platform::DivUp(bend_twist_constraint_constant.strand_size, work_group_invocations), 1,
                          1);
            Platform::EverythingBarrier(vk_command_buffer);
          }
          break;
        case ProjectMode::AlternatingDirection: {
          if (physics_parameters.frame_index % 2 == 0) {
            forward_projection();
          } else {
            backward_projection();
          }
          break;
        }
        case ProjectMode::Forward:
          forward_projection();
          break;
        case ProjectMode::Backward:
          backward_projection();
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
  if (!bundle_stretch_shear_offset_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/RandomBundleShearStretchOffset.comp");
    bundle_stretch_shear_offset_pipeline = std::make_shared<ComputePipeline>();
    bundle_stretch_shear_offset_pipeline->compute_shader = shader;
    bundle_stretch_shear_offset_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = bundle_stretch_shear_offset_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RandomBundleShearStretchConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bundle_stretch_shear_offset_pipeline->Initialize();
  }

  if (!bundle_bend_twist_offset_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/RandomBundleBendTwistOffset.comp");
    bundle_bend_twist_offset_pipeline = std::make_shared<ComputePipeline>();
    bundle_bend_twist_offset_pipeline->compute_shader = shader;
    bundle_bend_twist_offset_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);

    auto& push_constant_range = bundle_bend_twist_offset_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RandomBundleBendTwistConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    bundle_bend_twist_offset_pipeline->Initialize();
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

    auto& push_constant_range = bundle_offset_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RandomBundleConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
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

    auto& push_constant_range = bundle_apply_segments_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(RandomBundleApplySegmentsConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    bundle_apply_segments_pipeline->Initialize();
  }

  if (!connections_correction_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->Set(ShaderType::Compute, Platform::Constants::shader_global_defines,
                std::filesystem::path("./EcoSysLabResources") /
                    "Shaders/Compute/DynamicStrands/Constraints/RandomBundleApplyConnections.comp");

    connections_correction_pipeline = std::make_shared<ComputePipeline>();
    connections_correction_pipeline->compute_shader = shader;
    connections_correction_pipeline->descriptor_set_layouts.emplace_back(DynamicStrands::strands_layout);
    auto& stretch_shear_push_constant_range = connections_correction_pipeline->push_constant_ranges.emplace_back();
    stretch_shear_push_constant_range.size = sizeof(RandomBundleApplyConnectionsConstant);
    stretch_shear_push_constant_range.offset = 0;
    stretch_shear_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
    connections_correction_pipeline->Initialize();
  }
}

void DsRandomBundle::ProjectPositionConstraint(const DynamicStrands::PhysicsParameters& physics_parameters,
                                               const DynamicStrands& target_dynamic_strands) {
  if (target_dynamic_strands.segment_pairs.empty())
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  RandomBundleConstant constraint_constant;
  constraint_constant.segment_size = static_cast<uint32_t>(target_dynamic_strands.segment_data_list.size());
  constraint_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);
  constraint_constant.over_relaxation = over_relaxation;

  RandomBundleBendTwistConstant bend_twist_constraint_constant;
  bend_twist_constraint_constant.segment_size = static_cast<uint32_t>(target_dynamic_strands.segment_data_list.size());
  bend_twist_constraint_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);
  bend_twist_constraint_constant.over_relaxation = bend_twist_over_relaxation;

  RandomBundleShearStretchConstant stretch_shear_constraint_constant;
  stretch_shear_constraint_constant.segment_size =
      static_cast<uint32_t>(target_dynamic_strands.segment_data_list.size());
  stretch_shear_constraint_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);

  RandomBundleApplySegmentsConstant constraint_apply_segments_constant;
  constraint_apply_segments_constant.segment_size =
      static_cast<uint32_t>(target_dynamic_strands.segment_data_list.size());
  constraint_apply_segments_constant.inv_time_step = 1.f / (physics_parameters.time_step / physics_parameters.sub_step);

  RandomBundleApplyConnectionsConstant constraint_apply_connections_constant;
  constraint_apply_connections_constant.connection_size =
      static_cast<uint32_t>(target_dynamic_strands.connections.size());
  constraint_apply_connections_constant.inv_time_step =
      1.f / (physics_parameters.time_step / physics_parameters.sub_step);

  const uint32_t work_group_invocations =
      Platform::Constants::compute_work_group_invocations;

  Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
    for (int sub_iteration_index = 0; sub_iteration_index < sub_iteration; sub_iteration_index++) {
      const auto apply_segment_offset = [&]() {
        bundle_apply_segments_pipeline->Bind(vk_command_buffer);
        bundle_apply_segments_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        bundle_apply_segments_pipeline->PushConstant(vk_command_buffer, 0, constraint_apply_segments_constant);
        vkCmdDispatch(vk_command_buffer,
                      Platform::DivUp(constraint_apply_segments_constant.segment_size, work_group_invocations), 1,
                      1);
        Platform::EverythingBarrier(vk_command_buffer);
      };
      const auto correct_connections = [&]() {
        connections_correction_pipeline->Bind(vk_command_buffer);
        connections_correction_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        connections_correction_pipeline->PushConstant(vk_command_buffer, 0, constraint_apply_connections_constant);
        vkCmdDispatch(
            vk_command_buffer,
            Platform::DivUp(constraint_apply_connections_constant.connection_size, work_group_invocations), 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      };
      const auto calculate_stretch_shear_offset = [&]() {
        bundle_stretch_shear_offset_pipeline->Bind(vk_command_buffer);
        bundle_stretch_shear_offset_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        bundle_stretch_shear_offset_pipeline->PushConstant(vk_command_buffer, 0, stretch_shear_constraint_constant);
        vkCmdDispatch(vk_command_buffer,
                      Platform::DivUp(stretch_shear_constraint_constant.segment_size, work_group_invocations), 1,
                      1);
        Platform::EverythingBarrier(vk_command_buffer);
      };
      const auto calculate_bend_twist_offset = [&]() {
        bundle_bend_twist_offset_pipeline->Bind(vk_command_buffer);
        bundle_bend_twist_offset_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        bundle_bend_twist_offset_pipeline->PushConstant(vk_command_buffer, 0, bend_twist_constraint_constant);
        vkCmdDispatch(vk_command_buffer,
                      Platform::DivUp(bend_twist_constraint_constant.segment_size, work_group_invocations), 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      };
      const auto calculate_offset = [&]() {
        bundle_offset_pipeline->Bind(vk_command_buffer);
        bundle_offset_pipeline->BindDescriptorSet(
            vk_command_buffer, 0,
            target_dynamic_strands.strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        bundle_offset_pipeline->PushConstant(vk_command_buffer, 0, constraint_constant);
        vkCmdDispatch(vk_command_buffer, Platform::DivUp(constraint_constant.segment_size, work_group_invocations),
                      1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      };

      if (enable_bundle) {
        calculate_offset();
      }
      if (enable_bend_twist) {
        calculate_bend_twist_offset();
        apply_segment_offset();
        correct_connections();
      }
      if (enable_stretch_shear) {
        calculate_stretch_shear_offset();
        apply_segment_offset();
        correct_connections();
      }
    }
  });
}

bool DsRandomBundle::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("Random Bundle")) {
    if (ImGui::Checkbox("Enable", &enabled))
      changed = true;
    if (enabled) {
      if (ImGui::Checkbox("Enable bundle", &enable_bundle))
        changed = true;

      if (ImGui::Checkbox("Enable bend twist", &enable_bend_twist))
        changed = true;

      if (ImGui::Checkbox("Enable stretch shear", &enable_stretch_shear))
        changed = true;
    }
    if (ImGui::DragInt("Sub iteration", &sub_iteration, 1, 1, 100))
      changed = true;

    if (ImGui::DragFloat("Over relaxation", &over_relaxation, 0.01f, 1, 10.f))
      changed = true;
    if (ImGui::DragFloat("Bend Twist over relaxation", &bend_twist_over_relaxation, 0.01f, 1, 10.f))
      changed = true;
    ImGui::TreePop();
  }
  return changed;
}