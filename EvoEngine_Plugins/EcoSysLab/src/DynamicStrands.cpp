#include "DynamicStrands.hpp"
#include "DsColliders.hpp"
#include "DsConstraints.hpp"
#include "DsPhysics.hpp"
#include "DynamicStrandUtils.hpp"
#include "FoliageDescriptor.hpp"
#include "Shader.hpp"
#include "UVMapUtils.hpp"
#include "glm/gtc/matrix_access.hpp"
#include "glm/gtx/quaternion.hpp"
using namespace eco_sys_lab_plugin;

#ifdef USE_CGAL
inline glm::vec3 cgal_to_glm(const Point_CGAL& p) {
  return {p.x(), p.y(), p.z()};
}
#endif
void DynamicStrands::Physics(const PhysicsParameters& physics_parameters,
                             const std::function<void()>& pre_step_action) {
  if (pre_step)
    pre_step->Execute(physics_parameters, *this);
  pre_step_action();
  for (int sub_step_index = 0; sub_step_index < physics_parameters.sub_step; sub_step_index++) {
    if (prediction)
      prediction->Execute(physics_parameters, *this);
    if (sub_step_index == 0) {
      if (physics_parameters.enable_segment_breaking || physics_parameters.enable_segment_disconnection ||
          physics_parameters.enable_foliage_detachment) {
        breaking->Execute(physics_parameters, *this);
        CalculateGroups(physics_parameters);
      }
    }
    for (int iteration_i = 0; iteration_i < physics_parameters.position_constraint_iteration; iteration_i++) {
      for (const auto& c : constraints) {
        if (c->enabled)
          c->ProjectPositionConstraint(physics_parameters, *this);
      }
    }
    const auto scene = Application::GetActiveScene();
    const auto* box_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsBoxCollider>();
    const auto* sphere_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsSphereCollider>();
    const auto* cylinder_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsCylinderCollider>();
    const auto for_each_collider_entity =
        [&](const std::function<void(const std::shared_ptr<IDsCollider>& dts)>& action) {
          if (box_collider_entities && !box_collider_entities->empty()) {
            for (const auto& i : *box_collider_entities) {
              const auto box_collider = scene->GetOrSetPrivateComponent<DsBoxCollider>(i).lock();
              if (scene->IsEntityEnabled(i) && box_collider->IsEnabled())
                action(std::dynamic_pointer_cast<IDsCollider>(box_collider));
            }
          }
          if (sphere_collider_entities && !sphere_collider_entities->empty()) {
            for (const auto& i : *sphere_collider_entities) {
              const auto sphere_collider = scene->GetOrSetPrivateComponent<DsSphereCollider>(i).lock();
              if (scene->IsEntityEnabled(i) && sphere_collider->IsEnabled())
                action(std::dynamic_pointer_cast<IDsCollider>(sphere_collider));
            }
          }
          if (cylinder_collider_entities && !cylinder_collider_entities->empty()) {
            for (const auto& i : *cylinder_collider_entities) {
              const auto cylinder_collider = scene->GetOrSetPrivateComponent<DsCylinderCollider>(i).lock();
              if (scene->IsEntityEnabled(i) && cylinder_collider->IsEnabled())
                action(std::dynamic_pointer_cast<IDsCollider>(cylinder_collider));
            }
          }
        };
    for_each_collider_entity([&](const std::shared_ptr<IDsCollider>& dts) {
      dts->ProjectPositionConstraint(physics_parameters, *this);
    });
    if (velocity_update)
      velocity_update->Execute(physics_parameters, *this);

    for (int iteration_i = 0; iteration_i < physics_parameters.velocity_constraint_iteration; iteration_i++) {
      for (const auto& c : constraints) {
        if (c->enabled)
          c->ProjectVelocityConstraint(physics_parameters, *this);
      }
    }

    for_each_collider_entity([&](const std::shared_ptr<IDsCollider>& dts) {
      dts->ProjectVelocityConstraint(physics_parameters, *this);
    });

    simulated_time += physics_parameters.time_step / static_cast<float>(physics_parameters.sub_step);
  }
  if (physics_parameters.enable_segment_collision) {
    dynamic_hashed_grid->BuildGrid(physics_parameters, *this);
    segment_collision->Execute(physics_parameters, *this);
  }

  frame_index++;
}

struct TetrahedronFilteringPushConstant {
  uint32_t tetrahedrons_size = 0;
  float alpha = 0.0f;
  float bifurcation_alpha = 0.0f;
  float max_dist_squared = 0.0f;
  int render_complex = 0;
  float degen_triangle_threshold = 0.0f;
  float break_threshold = 0.02f;
  int persistent_damage;
};

DynamicStrands::DynamicStrands() {
#ifdef USE_RENDERDOC
  if (rdoc_api == nullptr) {
    if (HMODULE mod = GetModuleHandleA("renderdoc.dll")) {
      pRENDERDOC_GetAPI RENDERDOC_GetAPI = (pRENDERDOC_GetAPI)GetProcAddress(mod, "RENDERDOC_GetAPI");
      int ret = RENDERDOC_GetAPI(eRENDERDOC_API_Version_1_1_2, (void**)&rdoc_api);
      assert(ret == 1);
    }
  }
#endif

  if (!strands_layout) {
    strands_layout = std::make_shared<DescriptorSetLayout>();
    strands_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(6, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(7, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(8, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(9, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->Initialize();
  }
  VkBufferCreateInfo buffer_create_info{};
  buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  buffer_create_info.usage =
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
  buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  buffer_create_info.size = 1;
  VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
  buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  device_strands_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_nodes_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segments_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_pairs_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_data_list_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_uniform_particles_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_delaunay_tetrahedrons_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_hashed_grid_elements_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_hashed_grid_cell_starts_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_foliage_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
  strands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : strands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(strands_layout);
  }
  pre_step = std::make_shared<DsPreStep>();
  prediction = std::make_shared<DsPrediction>();
  velocity_update = std::make_shared<DsVelocityUpdate>();
  dynamic_hashed_grid = std::make_shared<DsDynamicHashedGrid>();
  segment_collision = std::make_shared<DsSegmentCollision>();
  breaking = std::make_shared<DsBreaking>();

  BuildRenderComputePipelines();
  BuildBranchesRenderingPipelines();
  BuildSmallSegmentsRenderingPipelines();
  BuildFoliageRenderingPipelines();
}

void DynamicStrands::BuildRenderComputePipelines() {
  // Tetrahedrons
  branches_tetrahedron_filtering_pipeline = std::make_shared<ComputePipeline>();
  branches_tetrahedron_filtering_pipeline->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Compute/DynamicStrands/Rendering/TetrahedronFiltering.comp");
  branches_tetrahedron_filtering_pipeline->descriptor_set_layouts.emplace_back(strands_layout);

  auto& tetrahedron_filtering_push_constant_range =
      branches_tetrahedron_filtering_pipeline->push_constant_ranges.emplace_back();
  tetrahedron_filtering_push_constant_range.size = sizeof(TetrahedronFilteringPushConstant);
  tetrahedron_filtering_push_constant_range.offset = 0;
  tetrahedron_filtering_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  branches_tetrahedron_filtering_pipeline->Initialize();

  // Triangles
  branches_triangle_filtering_pipeline = std::make_shared<ComputePipeline>();
  branches_triangle_filtering_pipeline->compute_shader =
      Shader::CreateTemporary(ShaderType::Compute, Platform::Constants::shader_global_defines,
                              std::filesystem::path("./EcoSysLabResources") /
                                  "Shaders/Compute/DynamicStrands/Rendering/TriangleFiltering.comp");
  branches_triangle_filtering_pipeline->descriptor_set_layouts.emplace_back(strands_layout);

  auto& triangle_filtering_push_constant_range =
      branches_triangle_filtering_pipeline->push_constant_ranges.emplace_back();
  triangle_filtering_push_constant_range.size = sizeof(TetrahedronFilteringPushConstant);
  triangle_filtering_push_constant_range.offset = 0;
  triangle_filtering_push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  branches_triangle_filtering_pipeline->Initialize();
}

void DynamicStrands::RenderCompute(const BranchesRenderParameters& branches_render_parameters,
                                   const SmallSegmentsRenderParameters& small_segments_render_parameters,
                                   const FoliageRenderParameters& foliage_render_parameters) const {
  if (segments.empty())
    return;
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  // Tetrahedrons
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    TetrahedronFilteringPushConstant filtering_push_constant;
    filtering_push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
    filtering_push_constant.alpha = branches_render_parameters.alpha;
    filtering_push_constant.bifurcation_alpha = branches_render_parameters.bifurcation_alpha;
    filtering_push_constant.max_dist_squared = branches_render_parameters.max_dist_squared;
    filtering_push_constant.render_complex = branches_render_parameters.render_complex ? 1 : 0;
    filtering_push_constant.degen_triangle_threshold =
        pow(10.0f, -branches_render_parameters.degen_triangle_threshold_logairthmic);
    filtering_push_constant.break_threshold = branches_render_parameters.break_threshold;
    filtering_push_constant.persistent_damage = branches_render_parameters.persistent_damage ? 1 : 0;

    branches_tetrahedron_filtering_pipeline->Bind(vk_command_buffer);
    branches_tetrahedron_filtering_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_tetrahedron_filtering_pipeline->PushConstant(vk_command_buffer, 0, filtering_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(filtering_push_constant.tetrahedrons_size, work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });

  // Triangles
  Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
    TetrahedronFilteringPushConstant filtering_push_constant;
    filtering_push_constant.tetrahedrons_size = delaunay_tetrahedrons.size();
    filtering_push_constant.alpha = branches_render_parameters.alpha;
    filtering_push_constant.bifurcation_alpha = branches_render_parameters.bifurcation_alpha;
    filtering_push_constant.max_dist_squared = branches_render_parameters.max_dist_squared;
    filtering_push_constant.render_complex = branches_render_parameters.render_complex ? 1 : 0;
    filtering_push_constant.degen_triangle_threshold =
        pow(10.0f, -branches_render_parameters.degen_triangle_threshold_logairthmic);
    filtering_push_constant.break_threshold = branches_render_parameters.break_threshold;
    branches_triangle_filtering_pipeline->Bind(vk_command_buffer);
    branches_triangle_filtering_pipeline->BindDescriptorSet(
        vk_command_buffer, 0, strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
    branches_triangle_filtering_pipeline->PushConstant(vk_command_buffer, 0, filtering_push_constant);
    vkCmdDispatch(vk_command_buffer, Platform::DivUp(filtering_push_constant.tetrahedrons_size, work_group_invocations),
                  1, 1);
    Platform::EverythingBarrier(vk_command_buffer);
  });
}

uint32_t DynamicStrands::GetFrameIndex() const {
  return frame_index;
}

float DynamicStrands::GetSimulatedTime() const {
  return simulated_time;
}

bool DynamicStrands::InitializeParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Min segment length", &min_segment_length, 0.001f, 0.001f, max_segment_length))
    changed = true;
  if (ImGui::DragFloat("Max segment length", &max_segment_length, 0.001f, min_segment_length, 1.0f))
    changed = true;

  if (ImGui::DragInt("Uniform subdivision", &uniform_subdivision, 1, 1, 16)) {
    uniform_subdivision = glm::clamp(uniform_subdivision, 1, 16);
    changed = true;
  }
  if (ImGui::DragFloat("Neighbor vertical range", &neighbor_vertical_range, 0.01f, 0.01f, 10.0f))
    changed = true;
  if (ImGui::DragFloat("Neighbor horizontal range", &neighbor_horizontal_range, 0.01f, 0.01f, 10.0f))
    changed = true;
  if (ImGui::TreeNode("Material Properties")) {
    PlottedDistributionSettings wood_density_settings{};
    if (wood_density.OnInspect("Wood Density", wood_density_settings))
      changed = true;
    PlottedDistributionSettings wood_young_settings{};
    if (max_youngs_modulus.OnInspect("Wood Young's modulus", wood_young_settings))
      changed = true;
    PlottedDistributionSettings wood_bending_settings{};
    if (max_bending_modulus.OnInspect("Wood Bending modulus", wood_bending_settings))
      changed = true;
    PlottedDistributionSettings wood_torsion_settings{};
    if (max_torsion_modulus.OnInspect("Wood Torsion modulus", wood_torsion_settings))
      changed = true;

    PlottedDistributionSettings max_bend_strain_settings{};
    if (max_bend_strain.OnInspect("Max bend strain", max_bend_strain_settings))
      changed = true;
    PlottedDistributionSettings max_twist_strain_settings{};
    if (max_twist_strain.OnInspect("Max twist strain", max_twist_strain_settings))
      changed = true;

    PlottedDistributionSettings max_bundle_strain_settings{};
    if (max_bundle_strain.OnInspect("Max bundle strain", max_bundle_strain_settings))
      changed = true;
    PlottedDistributionSettings max_connectivity_strain_settings{};
    if (max_connectivity_strain.OnInspect("Max connectivity strain", max_connectivity_strain_settings))
      changed = true;

    if (leaf_position_alpha.OnInspect("Leaf position alpha"))
      changed = true;

    if (leaf_rotation_alpha.OnInspect("Leaf rotation alpha"))
      changed = true;

    if (max_leaf_position_strain.OnInspect("Max leaf position strain"))
      changed = true;

    if (max_leaf_rotation_strain.OnInspect("Max leaf rotation strain"))
      changed = true;

    ImGui::TreePop();
  }

  editor_layer->DragAndDropButton<FoliageDescriptor>(foliage_descriptor, "Foliage Descriptor");

  if (ImGui::TreeNode("Meshing Properties")) {
#ifdef USE_CGAL
    if (ImGui::Checkbox("Use CGAL", &use_cgal))
      changed = true;
#endif  // USE_CGAL
    if (ImGui::Checkbox("Triangulate per bundle", &triangulate_per_bundle))
      changed = true;
    if (ImGui::DragFloat("Alpha", &alpha, 0.000001f, 0.0f, 1.0f, "%.6f"))
      changed = true;
    if (ImGui::DragFloat("Bifurcation Alpha", &bifurcation_alpha, 0.000001f, 0.0f, 1.0f, "%.6f"))
      changed = true;
    if (ImGui::DragFloat("Max Distance Squared", &max_dist_squared, 0.000001f, 0.0f, 1.0f, "%.6f"))
      changed = true;

    ImGui::TreePop();
  }

  return changed;
}

bool DynamicStrands::PhysicsParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Time step", &time_step, 0.001f, 0.001f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Sub step", &sub_step, 1, 1, 100)) {
    changed = true;
  }
  if (ImGui::Checkbox("Segment breaking", &enable_segment_breaking)) {
    changed = true;
  }
  if (enable_segment_breaking) {
    if (ImGui::DragInt("Segment breaking detection frame", &segment_breaking_detection_frame, 1, 1, 500))
      changed = true;
  }
  if (ImGui::Checkbox("Segment disconnection", &enable_segment_disconnection)) {
    changed = true;
  }
  if (enable_segment_disconnection) {
    if (ImGui::DragInt("Segment disconnection detection frame", &segment_disconnection_detection_frame, 1, 1, 500))
      changed = true;
  }

  if (ImGui::Checkbox("Foliage detachment", &enable_foliage_detachment)) {
    changed = true;
  }
  if (enable_foliage_detachment) {
    if (ImGui::DragInt("Foliage detachment detection frame", &foliage_detachment_detection_frame, 1, 1, 500))
      changed = true;
  }

  if (ImGui::Checkbox("Dynamic Grouping", &dynamic_grouping)) {
    changed = true;
  }
  if (!dynamic_grouping) {
    if (ImGui::DragInt("Grouping iteration", &grouping_iteration, 1, 1, 500)) {
      grouping_iteration = glm::clamp(grouping_iteration, 1, 500);
      changed = true;
    }
  }
  if (ImGui::Checkbox("Segment collision", &enable_segment_collision)) {
    changed = true;
  }
  if (ImGui::DragInt("Position constraint iteration", &position_constraint_iteration, 1, 1, 50))
    changed = true;
  if (ImGui::DragInt("Velocity constraint iteration", &velocity_constraint_iteration, 1, 1, 50))
    changed = true;
  if (ImGui::DragFloat("Segment Velocity damping", &segment_velocity_damping, 0.0001f, 0.f, 1.f, "%.4f"))
    changed = true;
  if (ImGui::DragFloat("Segment Angular velocity damping", &segment_angular_velocity_damping, 0.00001f, 0.f, 1.f,
                       "%.5f"))
    changed = true;
  if (ImGui::DragFloat("Leaf Velocity damping", &leaf_velocity_damping, 0.0001f, 0.f, 1.f, "%.4f"))
    changed = true;
  if (ImGui::DragFloat("Leaf Angular velocity damping", &leaf_angular_velocity_damping, 0.00001f, 0.f, 1.f, "%.5f"))
    changed = true;

  if (ImGui::DragFloat3("Gravity", &gravity.x, 1.f))
    changed = true;

  return changed;
}

void DynamicStrands::UpdateBindings() const {
  if (segments.empty())
    return;
  const auto current_frame_index = Platform::GetCurrentFrameIndex();

  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(0, device_strands_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(1, device_nodes_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(2, device_segments_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(3, device_segment_pairs_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(4, device_segment_data_list_buffer, 0);

  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(5, device_uniform_particles_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(6, device_delaunay_tetrahedrons_buffer,
                                                                              0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(7, device_hashed_grid_elements_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(8, device_hashed_grid_cell_starts_buffer,
                                                                              0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(9, device_foliage_buffer, 0);
  for (const auto& c : constraints) {
    c->UpdateBindings();
  }
}

glm::vec3 DynamicStrands::GpuSegment::GetCenterX0() const {
  return (particle0.x0 + particle1.x0) * .5f;
}

void DynamicStrands::Upload() {
  device_strands_buffer->UploadVector(strands);
  device_nodes_buffer->UploadVector(nodes);
  device_segments_buffer->UploadVector(segments);
  device_segment_pairs_buffer->UploadVector(segment_pairs);
  device_segment_data_list_buffer->UploadVector(segment_data_list);
  device_uniform_particles_buffer->UploadVector(uniform_particles);
  device_delaunay_tetrahedrons_buffer->UploadVector(delaunay_tetrahedrons);
  device_hashed_grid_elements_buffer->UploadVector(hashed_grid_elements);
  device_hashed_grid_cell_starts_buffer->UploadVector(hashed_grid_cell_starts);
  device_foliage_buffer->UploadVector(foliage);
  for (const auto& c : constraints) {
    c->UploadData();
  }
  UpdateBindings();
  frame_index = 0;
}

void DynamicStrands::Download() {
  if (!strands.empty())
    device_strands_buffer->DownloadVector(strands, strands.size());
  if (!nodes.empty())
    device_nodes_buffer->DownloadVector(nodes, nodes.size());
  if (!segments.empty())
    device_segments_buffer->DownloadVector(segments, segments.size());
  if (!segment_pairs.empty())
    device_segment_pairs_buffer->DownloadVector(segment_pairs, segment_pairs.size());
  if (!segment_data_list.empty())
    device_segment_data_list_buffer->DownloadVector(segment_data_list, segment_data_list.size());
  if (!uniform_particles.empty())
    device_uniform_particles_buffer->DownloadVector(uniform_particles, uniform_particles.size());
  if (!delaunay_tetrahedrons.empty())
    device_delaunay_tetrahedrons_buffer->DownloadVector(delaunay_tetrahedrons, delaunay_tetrahedrons.size());
  if (!hashed_grid_elements.empty())
    device_hashed_grid_elements_buffer->DownloadVector(hashed_grid_elements, hashed_grid_elements.size());
  if (!hashed_grid_cell_starts.empty())
    device_hashed_grid_cell_starts_buffer->DownloadVector(hashed_grid_cell_starts, hashed_grid_cell_starts.size());
  if (!foliage.empty())
    device_foliage_buffer->DownloadVector(foliage, foliage.size());
  for (const auto& c : constraints) {
    c->DownloadData();
  }
}

void DynamicStrands::CalculateGroups(const PhysicsParameters& physics_parameters) const {
  if (segments.empty())
    return;

  // Below will be executed at the start of next frame.
  struct GroupingPushConstant {
    uint32_t segment_size;
  };

  static std::shared_ptr<ComputePipeline> reset_pipeline, step_pipeline, dynamic_step_pipeline, apply_pipeline{};
  static std::shared_ptr<Buffer> feedback_buffer;
  static std::shared_ptr<Buffer> new_group_index_buffer;

  static std::shared_ptr<DescriptorSetLayout> grouping_layout{};
  static std::shared_ptr<DescriptorSetLayout> dynamic_grouping_layout{};
  static std::shared_ptr<DescriptorSet> grouping_descriptor_set{};
  static std::shared_ptr<DescriptorSet> dynamic_grouping_descriptor_set{};

  if (!reset_pipeline) {
    std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Grouping/Reset.comp");
    reset_pipeline = std::make_shared<ComputePipeline>();
    reset_pipeline->compute_shader = shader;
    reset_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    auto& push_constant_range = reset_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroupingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    reset_pipeline->Initialize();
  }

  if (!grouping_layout) {
    grouping_layout = std::make_shared<DescriptorSetLayout>();
    grouping_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    grouping_layout->Initialize();
  }

  if (!dynamic_grouping_layout) {
    dynamic_grouping_layout = std::make_shared<DescriptorSetLayout>();
    dynamic_grouping_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                   0);
    dynamic_grouping_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                   0);
    dynamic_grouping_layout->Initialize();
  }

  if (!step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Grouping/Step.comp");
    step_pipeline = std::make_shared<ComputePipeline>();
    step_pipeline->compute_shader = shader;
    step_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    step_pipeline->descriptor_set_layouts.emplace_back(grouping_layout);
    auto& push_constant_range = step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroupingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    step_pipeline->Initialize();
  }
  if (!dynamic_step_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Grouping/DynamicStep.comp");
    dynamic_step_pipeline = std::make_shared<ComputePipeline>();
    dynamic_step_pipeline->compute_shader = shader;
    dynamic_step_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    dynamic_step_pipeline->descriptor_set_layouts.emplace_back(dynamic_grouping_layout);
    auto& push_constant_range = dynamic_step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroupingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    dynamic_step_pipeline->Initialize();
  }
  if (!apply_pipeline) {
    static std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Grouping/Apply.comp");
    apply_pipeline = std::make_shared<ComputePipeline>();
    apply_pipeline->compute_shader = shader;
    apply_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    apply_pipeline->descriptor_set_layouts.emplace_back(grouping_layout);
    auto& push_constant_range = apply_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroupingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    apply_pipeline->Initialize();
  }
  if (!new_group_index_buffer) {
    VkBufferCreateInfo buffer_create_info{};
    buffer_create_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    buffer_create_info.usage =
        VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    buffer_create_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    buffer_create_info.size = 1;
    VmaAllocationCreateInfo buffer_vma_allocation_create_info{};
    buffer_vma_allocation_create_info.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
    feedback_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
    new_group_index_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  }
  if (!dynamic_grouping_descriptor_set) {
    dynamic_grouping_descriptor_set = std::make_shared<DescriptorSet>(dynamic_grouping_layout);
  }
  if (!grouping_descriptor_set) {
    grouping_descriptor_set = std::make_shared<DescriptorSet>(grouping_layout);
  }
  const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

  GroupingPushConstant push_constant;
  push_constant.segment_size = segments.size();
  const auto current_frame_index = Platform::GetCurrentFrameIndex();
  const auto group_size = Platform::DivUp(segments.size(), work_group_invocations);
  std::vector<uint32_t> feedback(group_size);

  new_group_index_buffer->Resize(sizeof(int) * segments.size());
  grouping_descriptor_set->UpdateBufferDescriptorBinding(0, new_group_index_buffer);

  if (physics_parameters.dynamic_grouping) {
    const auto start_time = Times::Now();
    feedback_buffer->Resize(sizeof(uint32_t) * group_size);
    dynamic_grouping_descriptor_set->UpdateBufferDescriptorBinding(0, new_group_index_buffer);
    dynamic_grouping_descriptor_set->UpdateBufferDescriptorBinding(1, feedback_buffer);

    Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
      reset_pipeline->Bind(vk_command_buffer);
      reset_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                        strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      reset_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      vkCmdDispatch(vk_command_buffer, group_size, 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
    });
    bool updated = true;
    const auto step = [&]() {
      Platform::ImmediateSubmit([&](const VkCommandBuffer vk_command_buffer) {
        vkCmdFillBuffer(vk_command_buffer, feedback_buffer->GetVkBuffer(), 0, VK_WHOLE_SIZE, 0);
        Platform::EverythingBarrier(vk_command_buffer);
        dynamic_step_pipeline->Bind(vk_command_buffer);
        dynamic_step_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                                 strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        dynamic_step_pipeline->BindDescriptorSet(vk_command_buffer, 1,
                                                 dynamic_grouping_descriptor_set->GetVkDescriptorSet());
        dynamic_step_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        vkCmdDispatch(vk_command_buffer, group_size, 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);

        apply_pipeline->Bind(vk_command_buffer);
        apply_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                          strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        apply_pipeline->BindDescriptorSet(vk_command_buffer, 1, grouping_descriptor_set->GetVkDescriptorSet());
        apply_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        vkCmdDispatch(vk_command_buffer, group_size, 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      });
      feedback_buffer->DownloadVector(feedback, feedback.size());
    };
    static int max_iterations = 0;
    int iterations = 0;
    while (updated) {
      updated = false;
      step();
      for (const auto& i : feedback) {
        if (i != 0) {
          updated = true;
          break;
        }
      }
      iterations++;
    }
    max_iterations = glm::max(iterations, max_iterations);
    // EVOENGINE_LOG("Iterations: " + std::to_string(iterations), + ", max: " + std::to_string(max_iterations));
    const auto method3_time = std::to_string(Times::Now() - start_time);

  } else {
    Platform::RecordCommandsMainQueue([&](const VkCommandBuffer vk_command_buffer) {
      reset_pipeline->Bind(vk_command_buffer);
      reset_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                        strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
      reset_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
      vkCmdDispatch(vk_command_buffer, group_size, 1, 1);
      Platform::EverythingBarrier(vk_command_buffer);
      for (int iteration = 0; iteration < physics_parameters.grouping_iteration; iteration++) {
        step_pipeline->Bind(vk_command_buffer);
        step_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        step_pipeline->BindDescriptorSet(vk_command_buffer, 1, grouping_descriptor_set->GetVkDescriptorSet());
        step_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        vkCmdDispatch(vk_command_buffer, group_size, 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);

        apply_pipeline->Bind(vk_command_buffer);
        apply_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                          strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        apply_pipeline->BindDescriptorSet(vk_command_buffer, 1, grouping_descriptor_set->GetVkDescriptorSet());
        apply_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        vkCmdDispatch(vk_command_buffer, group_size, 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      }
    });
  }
}

void DynamicStrands::Clear() {
  strands.clear();
  nodes.clear();
  segments.clear();
  segment_pairs.clear();
  segment_data_list.clear();
  uniform_particles.clear();
  delaunay_tetrahedrons.clear();
  hashed_grid_elements.clear();
  hashed_grid_cell_starts.clear();
}

glm::vec3 DynamicStrands::ComputeInertiaTensorBox(const float mass, const float width, const float height,
                                                  const float depth) {
  return {
      mass / 12.f * (height * height + depth * depth),
      mass / 12.f * (width * width + depth * depth),
      mass / 12.f * (width * width + height * height),
  };
}

glm::vec3 DynamicStrands::ComputeInertiaTensorRod(const float mass, const float radius, const float length) {
  float factor = mass / 12.f * (3.f * radius * radius + length * length);
  return {factor, factor, mass / 2.f * radius * radius};
}

void DynamicStrands::ComputeDelaunayPerBundle(std::vector<GpuDelaunayTetrahedron>& tetrahedrons, bool use_cgal) {
  int max_dist_from_root = 0;

  for (int i = 0; i < uniform_particles.size(); i++) {
    max_dist_from_root = std::max(max_dist_from_root, uniform_particles[i].segment_index);
  }

  std::vector<std::map<int, std::vector<size_t>>> bundle_maps(max_dist_from_root + 1);
  std::vector<size_t> offsets(max_dist_from_root + 1, 0);
  std::vector<std::vector<size_t>> particle_adjacent_tets(uniform_particles.size(), std::vector<size_t>{});

  for (int i = 0; i < uniform_particles.size(); i++) {
    auto& particle = uniform_particles[i];
    auto& node_handle = particle.node_index;

    if (bundle_maps[particle.segment_index].find(node_handle) == bundle_maps[particle.segment_index].end()) {
      bundle_maps[particle.segment_index][node_handle] = std::vector<size_t>();
    }

    bundle_maps[particle.segment_index][node_handle].push_back(i);
  }

  // TODO: "squish" each bundle such that no internal degenerate tetrahedrons occur
#ifdef USE_CGAL
  // triangulate each bundle:
  for (int d = 0; d < bundle_maps.size(); d++) {
    offsets[d] = tetrahedrons.size();
    auto& map = bundle_maps[d];
    for (auto& kv_pair : map) {
      auto& bundle = kv_pair.second;
      std::vector<std::pair<Point_CGAL, unsigned>> points;

      if (bundle.size() < 3) {
        continue;
      }

      for (size_t i : bundle) {
        auto& particle = uniform_particles[i];

        if (particle.next_particle_handle == -1) {
          continue;
          // TODO: probably even means we can skip this bundle entirely
        }

        auto& next_particle = uniform_particles[particle.next_particle_handle];

        float squish_weight = 0.0f;

        glm::vec3 squished_position =
            squish_weight * particle.position + (1.0f - squish_weight) * next_particle.position;

        Point_CGAL p0_cgal(particle.position[0], particle.position[1], particle.position[2]);
        Point_CGAL p1_cgal(squished_position[0], squished_position[1], squished_position[2]);

        points.emplace_back(p0_cgal, i);
        points.emplace_back(p1_cgal, particle.next_particle_handle);
      }

      CGALDelaunay(points, tetrahedrons);
    }
  }
#else
  for (int d = 0; d < bundle_maps.size(); d++) {
    offsets[d] = tetrahedrons.size();
    auto& map = bundle_maps[d];
    for (auto& kv_pair : map) {
      auto& bundle = kv_pair.second;
      std::vector<glm::vec3> points;
      std::vector<size_t> indices;

      if (bundle.size() < 3) {
        continue;
      }

      int end_of_strand_count = 0;
      for (size_t i : bundle) {
        auto& particle = uniform_particles[i];

        glm::vec3 p0(particle.position[0], particle.position[1], particle.position[2]);
        points.emplace_back(p0);
        indices.emplace_back(i);

        if (particle.next_particle_handle == -1) {
          end_of_strand_count++;
        } else {
          auto& next_particle = uniform_particles[particle.next_particle_handle];

          float squish_weight = 0.0f;

          glm::vec3 squished_position =
              squish_weight * particle.position + (1.0f - squish_weight) * next_particle.position;

          glm::vec3 p1(squished_position[0], squished_position[1], squished_position[2]);

          points.emplace_back(p1);
          indices.emplace_back(particle.next_particle_handle);
        }
      }

      if (end_of_strand_count == bundle.size()) {
        continue;
      }

      TetDelaunay(points, indices, tetrahedrons);
    }
  }
#endif

  std::mutex mtx;

  Jobs::RunParallelFor(tetrahedrons.size(), [&](const size_t tet_index) {
    auto& tet = tetrahedrons[tet_index];

    for (size_t i = 0; i < 4; i++) {
      if (tet.indices[i] == -1) {
        continue;
      }

      if (tet.indices[i] >= particle_adjacent_tets.size()) {
        EVOENGINE_ERROR("particle index out of range, skipping!");
        continue;
      }

      mtx.lock();
      particle_adjacent_tets[tet.indices[i]].emplace_back(tet_index);
      mtx.unlock();
    }
  });

  // now glue them back together
  for (int particle_index = 0; particle_index < uniform_particles.size(); particle_index++) {
    auto& adjacent_tets = particle_adjacent_tets[particle_index];

    // can't be too many, brute force should work here;
    for (size_t i = 0; i < adjacent_tets.size(); i++) {
      for (size_t j = i + 1; j < adjacent_tets.size(); j++) {
        auto& tet0 = tetrahedrons[adjacent_tets[i]];
        auto& tet1 = tetrahedrons[adjacent_tets[j]];

        // check if the two share a face
        size_t occurs_in_both = 0;
        size_t b_in_both = 0;

        for (size_t i = 0; i < 4; i++) {
          for (size_t j = 0; j < 4; j++) {
            if (tet0.indices[i] == tet1.indices[j] && tet0.indices[i] != -1) {
              occurs_in_both++;
            }
          }
        }

        if (occurs_in_both != 3) {
          continue;
        }
        const auto mismatch_indices = DynamicStrandUtils::CompareIndices(tet0.indices, tet1.indices);

        tet0.neighbor_tet_ids[mismatch_indices.first] = adjacent_tets[j];
        tet1.neighbor_tet_ids[mismatch_indices.second] = adjacent_tets[i];
      }
    }
  }
}

#ifdef USE_CGAL
void DynamicStrands::CGALDelaunay(const std::vector<std::pair<Point_CGAL, unsigned>>& points,
                                  std::vector<GpuDelaunayTetrahedron>& tetrahedrons) {
  Delaunay_CGAL dt;
  dt.insert(points.begin(), points.end());

  // TODO: parallel for
  for (auto cell_it = dt.all_cells_begin(); cell_it != dt.all_cells_end(); cell_it++) {
    auto& cell = *cell_it;
    auto& tetrahedron = dt.tetrahedron(cell_it);
    int indices[4];
    for (size_t i = 0; i < 4; i++) {
      indices[i] = cell.vertex(i)->info();
    }
    if (!DynamicStrandUtils::IsValid(indices, uniform_particles.size())) {
      continue;  // discard this tetrahedron
    }

    // only take tetrahedra that sit between two neighboring planes
    if (!DynamicStrandUtils::IsBetweenPlanes(indices, uniform_particles)) {
      continue;
    }

    GpuDelaunayTetrahedron gpu_tet;
    for (size_t i = 0; i < 4; i++) {
      gpu_tet.indices[i] = indices[i];
      gpu_tet.neighbor_tet_ids[i] = -1;
      gpu_tet.is_bark[i] = -1;
    }
    // set up debugging members
    gpu_tet.color = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
    for (int& i : gpu_tet.render_neighbor) {
      i = -1;
    }
    gpu_tet.task_looked_at = 0;
    gpu_tet.mesh_looked_at = 0;
    gpu_tet.inside = 0;
    gpu_tet.triangles_accepted = 0;

    // check orientation of the tetrahedron
    const float d = DynamicStrandUtils::PointPlaneDistance(
        uniform_particles[gpu_tet.indices[3]].position, uniform_particles[gpu_tet.indices[0]].position,
        uniform_particles[gpu_tet.indices[1]].position, uniform_particles[gpu_tet.indices[2]].position);

    if (d > 0)  // point no. 3 is in front if the triangle 0 1 2, we need to correct this so the triangles face outwards
    {
      std::swap(gpu_tet.indices[1], gpu_tet.indices[2]);
    }

    // fill in neighbor indices
    // TODO: need to figure out how to check if a neighbor is valid
    for (size_t i = 0; i < 4; i++) {
      auto& neighbor = *cell.neighbor(i);
      int neighbor_indices[4];

      for (size_t j = 0; j < 4; j++) {
        neighbor_indices[j] = neighbor.vertex(j)->info();
      }

      if (!DynamicStrandUtils::IsValid(neighbor_indices, uniform_particles.size())) {
        continue;
      }

      if (!DynamicStrandUtils::IsBetweenPlanes(neighbor_indices, uniform_particles)) {
        continue;
      }

      const auto mismatch_indices = DynamicStrandUtils::CompareIndices(gpu_tet.indices, neighbor_indices);
      // TODO: according to CGAL documentation, this is guaranteed anyway, so we do not need to match both sides
      // store it such that the neighboring tetrahedron always consists of different indices
      // e. g. for the triangle 1 2 4 we store the corresponding neighboring index at position 3
      // gpu_tet.neighbor_tet_ids[mismatch_indices.first] = neighbor.index();
      EVOENGINE_ERROR("CGAL does not provide neighbor indices");
    }

    tetrahedrons.emplace_back(gpu_tet);
  }
}
#endif

void DynamicStrands::TetDelaunay(const std::vector<glm::vec3>& points, const std::vector<size_t>& particle_indices,
                                 std::vector<GpuDelaunayTetrahedron>& tetrahedrons) {
  const auto tets = Delaunay3D::GenerateTetrahedrons(points);

  std::vector<int> valid_index_map(tets.size(), -1);
  int valid_neighbors = 0;
  for (size_t orig_tet_index = 0; orig_tet_index < tets.size(); orig_tet_index++) {
    auto& tet = tets[orig_tet_index];
    int indices[4];
    bool invalid = false;
    for (size_t i = 0; i < 4; i++) {
      if (tet.v[i] >= particle_indices.size() || tet.v[i] < 0) {
        invalid = true;
        EVOENGINE_LOG("Tetrahedron is invalid");
        break;
      }
      indices[i] = particle_indices[tet.v[i]];
    }
    if (invalid) {
      continue;  // discard this tetrahedron
    }

    // only take tetrahedra that sit between two neighboring planes
    if (!DynamicStrandUtils::IsBetweenPlanes(indices, uniform_particles)) {
      continue;
    }

    GpuDelaunayTetrahedron gpu_tet;
    for (size_t i = 0; i < 4; i++) {
      gpu_tet.indices[i] = indices[i];
      gpu_tet.neighbor_tet_ids[i] = -1;
      gpu_tet.is_bark[i] = -1;
    }
    // set up debugging members
    gpu_tet.color = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
    for (int& i : gpu_tet.render_neighbor) {
      i = -1;
    }
    gpu_tet.task_looked_at = 0;
    gpu_tet.mesh_looked_at = 0;
    gpu_tet.inside = 0;
    gpu_tet.triangles_accepted = 0;

    // check orientation of the tetrahedron
    const float d = DynamicStrandUtils::PointPlaneDistance(
        uniform_particles[gpu_tet.indices[3]].position, uniform_particles[gpu_tet.indices[0]].position,
        uniform_particles[gpu_tet.indices[1]].position, uniform_particles[gpu_tet.indices[2]].position);

    if (d > 0)  // point no. 3 is in front if the triangle 0 1 2, we need to correct this so the triangles face outwards
    {
      std::swap(gpu_tet.indices[1], gpu_tet.indices[2]);
    }

    // fill in neighbor indices
    for (size_t i = 0; i < 4; i++) {
      // check if neighbor is valid
      if (tet.neighbor_tet_indices[i] >= tets.size() || tet.neighbor_tet_indices[i] < 0) {
        continue;
      }

      auto& neighbor = tets[tet.neighbor_tet_indices[i]];
      int neighbor_indices[4];

      bool invalid = false;
      for (size_t j = 0; j < 4; j++) {
        if (neighbor.v[j] >= particle_indices.size()) {
          invalid = true;
          break;
        }

        neighbor_indices[j] = particle_indices[neighbor.v[j]];
      }

      if (invalid) {
        continue;
      }

      if (!DynamicStrandUtils::IsValid(neighbor_indices, uniform_particles.size())) {
        continue;
      }

      if (!DynamicStrandUtils::IsBetweenPlanes(neighbor_indices, uniform_particles)) {
        continue;
      }

      // TODO: probably not needed
      const auto mismatch_indices = DynamicStrandUtils::CompareIndices(gpu_tet.indices, neighbor_indices);

      gpu_tet.neighbor_tet_ids[mismatch_indices.first] = tet.neighbor_tet_indices[i];
      valid_neighbors++;
    }

    valid_index_map[orig_tet_index] = tetrahedrons.size();
    tetrahedrons.emplace_back(gpu_tet);
  }

  // correct neighbor indices
  Jobs::RunParallelFor(tetrahedrons.size(), [&](const size_t tet_index) {
    auto& tet = tetrahedrons[tet_index];
    for (size_t i = 0; i < 4; i++) {
      if (tet.neighbor_tet_ids[i] == -1) {
        continue;
      }
      tet.neighbor_tet_ids[i] = valid_index_map[tet.neighbor_tet_ids[i]];
    }
  });

  EVOENGINE_LOG("Found " << valid_neighbors << " valid neighbors");
}

void DynamicStrands::ComputeDelaunay(std::vector<GpuDelaunayTetrahedron>& tetrahedrons, bool use_cgal) {
// TODO: maybe a different library will work here
#ifdef USE_CGAL
  if (use_cgal) {
    std::vector<std::pair<Point_CGAL, unsigned>> points;
    for (int i = 0; i < uniform_particles.size(); i++) {
      auto& particle = uniform_particles[i];
      glm::vec3 particle_pos = particle.position;
      Point_CGAL p_cgal(particle_pos[0], particle_pos[1], particle_pos[2]);
      points.emplace_back(p_cgal, i);
    }

    CGALDelaunay(points, tetrahedrons);
  }
#endif
  if (!use_cgal) {
    std::vector<glm::vec3> points;
    std::vector<size_t> indices;

    for (int i = 0; i < uniform_particles.size(); i++) {
      auto& particle = uniform_particles[i];
      glm::vec3& particle_pos = particle.position;
      points.emplace_back(particle_pos);
      indices.emplace_back(i);
    }

    TetDelaunay(points, indices, tetrahedrons);
  }
}
