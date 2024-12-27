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
void DynamicStrands::Physics(const PhysicsParameters& physics_parameters, const std::function<void()>& pre_step_action,
                             const std::function<void()>& sub_step_action) {
  pre_step_action();
  for (int sub_step_index = 0; sub_step_index < physics_parameters.sub_step; sub_step_index++) {
    if (pre_step)
      pre_step->Execute(physics_parameters, *this);
    sub_step_action();
    if (prediction)
      prediction->Execute(physics_parameters, *this);
    for (const auto& c : constraints) {
      if (c->enabled)
        for (int iteration_i = 0; iteration_i < physics_parameters.constraint_iteration; iteration_i++) {
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
              action(std::dynamic_pointer_cast<IDsCollider>(box_collider));
            }
          }
          if (sphere_collider_entities && !sphere_collider_entities->empty()) {
            for (const auto& i : *sphere_collider_entities) {
              const auto sphere_collider = scene->GetOrSetPrivateComponent<DsSphereCollider>(i).lock();
              action(std::dynamic_pointer_cast<IDsCollider>(sphere_collider));
            }
          }
          if (cylinder_collider_entities && !cylinder_collider_entities->empty()) {
            for (const auto& i : *cylinder_collider_entities) {
              const auto cylinder_collider = scene->GetOrSetPrivateComponent<DsCylinderCollider>(i).lock();
              action(std::dynamic_pointer_cast<IDsCollider>(cylinder_collider));
            }
          }
        };
    for_each_collider_entity([&](const std::shared_ptr<IDsCollider>& dts) {
      dts->ProjectPositionConstraint(physics_parameters, *this);
    });
    if (velocity_update)
      velocity_update->Execute(physics_parameters, *this);
  }
  dynamic_hashed_grid->BuildGrid(physics_parameters, *this);
  if (physics_parameters.enable_segment_collision) {
    segment_collision->Execute(physics_parameters, *this);
  }
  if (physics_parameters.enable_grouping && frame_index > 0) {
    CalculateGroups(physics_parameters);
  }
  frame_index++;
}

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
  wait_for_upload = true;
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

  BuildRenderingPipelines();
  BuildFoliageRenderingPipelines();
}

uint32_t DynamicStrands::GetFrameIndex() const {
  return frame_index;
}

bool DynamicStrands::WaitForUpload() const {
  return wait_for_upload;
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
    PlottedDistributionSettings wood_shear_settings{};
    if (max_shear_modulus.OnInspect("Wood Shear modulus", wood_shear_settings))
      changed = true;
    PlottedDistributionSettings wood_bending_settings{};
    if (max_bending_modulus.OnInspect("Wood Bending modulus", wood_bending_settings))
      changed = true;
    PlottedDistributionSettings wood_torsion_settings{};
    if (max_torsion_modulus.OnInspect("Wood Torsion modulus", wood_torsion_settings))
      changed = true;

    PlottedDistributionSettings max_bundle_strain_settings{};
    if (max_bundle_strain.OnInspect("Max bundle strain", max_bundle_strain_settings))
      changed = true;
    PlottedDistributionSettings max_shear_strain_settings{};
    if (max_shear_strain.OnInspect("Max shear strain", max_shear_strain_settings))
      changed = true;
    PlottedDistributionSettings max_stretch_strain_settings{};
    if (max_stretch_strain.OnInspect("Max stretch strain", max_stretch_strain_settings))
      changed = true;

    PlottedDistributionSettings max_bend_strain_settings{};
    if (max_bend_strain.OnInspect("Max bend strain", max_bend_strain_settings))
      changed = true;
    PlottedDistributionSettings max_twist_strain_settings{};
    if (max_twist_strain.OnInspect("Max twist strain", max_twist_strain_settings))
      changed = true;

    ImGui::TreePop();
  }

  editor_layer->DragAndDropButton<FoliageDescriptor>(foliage_descriptor, "Foliage Descriptor");

  if (ImGui::TreeNode("Meshing Properties")) {
#ifdef USE_CGAL
    if (ImGui::Checkbox("Use CGAL", &use_cgal))
      changed = true;
#endif  // USE_CGAL
    if (ImGui::DragInt("u-coordinate multiplier", &u_multiplier, 2, 2, 20))
      changed = true;

    if (ImGui::DragFloat("v-coordinate multiplier", &v_multiplier, 0.001f, 0.0f, 100.0f))
      changed = true;

    ImGui::TreePop();
  }

  return changed;
}

void DynamicStrands::Initialize(const InitializeParameters& initialize_parameters,
                                const StrandModelSkeleton& strand_model_skeleton,
                                const StrandModelStrandGroup& strand_model_strand_group,
                                const DtsStrandGroup& strand_group) {
  Clear();
  frame_index = 0;
  assert(initialize_parameters.root_transform.GetScale() == glm::vec3(1.0f));
  const auto& target_strands = strand_group.PeekStrands();
  const auto& target_strand_segments = strand_group.PeekStrandSegments();
  const auto& target_strand_segment_data_list = strand_group.PeekStrandSegmentDataList();
  strands.resize(target_strands.size());

  Jobs::RunParallelFor(target_strands.size(), [&](const size_t i) {
    auto& strand = strands[i];
    const auto& target_strand = target_strands[i];
    const auto& handles = target_strand.PeekStrandSegmentHandles();
    if (!handles.empty()) {
      strand.begin_segment_handle = handles.front();
      strand.end_segment_handle = handles.back();
    } else {
      strand.begin_segment_handle = -1;
      strand.end_segment_handle = -1;
    }
  });
  segments.resize(target_strand_segments.size());
  Jobs::RunParallelFor(target_strand_segments.size(), [&](const size_t segment_handle) {
    auto& segment = segments[segment_handle];
    const auto& target_strand_segment = target_strand_segments[segment_handle];
    const auto& target_strand_segment_data = target_strand_segment_data_list[segment_handle];
    segment.prev_handle = target_strand_segment.GetPrevHandle();
    segment.next_handle = target_strand_segment.GetNextHandle();
    segment.strand_handle = target_strand_segment.GetStrandHandle();
    segment.rest_length = strand_group.GetStrandSegmentLength(static_cast<int>(segment_handle));
    segment.color = target_strand_segment.end_color;

    segment.radius = target_strand_segment.end_thickness * .5f;
    segment.q0 = segment.q = segment.last_q =
        initialize_parameters.root_transform.GetRotation() * target_strand_segment.rotation;
    segment.torque = glm::vec3(0.f);
    // 0.6046 = area radio of the circle within its bounding equilateral triangle.
    const float ratio = target_strand_segment_data.initial_distance_to_boundary * segment.radius * 2.f /
                        initialize_parameters.max_distance_to_boundary;

    const float mass = segment.radius * segment.radius * glm::pi<float>() *
                       initialize_parameters.wood_density.GetValue(ratio) * segment.rest_length;
    segment.inertia_tensor = ComputeInertiaTensorRod(mass, segment.radius, segment.rest_length);
    segment.inv_inertia_tensor = 1.f / segment.inertia_tensor;
    segment.original_inv_mass = 1.f / mass;
    const float area = glm::pi<float>() * segment.radius * segment.radius;
    segment.max_stretching_modulus = initialize_parameters.max_youngs_modulus.GetValue(ratio) * 1e9f;
    segment.max_shearing_modulus = initialize_parameters.max_shear_modulus.GetValue(ratio) * 1e9f;
    segment.moisture_content = initialize_parameters.moisture_content.GetValue(ratio);
    segment.boundary_distance = target_strand_segment_data.initial_distance_to_boundary * segment.radius * 2.f;
    segment.stretching_alpha = 1.f / (segment.max_stretching_modulus * area / segment.rest_length);
    segment.shearing_alpha = 1.f / (segment.max_shearing_modulus * area / segment.rest_length);
    const float max_shear_strain = glm::max(0.001f, initialize_parameters.max_shear_strain.GetValue(ratio));
    const float max_stretch_strain = glm::max(0.001f, initialize_parameters.max_stretch_strain.GetValue(ratio));
    segment.shear_stretch_strain_limit = segment.max_shear_stretch_strain =
        glm::vec2(max_shear_strain, max_stretch_strain);

    const auto& strand_segment = strand_group.PeekStrandSegment(static_cast<int>(segment_handle));
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(static_cast<int>(segment_handle));
    auto& particle0 = segment.particle0;
    auto& particle1 = segment.particle1;
    segment.group_index = 0;
    particle0.x0 = particle0.x = particle0.last_x = glm::vec3(initialize_parameters.root_transform.TransformPoint(
        strand_group.GetStrandSegmentStart(static_cast<int>(segment_handle))));

    particle1.x0 = particle1.x = particle1.last_x =
        glm::vec3(initialize_parameters.root_transform.TransformPoint(strand_segment.end_position));

    particle0.acceleration = particle1.acceleration = glm::vec3(0.0);
    particle0.node_handle = particle1.node_handle = strand_segment_data.node_handle;
  });

  DtsStrandGroup uniformly_subdivided_strand_group;
  strand_model_strand_group.UniformlySubdivide<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData>(
      uniformly_subdivided_strand_group, initialize_parameters.uniform_subdivision,
      [&](const StrandHandle src_handle, DtsStrandData& strand_data) {

      },
      [&](const float start_root_distance, const float end_root_distance, const StrandSegmentHandle src_handle,
          const uint32_t original_segment_index, const float segment_t, DtsStrandSegmentData& segment_data,
          const uint32_t sub_segment_index) {
        const auto& src_segment_data = strand_model_strand_group.PeekStrandSegmentData(src_handle);
        segment_data.node_handle = src_segment_data.node_handle;
        segment_data.original_segment_handle = src_handle;
        segment_data.original_segment_index = original_segment_index;
        segment_data.segment_index = sub_segment_index;
        segment_data.original_segment_t = segment_t;
        segment_data.start_root_distance = start_root_distance;
        segment_data.end_root_distance = end_root_distance;
        const auto& strand_segment = strand_model_strand_group.PeekStrandSegment(src_handle);
        const auto& strand = strand_model_strand_group.PeekStrand(strand_segment.GetStrandHandle());
        const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();

        glm::vec2 p0, p1, p3;
        const glm::vec2 p2 = src_segment_data.profile_position;
        float d0, d1, d3;
        const float d2 = src_segment_data.initial_distance_to_boundary;
        if (src_handle == strand_segment_handles.front()) {
          d1 = d2;
          d0 = d1 * 2.0f - d2;

          p1 = p2;
          p0 = p1 * 2.0f - p2;
        } else if (strand_segment.GetPrevHandle() == strand_segment_handles.front()) {
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          d0 = d2;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = p2;
          p1 = prev_segment_data.profile_position;
        } else {
          const auto& prev_segment = strand_model_strand_group.PeekStrandSegment(strand_segment.GetPrevHandle());
          const auto& prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          const auto& prev_prev_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(prev_segment.GetPrevHandle());
          d0 = prev_prev_segment_data.initial_distance_to_boundary;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = prev_prev_segment_data.profile_position;
          p1 = prev_segment_data.profile_position;
        }
        if (src_handle == strand_segment_handles.back()) {
          d3 = d2 * 2.0f - d1;

          p3 = p2 * 2.0f - p1;
        } else {
          const auto& next_segment_data =
              strand_model_strand_group.PeekStrandSegmentData(strand_segment.GetNextHandle());
          d3 = next_segment_data.initial_distance_to_boundary;

          p3 = next_segment_data.profile_position;
        }
        segment_data.initial_distance_to_boundary = Strands::CubicInterpolation(d0, d1, d2, d3, segment_t);
        segment_data.profile_position = Strands::CubicInterpolation(p0, p1, p2, p3, segment_t);
      },
      (initialize_parameters.min_segment_length + initialize_parameters.max_segment_length) * .5f * .01f);

  uniform_particles.resize(uniformly_subdivided_strand_group.PeekStrandSegments().size() + target_strands.size());
  std::vector<int> uniform_particle_offsets(target_strands.size());
  if (!uniform_particle_offsets.empty())
    uniform_particle_offsets[0] = 0;
  for (uint32_t strand_index = 1; strand_index < target_strands.size(); strand_index++) {
    uniform_particle_offsets[strand_index] =
        uniform_particle_offsets[strand_index - 1] +
        uniformly_subdivided_strand_group.PeekStrand(strand_index - 1).PeekStrandSegmentHandles().size() + 1;
  }
  Jobs::RunParallelFor(target_strands.size(), [&](const size_t strand_index) {
    auto& random_subdivided_strand = target_strands[strand_index];
    auto& uniformly_subdivided_strand = uniformly_subdivided_strand_group.PeekStrand(strand_index);
    const auto uniform_particle_offset = uniform_particle_offsets[strand_index];

    auto& first_uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(
        uniformly_subdivided_strand.PeekStrandSegmentHandles()[0]);
    auto& first_uniform_particle = uniform_particles[uniform_particle_offset];
    int random_segment_walker_index = 0;
    first_uniform_particle.segment_handle =
        random_subdivided_strand.PeekStrandSegmentHandles()[random_segment_walker_index];
    first_uniform_particle.node_index = first_uniform_segment_data.node_handle;
    first_uniform_particle.t = 0.0f;
    first_uniform_particle.segment_index = 0;
    first_uniform_particle.prev_particle_handle = -1;
    first_uniform_particle.next_particle_handle = -1;
    first_uniform_particle.next_node_index = -1;
    first_uniform_particle.strand_index = strand_index;

    int last_index_with_new_node = 0;
    float previous_root_distance = 0.0f;
    for (int uniform_segment_index = 0;
         uniform_segment_index < uniformly_subdivided_strand.PeekStrandSegmentHandles().size();
         uniform_segment_index++) {
      const auto& uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(
          uniformly_subdivided_strand.PeekStrandSegmentHandles()[uniform_segment_index]);
      auto& uniform_particle = uniform_particles[uniform_particle_offset + 1 + uniform_segment_index];
      uniform_particle.node_index = uniform_segment_data.node_handle;
      uniform_particle.segment_index = uniform_segment_index + 1;
      uniform_particle.prev_particle_handle = uniform_particle_offset + uniform_segment_index;
      uniform_particles[uniform_particle_offset + uniform_segment_index].next_particle_handle =
          uniform_particle_offset + 1 + uniform_segment_index;
      uniform_particle.next_particle_handle = -1;  // will stay for the last particle of the strand
      uniform_particle.next_node_index = -1;       // will stay for the last particle of the strand
      uniform_particle.strand_index = strand_index;

      if (uniform_particles[uniform_particle_offset + 1 + last_index_with_new_node].node_index !=
          uniform_particle.node_index) {
        // write node index to all previous ones
        for (int i = last_index_with_new_node; i < uniform_segment_index + 1; i++) {
          uniform_particles[uniform_particle_offset + i].next_node_index = uniform_particle.node_index;
        }
        last_index_with_new_node = uniform_segment_index;
      }

      bool found = false;
      while (random_segment_walker_index < random_subdivided_strand.PeekStrandSegmentHandles().size()) {
        uniform_particle.segment_handle =
            random_subdivided_strand.PeekStrandSegmentHandles()[random_segment_walker_index];
        const auto& random_segment_data = strand_group.PeekStrandSegmentData(uniform_particle.segment_handle);
        if (random_segment_data.end_root_distance >= uniform_segment_data.end_root_distance) {
          // Get the start original_segment_t for random_segment.
          if (glm::abs(random_segment_data.end_root_distance - previous_root_distance) < glm::epsilon<float>()) {
            uniform_particle.t = 1.f;
          } else {
            uniform_particle.t = (uniform_segment_data.end_root_distance - previous_root_distance) /
                                 (random_segment_data.end_root_distance - previous_root_distance);
          }
          uniform_particle.distance_to_boundary = uniform_segment_data.initial_distance_to_boundary;
          found = true;
          break;
        }
        random_segment_walker_index++;
        previous_root_distance = random_segment_data.end_root_distance;
      }
      if (!found) {
        EVOENGINE_ERROR("Fault!");
      }
    }
  });

  Jobs::RunParallelFor(uniform_particles.size(), [&](const size_t uniform_particle_index) {
    auto& uniform_particle = uniform_particles[uniform_particle_index];
    const auto& segment = segments[uniform_particle.segment_handle];
    const auto& particle0 = segment.particle0;
    const auto& particle1 = segment.particle1;

    uniform_particle.position = glm::mix(particle0.x0, particle1.x0, uniform_particle.t);
    uniform_particle.normal = glm::vec3(0.0f);
    uniform_particle.deg = 0.0f;

    const auto& pipe = strand_model_skeleton.data.strand_group.PeekStrand(uniform_particle.strand_index);
    // set up UV map
    const auto& p0_ptr = UVMapUtils::GetEndParticle(strand_model_skeleton, uniform_particle.strand_index,
                                                    glm::floor(uniform_particle.t));
    const auto& p1_ptr = UVMapUtils::GetStartParticle(strand_model_skeleton, uniform_particle.strand_index,
                                                      glm::floor(uniform_particle.t));

    if (p0_ptr && p1_ptr) {
      uniform_particle.tex_coord.x = UVMapUtils::GetPipePolar(*p0_ptr, *p1_ptr, uniform_particle.t) /
                                     (2 * glm::pi<float>()) * initialize_parameters.u_multiplier;
      uniform_particle.tex_coord.y = uniform_particle.node_index * initialize_parameters.v_multiplier;
    } else {
      uniform_particle.tex_coord.x = 0.0f;
      uniform_particle.tex_coord.y = uniform_particle.node_index * initialize_parameters.v_multiplier;
    }
  });

  segment_data_list.resize(segments.size());
  std::vector<glm::vec3> projected_max_bounds(Jobs::GetWorkerSize());
  std::vector<glm::vec3> projected_min_bounds(Jobs::GetWorkerSize());
  std::vector<glm::vec3> max_bounds(Jobs::GetWorkerSize());
  std::vector<glm::vec3> min_bounds(Jobs::GetWorkerSize());
  for (auto& i : projected_max_bounds)
    i = glm::vec3(-FLT_MAX);
  for (auto& i : projected_min_bounds)
    i = glm::vec3(FLT_MAX);

  for (auto& i : max_bounds)
    i = glm::vec3(-FLT_MAX);
  for (auto& i : min_bounds)
    i = glm::vec3(FLT_MAX);

  float average_segment_length =
      (initialize_parameters.min_segment_length + initialize_parameters.max_segment_length) * 0.5f;

  const auto calculate_regularized_segment_p0 = [&](const int segment_handle) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    return glm::vec3(strand_segment_data.profile_position.x, strand_segment_data.profile_position.y,
                     strand_segment_data.start_root_distance / average_segment_length);
  };
  const auto calculate_regularized_segment_center = [&](const int segment_handle) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    return glm::vec3(strand_segment_data.profile_position.x, strand_segment_data.profile_position.y,
                     (strand_segment_data.start_root_distance + strand_segment_data.end_root_distance) * .5f /
                         average_segment_length);
  };
  const auto calculate_regularized_segment_p1 = [&](const int segment_handle) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    return glm::vec3(strand_segment_data.profile_position.x, strand_segment_data.profile_position.y,
                     strand_segment_data.end_root_distance / average_segment_length);
  };

  Jobs::RunParallelFor(segment_data_list.size(), [&](const auto segment_handle, const auto worker_i) {
    projected_max_bounds[worker_i] =
        glm::max(projected_max_bounds[worker_i], calculate_regularized_segment_p0(static_cast<int>(segment_handle)));
    projected_min_bounds[worker_i] =
        glm::min(projected_min_bounds[worker_i], calculate_regularized_segment_p0(static_cast<int>(segment_handle)));
    projected_max_bounds[worker_i] =
        glm::max(projected_max_bounds[worker_i], calculate_regularized_segment_p1(static_cast<int>(segment_handle)));
    projected_min_bounds[worker_i] =
        glm::min(projected_min_bounds[worker_i], calculate_regularized_segment_p1(static_cast<int>(segment_handle)));

    const auto pos = segments[segment_handle].GetCenterX0();

    max_bounds[worker_i] = glm::max(max_bounds[worker_i], pos);
    min_bounds[worker_i] = glm::min(min_bounds[worker_i], pos);
    max_bounds[worker_i] = glm::max(max_bounds[worker_i], pos);
    min_bounds[worker_i] = glm::min(min_bounds[worker_i], pos);

    for (int j = 0; j < BUNDLE_MAX_CONNECTION; j++) {
      segment_data_list[segment_handle].pair_handles[j] = -1;
    }
  });

  for (uint32_t strand_index = 0; strand_index < target_strands.size(); strand_index++) {
    auto& target_strand = target_strands[strand_index];
    const auto& segment_handles = target_strand.PeekStrandSegmentHandles();
    if (segment_handles.size() < 2)
      continue;

    auto& strand = strands[strand_index];
    strand.begin_segment_handle = segment_handles.front();
    strand.end_segment_handle = segment_handles.back();
    const int handle_index_offset = static_cast<int>(segment_pairs.size());
    strand.begin_segment_pair_handle = handle_index_offset;
    strand.end_segment_pair_handle = handle_index_offset + static_cast<int>(segment_handles.size()) - 2;
    segment_pairs.resize(segment_pairs.size() + segment_handles.size() - 1);
    for (int segment_handle_index = 0; segment_handle_index < static_cast<int>(segment_handles.size());
         segment_handle_index++) {
      const auto segment0_handle = segment_handles[segment_handle_index];
      auto& segment0 = segments[segment0_handle];
      if (segment_handle_index == static_cast<int>(segment_handles.size()) - 1)
        break;
      const auto segment_pair_handle = segment_handle_index + handle_index_offset;
      auto& segment_pair = segment_pairs[segment_pair_handle];

      segment_pair.segment0_handle = segment0_handle;
      segment_pair.segment1_handle = segment_handles[segment_handle_index + 1];

      const auto& segment1 = segments[segment_pair.segment1_handle];

      auto& segment0_data = segment_data_list[segment0_handle];
      auto& segment1_data = segment_data_list[segment_pair.segment1_handle];

      segment0_data.pair_handles[1] = segment_pair_handle;
      segment1_data.pair_handles[0] = segment_pair_handle;

      // particles[segment0.particle1_handle].connection_handle = segment_pair_handle;
      // particles[segment1.particle0_handle].connection_handle = segment_pair_handle;
    }
  }

  for (uint32_t strand_index = 0; strand_index < target_strands.size(); strand_index++) {
    auto& gpu_strand = strands[strand_index];
    gpu_strand.front_propagate_begin_segment_handle = -1;
    gpu_strand.back_propagate_begin_segment_handle = -1;
    gpu_strand.front_propagate_begin_segment_pair_handle = -1;
    gpu_strand.back_propagate_begin_segment_pair_handle = -1;
    gpu_strand.alternative_front_propagate_begin_segment_pair_handle = -1;
    gpu_strand.alternative_back_propagate_begin_segment_pair_handle = -1;
    gpu_strand.alternative_front_propagate_begin_segment_handle = -1;
    gpu_strand.alternative_back_propagate_begin_segment_handle = -1;
    if (gpu_strand.begin_segment_handle == -1) {
      continue;
    }
    gpu_strand.front_propagate_begin_segment_handle = gpu_strand.begin_segment_handle;
    if (gpu_strand.begin_segment_handle == gpu_strand.end_segment_handle) {
      gpu_strand.alternative_front_propagate_begin_segment_handle = gpu_strand.begin_segment_handle;
      continue;
    }
    gpu_strand.alternative_front_propagate_begin_segment_handle = segments[gpu_strand.begin_segment_handle].next_handle;

    gpu_strand.front_propagate_begin_segment_pair_handle = gpu_strand.begin_segment_pair_handle;
    if (gpu_strand.begin_segment_pair_handle == gpu_strand.end_segment_pair_handle) {
      gpu_strand.alternative_front_propagate_begin_segment_pair_handle = gpu_strand.begin_segment_pair_handle;
      continue;
    }
    gpu_strand.alternative_front_propagate_begin_segment_pair_handle = gpu_strand.begin_segment_pair_handle + 1;

    const int connection_size = gpu_strand.end_segment_pair_handle - gpu_strand.begin_segment_pair_handle + 1;
    gpu_strand.back_propagate_begin_segment_pair_handle =
        connection_size % 2 == 0 ? gpu_strand.end_segment_pair_handle : gpu_strand.end_segment_pair_handle - 1;

    gpu_strand.alternative_back_propagate_begin_segment_pair_handle =
        connection_size % 2 == 0 ? gpu_strand.end_segment_pair_handle - 1 : gpu_strand.end_segment_pair_handle;

    gpu_strand.back_propagate_begin_segment_handle =
        connection_size % 2 == 1 ? gpu_strand.end_segment_handle : segments[gpu_strand.end_segment_handle].prev_handle;

    gpu_strand.alternative_back_propagate_begin_segment_handle =
        connection_size % 2 == 1 ? segments[gpu_strand.end_segment_handle].prev_handle : gpu_strand.end_segment_handle;
  }
  connection_segment_pair_size = segment_pairs.size();

  auto projected_max_bound = glm::vec3(-FLT_MAX);
  auto projected_min_bound = glm::vec3(FLT_MAX);
  for (auto& i : projected_max_bounds)
    projected_max_bound = glm::max(i, projected_max_bound);
  for (auto& i : projected_min_bounds)
    projected_min_bound = glm::min(i, projected_min_bound);

  auto max_bound = glm::vec3(-FLT_MAX);
  auto min_bound = glm::vec3(FLT_MAX);
  for (auto& i : max_bounds)
    max_bound = glm::max(i, max_bound);
  for (auto& i : min_bounds)
    min_bound = glm::min(i, min_bound);

  struct SegmentInfo {
    glm::vec3 p0;
    glm::vec3 p1;
    glm::vec3 center_position;
    int node_handle;
    int strand_handle;
    int segment_handle;
  };

  VoxelGrid<std::vector<SegmentInfo>> projected_voxel_grid;
  constexpr auto projected_cell_size = 1.f;
  projected_voxel_grid.Initialize(projected_cell_size, projected_min_bound - glm::vec3(projected_cell_size) * 2.f,
                                  projected_max_bound + glm::vec3(projected_cell_size) * 2.f, {});

  for (int segment_handle = 0; segment_handle < segments.size(); segment_handle++) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    SegmentInfo s_d;
    s_d.p0 = calculate_regularized_segment_p0(segment_handle);
    s_d.p1 = calculate_regularized_segment_p1(segment_handle);
    s_d.center_position = calculate_regularized_segment_center(segment_handle);
    s_d.node_handle = strand_segment_data.node_handle;
    s_d.strand_handle = strand_group.PeekStrandSegment(segment_handle).GetStrandHandle();
    s_d.segment_handle = segment_handle;
    projected_voxel_grid.Ref(s_d.center_position).emplace_back(s_d);
  }
  std::multimap<float, std::map<std::pair<int, int>, std::pair<float, float>>> candidates;
  for (int segment_handle = 0; segment_handle < segments.size(); segment_handle++) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    const auto p0 = calculate_regularized_segment_p0(segment_handle);
    const auto p1 = calculate_regularized_segment_p1(segment_handle);

    const auto extended_p0 = p0 - glm::vec3(0, 0, initialize_parameters.neighbor_vertical_range);
    const auto extended_p1 = p1 + glm::vec3(0, 0, initialize_parameters.neighbor_vertical_range);

    const auto center = calculate_regularized_segment_center(segment_handle);
    const auto strand_handle = strand_group.PeekStrandSegment(segment_handle).GetStrandHandle();
    projected_voxel_grid.ForEach(
        center,
        glm::max(initialize_parameters.neighbor_vertical_range, initialize_parameters.neighbor_horizontal_range),
        [&](const std::vector<SegmentInfo>& list) {
          for (const auto& info : list) {
            if (info.segment_handle == segment_handle)
              continue;
            if (info.strand_handle == strand_handle) {
              continue;
            }
            //  Function to check if a point is inside a cylinder
            const auto cylinder_check = [](const glm::vec3& p0, const glm::vec3& p1, const float radius,
                                           const glm::vec3& point, float& horizontal_distance,
                                           float& vertical_distance) {
              // Calculate the direction vector of the cylinder's axis
              const glm::vec3 d_v = p1 - p0;
              const float height = glm::length(d_v);
              const glm::vec3 direction = glm::normalize(d_v);

              // Vector from p0 to point
              const glm::vec3 p0_p = point - p0;

              // Projection scalar
              const float t = glm::dot(p0_p, direction);
              // Check if projection is within the cylinder's height
              if (t < 0.0f || t > height) {
                return false;  // Outside the cylinder height
              }

              // Closest point on the cylinder's axis
              const glm::vec3 closest_point = p0 + t * direction;

              // Distance from point to the axis
              horizontal_distance = glm::length(point - closest_point);
              // Check if the distance is within the radius
              return horizontal_distance <= radius;
            };
            float horizontal_distance1, horizontal_distance2;
            float vertical_distance1, vertical_distance2;
            const auto check1 =
                cylinder_check(extended_p0, extended_p1, initialize_parameters.neighbor_horizontal_range, info.p0,
                               horizontal_distance1, vertical_distance1);
            const auto check2 =
                cylinder_check(extended_p0, extended_p1, initialize_parameters.neighbor_horizontal_range, info.p1,
                               horizontal_distance2, vertical_distance2);
            if (!check1 && !check2)
              continue;

            bool node_check = false;
            if (info.node_handle == strand_segment_data.node_handle)
              node_check = true;
            if (!node_check) {
              if (auto& node = strand_model_skeleton.PeekNode(strand_segment_data.node_handle);
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
            const auto horizontal_distance = glm::min(horizontal_distance1, horizontal_distance2);
            const auto pair = segment_handle <= info.segment_handle
                                  ? std::make_pair(segment_handle, info.segment_handle)
                                  : std::make_pair(info.segment_handle, segment_handle);

            const auto distance_pair = std::make_pair(horizontal_distance, 0.f);
            if (const auto search = candidates.find(horizontal_distance); search != candidates.end()) {
              search->second.emplace(pair, distance_pair);
            } else {
              candidates.insert({horizontal_distance, {}});
              candidates.find(horizontal_distance)->second.insert({pair, distance_pair});
            }
          }
        });
  }

  std::vector<uint32_t> counters(segments.size(), 2);
  for (const auto& candidate_set : candidates) {
    for (const auto& candidate : candidate_set.second) {
      auto& first = counters[candidate.first.first];
      auto& second = counters[candidate.first.second];
      if (first >= BUNDLE_MAX_CONNECTION || second >= BUNDLE_MAX_CONNECTION)
        continue;
      const auto pair_handle = static_cast<int>(segment_pairs.size());
      segment_pairs.emplace_back();
      auto& new_pair = segment_pairs.back();
      new_pair.segment0_handle = candidate.first.first;
      new_pair.segment1_handle = candidate.first.second;
      segment_data_list[candidate.first.first].pair_handles[first] = pair_handle;
      segment_data_list[candidate.first.second].pair_handles[second] = pair_handle;
      first++;
      second++;
    }
  }

  Jobs::RunParallelFor(segment_pairs.size(), [&](const auto pair_index) {
    auto& segment_pair = segment_pairs[pair_index];
    auto& segment0 = segments[segment_pair.segment0_handle];
    auto& segment1 = segments[segment_pair.segment1_handle];
    const bool direct_connection = segment_data_list[segment_pair.segment0_handle].pair_handles[1] == pair_index;
    auto& segment0_particle0 = segment0.particle0;
    auto& segment0_particle1 = segment0.particle1;
    auto& segment1_particle0 = segment1.particle0;
    auto& segment1_particle1 = segment1.particle1;
    const auto segment0_center_position = (segment0_particle0.x0 + segment0_particle1.x0) * .5f;
    const auto segment1_center_position = (segment1_particle0.x0 + segment1_particle1.x0) * .5f;
    segment_pair.segment0_particle0_offset =
        glm::vec4(glm::inverse(segment1.q0) * (segment0_particle0.x0 - segment1_center_position), 0.0f);
    segment_pair.segment0_particle1_offset =
        glm::vec4(glm::inverse(segment1.q0) * (segment0_particle1.x0 - segment1_center_position), 0.0f);

    segment_pair.segment1_particle0_offset =
        glm::vec4(glm::inverse(segment0.q0) * (segment1_particle0.x0 - segment0_center_position), 0.0f);
    segment_pair.segment1_particle1_offset =
        glm::vec4(glm::inverse(segment0.q0) * (segment1_particle1.x0 - segment0_center_position), 0.0f);

    segment_pair.rest_darboux_vector = glm::conjugate(segment0.q0) * segment1.q0;
    segment_pair.bend_twist_bundle_integrity = 1.0f;
    segment_pair.connectivity_integrity = direct_connection ? 1.0f : 0.0f;

    const float ratio0 =
        segment0.boundary_distance * segment0.radius * 2.f / initialize_parameters.max_distance_to_boundary;
    const float ratio1 =
        segment1.boundary_distance * segment1.radius * 2.f / initialize_parameters.max_distance_to_boundary;
    const float ratio = (ratio0 + ratio1) * .5f;

    segment_pair.max_bending_modulus = initialize_parameters.max_bending_modulus.GetValue(ratio) * 1e9f;
    segment_pair.max_torsion_modulus = initialize_parameters.max_torsion_modulus.GetValue(ratio) * 1e9f;
    const float average_segment_radius = (segment0.radius + segment1.radius) * .5f;
    const float average_segment_length = (segment0.rest_length + segment1.rest_length) * .5f;
    const auto second_moment_of_area = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.25f;
    const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.5f;
    segment_pair.bending_alpha =
        1.f / (segment_pair.max_bending_modulus * second_moment_of_area / glm::pow(average_segment_length, 3.f));
    segment_pair.torsion_alpha =
        1.f / (segment_pair.max_torsion_modulus * polar_moment_of_inertia / average_segment_length);
    const auto& q0 = segment0.q0;
    const auto& q1 = segment1.q0;
    segment_pair.rest_darboux_vector = glm::conjugate(q0) * q1;
    const float max_bend_strain = glm::max(0.001f, initialize_parameters.max_bend_strain.GetValue(ratio));
    const float max_twist_strain = glm::max(0.001f, initialize_parameters.max_twist_strain.GetValue(ratio));
    const float max_bundle_strain = glm::max(0.001f, initialize_parameters.max_bundle_strain.GetValue(ratio));
    segment_pair.max_bending_twist_bundle_strain = segment_pair.bending_twist_bundle_limit =
        glm::vec3(max_bend_strain, max_twist_strain, max_bundle_strain);
  });
  // set up nodes
  auto& skeleton_nodes = strand_model_skeleton.PeekRawNodes();
  nodes.resize(skeleton_nodes.size());

  for (size_t i = 0; i < skeleton_nodes.size(); i++) {
    nodes[i].prev_handle = skeleton_nodes[i].GetParentHandle();
  }

  if (!initialize_parameters.triangulate_per_bundle) {
    ComputeDelaunay(delaunay_tetrahedrons, initialize_parameters.use_cgal);
  } else {
    ComputeDelaunayPerBundle(delaunay_tetrahedrons, initialize_parameters.use_cgal);
  }
  for (const auto& i : constraints)
    i->InitializeData(initialize_parameters, strand_model_skeleton, strand_group, *this);

  hashed_grid_elements.resize(segments.size());
  hashed_grid_cell_starts.resize(HASH_GRID_CELL_SIZE);

  // Create foliage here.
  auto initialize_parameters_copy = initialize_parameters;
  auto fd = initialize_parameters_copy.foliage_descriptor.Get<FoliageDescriptor>();
  if (!fd)
    fd = ProjectManager::CreateTemporaryAsset<FoliageDescriptor>();
  const auto& node_list = strand_model_skeleton.PeekSortedNodeList();
  const auto tree_dim = strand_model_skeleton.max - strand_model_skeleton.min;

  VoxelGrid<std::vector<SegmentInfo>> voxel_grid;
  const auto current_leaf_size = fd->leaf_size * glm::length(tree_dim) * 0.1f;
  const auto cell_size = 2.f * (current_leaf_size.y + fd->position_variance * glm::length(tree_dim) * 0.1f) +
                         initialize_parameters.max_segment_length;
  voxel_grid.Initialize(cell_size, min_bound - glm::vec3(cell_size) * 2.f, max_bound + glm::vec3(cell_size) * 2.f, {});
  std::unordered_set<int> enabled_node_handles;
  for (int segment_handle = 0; segment_handle < segments.size(); segment_handle++) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    SegmentInfo s_d;
    const auto& segment = segments[segment_handle];
    s_d.p0 = segment.particle0.x0;
    s_d.p1 = segment.particle1.x0;
    s_d.center_position = (s_d.p0 + s_d.p1) * 0.5f;
    s_d.node_handle = strand_segment_data.node_handle;
    enabled_node_handles.emplace(s_d.node_handle);
    s_d.strand_handle = strand_group.PeekStrandSegment(segment_handle).GetStrandHandle();
    s_d.segment_handle = segment_handle;
    voxel_grid.Ref(s_d.center_position).emplace_back(s_d);
  }

  struct LeafInfo {
    Transform matrix;
    int node_handle;
  };
  std::vector<LeafInfo> leaf_infos;
  for (const auto& internode_handle : node_list) {
    if (enabled_node_handles.find(internode_handle) == enabled_node_handles.end())
      continue;
    const auto& strand_model_node = strand_model_skeleton.PeekNode(internode_handle);
    std::vector<glm::mat4> leaf_matrices;
    fd->GenerateFoliageMatrices(leaf_matrices, strand_model_node.info, glm::length(tree_dim));
    for (const auto& matrix : leaf_matrices) {
      auto& leaf_info = leaf_infos.emplace_back();
      leaf_info.node_handle = internode_handle;
      leaf_info.matrix.value = initialize_parameters.root_transform.value * matrix;
    }
  }
  foliage.resize(leaf_infos.size());
  Jobs::RunParallelFor(foliage.size(), [&](const auto foliage_index) {
    const auto& leaf_info = leaf_infos[foliage_index];
    auto& leaf = foliage[foliage_index];
    bool found = false;
    glm::vec3 center = leaf_info.matrix.GetPosition();
    int target_segment_handle = 0;
    float distance = FLT_MAX;

    float min_radius = 0.f;
    float max_radius = cell_size;
    while (!found) {
      voxel_grid.ForEach(center, min_radius, max_radius, [&](const std::vector<SegmentInfo>& list) {
        for (const auto& i : list) {
          if (i.node_handle == leaf_info.node_handle) {
            if (const auto new_distance = glm::distance(center, i.center_position); new_distance < distance) {
              found = true;
              distance = new_distance;
              target_segment_handle = i.segment_handle;
            }
          }
        }
      });
      min_radius = max_radius;
      max_radius += cell_size;
    }
    leaf.segment_handle = target_segment_handle;
    leaf.attachment_integrity = 1.f;
    leaf.q0 = leaf.q = leaf.last_q = leaf_info.matrix.GetRotation();
    leaf.x0 = leaf.x = leaf.last_x = leaf_info.matrix.GetPosition();
    leaf.rotation_integrity = 1.f;
    leaf.scale = leaf_info.matrix.GetScale();
    const auto& segment = segments[target_segment_handle];
    leaf.position_offset = glm::vec4(glm::inverse(segment.q0) * (leaf.x0 - segment.GetCenterX0()), 0.0f);

  });
  Upload();
}

bool DynamicStrands::PhysicsParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Time step", &time_step, 0.001f, 0.001f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Sub step", &sub_step, 1, 1, 100)) {
    changed = true;
  }
  if (ImGui::Checkbox("Breaking", &enable_breaking)) {
    changed = true;
  }
  if (ImGui::Checkbox("Disconnection", &enable_disconnection)) {
    changed = true;
  }
  if (ImGui::Checkbox("Grouping", &enable_grouping)) {
    changed = true;
  }
  if (ImGui::Checkbox("Segment Collision", &enable_segment_collision)) {
    changed = true;
  }
  if (ImGui::DragInt("Constraint Iteration", &constraint_iteration, 1, 1, 500))
    changed = true;
  if (ImGui::DragFloat("Velocity damping", &velocity_damping, 0.01f, 0.01f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Angular velocity damping", &angular_velocity_damping, 0.00001f, 0.0f, 1.0f, "%.5f"))
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
  wait_for_upload = true;
  Platform::AddTemporaryBufferSyncAction([&]() {
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
    wait_for_upload = false;
  });
}

void DynamicStrands::Download() {
  Platform::AddTemporaryBufferSyncAction([&]() {
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
  });
}
std::shared_ptr<ComputePipeline> reset_pipeline, step_pipeline, apply_pipeline{};
std::shared_ptr<Buffer> feedback_buffer;
std::shared_ptr<Buffer> new_group_index_buffer;
std::shared_ptr<DescriptorSetLayout> feedback_layout{};
std::shared_ptr<DescriptorSet> feedback_descriptor_set{};
void DynamicStrands::CalculateGroups(const PhysicsParameters& physics_parameters) const {
  if (segments.empty())
    return;
  struct GroupingPushConstant {
    uint32_t segment_size;
  };
  if (!reset_pipeline) {
    std::shared_ptr<Shader> shader{};
    shader = std::make_shared<Shader>();
    shader->TryCompile(
        ShaderType::Compute, Platform::Constants::shader_global_defines,
        std::filesystem::path("./EcoSysLabResources") / "Shaders/Compute/DynamicStrands/Grouping/Reset.comp");
    reset_pipeline = std::make_shared<ComputePipeline>();
    reset_pipeline->compute_shader = shader;
    reset_pipeline->descriptor_set_layouts.emplace_back(strands_layout);
    reset_pipeline->map_entries.emplace_back(Platform::Constants::compute_work_group_invocations);
    auto& push_constant_range = reset_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroupingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    reset_pipeline->Initialize();
  }
  if (!feedback_layout) {
    feedback_layout = std::make_shared<DescriptorSetLayout>();
    feedback_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    feedback_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
    feedback_layout->Initialize();
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
    step_pipeline->descriptor_set_layouts.emplace_back(feedback_layout);
    step_pipeline->map_entries.emplace_back(Platform::Constants::compute_work_group_invocations);
    auto& push_constant_range = step_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroupingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    step_pipeline->Initialize();
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
    apply_pipeline->descriptor_set_layouts.emplace_back(feedback_layout);
    apply_pipeline->map_entries.emplace_back(Platform::Constants::compute_work_group_invocations);
    auto& push_constant_range = apply_pipeline->push_constant_ranges.emplace_back();
    push_constant_range.size = sizeof(GroupingPushConstant);
    push_constant_range.offset = 0;
    push_constant_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

    apply_pipeline->Initialize();
  }
  if (!feedback_buffer || !new_group_index_buffer) {
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
  if (!feedback_descriptor_set) {
    feedback_descriptor_set = std::make_shared<DescriptorSet>(feedback_layout);
  }
  Platform::AddTemporaryBufferSyncAction([&]() {
    const uint32_t work_group_invocations = Platform::Constants::compute_work_group_invocations;

    const auto start_time = Times::Now();
    GroupingPushConstant push_constant;
    push_constant.segment_size = segments.size();
    const auto current_frame_index = Platform::GetCurrentFrameIndex();
    const auto group_size = Platform::DivUp(segments.size(), work_group_invocations);
    std::vector<uint32_t> feedback(group_size);
    feedback_buffer->Resize(sizeof(uint32_t) * group_size);
    feedback_descriptor_set->UpdateBufferDescriptorBinding(0, feedback_buffer);
    new_group_index_buffer->Resize(sizeof(int) * segments.size());
    feedback_descriptor_set->UpdateBufferDescriptorBinding(1, new_group_index_buffer);
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
        step_pipeline->Bind(vk_command_buffer);
        step_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                         strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        step_pipeline->BindDescriptorSet(vk_command_buffer, 1, feedback_descriptor_set->GetVkDescriptorSet());
        step_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        vkCmdDispatch(vk_command_buffer, group_size, 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);

        apply_pipeline->Bind(vk_command_buffer);
        apply_pipeline->BindDescriptorSet(vk_command_buffer, 0,
                                          strands_descriptor_sets[current_frame_index]->GetVkDescriptorSet());
        apply_pipeline->BindDescriptorSet(vk_command_buffer, 1, feedback_descriptor_set->GetVkDescriptorSet());
        apply_pipeline->PushConstant(vk_command_buffer, 0, push_constant);
        vkCmdDispatch(vk_command_buffer, group_size, 1, 1);
        Platform::EverythingBarrier(vk_command_buffer);
      });
      feedback_buffer->DownloadVector(feedback, feedback.size());
    };
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
    const auto method3_time = std::to_string(Times::Now() - start_time);
  });
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
  return {
      mass / 12.f * (radius * radius + length * length),
      mass / 12.f * (radius * radius + length * length),
      mass / 4.f * (radius * radius + radius * radius),
  };
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

        tet0.neighbors[mismatch_indices.first] = tet1.indices[mismatch_indices.second];
        tet1.neighbors[mismatch_indices.second] = tet0.indices[mismatch_indices.first];
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
      gpu_tet.neighbors[i] = -1;
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
      gpu_tet.neighbors[mismatch_indices.first] = neighbor_indices[mismatch_indices.second];
    }

    tetrahedrons.emplace_back(gpu_tet);
  }
}
#endif

void DynamicStrands::TetDelaunay(const std::vector<glm::vec3>& points, const std::vector<size_t>& particle_indices,
                                 std::vector<GpuDelaunayTetrahedron>& tetrahedrons) {
  const auto tets = Delaunay3D::GenerateTetrahedrons(points);

  int valid_neighbors = 0;
  for (const auto& tet : tets) {
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
      gpu_tet.neighbors[i] = -1;
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

      const auto mismatch_indices = DynamicStrandUtils::CompareIndices(gpu_tet.indices, neighbor_indices);
      // TODO: according to CGAL documentation, this is guaranteed anyway, so we do not need to match both sides
      // store it such that the neighboring tetrahedron always consists of different indices
      // e. g. for the triangle 1 2 4 we store the corresponding neighboring index at position 3
      gpu_tet.neighbors[mismatch_indices.first] = neighbor_indices[mismatch_indices.second];
      valid_neighbors++;
    }

    tetrahedrons.emplace_back(gpu_tet);
  }

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
