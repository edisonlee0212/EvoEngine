#include "DynamicStrands.hpp"
#include "DsPhysics.hpp"
#include "DsColliders.hpp"
#include "DsConstraints.hpp"
#include "Shader.hpp"
#include "glm/gtc/matrix_access.hpp"
#include "glm/gtx/quaternion.hpp"

#ifdef USE_CGAL
#  include <CGAL/Delaunay_triangulation_3.h>
#  include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#  include <CGAL/Triangulation_vertex_base_with_info_3.h>
#else
#  include "Delaunay.hpp"
#endif

#ifdef USE_CGAL
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_3<unsigned int, K> Vb;
typedef CGAL::Triangulation_data_structure_3<Vb> Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds> Delaunay_CGAL;
typedef K::Point_3 Point;
#endif
using namespace eco_sys_lab_plugin;

#ifdef USE_CGAL
inline glm::vec3 cgal_to_glm(const CGAL::Point_3<CGAL::Epick>& p) {
  return {p.x(), p.y(), p.z()};
}
#endif
void DynamicStrands::Physics(const PhysicsParameters& physics_parameters, const std::function<void()>& pre_step_action,
                             const std::function<void()>& sub_step_action) const {
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
}

DynamicStrands::DynamicStrands() {
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
  device_segments_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_particles_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_pairs_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_segment_data_list_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_uniform_particles_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_connections_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_delaunay_tetrahedrons_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_hashed_grid_elements_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_hashed_grid_cell_starts_buffer =
      std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

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
}

bool DynamicStrands::WaitForUpload() const {
  return wait_for_upload;
}

bool DynamicStrands::InitializeParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::Checkbox("Static root", &static_root)) {
    changed = true;
  }

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

  return changed;
}

void DynamicStrands::Initialize(const InitializeParameters& initialize_parameters,
                                const StrandModelSkeleton& strand_model_skeleton,
                                const StrandModelStrandGroup& strand_model_strand_group,
                                const DtsStrandGroup& strand_group) {
  Clear();
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
  Jobs::RunParallelFor(target_strand_segments.size(), [&](const size_t i) {
    auto& segment = segments[i];
    const auto& target_strand_segment = target_strand_segments[i];
    const auto& target_strand_segment_data = target_strand_segment_data_list[i];
    segment.prev_handle = target_strand_segment.GetPrevHandle();
    segment.next_handle = target_strand_segment.GetNextHandle();
    segment.strand_handle = target_strand_segment.GetStrandHandle();
    segment.rest_length = strand_group.GetStrandSegmentLength(static_cast<int>(i));
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
    segment.shear_stretch_strain_limit = segment.max_shear_stretch_strain = segment.shear_stretch_strain_limit =
        glm::vec4(max_shear_strain, max_shear_strain, max_stretch_strain, 0.0f);
  });

  particles.resize(segments.size() * 2);
  Jobs::RunParallelFor(segments.size(), [&](const size_t segment_handle) {
    auto& segment = segments[segment_handle];
    const auto& strand_segment = strand_group.PeekStrandSegment(static_cast<int>(segment_handle));
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(static_cast<int>(segment_handle));
    auto& particle0 = particles[segment_handle * 2];
    auto& particle1 = particles[segment_handle * 2 + 1];
    segment.particle0_handle = static_cast<int>(segment_handle) * 2;
    segment.particle1_handle = static_cast<int>(segment_handle) * 2 + 1;
    particle0.x0 = particle0.x = particle0.last_x =
        glm::vec4(initialize_parameters.root_transform.TransformPoint(
                      strand_group.GetStrandSegmentStart(static_cast<int>(segment_handle))),
                  0.0);

    particle1.x0 = particle1.x = particle1.last_x =
        glm::vec4(initialize_parameters.root_transform.TransformPoint(strand_segment.end_position), 0.0);

    particle0.acceleration = particle1.acceleration = glm::vec3(0.0);
    particle0.strand_handle = particle1.strand_handle = segment.strand_handle;
    particle0.node_handle = particle1.node_handle = strand_segment_data.node_handle;
    particle0.segment_handle = particle1.segment_handle = static_cast<int>(segment_handle);
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

    float previous_root_distance = 0.0f;
    for (int uniform_segment_index = 0;
         uniform_segment_index < uniformly_subdivided_strand.PeekStrandSegmentHandles().size();
         uniform_segment_index++) {
      const auto& uniform_segment_data = uniformly_subdivided_strand_group.PeekStrandSegmentData(
          uniformly_subdivided_strand.PeekStrandSegmentHandles()[uniform_segment_index]);
      auto& uniform_particle = uniform_particles[uniform_particle_offset + 1 + uniform_segment_index];
      uniform_particle.node_index = uniform_segment_data.node_handle;
      uniform_particle.segment_index = uniform_segment_index + 1;
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
    const auto& particle0 = particles[segment.particle0_handle];
    const auto& particle1 = particles[segment.particle1_handle];

    uniform_particle.position = glm::mix(particle0.x0, particle1.x0, uniform_particle.t);
  });

  for (uint32_t strand_index = 0; strand_index < target_strands.size(); strand_index++) {
    auto& target_strand = target_strands[strand_index];
    const auto& segment_handles = target_strand.PeekStrandSegmentHandles();
    if (segment_handles.size() < 2)
      continue;

    auto& strand = strands[strand_index];
    strand.begin_segment_handle = segment_handles.front();
    strand.end_segment_handle = segment_handles.back();
    const int handle_index_offset = static_cast<int>(connections.size());
    strand.begin_connection_handle = handle_index_offset;
    strand.end_connection_handle = handle_index_offset + static_cast<int>(segment_handles.size()) - 2;
    connections.resize(connections.size() + segment_handles.size() - 1);
    for (int segment_handle_index = 0; segment_handle_index < static_cast<int>(segment_handles.size());
         segment_handle_index++) {
      const auto segment0_handle = segment_handles[segment_handle_index];
      auto& segment0 = segments[segment0_handle];
      /*
      if (initialize_parameters.uniform_subdivision == 1) {
        segment0.prev_jump_handle = segment0.prev_handle;
        segment0.next_jump_handle = segment0.next_handle;

        strand.begin_jump_segment_handle = strand.begin_segment_handle;
        strand.end_jump_segment_handle = strand.end_segment_handle;
        strand.begin_jump_connection_handle = strand.begin_connection_handle;
        strand.end_jump_connection_handle = strand.end_connection_handle;

      } else if (segment_handle_index % initialize_parameters.uniform_subdivision ==
      initialize_parameters.uniform_subdivision - 1) { if (segment_handle_index /
      initialize_parameters.uniform_subdivision == 0) { strand.begin_jump_segment_handle = segment0_handle;
        }
        if (segment_handle_index == initialize_parameters.uniform_subdivision - 1) {
          segment0.prev_jump_handle = -1;

        } else {
          int jump = initialize_parameters.uniform_subdivision - 1;
          segment0.prev_jump_handle = segment0.prev_handle;
          while (jump > 0) {
            segment0.prev_jump_handle = segments[segment0.prev_jump_handle].prev_handle;
            jump--;
          }
        }
        if (segment_handle_index == static_cast<int>(segment_handles.size()) - 1) {
          segment0.next_jump_handle = -1;
          strand.end_jump_segment_handle = segment0_handle;
        } else {
          int jump = initialize_parameters.uniform_subdivision - 1;
          segment0.next_jump_handle = segment0.next_handle;
          while (jump > 0) {
            segment0.next_jump_handle = segments[segment0.next_jump_handle].next_handle;
            jump--;
          }
        }
      } else {
        segment0.prev_jump_handle = -1;
        segment0.next_jump_handle = -1;
      }
      */
      if (segment_handle_index == static_cast<int>(segment_handles.size()) - 1)
        break;
      const auto connection_handle = segment_handle_index + handle_index_offset;
      auto& connection = connections[connection_handle];

      connection.segment0_handle = segment0_handle;
      connection.segment1_handle = segment_handles[segment_handle_index + 1];

      const auto& segment1 = segments[connection.segment1_handle];
      connection.segment0_particle_handle = segment0.particle1_handle;
      connection.segment1_particle_handle = segment1.particle0_handle;
      if (segment_handle_index > 0) {
        connection.prev_handle = connection_handle - 1;
      } else {
        connection.prev_handle = -1;
      }
      if (segment_handle_index < static_cast<int>(segment_handles.size()) - 2) {
        connection.next_handle = connection_handle + 1;
      } else {
        connection.next_handle = -1;
      }
      particles[connection.segment0_particle_handle].connection_handle = connection_handle;
      particles[connection.segment1_particle_handle].connection_handle = connection_handle;
      /*
      if (initialize_parameters.uniform_subdivision == 1) {
        connection.prev_jump_handle = connection.prev_handle;
        connection.next_jump_handle = connection.next_handle;
      } else if (segment_handle_index % initialize_parameters.uniform_subdivision ==
      initialize_parameters.uniform_subdivision - 1) { if (segment_handle_index /
      initialize_parameters.uniform_subdivision == 0) { strand.begin_jump_connection_handle = connection_handle;
        }
        if (segment_handle_index == initialize_parameters.uniform_subdivision - 1) {
          connection.prev_jump_handle = -1;
        } else {
          connection.prev_jump_handle = connection_handle - initialize_parameters.uniform_subdivision;
        }
        if (segment_handle_index == static_cast<int>(segment_handles.size()) - 1 -
      initialize_parameters.uniform_subdivision) { connection.next_jump_handle = -1; strand.end_jump_connection_handle =
      connection_handle; } else { connection.next_jump_handle = connection_handle +
      initialize_parameters.uniform_subdivision;
        }
      } else {
        connection.prev_jump_handle = -1;
        connection.next_jump_handle = -1;
      }
      */

      const float ratio0 =
          segment0.boundary_distance * segment0.radius * 2.f / initialize_parameters.max_distance_to_boundary;
      const float ratio1 =
          segment1.boundary_distance * segment1.radius * 2.f / initialize_parameters.max_distance_to_boundary;
      const float ratio = (ratio0 + ratio1) * .5f;
      connection.max_bending_modulus = initialize_parameters.max_bending_modulus.GetValue(ratio) * 1e9f;
      connection.max_torsion_modulus = initialize_parameters.max_torsion_modulus.GetValue(ratio) * 1e9f;

      const float average_segment_radius = (segment0.radius + segment1.radius) * .5f;
      const float average_segment_length = (segment0.rest_length + segment1.rest_length) * .5f;

      const auto second_moment_of_area = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.25f;
      const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.5f;

      connection.bending_alpha =
          1.f / (connection.max_bending_modulus * second_moment_of_area / glm::pow(average_segment_length, 3.f));
      connection.torsion_alpha =
          1.f / (connection.max_torsion_modulus * polar_moment_of_inertia / average_segment_length);

      const auto& q0 = segment0.q0;
      const auto& q1 = segment1.q0;
      connection.moisture_content = (segment0.moisture_content + segment1.moisture_content) * 0.5f;
      connection.boundary_distance = (segment0.boundary_distance + segment1.boundary_distance) * 0.5f;
      connection.rest_darboux_vector = glm::conjugate(q0) * q1;
      connection.bend_twist_valid = 1.f;
      connection.connectivity_valid = 1.f;
      const float max_bend_strain = glm::max(0.001f, initialize_parameters.max_bend_strain.GetValue(ratio));
      const float max_twist_strain = glm::max(0.001f, initialize_parameters.max_twist_strain.GetValue(ratio));


      connection.max_bend_twist_strain = connection.bend_twist_strain_limit =
          glm::vec3(max_bend_strain, max_bend_strain, max_twist_strain);
    }
  }

  std::vector<glm::vec3> max_bounds(Jobs::GetWorkerSize());
  std::vector<glm::vec3> min_bounds(Jobs::GetWorkerSize());
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

  segment_data_list.resize(segments.size());
  Jobs::RunParallelFor(segment_data_list.size(), [&](const auto segment_handle, const auto worker_i) {
    max_bounds[worker_i] =
        glm::max(max_bounds[worker_i], calculate_regularized_segment_p0(static_cast<int>(segment_handle)));
    min_bounds[worker_i] =
        glm::min(min_bounds[worker_i], calculate_regularized_segment_p0(static_cast<int>(segment_handle)));
    max_bounds[worker_i] =
        glm::max(max_bounds[worker_i], calculate_regularized_segment_p1(static_cast<int>(segment_handle)));
    min_bounds[worker_i] =
        glm::min(min_bounds[worker_i], calculate_regularized_segment_p1(static_cast<int>(segment_handle)));

    for (int j = 0; j < BUNDLE_MAX_CONNECTION; j++) {
      segment_data_list[segment_handle].pair_handles[j] = -1;
    }
  });

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

  VoxelGrid<std::vector<SegmentInfo>> voxel_grid;
  constexpr auto cell_size = 1.f;
  voxel_grid.Initialize(cell_size, min_bound - glm::vec3(cell_size) * 2.f, max_bound + glm::vec3(cell_size) * 2.f, {});
  for (int segment_handle = 0; segment_handle < segments.size(); segment_handle++) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    SegmentInfo s_d;
    s_d.p0 = calculate_regularized_segment_p0(segment_handle);
    s_d.p1 = calculate_regularized_segment_p1(segment_handle);
    s_d.center_position = calculate_regularized_segment_center(segment_handle);
    s_d.node_handle = strand_segment_data.node_handle;
    s_d.strand_handle = strand_group.PeekStrandSegment(segment_handle).GetStrandHandle();
    s_d.segment_handle = segment_handle;
    voxel_grid.Ref(s_d.center_position).emplace_back(s_d);
  }
  std::multimap<float, std::map<std::pair<int, int>, std::pair<float, float>>> candidates;

  for (const auto& connection : connections) {
    const auto& segment0_handle = connection.segment0_handle;
    const auto& segment1_handle = connection.segment1_handle;
    const auto pair =
        std::make_pair(glm::min(segment0_handle, segment1_handle), glm::max(segment0_handle, segment1_handle));
    constexpr auto distance_pair = std::make_pair(0.f, 0.f);
    if (const auto search = candidates.find(0.0f); search != candidates.end()) {
      search->second.emplace(pair, distance_pair);
    } else {
      candidates.insert({-1.0f, {}});
      candidates.find(-1.0f)->second.insert({pair, distance_pair});
    }
  }
  for (int segment_handle = 0; segment_handle < segments.size(); segment_handle++) {
    const auto& strand_segment_data = strand_group.PeekStrandSegmentData(segment_handle);
    const auto p0 = calculate_regularized_segment_p0(segment_handle);
    const auto p1 = calculate_regularized_segment_p1(segment_handle);

    const auto extended_p0 = p0 - glm::vec3(0, 0, initialize_parameters.neighbor_vertical_range);
    const auto extended_p1 = p1 + glm::vec3(0, 0, initialize_parameters.neighbor_vertical_range);

    const auto center = calculate_regularized_segment_center(segment_handle);
    const auto strand_handle = strand_group.PeekStrandSegment(segment_handle).GetStrandHandle();
    voxel_grid.ForEach(
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
  segment_pairs.clear();
  std::vector<uint32_t> counters(segments.size(), 0);

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
      const auto& horizontal_distance = candidate.second.first;
      const auto& vertical_distance = candidate.second.second;

      const auto& segment0 = segments[candidate.first.first];
      const auto& segment1 = segments[candidate.first.second];
      auto& segment0_particle0 = particles[segment0.particle0_handle];
      auto& segment0_particle1 = particles[segment0.particle1_handle];

      auto& segment1_particle0 = particles[segment1.particle0_handle];
      auto& segment1_particle1 = particles[segment1.particle1_handle];

      const auto segment0_center_position = (segment0_particle0.x0 + segment0_particle1.x0) * .5f;
      const auto segment1_center_position = (segment1_particle0.x0 + segment1_particle1.x0) * .5f;

      const float segment_length = 1.f;
      // glm::distance(segment0_center_position, segment1_center_position);
      const float segment_radius = 1.f;
      //(segment0.radius + segment1.radius) * .5f;
      const float ratio0 =
          segment0.boundary_distance * segment0.radius * 2.f / initialize_parameters.max_distance_to_boundary;
      const float ratio1 =
          segment1.boundary_distance * segment1.radius * 2.f / initialize_parameters.max_distance_to_boundary;
      const float ratio = (ratio0 + ratio1) * .5f;
      new_pair.max_bending_modulus = initialize_parameters.max_bending_modulus.GetValue(ratio) * 1e9f;
      new_pair.max_torsion_modulus = initialize_parameters.max_torsion_modulus.GetValue(ratio) * 1e9f;

      const auto second_moment_of_area = glm::pi<float>() * std::pow(segment_radius, 4.f) * 0.25f;
      const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(segment_radius, 4.f) * 0.5f;

      new_pair.bending_alpha =
          1.f / (new_pair.max_bending_modulus * second_moment_of_area / glm::pow(segment_length, 3.f));
      new_pair.twisting_alpha = 1.f / (new_pair.max_torsion_modulus * polar_moment_of_inertia / segment_length);
      const float max_bend_strain = glm::max(0.001f, initialize_parameters.max_bend_strain.GetValue(ratio));
      const float max_twist_strain = glm::max(0.001f, initialize_parameters.max_twist_strain.GetValue(ratio));

      const float max_bundle_strain = glm::max(0.001f, initialize_parameters.max_bundle_strain.GetValue(ratio));
      new_pair.max_bending_twist_bundle_strain = new_pair.bending_twist_bundle_limit =
          glm::vec4(max_bend_strain, max_bend_strain, max_twist_strain, max_bundle_strain);
      new_pair.valid = 1;
      segment_data_list[candidate.first.first].pair_handles[first] = pair_handle;
      segment_data_list[candidate.first.second].pair_handles[second] = pair_handle;
      first++;
      second++;
    }
  }

  Jobs::RunParallelFor(segment_pairs.size(), [&](const auto pair_index) {
    auto& pair = segment_pairs[pair_index];
    auto& segment0 = segments[pair.segment0_handle];
    auto& segment1 = segments[pair.segment1_handle];
    auto& segment0_particle0 = particles[segment0.particle0_handle];
    auto& segment0_particle1 = particles[segment0.particle1_handle];

    auto& segment1_particle0 = particles[segment1.particle0_handle];
    auto& segment1_particle1 = particles[segment1.particle1_handle];

    const auto segment0_center_position = (segment0_particle0.x0 + segment0_particle1.x0) * .5f;
    const auto segment1_center_position = (segment1_particle0.x0 + segment1_particle1.x0) * .5f;

    pair.segment0_particle0_offset =
        glm::vec4(glm::inverse(segment1.q0) * (segment0_particle0.x0 - segment1_center_position), 0.0f);
    pair.segment0_particle1_offset =
        glm::vec4(glm::inverse(segment1.q0) * (segment0_particle1.x0 - segment1_center_position), 0.0f);

    pair.segment1_particle0_offset =
        glm::vec4(glm::inverse(segment0.q0) * (segment1_particle0.x0 - segment0_center_position), 0.0f);
    pair.segment1_particle1_offset =
        glm::vec4(glm::inverse(segment0.q0) * (segment1_particle1.x0 - segment0_center_position), 0.0f);

    pair.rest_darboux_vector = glm::conjugate(segment0.q0) * segment1.q0;
  });

  ComputeDelaunay(delaunay_tetrahedrons);
  for (const auto& i : constraints)
    i->InitializeData(initialize_parameters, strand_model_skeleton, strand_group, *this);

  hashed_grid_elements.resize(segments.size());
  hashed_grid_cell_starts.resize(HASH_GRID_CELL_SIZE);
  Upload();
}

bool DynamicStrands::PhysicsParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragFloat("Time step", &time_step, 0.001f, 0.001f, 1.0f))
    changed = true;
  if (ImGui::DragInt("Sub step", &sub_step, 1, 1, 100)) {
    changed = true;
  }
  if (ImGui::Checkbox("Breaking", &allow_breaking)) {
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
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(1, device_segments_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(2, device_particles_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(3, device_segment_pairs_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(4, device_segment_data_list_buffer, 0);

  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(5, device_uniform_particles_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(6, device_connections_buffer, 0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(7, device_delaunay_tetrahedrons_buffer,
                                                                              0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(8, device_hashed_grid_elements_buffer,
                                                                              0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(9, device_hashed_grid_cell_starts_buffer,
                                                                              0);
  for (const auto& c : constraints) {
    c->UpdateBindings();
  }
}

void DynamicStrands::Upload() {
  wait_for_upload = true;
  Platform::AddTemporaryBufferSyncAction([&]() {
    device_strands_buffer->UploadVector(strands);
    device_segments_buffer->UploadVector(segments);
    device_particles_buffer->UploadVector(particles);
    device_segment_pairs_buffer->UploadVector(segment_pairs);
    device_segment_data_list_buffer->UploadVector(segment_data_list);
    device_uniform_particles_buffer->UploadVector(uniform_particles);
    device_connections_buffer->UploadVector(connections);
    device_delaunay_tetrahedrons_buffer->UploadVector(delaunay_tetrahedrons);
    device_hashed_grid_elements_buffer->UploadVector(hashed_grid_elements);
    device_hashed_grid_cell_starts_buffer->UploadVector(hashed_grid_cell_starts);

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
    if (!segments.empty())
      device_segments_buffer->DownloadVector(segments, segments.size());
    if (!particles.empty())
      device_particles_buffer->DownloadVector(particles, particles.size());
    if (!segment_pairs.empty())
      device_segment_pairs_buffer->DownloadVector(segment_pairs, segment_pairs.size());
    if (!segment_data_list.empty())
      device_segment_data_list_buffer->DownloadVector(segment_data_list, segment_data_list.size());
    if (!uniform_particles.empty())
      device_uniform_particles_buffer->DownloadVector(uniform_particles, uniform_particles.size());
    if (!connections.empty())
      device_connections_buffer->DownloadVector(connections, connections.size());
    if (!delaunay_tetrahedrons.empty())
      device_delaunay_tetrahedrons_buffer->DownloadVector(delaunay_tetrahedrons, delaunay_tetrahedrons.size());

    if (!hashed_grid_elements.empty())
      device_hashed_grid_elements_buffer->DownloadVector(hashed_grid_elements, hashed_grid_elements.size());
    if (!hashed_grid_cell_starts.empty())
      device_hashed_grid_cell_starts_buffer->DownloadVector(hashed_grid_cell_starts, hashed_grid_cell_starts.size());

    for (const auto& c : constraints) {
      c->DownloadData();
    }
  });
}

void DynamicStrands::Clear() {
  strands.clear();
  segments.clear();
  particles.clear();
  segment_pairs.clear();
  segment_data_list.clear();
  uniform_particles.clear();
  connections.clear();
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

void DynamicStrands::ComputeDelaunay(std::vector<GpuDelaunayTetrahedron>& tetrahedrons) {
  const auto point_plane_distance = [&](const glm::vec3& target_point, const glm::vec3& target_a,
                                        const glm::vec3& target_b, const glm::vec3& target_c) {
    // Compute the normal of the triangle
    const glm::vec3 ab = target_b - target_a;
    const glm::vec3 ac = target_c - target_a;
    const glm::vec3 normal = glm::normalize(glm::cross(ab, ac));

    // Compute signed distance from point to triangle's plane
    const float distance = glm::dot(normal, target_point - target_a);

    return distance;
  };

  /// @brief Given two arrays of size 4, each containing one element that is not in
  /// the other, compute the respective indices of these elements in the arrays
  /// @param a index array of size 4
  /// @param b index array of size 4
  /// @return pair of indices:
  /// first indicates the position in a that does not occur in b,
  /// second indicates the position in b that does not occur in a.
  const auto compare_indices = [&](const int a[4], const int b[4]) {
    std::vector a_in_both(4, false);
    std::vector b_in_both(4, false);

    for (size_t i = 0; i < 4; i++) {
      for (size_t j = 0; j < 4; j++) {
        if (a[i] == b[j]) {
          a_in_both[i] = true;
          b_in_both[j] = true;
        }
      }
    }

    int a_not_in_b = -1, b_not_in_a = -1;

    for (int i = 0; i < 4; i++) {
      if (!a_in_both[i]) {
        a_not_in_b = i;
      }
      if (!b_in_both[i]) {
        b_not_in_a = i;
      }
    }

    if (a_not_in_b >= 4 || b_not_in_a >= 4) {
      return std::make_pair(a_not_in_b, b_not_in_a);
      // throw std::exception("Did not find mismatched indices!");
    }

    return std::make_pair(a_not_in_b, b_not_in_a);
  };
#ifdef USE_CGAL
  const auto is_valid = [&](const int target_indices[4], const std::vector<int>& particle_indices) {
    for (size_t i = 0; i < 4; i++) {
      if (static_cast<unsigned>(target_indices[i]) >= particle_indices.size()) {
        // EVOENGINE_ERROR("tetrahedron vertex index out of range, will be discarded: " << target_indices[i]);
        return false;
      }
    }

    // check if all indices are distinct
    for (size_t i = 0; i < 4; i++) {
      for (size_t j = i + 1; j < 4; j++) {
        if (target_indices[i] == target_indices[j]) {
          return false;
        }
      }
    }

    return true;
  };
#else
  const auto is_valid = [&](const int tet_vertices[4], const std::vector<glm::vec3>& points) {
    for (size_t i = 0; i < 4; i++) {
      if (tet_vertices[i] >= static_cast<int>(points.size())) {
        EVOENGINE_ERROR("tetrahedron vertex index out of range, will be discarded: " << tet_vertices[i]);
        return false;
      }
    }

    for (size_t i = 0; i < 4; i++) {
      for (size_t j = i + 1; j < 4; j++) {
        if (tet_vertices[i] == tet_vertices[j]) {
          return false;
        }
      }
    }
  };
#endif
// TODO: maybe a different library will work here
#ifdef USE_CGAL
  std::vector<std::pair<Point, unsigned>> points;
  std::vector<int> particle_indices;
  for (int i = 0; i < particles.size(); i++) {
    // For duplicate particles we only use one of them.
    // if (particles[i].connection_handle >= 0 &&
    //    connections[particles[i].connection_handle].segment0_particle_handle == i)
    //  continue;
    auto& particle = particles[i];
    glm::vec3 particle_pos = particle.x0;
    if (particle.connection_handle) {
      auto& segment = segments[particle.segment_handle];
      auto& connection = connections[particle.connection_handle];
      glm::vec3 front = segment.q * glm::vec3(0, 0, -1);
      if (connection.segment0_particle_handle == i) {
        particle_pos -= front * segment.rest_length * 0.25f;
      } else {
        particle_pos += front * segment.rest_length * 0.25f;
      }
    }
    Point p_cgal(particle_pos[0], particle_pos[1], particle_pos[2]);
    points.emplace_back(p_cgal, points.size());
    particle_indices.emplace_back(i);
  }

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
    if (!is_valid(indices, particle_indices)) {
      continue;  // discard this tetrahedron
    }
    GpuDelaunayTetrahedron gpu_tet;
    for (size_t i = 0; i < 4; i++) {
      gpu_tet.indices[i] = particle_indices[indices[i]];
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
    const float d = point_plane_distance(particles[gpu_tet.indices[3]].x0, particles[gpu_tet.indices[0]].x0,
                                         particles[gpu_tet.indices[1]].x0, particles[gpu_tet.indices[2]].x0);

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

      if (!is_valid(neighbor_indices, particle_indices)) {
        continue;
      }

      const auto mismatch_indices = compare_indices(gpu_tet.indices, neighbor_indices);
      // TODO: according to CGAL documentation, this is guaranteed anyway, so we do not need to match both sides
      // store it such that the neighboring tetrahedron always consists of different indices
      // e. g. for the triangle 1 2 4 we store the corresponding neighboring index at position 3
      gpu_tet.neighbors[particle_indices[mismatch_indices.first]] =
          neighbor_indices[particle_indices[mismatch_indices.second]];
    }

    tetrahedrons.emplace_back(gpu_tet);
  }

#else
  std::vector<glm::vec3> points;

  for (uint32_t i = 0; i < particles.size(); i++) {
    auto& particle = particles[i];
    glm::vec3 particle_pos = particle.x0;
    if (particle.connection_handle) {
      auto& segment = segments[particle.segment_handle];
      auto& connection = connections[particle.connection_handle];
      glm::vec3 front = segment.q * glm::vec3(0, 0, -1);
      if (connection.segment0_particle_handle == i) {
        particle_pos -= front * segment.rest_length * 0.25f;
      } else {
        particle_pos += front * segment.rest_length * 0.25f;
      }
    }
    points.emplace_back(particle_pos);
  }
  const auto tets = Delaunay3D::GenerateTetrahedrons(points);

  tetrahedrons.reserve(tets.size());
  for (const auto& tet : tets) {
    if (!is_valid(tet.v, points)) {
      continue;  // discard this tetrahedron
    }
    auto& gpu_tet = tetrahedrons.emplace_back();
    for (size_t i = 0; i < 4; i++) {
      gpu_tet.indices[i] = tet.v[i];
      gpu_tet.neighbors[i] = -1;
    }
    // set up debugging members
    gpu_tet.color = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);
    gpu_tet.task_looked_at = 0;
    gpu_tet.mesh_looked_at = 0;
    gpu_tet.inside = 0;
    gpu_tet.triangles_accepted = 0;

    // check orientation of the tetrahedron

    if (const float d = point_plane_distance(particles[gpu_tet.indices[3]].x0, particles[gpu_tet.indices[0]].x0,
                                             particles[gpu_tet.indices[1]].x0, particles[gpu_tet.indices[2]].x0);
        d > 0)  // point no. 3 is in front if the triangle 0 1 2, we need to correct this so the triangles face outwards
    {
      std::swap(gpu_tet.indices[1], gpu_tet.indices[2]);
    }

    // fill in neighbor indices
    // TODO: need to figure out how to check if a neighbor is valid
    const auto& neighbor_indices = tet.neighbor_tet_indices;
    for (const auto& neighbor_index : neighbor_indices) {
      if (neighbor_index < 0 || neighbor_index >= tets.size())
        continue;
      const auto& neighbor = tets[neighbor_index];
      if (!is_valid(neighbor.v, points)) {
        continue;
      }
      const auto mismatch_indices = compare_indices(gpu_tet.indices, neighbor.v);
      // TODO: according to CGAL documentation, this is guaranteed anyway, so we do not need to match both sides
      // store it such that the neighboring tetrahedron always consists of different indices
      // e.g. for the triangle 1 2 4 we store the corresponding neighboring index at position 3
      gpu_tet.neighbors[mismatch_indices.first] = neighbor_indices[mismatch_indices.second];
    }
  }
#endif
}
