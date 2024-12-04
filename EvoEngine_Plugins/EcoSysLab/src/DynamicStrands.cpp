#include "DynamicStrands.hpp"
#include "DsColliders.hpp"
#include "DsConstraints.hpp"
#include "DynamicStrandUtils.hpp"
#include "Shader.hpp"
#include "glm/gtc/matrix_access.hpp"
#include "glm/gtx/quaternion.hpp"

using namespace eco_sys_lab_plugin;

#ifdef USE_CGAL
inline glm::vec3 cgal_to_glm(const Point_CGAL& p) {
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
  // Handle collision
  // Handle velocity constraints (Frictions, etc.)
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
  device_uniform_particles_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_connections_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  device_delaunay_tetrahedrons_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);
  device_nodes_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
  strands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : strands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(strands_layout);
  }

  pre_step = std::make_shared<DsPreStep>();
  prediction = std::make_shared<DsPrediction>();

  velocity_update = std::make_shared<DsVelocityUpdate>();
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
  if (ImGui::DragFloat("Wood Density", &wood_density, 0.01f, 0.01f, 3.0f))
    changed = true;
  if (youngs_modulus.OnInspect("Wood Young's modulus"))
    changed = true;
  if (shear_modulus.OnInspect("Wood Shear modulus"))
    changed = true;

  if (bending_modulus.OnInspect("Wood Bending modulus"))
    changed = true;
  if (torsion_modulus.OnInspect("Wood Torsion modulus"))
    changed = true;

  if (ImGui::DragFloat("Velocity damping", &velocity_damping, 0.01f, 0.01f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Angular velocity damping", &angular_velocity_damping, 0.00001f, 0.0f, 1.0f, "%.5f"))
    changed = true;

  if (ImGui::DragFloat("Neighbor vertical range", &neighbor_vertical_range, 0.01f, 0.01f, 10.0f))
    changed = true;

  if (ImGui::DragFloat("Neighbor horizontal range", &neighbor_horizontal_range, 0.01f, 0.01f, 10.0f))
    changed = true;

  if (max_neighbor_strain.OnInspect("Max neighbor strain", 0.01f))
    changed = true;
  if (ImGui::DragFloat("Min neighbor strain", &min_neighbor_strain, 0.001f, 0.00f, 1.0f))
    changed = true;
  if (max_stretch_shear_strain.OnInspect("Max stretch/shear strain", 0.01f))
    changed = true;
  if (ImGui::DragFloat3("Min stretch/shear strain", &min_stretch_shear_strain.x, 0.001f, 0.00f, 1.0f))
    changed = true;
  if (max_bend_twist_strain.OnInspect("Max bend/twist strain", 0.01f))
    changed = true;
  if (ImGui::DragFloat3("Min bend/twist strain", &min_bend_twist_strain.x, 0.001f, 0.00f, 1.0f))
    changed = true;
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

    if (target_strand_segment_data.initial_distance_to_boundary < 8.f) {
      segment.color = glm::mix(glm::vec4(0.6, 0.3, 0, 1), glm::vec4(0, 0, 1, 1),
                               glm::clamp(target_strand_segment_data.initial_distance_to_boundary / 8.f, 0.0f, 1.0f));
    } else if (target_strand_segment_data.initial_distance_to_boundary < 24.f) {
      segment.color =
          glm::mix(glm::vec4(0, 0, 1, 1), glm::vec4(1, 0, 0, 1),
                   glm::clamp((target_strand_segment_data.initial_distance_to_boundary - 8.f) / 16.f, 0.0f, 1.0f));
    } else {
      segment.color =
          glm::mix(glm::vec4(1, 0, 0, 1), glm::vec4(0, 1, 0, 1),
                   glm::clamp((target_strand_segment_data.initial_distance_to_boundary - 24.f) / 32.f, 0.0f, 1.0f));
    }
    segment.radius = target_strand_segment.end_thickness * .5f;
    segment.damping = initialize_parameters.angular_velocity_damping;
    segment.q0 = segment.q = segment.last_q =
        initialize_parameters.root_transform.GetRotation() * target_strand_segment.rotation;
    segment.torque = glm::vec3(0.f);
    // 0.6046 = area radio of the circle within its bounding equilateral triangle.
    const float mass = segment.radius * segment.radius * glm::pi<float>() * initialize_parameters.wood_density *
                       segment.rest_length * 0.6046;
    segment.inertia_tensor = ComputeInertiaTensorRod(mass, segment.radius, segment.rest_length);
    segment.inv_inertia_tensor = 1.f / segment.inertia_tensor;
    segment.original_inv_mass = 1.f / mass;
    const float area = glm::pi<float>() * segment.radius * segment.radius;
    const float youngs_modulus = initialize_parameters.youngs_modulus.GetValue() * 1e9f;
    const float shear_modulus = initialize_parameters.shear_modulus.GetValue() * 1e9f;
    segment.stretching_alpha = 1.f / (youngs_modulus * area / segment.rest_length);
    segment.shearing_alpha = 1.f / (shear_modulus * area / segment.rest_length);

    segment.max_stretch_shear_strain = glm::vec4(glm::max(initialize_parameters.min_stretch_shear_strain,
                                                          initialize_parameters.max_stretch_shear_strain.GetValue()),
                                                 0.0f);
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
    particle0.damping = particle1.damping = initialize_parameters.velocity_damping;
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

    // set them again at the end of struct
    particle0.strand_handle2 = particle1.strand_handle2 = segment.strand_handle;
    particle0.node_handle2 = particle1.node_handle2 = strand_segment_data.node_handle;
    particle0.segment_handle2 = particle1.segment_handle2 = static_cast<int>(segment_handle);
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
    first_uniform_particle.next_particle_handle - -1;
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
      uniform_particle.next_particle_handle = -1; // will stay for the last particle of the strand
      uniform_particle.next_node_index = -1; // will stay for the last particle of the strand
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
      const float torsion_modulus = initialize_parameters.torsion_modulus.GetValue() * 1e9f;
      const float bending_modulus = initialize_parameters.bending_modulus.GetValue() * 1e9f;

      const float average_segment_radius = (segment0.radius + segment1.radius) * .5f;
      const float average_segment_length = (segment0.rest_length + segment1.rest_length) * .5f;

      const auto second_moment_of_area = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.25f;
      const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.5f;

      connection.bending_alpha =
          1.f / (bending_modulus * second_moment_of_area / glm::pow(average_segment_length, 3.f));
      connection.torsion_alpha = 1.f / (torsion_modulus * polar_moment_of_inertia / average_segment_length);
      const auto& q0 = segment0.q0;
      const auto& q1 = segment1.q0;

      connection.rest_darboux_vector = glm::conjugate(q0) * q1;
      connection.bend_twist_strain_valid.w = 1.0;
      connection.max_bend_twist_strain = glm::vec4(
          glm::max(initialize_parameters.max_bend_twist_strain.GetValue(), initialize_parameters.min_bend_twist_strain),
          0.0f);
    }
  }

  // set up nodes
  auto& skeleton_nodes = strand_model_skeleton.PeekRawNodes();
  nodes.resize(skeleton_nodes.size());

  for (size_t i = 0; i < skeleton_nodes.size(); i++) {
    nodes[i].prev_handle = skeleton_nodes[i].GetParentHandle();
  }

  ComputeDelaunay(delaunay_tetrahedrons);
  for (const auto& i : constraints)
    i->InitializeData(initialize_parameters, strand_model_skeleton, strand_group, *this);

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
  if (ImGui::DragInt("Constraint Iteration", &constraint_iteration, 1, 1, 500))
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
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(3, device_uniform_particles_buffer, 0);

  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(4, device_connections_buffer, 0);

  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(5, device_delaunay_tetrahedrons_buffer,
                                                                              0);
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(6, device_nodes_buffer, 0); 

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
    device_uniform_particles_buffer->UploadVector(uniform_particles);
    device_connections_buffer->UploadVector(connections);

    device_delaunay_tetrahedrons_buffer->UploadVector(delaunay_tetrahedrons);
    device_nodes_buffer->UploadVector(nodes);
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

    if (!uniform_particles.empty())
      device_uniform_particles_buffer->DownloadVector(uniform_particles, uniform_particles.size());

    if (!connections.empty())
      device_connections_buffer->DownloadVector(connections, connections.size());

    if (!delaunay_tetrahedrons.empty())
      device_delaunay_tetrahedrons_buffer->DownloadVector(delaunay_tetrahedrons, delaunay_tetrahedrons.size());

    if (!nodes.empty())
      device_nodes_buffer->DownloadVector(nodes, nodes.size());

    for (const auto& c : constraints) {
      c->DownloadData();
    }
  });
}

void DynamicStrands::Clear() {
  strands.clear();
  segments.clear();
  particles.clear();
  uniform_particles.clear();
  connections.clear();

  delaunay_tetrahedrons.clear();
  nodes.clear();
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

void DynamicStrands::ComputeDelaunayPerBundle(std::vector<GpuDelaunayTetrahedron>& tetrahedrons) {
  int max_dist_from_root = 0;

  for (int i = 0; i < particles.size(); i++) {

    // only take start of bundle
    // TODO: needs to change
    if (i % 2 == 0) {
      max_dist_from_root = std::max(max_dist_from_root, particles[i].hop_distance_to_root);
    }
  }

  std::vector<std::map<int, std::vector<size_t> > > bundle_maps(max_dist_from_root + 1);
  std::vector<size_t> offsets(max_dist_from_root + 1, 0);
  std::vector<std::vector<size_t>> particle_adjacent_tets(particles.size());

  for (int i = 0; i < segments.size(); i++) {

    auto& segment = segments[i];
    auto& particle0 = particles[segment.particle0_handle];
    auto& node_handle = particle0.node_handle;

    if (bundle_maps[particle0.hop_distance_to_root].find(node_handle) ==
        bundle_maps[particle0.hop_distance_to_root].end()) {

      bundle_maps[particle0.hop_distance_to_root][node_handle] = std::vector<size_t>();
    }

    bundle_maps[particle0.hop_distance_to_root][node_handle].push_back(i);
  }

  // TODO: "squish" each bundle such that no internal degenerate tetrahedrons occur

  // triangulate each bundle:
  for (int d = 0; d < bundle_maps.size(); d++) {
    offsets[d] = tetrahedrons.size();
    auto& map = bundle_maps[d];
    for (auto& kv_pair : map) {
      auto& bundle = kv_pair.second;
      std::vector<std::pair<Point_CGAL, unsigned> > points;

      for (size_t i : bundle) {
        auto& segment = segments[i];

        glm::vec3 particle0_pos = particles[segment.particle0_handle].x0;
        glm::vec3 particle1_pos = particles[segment.particle1_handle].x0;

        Point_CGAL p0_cgal(particle0_pos[0], particle0_pos[1], particle0_pos[2]);
        Point_CGAL p1_cgal(particle1_pos[0], particle1_pos[1], particle1_pos[2]);
        points.emplace_back(p0_cgal, segment.particle0_handle);
        points.emplace_back(p1_cgal, segment.particle1_handle);
      }

      CGALDelaunay(points, tetrahedrons); 
    }
  }

  Jobs::RunParallelFor(tetrahedrons.size(), [&](const size_t tet_index) {
    auto& tet = tetrahedrons[tet_index];

    for (size_t i = 0; i < 4; i++) {
      particle_adjacent_tets[tet.indices[i]].emplace_back(tet_index);
    }
  });

}

void DynamicStrands::CGALDelaunay(const std::vector<std::pair<Point_CGAL, unsigned> >& points,
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

      const auto mismatch_indices = DynamicStrandUtils::CompareIndices(gpu_tet.indices, neighbor_indices);
      // TODO: according to CGAL documentation, this is guaranteed anyway, so we do not need to match both sides
      // store it such that the neighboring tetrahedron always consists of different indices
      // e. g. for the triangle 1 2 4 we store the corresponding neighboring index at position 3
      gpu_tet.neighbors[mismatch_indices.first] = neighbor_indices[mismatch_indices.second];
    }

    tetrahedrons.emplace_back(gpu_tet);
  }
}

void DynamicStrands::ComputeDelaunay(std::vector<GpuDelaunayTetrahedron>& tetrahedrons) {
 
// TODO: maybe a different library will work here
#ifdef USE_CGAL
  std::vector<std::pair<Point_CGAL, unsigned> > points;
  for (int i = 0; i < uniform_particles.size(); i++) {
    auto& particle = uniform_particles[i];
    glm::vec3 particle_pos = particle.position;
    Point_CGAL p_cgal(particle_pos[0], particle_pos[1], particle_pos[2]);
    points.emplace_back(p_cgal, i);
  }

  CGALDelaunay(points, tetrahedrons);
 

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
