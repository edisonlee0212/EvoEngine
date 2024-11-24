#include "DynamicStrands.hpp"

#include "DynamicStrandsPhysics.hpp"
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
DynamicStrands::DynamicStrands() {
  if (!strands_layout) {
    strands_layout = std::make_shared<DescriptorSetLayout>();
    strands_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);
    strands_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);

    strands_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_ALL, 0);

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
  device_connections_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  device_delaunay_tetrahedrons_buffer = std::make_shared<Buffer>(buffer_create_info, buffer_vma_allocation_create_info);

  const auto max_frame_in_flight = Platform::GetMaxFramesInFlight();
  strands_descriptor_sets.resize(max_frame_in_flight);
  for (auto& i : strands_descriptor_sets) {
    i = std::make_shared<DescriptorSet>(strands_layout);
  }

  pre_step = std::make_shared<DynamicStrandsPreStep>();
  prediction = std::make_shared<DynamicStrandsPrediction>();
}

bool DynamicStrands::WaitForUpload() const {
  return wait_for_upload;
}

bool DynamicStrands::InitializeParameters::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::DragInt("Sub segment count", &sub_segment, 1, 1, 100)) {
    sub_segment = glm::clamp(sub_segment, 1, 100);
    changed = true;
  }
  if (ImGui::DragFloat("Wood Density", &wood_density, 0.01f, 0.01f, 3.0f))
    changed = true;
#ifdef USE_XPBD
  if (wood_young_modulus.OnInspect("Wood Young's modulus"))
    changed = true;
  if (wood_torsion_modulus.OnInspect("Wood Torsion modulus"))
    changed = true;
#else
  if (shear_stiffness.OnInspect("Shear stiffness"))
    changed = true;
  if (stretch_stiffness.OnInspect("Stretch stiffness"))
    changed = true;
  if (bending_stiffness.OnInspect("Bending stiffness"))
    changed = true;
  if (twisting_stiffness.OnInspect("Twisting stiffness"))
    changed = true;
#endif
  if (neighbor_rotation_stiffness.OnInspect("Neighbor rotation stiffness"))
    changed = true;
  if (neighbor_position_stiffness.OnInspect("Neighbor position stiffness"))
    changed = true;

  if (ImGui::DragFloat("Velocity damping", &velocity_damping, 0.01f, 0.01f, 1.0f))
    changed = true;
  if (ImGui::DragFloat("Angular velocity damping", &angular_velocity_damping, 0.01f, 0.01f, 1.0f))
    changed = true;

  if (ImGui::DragFloat("Neighbor range", &neighbor_range, 0.01f, 0.01f, 10.0f))
    changed = true;
  if (max_neighbor_strain.OnInspect("Max neighbor strain", 0.01f))
    changed = true;
  if (max_stretch_shear_strain.OnInspect("Max stretch/shear strain", 0.01f))
    changed = true;
  if (max_bend_twist_strain.OnInspect("Max bend/twist strain", 0.01f))
    changed = true;

  return changed;
}

void DynamicStrands::Initialize(const InitializeParameters& initialize_parameters,
                                const StrandModelSkeleton& strand_model_skeleton, const DtsStrandGroup& strand_group) {
  Clear();
  assert(initialize_parameters.root_transform.GetScale() == glm::vec3(1.0f));
  const auto& target_strands = strand_group.PeekStrands();
  const auto& target_strand_segments = strand_group.PeekStrandSegments();
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
    segment.prev_handle = target_strand_segment.GetPrevHandle();
    segment.next_handle = target_strand_segment.GetNextHandle();
    segment.strand_handle = target_strand_segment.GetStrandHandle();
    segment.rest_length = strand_group.GetStrandSegmentLength(static_cast<int>(i));

    segment.color = target_strand_segment.end_color;
    segment.radius = target_strand_segment.end_thickness * .5f;
    segment.damping = initialize_parameters.angular_velocity_damping;
    segment.q0 = segment.q = segment.last_q = segment.old_q =
        initialize_parameters.root_transform.GetRotation() * target_strand_segment.rotation;
    segment.torque = glm::vec3(0.f);
    // 0.6046 = area radio of the circle within its bounding equilateral triangle.
    const float mass = segment.radius * segment.radius * glm::pi<float>() * initialize_parameters.wood_density *
                       segment.rest_length * 0.6046;
    segment.inertia_tensor = ComputeInertiaTensorRod(mass, segment.radius, segment.rest_length);
    segment.inv_inertia_tensor = 1.f / segment.inertia_tensor;
    segment.original_inv_mass = 1.f / mass;
#ifdef USE_XPBD
    const float area = glm::pi<float>() * segment.radius * segment.radius;
    const float youngs_modulus = initialize_parameters.wood_young_modulus.GetValue() * 1e9f;
    const float torsion_modulus = initialize_parameters.wood_torsion_modulus.GetValue() * 1e9f;
    segment.stretching_stiffness = youngs_modulus * area / segment.rest_length;
    segment.shearing_stiffness = torsion_modulus * area / segment.rest_length;
#else
    segment.stretching_stiffness = glm::clamp(initialize_parameters.stretch_stiffness.GetValue(), 0.0f, 1.0f);
    segment.shearing_stiffness = glm::clamp(initialize_parameters.shear_stiffness.GetValue(), 0.0f, 1.0f);
#endif
    segment.max_stretch_shear_strain = glm::vec4(glm::max(glm::vec3(0.0f), initialize_parameters.max_stretch_shear_strain.GetValue()), 0.0f);
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
    particle0.x0 = particle0.x = particle0.last_x = particle0.old_x =
        glm::vec4(initialize_parameters.root_transform.TransformPoint(
                      strand_group.GetStrandSegmentStart(static_cast<int>(segment_handle))),
                  0.0);

    particle1.x0 = particle1.x = particle1.last_x = particle1.old_x =
        glm::vec4(initialize_parameters.root_transform.TransformPoint(strand_segment.end_position), 0.0);

    particle0.acceleration = particle1.acceleration = glm::vec3(0.0);
    particle0.strand_handle = particle1.strand_handle = segment.strand_handle;
    particle0.node_handle = particle1.node_handle = strand_segment_data.node_handle;
    particle0.segment_handle = particle1.segment_handle = static_cast<int>(segment_handle);
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

      if (initialize_parameters.sub_segment == 1) {
        segment0.prev_jump_handle = segment0.prev_handle;
        segment0.next_jump_handle = segment0.next_handle;

        strand.begin_jump_segment_handle = strand.begin_segment_handle;
        strand.end_jump_segment_handle = strand.end_segment_handle;
        strand.begin_jump_connection_handle = strand.begin_connection_handle;
        strand.end_jump_connection_handle = strand.end_connection_handle;

      } else if (segment_handle_index % initialize_parameters.sub_segment == initialize_parameters.sub_segment - 1) {
        if (segment_handle_index / initialize_parameters.sub_segment == 0) {
          strand.begin_jump_segment_handle = segment0_handle;
        }
        if (segment_handle_index == initialize_parameters.sub_segment - 1) {
          segment0.prev_jump_handle = -1;

        } else {
          int jump = initialize_parameters.sub_segment - 1;
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
          int jump = initialize_parameters.sub_segment - 1;
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

      if (initialize_parameters.sub_segment == 1) {
        connection.prev_jump_handle = connection.prev_handle;
        connection.next_jump_handle = connection.next_handle;
      } else if (segment_handle_index % initialize_parameters.sub_segment == initialize_parameters.sub_segment - 1) {
        if (segment_handle_index / initialize_parameters.sub_segment == 0) {
          strand.begin_jump_connection_handle = connection_handle;
        }
        if (segment_handle_index == initialize_parameters.sub_segment - 1) {
          connection.prev_jump_handle = -1;
        } else {
          connection.prev_jump_handle = connection_handle - initialize_parameters.sub_segment;
        }
        if (segment_handle_index == static_cast<int>(segment_handles.size()) - 1 - initialize_parameters.sub_segment) {
          connection.next_jump_handle = -1;
          strand.end_jump_connection_handle = connection_handle;
        } else {
          connection.next_jump_handle = connection_handle + initialize_parameters.sub_segment;
        }
      } else {
        connection.prev_jump_handle = -1;
        connection.next_jump_handle = -1;
      }

#ifdef USE_XPBD
      const float youngs_modulus = initialize_parameters.wood_young_modulus.GetValue() * 1e9f;
      const float shear_modulus = initialize_parameters.wood_torsion_modulus.GetValue() * 1e9f;

      const float average_segment_radius = (segment0.radius + segment1.radius) * .5f;
      const float average_segment_length = (segment0.rest_length + segment1.rest_length) * .5f;

      const auto second_moment_of_area = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.25f;
      const auto polar_moment_of_inertia = glm::pi<float>() * std::pow(average_segment_radius, 4.f) * 0.5f;

      connection.bending_stiffness = youngs_modulus * second_moment_of_area / glm::pow(average_segment_length, 3.f);
      connection.twisting_stiffness = shear_modulus * polar_moment_of_inertia / average_segment_length;
#else
      connection.bending_stiffness = glm::clamp(initialize_parameters.bending_stiffness.GetValue(), 0.0f, 1.0f);
      connection.twisting_stiffness = glm::clamp(initialize_parameters.twisting_stiffness.GetValue(), 0.0f, 1.0f);
#endif
      const auto& q0 = segment0.q0;
      const auto& q1 = segment1.q0;

      connection.rest_darboux_vector = glm::conjugate(q0) * q1;
      connection.bend_twist_strain_valid.w = 1.0;
      connection.max_bend_twist_strain = glm::vec4(glm::max(initialize_parameters.max_bend_twist_strain.GetValue(), glm::vec3(0.0f)), 0.0f);
    }
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
  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(3, device_connections_buffer, 0);

  strands_descriptor_sets[current_frame_index]->UpdateBufferDescriptorBinding(4, device_delaunay_tetrahedrons_buffer,
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
    device_connections_buffer->UploadVector(connections);

    device_delaunay_tetrahedrons_buffer->UploadVector(delaunay_tetrahedrons);
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

    if (!connections.empty())
      device_connections_buffer->DownloadVector(connections, connections.size());

    if (!delaunay_tetrahedrons.empty())
      device_delaunay_tetrahedrons_buffer->DownloadVector(delaunay_tetrahedrons, delaunay_tetrahedrons.size());
    for (const auto& c : constraints) {
      c->DownloadData();
    }
  });
}

void DynamicStrands::Clear() {
  strands.clear();
  segments.clear();
  particles.clear();
  connections.clear();

  delaunay_tetrahedrons.clear();
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
  std::vector<std::pair<Point, unsigned> > points;
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
