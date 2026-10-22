#include <cmath>
#include <glm/gtx/quaternion.hpp>
#include "DynamicTreeSkeleton.hpp"

using namespace eco_sys_lab_package;

namespace {
constexpr glm::vec3 rod_axis{1, 0, 0};
const glm::quat axis_to_shoot = glm::rotation(rod_axis, glm::vec3(0, 0, -1));

bool Finite(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}
}  // namespace

bool DynamicTreeSkeleton::InitializePhysics(Tree& tree) {
  ResetPhysics(tree);
  const auto& skeleton = tree.shoot_model.PeekShootSkeleton();
  const auto& sorted = skeleton.PeekSortedNodeList();
  if (sorted.empty())
    return false;
  const auto& nodes = skeleton.PeekRawNodes();
  physics_node_rods_.assign(nodes.size(), -1);
  std::vector<int> end_vertices(nodes.size(), -1);
  for (const auto handle : sorted) {
    const auto& node = skeleton.PeekNode(handle);
    const auto& info = node.info;
    const int parent = node.GetParentHandle();
    int start = parent < 0 ? -1 : end_vertices[parent];
    if (start < 0) {
      start = static_cast<int>(physics_vertices_.size());
      physics_vertices_.push_back({info.global_position, {}, 0, true});
    }
    if (info.length <= 1e-6f || !Finite(info.global_position)) {
      end_vertices[handle] = start;
      continue;
    }
    const glm::vec3 end = physics_vertices_[start].position + info.GetGlobalDirection() * info.length;
    if (!Finite(end)) {
      end_vertices[handle] = start;
      continue;
    }
    const int finish = static_cast<int>(physics_vertices_.size());
    physics_vertices_.push_back({end, {}, 0, false});
    end_vertices[handle] = finish;
    PhysicsRod rod;
    rod.start = start;
    rod.end = finish;
    rod.node = handle;
    rod.length = info.length;
    rod.rotation = rod.rest_rotation = glm::normalize(info.global_rotation * axis_to_shoot);
    rod.visual_offset =
        parent < 0 ? glm::vec3{} : info.global_position - skeleton.PeekNode(parent).info.GetGlobalEndPosition();
    rod.pinned = parent < 0 || (physics_node_rods_[parent] < 0 && skeleton.PeekNode(parent).GetParentHandle() < 0);
    physics_node_rods_[handle] = static_cast<int>(physics_rods_.size());
    physics_rods_.push_back(rod);
    const float radius = glm::max(1e-4f, info.thickness);
    const float mass = glm::max(1e-6f, physics_settings.density * 3.14159265f * radius * radius * info.length);
    physics_vertices_[start].inverse_mass += 0.5f * mass;
    physics_vertices_[finish].inverse_mass += 0.5f * mass;
  }
  if (physics_rods_.empty()) {
    ResetPhysics(tree);
    return false;
  }
  physics_vertex_rods_.resize(physics_vertices_.size());
  physics_rod_angles_.resize(physics_rods_.size());
  physics_predicted_.resize(physics_vertices_.size());
  physics_previous_.resize(physics_vertices_.size());
  for (int i = 0; i < physics_rods_.size(); ++i) {
    const auto& rod = physics_rods_[i];
    physics_vertex_rods_[rod.start].push_back(i);
    physics_vertex_rods_[rod.end].push_back(i);
  }
  for (const auto& incident : physics_vertex_rods_) {
    for (size_t i = 0; i < incident.size(); ++i) {
      for (size_t j = i + 1; j < incident.size(); ++j) {
        const auto& first = physics_rods_[incident[i]];
        const auto& second = physics_rods_[incident[j]];
        const int angle_index = static_cast<int>(physics_angles_.size());
        physics_angles_.push_back({incident[i], incident[j], glm::inverse(first.rest_rotation) * second.rest_rotation});
        physics_rod_angles_[incident[i]].push_back(angle_index);
        physics_rod_angles_[incident[j]].push_back(angle_index);
      }
    }
  }
  physics_rest_positions_.reserve(physics_vertices_.size());
  for (auto& vertex : physics_vertices_) {
    physics_rest_positions_.push_back(vertex.position);
    if (vertex.pinned)
      vertex.inverse_mass = 0;
    else
      vertex.inverse_mass = 1.0f / glm::max(vertex.inverse_mass, 1e-6f);
  }
  model_revision_ = tree.GetShootModelRevision().content;
  return true;
}

void DynamicTreeSkeleton::StepPhysics(Tree& tree) {
  if (!HasPhysics(tree)) {
    if (!physics_rods_.empty())
      ResetPhysics(tree);
    return;
  }
  if (!physics_running)
    return;
  const int substeps = glm::clamp(physics_settings.substeps, 1, 64);
  const int iterations = glm::clamp(physics_settings.iterations, 1, 64);
  const float h = glm::clamp(physics_settings.time_step, 1e-5f, 0.1f) / substeps;
  const float stretch = glm::max(0.0f, physics_settings.stretch_stiffness);
  const float bend = glm::max(0.0f, physics_settings.bend_stiffness);
  for (int step = 0; step < substeps; ++step) {
    for (int i = 0; i < physics_vertices_.size(); ++i) {
      auto& vertex = physics_vertices_[i];
      physics_previous_[i] = vertex.position;
      if (vertex.pinned) {
        vertex.position = physics_rest_positions_[i] + physics_settings.anchor_offset;
        vertex.velocity = {};
        physics_predicted_[i] = vertex.position;
        continue;
      }
      const auto force = physics_settings.force_node >= 0 && physics_settings.force_node < physics_node_rods_.size() &&
                                 physics_node_rods_[physics_settings.force_node] >= 0 &&
                                 physics_rods_[physics_node_rods_[physics_settings.force_node]].end == i
                             ? physics_settings.tip_force
                             : glm::vec3{};
      physics_predicted_[i] =
          vertex.position + h * vertex.velocity + h * h * (physics_settings.gravity + force * vertex.inverse_mass);
    }
    for (int iteration = 0; iteration < iterations; ++iteration) {
      for (int i = 0; i < physics_vertices_.size(); ++i) {
        auto& vertex = physics_vertices_[i];
        if (vertex.pinned)
          continue;
        const float inertia = 1.0f / (vertex.inverse_mass * h * h);
        glm::vec3 numerator = inertia * physics_predicted_[i];
        float denominator = inertia;
        for (const int rod_index : physics_vertex_rods_[i]) {
          const auto& rod = physics_rods_[rod_index];
          const bool start = rod.start == i;
          const int other = start ? rod.end : rod.start;
          const float stiffness = stretch / rod.length;
          numerator += stiffness * (physics_vertices_[other].position +
                                    (start ? -1.0f : 1.0f) * rod.length * (rod.rotation * rod_axis));
          denominator += stiffness;
        }
        vertex.position = numerator / denominator;
      }
      for (int i = 0; i < physics_rods_.size(); ++i) {
        auto& rod = physics_rods_[i];
        if (rod.pinned)
          continue;
        const glm::vec3 displacement = physics_vertices_[rod.end].position - physics_vertices_[rod.start].position;
        if (glm::length2(displacement) < 1e-12f)
          continue;
        const glm::quat axis_rotation = glm::rotation(rod.rotation * rod_axis, glm::normalize(displacement));
        const glm::quat stretch_target = glm::normalize(axis_rotation * rod.rotation);
        const float stretch_weight = stretch / rod.length;
        glm::vec4 sum =
            stretch_weight * glm::vec4(stretch_target.x, stretch_target.y, stretch_target.z, stretch_target.w);
        float total = stretch_weight;
        for (const int angle_index : physics_rod_angles_[i]) {
          const auto& angle = physics_angles_[angle_index];
          const bool first = angle.first == i;
          const auto& other = physics_rods_[first ? angle.second : angle.first];
          const glm::quat target =
              glm::normalize(other.rotation * (first ? glm::inverse(angle.rest_relative) : angle.rest_relative));
          const float weight = bend / (0.5f * (rod.length + other.length));
          const float sign = glm::dot(target, stretch_target) < 0 ? -1.0f : 1.0f;
          sum += sign * weight * glm::vec4(target.x, target.y, target.z, target.w);
          total += weight;
        }
        if (total > 0 && glm::length2(sum) > 1e-12f)
          rod.rotation = glm::normalize(glm::quat(sum.w, sum.x, sum.y, sum.z));
      }
    }
    for (int i = 0; i < physics_vertices_.size(); ++i) {
      auto& vertex = physics_vertices_[i];
      if (!vertex.pinned)
        vertex.velocity = (vertex.position - physics_previous_[i]) / h;
      vertex.velocity *= glm::max(0.0f, 1.0f - physics_settings.damping * h);
      if (!Finite(vertex.position) || !Finite(vertex.velocity)) {
        ResetPhysics(tree);
        return;
      }
    }
  }
  const auto& skeleton = tree.shoot_model.PeekShootSkeleton();
  auto& pose = tree.shoot_model.physics_pose_;
  for (const auto handle : skeleton.PeekSortedNodeList()) {
    const auto& node = skeleton.PeekNode(handle);
    const auto& info = node.info;
    const int parent = node.GetParentHandle();
    auto& current = pose[handle];
    current = {info.global_position, info.global_rotation, info.regulated_global_rotation, info.length};
    glm::quat parent_delta{1, 0, 0, 0};
    if (parent >= 0) {
      const auto& parent_info = skeleton.PeekNode(parent).info;
      parent_delta = glm::normalize(pose[parent].rotation * glm::inverse(parent_info.global_rotation));
      const glm::vec3 parent_end =
          pose[parent].position + pose[parent].rotation * glm::vec3(0, 0, -1) * pose[parent].length;
      current.position = parent_end + parent_delta * (info.global_position - parent_info.GetGlobalEndPosition());
    } else {
      current.position += physics_settings.anchor_offset;
    }
    const int rod_index = physics_node_rods_[handle];
    if (rod_index < 0) {
      current.rotation = glm::normalize(parent_delta * current.rotation);
      current.regulated_rotation = glm::normalize(parent_delta * current.regulated_rotation);
      continue;
    }
    const auto& rod = physics_rods_[rod_index];
    const glm::vec3 displacement = physics_vertices_[rod.end].position - physics_vertices_[rod.start].position;
    const glm::quat frame =
        glm::length2(displacement) > 1e-12f
            ? glm::normalize(glm::rotation(rod.rotation * rod_axis, glm::normalize(displacement)) * rod.rotation)
            : rod.rotation;
    const glm::quat delta = glm::normalize(frame * glm::inverse(rod.rest_rotation));
    if (parent < 0)
      current.position = physics_vertices_[rod.start].position;
    else
      current.position = pose[parent].position + pose[parent].rotation * glm::vec3(0, 0, -1) * pose[parent].length +
                         parent_delta * rod.visual_offset;
    current.rotation = glm::normalize(frame * glm::inverse(axis_to_shoot));
    current.regulated_rotation = glm::normalize(delta * info.regulated_global_rotation);
    current.length = glm::length(physics_vertices_[rod.end].position - physics_vertices_[rod.start].position);
  }
}
