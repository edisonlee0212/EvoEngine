#include <cmath>
#include <glm/gtx/quaternion.hpp>
#include "DynamicTreeSkeleton.hpp"

using namespace eco_sys_lab_package;

namespace {
constexpr glm::vec3 rod_axis{1, 0, 0};
constexpr float reference_radius = 0.02f;
constexpr float reference_thickness = 0.05f;
const glm::quat axis_to_shoot = glm::rotation(rod_axis, glm::vec3(0, 0, -1));

bool Finite(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

}  // namespace

bool DynamicTreeSkeleton::InitializePhysics(Tree& tree) {
  const auto scene = tree.GetScene();
  return InitializePhysics(tree, scene ? scene->GetDataComponent<GlobalTransform>(tree.GetOwner()) : GlobalTransform{});
}

bool DynamicTreeSkeleton::InitializePhysics(Tree& tree, const GlobalTransform& model) {
  if (!physics_properties_initialized_) {
    const auto descriptor = tree.tree_descriptor_ref.Get<TreeDescriptor>();
    if (descriptor && static_cast<const TreePhysicsProperties&>(physics_settings) == TreePhysicsProperties{})
      SetPhysicsProperties(descriptor->physics_properties);
    physics_properties_initialized_ = true;
  }
  ResetPhysics();
  physics_density_ = glm::max(physics_settings.density, 0.001f);
  previous_model_ = model.value;
  initial_scale_ = model.GetScale();
  initial_rotation_ = model.GetRotation();
  const auto& skeleton = tree.shoot_model.PeekShootSkeleton();
  const auto& sorted = skeleton.PeekSortedNodeList();
  if (sorted.empty())
    return false;
  const auto& nodes = skeleton.PeekRawNodes();
  physics_node_rods_.assign(nodes.size(), -1);
  std::vector<int> end_vertices(nodes.size(), -1);
  std::vector<uint32_t> branch_orders(nodes.size());
  for (const auto handle : sorted) {
    const auto& node = skeleton.PeekNode(handle);
    const auto& info = node.info;
    const int parent = node.GetParentHandle();
    branch_orders[handle] = parent < 0 ? 0 : branch_orders[parent] + (node.IsApical() ? 0 : 1);
    int start = parent < 0 ? -1 : end_vertices[parent];
    if (start < 0) {
      start = static_cast<int>(physics_vertices_.size());
      physics_vertices_.push_back({glm::vec3(model.value * glm::vec4(info.global_position, 1.0f)), 0, 0, true});
    }
    if (info.length <= 1e-6f || !Finite(info.global_position)) {
      end_vertices[handle] = start;
      continue;
    }
    const glm::vec3 end = physics_vertices_[start].position +
                          glm::vec3(model.value * glm::vec4(info.GetGlobalDirection() * info.length, 0.0f));
    if (!Finite(end) || glm::length2(end - physics_vertices_[start].position) < 1e-12f) {
      end_vertices[handle] = start;
      continue;
    }
    const int finish = static_cast<int>(physics_vertices_.size());
    physics_vertices_.push_back({end, 0, 0, false});
    end_vertices[handle] = finish;
    PhysicsRod rod;
    rod.start = start;
    rod.end = finish;
    rod.node = handle;
    rod.branch_order = branch_orders[handle];
    rod.length = glm::length(end - physics_vertices_[start].position);
    const glm::vec3 scale = glm::abs(initial_scale_);
    const auto& mesh = tree.tree_mesh_generator_settings;
    const float thickness = (mesh.override_radius ? mesh.radius : info.thickness) * mesh.radius_multiplier;
    rod.radius = 0.5f * glm::max(0.0f, thickness) * glm::max(scale.x, glm::max(scale.y, scale.z));
    rod.rotation = rod.rest_rotation =
        glm::normalize(glm::rotation(initial_rotation_ * info.GetGlobalDirection(),
                                     glm::normalize(end - physics_vertices_[start].position)) *
                       initial_rotation_ * info.global_rotation * axis_to_shoot);
    rod.pinned = parent < 0 || (physics_node_rods_[parent] < 0 && skeleton.PeekNode(parent).GetParentHandle() < 0);
    physics_node_rods_[handle] = static_cast<int>(physics_rods_.size());
    physics_rods_.push_back(rod);
  }
  if (physics_rods_.empty()) {
    ResetPhysics();
    return false;
  }
  physics_vertex_rods_.resize(physics_vertices_.size());
  physics_rod_angles_.resize(physics_rods_.size());
  std::vector<float> structural_lengths(physics_vertices_.size(), 0.0f);
  // The tree example counts each vertex's incoming edge in its structural length.
  for (const auto& rod : physics_rods_)
    structural_lengths[rod.end] = rod.length;
  for (auto it = physics_rods_.rbegin(); it != physics_rods_.rend(); ++it)
    structural_lengths[it->start] += it->length + structural_lengths[it->end];
  std::vector<float> rod_bend_scales(physics_rods_.size());
  for (int i = 0; i < physics_rods_.size(); ++i) {
    auto& rod = physics_rods_[i];
    const float thickness_ratio = glm::max(1e-4f, skeleton.PeekNode(rod.node).info.thickness) / reference_thickness;
    const float area_scale = thickness_ratio * thickness_ratio;
    rod.structural_length_scale = 0.5f * (structural_lengths[rod.start] + structural_lengths[rod.end]) * area_scale;
    rod_bend_scales[i] = std::pow(thickness_ratio, glm::max(0.0f, physics_settings.bend_thickness_exponent));
    const float volume_over_pi = physics_settings.use_local_rod_mass
                                     ? rod.radius * rod.radius * rod.length
                                     : reference_radius * reference_radius * rod.structural_length_scale;
    rod.mass = glm::max(1e-6f, physics_density_ * 3.14159265f * volume_over_pi);
    physics_vertices_[rod.start].inverse_mass += 0.5f * rod.mass;
    physics_vertices_[rod.end].inverse_mass += 0.5f * rod.mass;
    physics_vertices_[rod.start].radius = glm::max(physics_vertices_[rod.start].radius, rod.radius);
    physics_vertices_[rod.end].radius = glm::max(physics_vertices_[rod.end].radius, rod.radius);
    physics_vertex_rods_[rod.start].push_back(i);
    physics_vertex_rods_[rod.end].push_back(i);
  }
  for (int vertex = 0; vertex < physics_vertex_rods_.size(); ++vertex) {
    const auto& incident = physics_vertex_rods_[vertex];
    for (const int first_index : incident) {
      const auto& first = physics_rods_[first_index];
      if (first.end != vertex)
        continue;
      for (const int second_index : incident) {
        const auto& second = physics_rods_[second_index];
        if (second.start != vertex)
          continue;
        const float average_length = 0.25f * (structural_lengths[first.start] + structural_lengths[first.end] +
                                              structural_lengths[second.start] + structural_lengths[second.end]);
        const int angle_index = static_cast<int>(physics_angles_.size());
        physics_angles_.push_back(
            {first_index, second_index, glm::inverse(first.rest_rotation) * second.rest_rotation,
             4.0f * std::pow(average_length, glm::max(0.0f, physics_settings.bend_downstream_exponent)) *
                 glm::min(rod_bend_scales[first_index], rod_bend_scales[second_index])});
        physics_rod_angles_[first_index].push_back(angle_index);
        physics_rod_angles_[second_index].push_back(angle_index);
      }
    }
  }
  physics_rest_positions_.reserve(physics_vertices_.size());
  const glm::mat4 inverse_initial_model = glm::inverse(previous_model_);
  for (auto& vertex : physics_vertices_) {
    physics_rest_positions_.push_back(glm::vec3(inverse_initial_model * glm::vec4(vertex.position, 1.0f)));
    if (vertex.pinned)
      vertex.inverse_mass = 0;
    else
      vertex.inverse_mass = 1.0f / glm::max(vertex.inverse_mass, 1e-6f);
  }
  model_revision_ = tree.GetShootModelRevision().content;
  return true;
}
