#include <algorithm>
#include <cmath>
#include <glm/gtx/quaternion.hpp>
#include "DsColliders.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "TreePhysicsCollision.hpp"

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
  ResetPhysics(tree);
  previous_model_ = model.value;
  initial_scale_ = model.GetScale();
  initial_rotation_ = model.GetRotation();
  previous_rotation_ = initial_rotation_;
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
      physics_vertices_.push_back({glm::vec3(model.value * glm::vec4(info.global_position, 1.0f)), {}, 0, 0, true});
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
    physics_vertices_.push_back({end, {}, 0, 0, false});
    end_vertices[handle] = finish;
    PhysicsRod rod;
    rod.start = start;
    rod.end = finish;
    rod.node = handle;
    rod.length = glm::length(end - physics_vertices_[start].position);
    const glm::vec3 scale = glm::abs(initial_scale_);
    const auto& mesh = tree.tree_mesh_generator_settings;
    const float thickness = (mesh.override_radius ? mesh.radius : info.thickness) * mesh.radius_multiplier;
    rod.radius = 0.5f * glm::max(0.0f, thickness) * glm::max(scale.x, glm::max(scale.y, scale.z));
    rod.rotation = rod.rest_rotation =
        glm::normalize(glm::rotation(initial_rotation_ * info.GetGlobalDirection(),
                                     glm::normalize(end - physics_vertices_[start].position)) *
                       initial_rotation_ * info.global_rotation * axis_to_shoot);
    rod.visual_offset =
        parent < 0 ? glm::vec3{} : info.global_position - skeleton.PeekNode(parent).info.GetGlobalEndPosition();
    rod.pinned = parent < 0 || (physics_node_rods_[parent] < 0 && skeleton.PeekNode(parent).GetParentHandle() < 0);
    physics_node_rods_[handle] = static_cast<int>(physics_rods_.size());
    physics_rods_.push_back(rod);
  }
  if (physics_rods_.empty()) {
    ResetPhysics(tree);
    return false;
  }
  physics_vertex_rods_.resize(physics_vertices_.size());
  physics_rod_angles_.resize(physics_rods_.size());
  physics_predicted_.resize(physics_vertices_.size());
  physics_previous_.resize(physics_vertices_.size());
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
    rod_bend_scales[i] = area_scale * area_scale;
    rod.mass = glm::max(1e-6f, physics_settings.density * 3.14159265f * reference_radius * reference_radius *
                                   rod.structural_length_scale);
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
        physics_angles_.push_back({first_index, second_index, glm::inverse(first.rest_rotation) * second.rest_rotation,
                                   4.0f * std::pow(average_length, 1.8f) *
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

void DynamicTreeSkeleton::StepPhysics(Tree& tree) {
  const auto scene = tree.GetScene();
  StepPhysics(tree, scene ? scene->GetDataComponent<GlobalTransform>(tree.GetOwner()) : GlobalTransform{});
}

void DynamicTreeSkeleton::StepPhysics(Tree& tree, const GlobalTransform& model) {
  if (!HasPhysics(tree)) {
    if (!physics_rods_.empty())
      ResetPhysics(tree);
    return;
  }
  if (glm::any(glm::greaterThan(glm::abs(model.GetScale() - initial_scale_), glm::vec3(1e-4f)))) {
    ResetPhysics(tree);
    return;
  }
  if (!physics_running) {
    if (model.value == previous_model_)
      return;
    const glm::mat4 delta = model.value * glm::inverse(previous_model_);
    const glm::quat motion_rotation = glm::normalize(model.GetRotation() * glm::inverse(previous_rotation_));
    for (auto& vertex : physics_vertices_) {
      vertex.position = glm::vec3(delta * glm::vec4(vertex.position, 1.0f));
      vertex.velocity = motion_rotation * vertex.velocity;
    }
    for (auto& rod : physics_rods_)
      rod.rotation = glm::normalize(motion_rotation * rod.rotation);
    previous_model_ = model.value;
    previous_rotation_ = model.GetRotation();
    return;
  }
  const int substeps = glm::clamp(physics_settings.substeps, 1, 64);
  const int iterations = glm::clamp(physics_settings.iterations, 1, 64);
  const float h = glm::clamp(physics_settings.time_step, 1e-5f, 0.1f) / substeps;
  const float stretch = glm::max(0.0f, physics_settings.stretch_stiffness);
  const float bend = glm::max(0.0f, physics_settings.bend_stiffness);
  const glm::quat model_rotation = model.GetRotation();
  struct BoxContact {
    glm::vec3 center;
    glm::quat rotation;
    glm::vec3 half_extent;
  };
  struct SphereContact {
    glm::vec3 center;
    float radius;
  };
  struct CylinderContact {
    glm::vec3 center;
    glm::quat rotation;
    float radius;
    float half_height;
  };
  std::vector<BoxContact> boxes;
  std::vector<SphereContact> spheres;
  std::vector<CylinderContact> cylinders;
  if (const auto scene = tree.GetScene(); scene && (physics_settings.fracture_enabled || has_detached_branches_)) {
    if (const auto* entities = scene->UnsafeGetPrivateComponentOwnersList<DsBoxCollider>()) {
      for (const auto entity : *entities) {
        if (!scene->IsEntityEnabled(entity))
          continue;
        const auto collider = scene->GetOrSetPrivateComponent<DsBoxCollider>(entity).lock();
        if (!collider || !collider->IsEnabled())
          continue;
        const auto transform = scene->GetDataComponent<GlobalTransform>(entity);
        boxes.push_back({transform.GetPosition(), transform.GetRotation(),
                         glm::max(glm::abs(collider->scale * transform.GetScale()), glm::vec3(0.001f))});
      }
    }
    if (const auto* entities = scene->UnsafeGetPrivateComponentOwnersList<DsSphereCollider>()) {
      for (const auto entity : *entities) {
        if (!scene->IsEntityEnabled(entity))
          continue;
        const auto collider = scene->GetOrSetPrivateComponent<DsSphereCollider>(entity).lock();
        if (!collider || !collider->IsEnabled())
          continue;
        const auto transform = scene->GetDataComponent<GlobalTransform>(entity);
        const auto scale = transform.GetScale();
        spheres.push_back({transform.GetPosition(),
                           glm::max(0.001f, collider->radius * glm::max(scale.x, glm::max(scale.y, scale.z)))});
      }
    }
    if (const auto* entities = scene->UnsafeGetPrivateComponentOwnersList<DsCylinderCollider>()) {
      for (const auto entity : *entities) {
        if (!scene->IsEntityEnabled(entity))
          continue;
        const auto collider = scene->GetOrSetPrivateComponent<DsCylinderCollider>(entity).lock();
        if (!collider || !collider->IsEnabled())
          continue;
        const auto transform = scene->GetDataComponent<GlobalTransform>(entity);
        const auto scale = transform.GetScale();
        cylinders.push_back({transform.GetPosition(), transform.GetRotation(),
                             glm::max(0.001f, collider->radius * glm::max(scale.x, scale.z)),
                             glm::max(0.001f, collider->height * scale.y)});
      }
    }
  }
  std::vector<bool> supported(physics_vertices_.size());
  std::vector<glm::vec3> support_normals(physics_vertices_.size(), glm::vec3(0));
  struct PivotVelocity {
    int pivot;
    int free;
    glm::vec3 angular;
  };
  std::vector<PivotVelocity> pivot_velocities;
  const auto solve_contacts = [&] {
    if (boxes.empty() && spheres.empty() && cylinders.empty())
      return false;
    supported.resize(physics_vertices_.size());
    support_normals.resize(physics_vertices_.size(), glm::vec3(0));
    bool contacted = false;
    for (const auto& rod : physics_rods_) {
      if (!rod.fallen)
        continue;
      auto& start = physics_vertices_[rod.start];
      auto& end = physics_vertices_[rod.end];
      const auto may_touch = [&](const glm::vec3& center, const float radius) {
        const glm::vec3 segment = end.position - start.position;
        const float t = glm::clamp(glm::dot(center - start.position, segment) / glm::max(glm::length2(segment), 1e-12f),
                                   0.0f, 1.0f);
        return glm::length2(glm::mix(start.position, end.position, t) - center) < radius * radius;
      };
      const auto solve_volume = [&](const glm::vec3& center, const auto& project) {
        const glm::vec3 segment = end.position - start.position;
        const float nearest = glm::clamp(
            glm::dot(center - start.position, segment) / glm::max(glm::length2(segment), 1e-12f), 0.0f, 1.0f);
        for (const float t : {0.0f, 0.5f, 1.0f, nearest}) {
          const glm::vec3 point = glm::mix(start.position, end.position, t);
          const glm::vec3 projected = project(point);
          const glm::vec3 previous = glm::mix(physics_previous_[rod.start], physics_previous_[rod.end], t);
          if (glm::dot(projected - point, -physics_settings.gravity) > 1e-6f) {
            const glm::vec3 normal = glm::normalize(projected - point);
            support_normals[rod.start] += (1.0f - t) * normal;
            support_normals[rod.end] += t * normal;
            if (t == 0.0f)
              supported[rod.start] = true;
            else if (t == 1.0f)
              supported[rod.end] = true;
          }
          contacted |= SolveRodContact(start.position, end.position, start.inverse_mass, end.inverse_mass, t, projected,
                                       previous, physics_settings.contact_friction);
        }
      };
      for (const auto& box : boxes) {
        const glm::quat inverse_rotation = glm::inverse(box.rotation);
        const glm::vec3 local_start = inverse_rotation * (start.position - box.center);
        const glm::vec3 local_end = inverse_rotation * (end.position - box.center);
        if (glm::any(glm::greaterThan(glm::min(local_start, local_end) - rod.radius, box.half_extent)) ||
            glm::any(glm::lessThan(glm::max(local_start, local_end) + rod.radius, -box.half_extent)))
          continue;
        solve_volume(box.center, [&](const glm::vec3& point) {
          return ProjectOutsideBox(point, box.center, box.rotation, box.half_extent, rod.radius);
        });
      }
      for (const auto& sphere : spheres) {
        if (may_touch(sphere.center, sphere.radius + rod.radius))
          solve_volume(sphere.center, [&](const glm::vec3& point) {
            return ProjectOutsideSphere(point, sphere.center, sphere.radius, rod.radius);
          });
      }
      for (const auto& cylinder : cylinders) {
        if (may_touch(cylinder.center, glm::length(glm::vec2(cylinder.radius, cylinder.half_height)) + rod.radius))
          solve_volume(cylinder.center, [&](const glm::vec3& point) {
            return ProjectOutsideCylinder(point, cylinder.center, cylinder.rotation, cylinder.radius,
                                          cylinder.half_height, rod.radius);
          });
      }
    }
    return contacted;
  };
  const auto solve_fallen_lengths = [&] {
    for (const auto& rod : physics_rods_) {
      if (!rod.fallen)
        continue;
      auto& start = physics_vertices_[rod.start];
      auto& end = physics_vertices_[rod.end];
      const glm::vec3 displacement = end.position - start.position;
      const float length = glm::length(displacement);
      const float weight = start.inverse_mass + end.inverse_mass;
      if (length < 1e-6f || weight <= 0)
        continue;
      const glm::vec3 correction = (length - rod.length) * displacement / (length * weight);
      start.position += start.inverse_mass * correction;
      end.position -= end.inverse_mass * correction;
    }
  };
  for (int step = 0; step < substeps; ++step) {
    supported.assign(physics_vertices_.size(), false);
    support_normals.assign(physics_vertices_.size(), glm::vec3(0));
    pivot_velocities.clear();
    const float anchor_fraction = static_cast<float>(step + 1) / substeps;
    const glm::quat anchor_rotation = glm::normalize(glm::slerp(previous_rotation_, model_rotation, anchor_fraction) *
                                                     glm::inverse(initial_rotation_));
    for (int i = 0; i < physics_vertices_.size(); ++i) {
      auto& vertex = physics_vertices_[i];
      physics_previous_[i] = vertex.position;
      if (vertex.pinned) {
        const glm::vec4 rest_local(physics_rest_positions_[i] + physics_settings.anchor_offset, 1.0f);
        vertex.position =
            glm::mix(glm::vec3(previous_model_ * rest_local), glm::vec3(model.value * rest_local), anchor_fraction);
        vertex.velocity = {};
        physics_predicted_[i] = vertex.position;
        continue;
      }
      physics_predicted_[i] = vertex.position + h * vertex.velocity + h * h * physics_settings.gravity;
      vertex.position = physics_predicted_[i];
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
          const float stiffness = stretch * rod.structural_length_scale / rod.length;
          numerator += stiffness * (physics_vertices_[other].position +
                                    (start ? -1.0f : 1.0f) * rod.length * (rod.rotation * rod_axis));
          denominator += stiffness;
        }
        vertex.position = numerator / denominator;
      }
      if (solve_contacts())
        solve_fallen_lengths();
      for (int i = 0; i < physics_rods_.size(); ++i) {
        auto& rod = physics_rods_[i];
        if (rod.pinned) {
          rod.rotation = glm::normalize(anchor_rotation * rod.rest_rotation);
          continue;
        }
        const glm::vec3 displacement = physics_vertices_[rod.end].position - physics_vertices_[rod.start].position;
        if (glm::length2(displacement) < 1e-12f)
          continue;
        const glm::quat axis_rotation = glm::rotation(rod.rotation * rod_axis, glm::normalize(displacement));
        const glm::quat stretch_target = glm::normalize(axis_rotation * rod.rotation);
        const float stretch_weight = stretch * rod.structural_length_scale;
        glm::vec4 sum =
            stretch_weight * glm::vec4(stretch_target.x, stretch_target.y, stretch_target.z, stretch_target.w);
        float total = stretch_weight;
        for (const int angle_index : physics_rod_angles_[i]) {
          const auto& angle = physics_angles_[angle_index];
          if (angle.broken)
            continue;
          const bool first = angle.first == i;
          const auto& other = physics_rods_[first ? angle.second : angle.first];
          const glm::quat target =
              glm::normalize(other.rotation * (first ? glm::inverse(angle.rest_relative) : angle.rest_relative));
          const float weight = bend * angle.bend_scale / (0.5f * (rod.length + other.length));
          const float sign = glm::dot(target, stretch_target) < 0 ? -1.0f : 1.0f;
          sum += sign * weight * glm::vec4(target.x, target.y, target.z, target.w);
          total += weight;
        }
        if (total > 0 && glm::length2(sum) > 1e-12f)
          rod.rotation = glm::normalize(glm::quat(sum.w, sum.x, sum.y, sum.z));
      }
    }
    if (physics_settings.fracture_enabled) {
      bool newly_detached = false;
      for (auto& angle : physics_angles_) {
        if (angle.broken)
          continue;
        const auto& parent = physics_rods_[angle.first];
        const auto& child = physics_rods_[angle.second];
        const float limit = physics_settings.FractureBendLimit(child.radius, 0.5f * (parent.length + child.length));
        const glm::vec3 displacement = physics_vertices_[child.end].position - physics_vertices_[child.start].position;
        if (glm::length2(displacement) < 1e-12f)
          continue;
        const glm::vec3 rest_direction = parent.rotation * angle.rest_relative * rod_axis;
        const float bend_angle =
            std::acos(glm::clamp(glm::dot(glm::normalize(displacement), rest_direction), -1.0f, 1.0f));
        if (bend_angle <= limit) {
          angle.over_limit_time = glm::max(0.0f, angle.over_limit_time - 0.01f * h);
          continue;
        }
        angle.over_limit_time += h * glm::min(10.0f, bend_angle / glm::max(limit, 1e-4f));
        if (angle.over_limit_time < glm::max(0.0f, physics_settings.fracture_duration))
          continue;
        const int old_start = child.start;
        const int new_start = static_cast<int>(physics_vertices_.size());
        PhysicsVertex separated = physics_vertices_[old_start];
        const glm::vec3 previous_position = physics_previous_[old_start];
        const glm::vec3 rest_position = physics_rest_positions_[old_start];
        separated.pinned = false;
        separated.fallen = true;
        separated.radius = child.radius;
        separated.inverse_mass = 2.0f / child.mass;
        physics_vertices_.push_back(separated);
        physics_predicted_.push_back(separated.position);
        physics_previous_.push_back(previous_position);
        physics_rest_positions_.push_back(rest_position);
        auto& incident = physics_vertex_rods_[old_start];
        incident.erase(std::remove(incident.begin(), incident.end(), angle.second), incident.end());
        if (!physics_vertices_[old_start].pinned) {
          float mass = 0;
          float radius = 0;
          for (const int index : incident) {
            mass += 0.5f * physics_rods_[index].mass;
            radius = glm::max(radius, physics_rods_[index].radius);
          }
          physics_vertices_[old_start].inverse_mass = 1.0f / glm::max(mass, 1e-6f);
          physics_vertices_[old_start].radius = radius;
        }
        physics_vertex_rods_.push_back({angle.second});
        auto& detached_rod = physics_rods_[angle.second];
        detached_rod.start = new_start;
        detached_rod.detached = true;
        has_detached_branches_ = true;
        newly_detached = true;
        angle.broken = true;
      }
      if (newly_detached) {
        const auto& skeleton = tree.shoot_model.PeekShootSkeleton();
        for (const auto handle : skeleton.PeekSortedNodeList()) {
          const int index = physics_node_rods_[handle];
          if (index < 0)
            continue;
          auto& rod = physics_rods_[index];
          const int parent_handle = skeleton.PeekNode(handle).GetParentHandle();
          if (rod.detached || (parent_handle >= 0 && physics_node_rods_[parent_handle] >= 0 &&
                               physics_rods_[physics_node_rods_[parent_handle]].fallen))
            rod.fallen = true;
          if (rod.fallen) {
            physics_vertices_[rod.start].fallen = true;
            physics_vertices_[rod.end].fallen = true;
          }
        }
      }
    }
    if (solve_contacts()) {
      for (int iteration = 0; iteration < 3; ++iteration) {
        solve_fallen_lengths();
        solve_contacts();
      }
      for (auto& rod : physics_rods_) {
        if (!rod.fallen)
          continue;
        const glm::vec3 displacement = physics_vertices_[rod.end].position - physics_vertices_[rod.start].position;
        if (glm::length2(displacement) > 1e-12f)
          rod.rotation =
              glm::normalize(glm::rotation(rod.rotation * rod_axis, glm::normalize(displacement)) * rod.rotation);
      }
    }
    for (const auto& rod : physics_rods_) {
      if (!rod.fallen || supported[rod.start] == supported[rod.end])
        continue;
      const int pivot_index = supported[rod.start] ? rod.start : rod.end;
      const int free_index = supported[rod.start] ? rod.end : rod.start;
      const auto& pivot = physics_vertices_[pivot_index];
      const auto& free = physics_vertices_[free_index];
      const glm::vec3 arm = free.position - pivot.position;
      const float length_squared = glm::length2(arm);
      if (length_squared < 1e-12f)
        continue;
      // A uniform rod about one end has I = mL^2/3; incoming linear momentum also turns the rod at impact.
      const glm::vec3 angular_velocity = (glm::cross(arm, free.velocity + 0.5f * pivot.velocity) +
                                          1.5f * h * glm::cross(arm, physics_settings.gravity)) /
                                         length_squared;
      pivot_velocities.push_back({pivot_index, free_index, angular_velocity});
    }
    for (int i = 0; i < physics_vertices_.size(); ++i) {
      auto& vertex = physics_vertices_[i];
      if (!vertex.pinned)
        vertex.velocity = (vertex.position - physics_previous_[i]) / h;
      vertex.velocity *= glm::max(0.0f, 1.0f - physics_settings.damping * h);
      if (glm::length2(support_normals[i]) > 1e-12f)
        vertex.velocity = ApplySupportFriction(vertex.velocity, glm::normalize(support_normals[i]),
                                               physics_settings.gravity, h, physics_settings.contact_friction);
      if (!Finite(vertex.position) || !Finite(vertex.velocity)) {
        ResetPhysics(tree);
        return;
      }
    }
    for (const auto& pivot : pivot_velocities) {
      auto& free = physics_vertices_[pivot.free];
      const auto& support = physics_vertices_[pivot.pivot];
      free.velocity = support.velocity + glm::cross(pivot.angular, free.position - support.position) *
                                             glm::max(0.0f, 1.0f - physics_settings.damping * h);
      if (!Finite(free.velocity)) {
        ResetPhysics(tree);
        return;
      }
    }
  }
  previous_model_ = model.value;
  previous_rotation_ = model_rotation;
  const auto& skeleton = tree.shoot_model.PeekShootSkeleton();
  auto& pose = tree.shoot_model.physics_pose_;
  const glm::mat4 inverse_model = glm::inverse(model.value);
  const glm::quat inverse_rotation = glm::inverse(model_rotation);
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
    } else
      current.position += physics_settings.anchor_offset;
    const int rod_index = physics_node_rods_[handle];
    if (rod_index < 0) {
      current.rotation = glm::normalize(parent_delta * current.rotation);
      current.regulated_rotation = glm::normalize(parent_delta * current.regulated_rotation);
      continue;
    }
    const auto& rod = physics_rods_[rod_index];
    current.detached = rod.detached;
    const glm::vec3 displacement = physics_vertices_[rod.end].position - physics_vertices_[rod.start].position;
    const glm::quat frame =
        glm::length2(displacement) > 1e-12f
            ? glm::normalize(glm::rotation(rod.rotation * rod_axis, glm::normalize(displacement)) * rod.rotation)
            : rod.rotation;
    const glm::quat local_frame = glm::normalize(inverse_rotation * frame);
    const glm::quat delta = glm::normalize(local_frame * glm::inverse(info.global_rotation * axis_to_shoot));
    if (parent < 0 || rod.detached)
      current.position = glm::vec3(inverse_model * glm::vec4(physics_vertices_[rod.start].position, 1.0f));
    else
      current.position = pose[parent].position + pose[parent].rotation * glm::vec3(0, 0, -1) * pose[parent].length +
                         parent_delta * rod.visual_offset;
    current.rotation = glm::normalize(local_frame * glm::inverse(axis_to_shoot));
    current.regulated_rotation = glm::normalize(delta * info.regulated_global_rotation);
    current.length = glm::length(glm::vec3(
        inverse_model * glm::vec4(physics_vertices_[rod.end].position - physics_vertices_[rod.start].position, 0.0f)));
  }
}
