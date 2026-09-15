#include "EntityBatchInspector.hpp"

#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "Scene.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

using namespace evo_engine;

namespace {
constexpr float kMixedEpsilon = 0.0001f;

bool Near(const glm::vec3& a, const glm::vec3& b) {
  return glm::all(glm::lessThanEqual(glm::abs(a - b), glm::vec3(kMixedEpsilon)));
}

bool IsFiniteVector(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool IsValidBound(const Bound& bound) {
  return IsFiniteVector(bound.min) && IsFiniteVector(bound.max) && glm::all(glm::lessThanEqual(bound.min, bound.max));
}

void AccumulateBound(const Bound& local_bound, const glm::mat4& transform, Bound& world_bound, bool& accumulated) {
  if (!IsValidBound(local_bound))
    return;
  auto transformed = local_bound;
  transformed.ApplyTransform(transform);
  if (!IsValidBound(transformed))
    return;
  world_bound.min = glm::min(world_bound.min, transformed.min);
  world_bound.max = glm::max(world_bound.max, transformed.max);
  accumulated = true;
}

template <typename Enumerate>
std::vector<EntityBatchComponent> BuildCommonComponents(const std::vector<Entity>& targets, Enumerate enumerate,
                                                        size_t& hidden_types) {
  std::vector<EntityBatchComponent> ordered;
  std::unordered_map<size_t, size_t> counts;
  std::unordered_set<size_t> union_types;
  for (size_t target_index = 0; target_index < targets.size(); ++target_index) {
    std::unordered_set<size_t> entity_types;
    enumerate(targets[target_index], [&](const EntityBatchComponent& component) {
      if (!entity_types.emplace(component.type_index).second)
        return;
      union_types.emplace(component.type_index);
      counts[component.type_index]++;
      if (target_index == 0)
        ordered.emplace_back(component);
    });
  }
  ordered.erase(std::remove_if(ordered.begin(), ordered.end(),
                               [&](const EntityBatchComponent& component) {
                                 return counts[component.type_index] != targets.size();
                               }),
                ordered.end());
  hidden_types = union_types.size() - ordered.size();
  return ordered;
}

template <typename Read>
EntityBatchValue<glm::vec3> ReadVector(const std::vector<Entity>& targets, Read read) {
  EntityBatchValue<glm::vec3> result;
  if (targets.empty())
    return result;
  result.value = read(targets.front());
  for (size_t i = 1; i < targets.size(); ++i) {
    const auto value = read(targets[i]);
    for (int axis = 0; axis < 3; ++axis)
      result.mixed_axes[axis] |= std::abs(result.value[axis] - value[axis]) > kMixedEpsilon;
    result.mixed |= !Near(result.value, value);
  }
  return result;
}
}  // namespace

EntityInspectorDispatch EntityBatchInspector::ResolveInspectorDispatch(const size_t target_count,
                                                                       const bool single_inspector_available,
                                                                       const bool batch_inspector_available,
                                                                       const bool force_batch) {
  if (target_count == 0)
    return EntityInspectorDispatch::Unsupported;
  if (target_count == 1 && single_inspector_available && !force_batch)
    return EntityInspectorDispatch::Single;
  return batch_inspector_available ? EntityInspectorDispatch::Batch : EntityInspectorDispatch::Unsupported;
}

EntityBatchInspectionContext EntityBatchInspector::BuildContext(const std::shared_ptr<Scene>& scene,
                                                                const std::vector<Entity>& exact_targets,
                                                                const Entity primary) {
  EntityBatchInspectionContext context;
  if (!scene)
    return context;
  for (const auto entity : exact_targets)
    if (scene->IsEntityValid(entity))
      context.targets.emplace_back(entity);
  if (context.targets.empty())
    return context;
  context.primary = std::find(context.targets.begin(), context.targets.end(), primary) != context.targets.end()
                        ? primary
                        : context.targets.back();
  auto component_order_targets = context.targets;
  std::rotate(component_order_targets.begin(),
              std::find(component_order_targets.begin(), component_order_targets.end(), context.primary),
              std::find(component_order_targets.begin(), component_order_targets.end(), context.primary) + 1);
  context.common_data_components = BuildCommonComponents(
      component_order_targets,
      [&](const Entity entity, const auto& emit) {
        scene->UnsafeForEachDataComponent(entity, [&](const DataComponentType& type, void*) {
          if (type.type_index != typeid(TransformUpdateFlag).hash_code() &&
              type.type_index != typeid(GlobalTransform).hash_code())
            emit(EntityBatchComponent{type.type_index, type.type_size, type.type_name});
        });
      },
      context.hidden_data_component_types);
  context.common_private_components = BuildCommonComponents(
      component_order_targets,
      [&](const Entity entity, const auto& emit) {
        scene->ForEachPrivateComponent(entity, [&](PrivateComponentElement& element) {
          emit(EntityBatchComponent{element.type_index, 0, element.private_component_data->GetTypeName()});
        });
      },
      context.hidden_private_component_types);
  return context;
}

std::vector<Entity> EntityBatchInspector::BuildGizmoParticipants(const std::shared_ptr<Scene>& scene,
                                                                 const std::vector<Entity>& exact_targets) {
  std::vector<Entity> participants;
  if (!scene)
    return participants;
  std::unordered_set<Entity, Entity> selected;
  for (const auto entity : exact_targets)
    if (scene->IsEntityValid(entity))
      selected.emplace(entity);
  for (const auto entity : exact_targets) {
    if (selected.find(entity) == selected.end())
      continue;
    auto ancestor = scene->GetParent(entity);
    bool covered = false;
    while (scene->IsEntityValid(ancestor)) {
      if (selected.find(ancestor) != selected.end()) {
        covered = true;
        break;
      }
      ancestor = scene->GetParent(ancestor);
    }
    if (!covered)
      participants.emplace_back(entity);
  }
  return participants;
}

EntityBatchSelectionBound EntityBatchInspector::BuildSelectionWorldBound(const std::shared_ptr<Scene>& scene,
                                                                         const std::vector<Entity>& roots) {
  EntityBatchSelectionBound result;
  if (!scene)
    return result;

  std::unordered_set<Entity, Entity> visited;
  std::vector<Entity> valid_roots;
  for (const auto root : roots) {
    if (!scene->IsEntityValid(root))
      continue;
    valid_roots.emplace_back(root);
    auto entities = scene->GetDescendants(root);
    entities.emplace_back(root);
    for (const auto entity : entities) {
      if (!scene->IsEntityValid(entity) || !scene->IsEntityEnabled(entity) || !visited.emplace(entity).second)
        continue;
      const auto transform = scene->GetDataComponent<GlobalTransform>(entity).value;
      if (scene->HasPrivateComponent<MeshRenderer>(entity)) {
        const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(entity).lock();
        if (renderer && renderer->IsEnabled())
          if (const auto mesh = renderer->mesh.Get<Mesh>())
            AccumulateBound(mesh->GetBound(), transform, result.world_bound, result.has_renderable_bounds);
      }
      if (scene->HasPrivateComponent<SkinnedMeshRenderer>(entity)) {
        const auto renderer = scene->GetOrSetPrivateComponent<SkinnedMeshRenderer>(entity).lock();
        if (renderer && renderer->IsEnabled())
          if (const auto mesh = renderer->skinned_mesh.Get<SkinnedMesh>())
            AccumulateBound(mesh->GetBound(), transform, result.world_bound, result.has_renderable_bounds);
      }
      if (scene->HasPrivateComponent<Particles>(entity)) {
        const auto particles = scene->GetOrSetPrivateComponent<Particles>(entity).lock();
        if (particles && particles->IsEnabled())
          AccumulateBound(particles->bounding_box, transform, result.world_bound, result.has_renderable_bounds);
      }
      if (scene->HasPrivateComponent<StrandsRenderer>(entity)) {
        const auto renderer = scene->GetOrSetPrivateComponent<StrandsRenderer>(entity).lock();
        if (renderer && renderer->IsEnabled())
          if (const auto strands = renderer->strands.Get<Strands>())
            AccumulateBound(strands->GetBound(), transform, result.world_bound, result.has_renderable_bounds);
      }
    }
  }

  if (!result.has_renderable_bounds) {
    for (const auto root : valid_roots) {
      const auto position = scene->GetDataComponent<GlobalTransform>(root).GetPosition();
      if (!IsFiniteVector(position))
        continue;
      result.world_bound.min = glm::min(result.world_bound.min, position);
      result.world_bound.max = glm::max(result.world_bound.max, position);
    }
  }
  result.valid = IsValidBound(result.world_bound);
  return result;
}

Entity EntityBatchInspector::FindGizmoReference(const std::shared_ptr<Scene>& scene,
                                                const std::vector<Entity>& participants, Entity primary) {
  if (!scene || participants.empty())
    return {};
  if (std::find(participants.begin(), participants.end(), primary) != participants.end())
    return primary;
  while (scene->IsEntityValid(primary)) {
    primary = scene->GetParent(primary);
    if (std::find(participants.begin(), participants.end(), primary) != participants.end())
      return primary;
  }
  return participants.back();
}

EntityBatchValue<glm::vec3> EntityBatchInspector::ReadLocalPosition(const std::shared_ptr<Scene>& scene,
                                                                    const std::vector<Entity>& targets) {
  return ReadVector(targets, [&](const Entity entity) {
    return scene->GetDataComponent<Transform>(entity).GetPosition();
  });
}

EntityBatchValue<glm::vec3> EntityBatchInspector::ReadLocalRotationDegrees(const std::shared_ptr<Scene>& scene,
                                                                           const std::vector<Entity>& targets) {
  return ReadVector(targets, [&](const Entity entity) {
    glm::vec3 position, rotation, scale;
    scene->GetDataComponent<Transform>(entity).Decompose(position, rotation, scale);
    return glm::degrees(rotation);
  });
}

EntityBatchValue<glm::vec3> EntityBatchInspector::ReadLocalScale(const std::shared_ptr<Scene>& scene,
                                                                 const std::vector<Entity>& targets) {
  return ReadVector(targets, [&](const Entity entity) {
    return scene->GetDataComponent<Transform>(entity).GetScale();
  });
}

bool EntityBatchInspector::WriteLocalTransformField(const std::shared_ptr<Scene>& scene,
                                                    const std::vector<Entity>& targets, const int field,
                                                    const glm::vec3& value, const int axis) {
  if (!scene || targets.empty() || !std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))
    return false;
  std::vector<Transform> values;
  values.reserve(targets.size());
  for (const auto entity : targets) {
    if (!scene->IsEntityValid(entity) || !scene->HasDataComponent<Transform>(entity))
      return false;
    auto transform = scene->GetDataComponent<Transform>(entity);
    glm::vec3 position, rotation, scale;
    transform.Decompose(position, rotation, scale);
    rotation = glm::degrees(rotation);
    auto patch = [&](glm::vec3& target) {
      if (axis >= 0 && axis < 3)
        target[axis] = value[axis];
      else
        target = value;
    };
    if (field == 0)
      patch(position);
    else if (field == 1)
      patch(rotation);
    else if (field == 2)
      patch(scale);
    else
      return false;
    transform.SetValue(position, glm::radians(rotation), scale);
    values.emplace_back(transform);
  }
  for (size_t i = 0; i < targets.size(); ++i)
    scene->SetDataComponent(targets[i], values[i]);
  return true;
}

bool EntityBatchInspector::WriteRelativeLocalTransformField(const std::shared_ptr<Scene>& scene,
                                                            const std::vector<Entity>& targets,
                                                            const std::vector<Transform>& original_transforms,
                                                            const int field, const int axis, const float start_value,
                                                            const float current_value) {
  if (!scene || targets.empty() || targets.size() != original_transforms.size() || axis < 0 || axis >= 3 || field < 0 ||
      field > 2 || !std::isfinite(start_value) || !std::isfinite(current_value))
    return false;
  std::vector<Transform> values;
  values.reserve(targets.size());
  const float delta = current_value - start_value;
  const float scale_ratio = std::abs(start_value) > 1.0e-6f ? current_value / start_value : 0.0f;
  for (size_t i = 0; i < targets.size(); ++i) {
    if (!scene->IsEntityValid(targets[i]) || !scene->HasDataComponent<Transform>(targets[i]))
      return false;
    auto transform = original_transforms[i];
    glm::vec3 position, rotation, scale;
    if (!transform.Decompose(position, rotation, scale))
      return false;
    rotation = glm::degrees(rotation);
    if (field == 0)
      position[axis] += delta;
    else if (field == 1)
      rotation[axis] += delta;
    else if (std::abs(start_value) > 1.0e-6f)
      scale[axis] *= scale_ratio;
    else
      scale[axis] += delta;
    transform.SetValue(position, glm::radians(rotation), scale);
    values.emplace_back(transform);
  }
  for (size_t i = 0; i < targets.size(); ++i)
    scene->SetDataComponent(targets[i], values[i]);
  return true;
}

bool EntityBatchInspector::TryApplyGizmoTransform(const glm::mat4& initial_handle, const glm::mat4& manipulated_handle,
                                                  const glm::mat4& participant_world,
                                                  const EntityBatchGizmoOperation operation,
                                                  const EntityBatchGizmoPivot pivot_mode,
                                                  const EntityBatchGizmoOrientation orientation_mode,
                                                  glm::mat4& candidate_world) {
  Transform initial, manipulated, participant;
  initial.value = initial_handle;
  manipulated.value = manipulated_handle;
  participant.value = participant_world;
  glm::vec3 initial_position, manipulated_position, initial_scale, manipulated_scale;
  glm::vec3 participant_position, participant_scale;
  glm::quat initial_rotation, manipulated_rotation, participant_rotation;
  if (!initial.Decompose(initial_position, initial_rotation, initial_scale) ||
      !manipulated.Decompose(manipulated_position, manipulated_rotation, manipulated_scale) ||
      !participant.Decompose(participant_position, participant_rotation, participant_scale))
    return false;
  if (glm::any(glm::lessThanEqual(glm::abs(initial_scale), glm::vec3(1.0e-8f))))
    return false;

  const auto world_rotation_delta = glm::normalize(manipulated_rotation * glm::inverse(initial_rotation));
  const auto local_rotation_delta = glm::normalize(glm::inverse(initial_rotation) * manipulated_rotation);
  const auto scale_delta = manipulated_scale / initial_scale;
  if (operation == EntityBatchGizmoOperation::Translate) {
    candidate_world = glm::translate(manipulated_position - initial_position) * participant_world;
  } else if (operation == EntityBatchGizmoOperation::Rotate) {
    if (pivot_mode == EntityBatchGizmoPivot::Center) {
      candidate_world = glm::translate(initial_position) * glm::mat4_cast(world_rotation_delta) *
                        glm::translate(-initial_position) * participant_world;
    } else {
      participant.SetValue(participant_position,
                           orientation_mode == EntityBatchGizmoOrientation::Local
                               ? glm::normalize(participant_rotation * local_rotation_delta)
                               : glm::normalize(world_rotation_delta * participant_rotation),
                           participant_scale);
      candidate_world = participant.value;
    }
  } else if (pivot_mode == EntityBatchGizmoPivot::Pivot && orientation_mode == EntityBatchGizmoOrientation::Local) {
    participant.SetValue(participant_position, participant_rotation, participant_scale * scale_delta);
    candidate_world = participant.value;
  } else {
    const auto origin = pivot_mode == EntityBatchGizmoPivot::Center ? initial_position : participant_position;
    const auto basis =
        orientation_mode == EntityBatchGizmoOrientation::Local ? glm::mat4_cast(initial_rotation) : glm::mat4(1.0f);
    candidate_world = glm::translate(origin) * basis * glm::scale(scale_delta) * glm::inverse(basis) *
                      glm::translate(-origin) * participant_world;
  }
  for (glm::length_t column = 0; column < 4; ++column)
    for (glm::length_t row = 0; row < 4; ++row)
      if (!std::isfinite(candidate_world[column][row]))
        return false;
  return true;
}
