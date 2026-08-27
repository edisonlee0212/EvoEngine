#pragma once

#include "Bound.hpp"
#include "Entity.hpp"
#include "Transform.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace evo_engine {
class EVOENGINE_API Scene;

struct EntityBatchComponent final {
  size_t type_index = 0;
  size_t type_size = 0;
  std::string type_name;
};

struct EntityBatchInspectionContext final {
  std::vector<Entity> targets;
  Entity primary{};
  std::vector<EntityBatchComponent> common_data_components;
  std::vector<EntityBatchComponent> common_private_components;
  size_t hidden_data_component_types = 0;
  size_t hidden_private_component_types = 0;
};

struct EntityBatchSelectionBound final {
  Bound world_bound;
  bool has_renderable_bounds = false;
  bool valid = false;
};

enum class EntityBatchGizmoOperation : uint8_t { Translate, Rotate, Scale };
enum class EntityBatchGizmoPivot : uint8_t { Pivot, Center };
enum class EntityBatchGizmoOrientation : uint8_t { Local, Global };
enum class EntityInspectorDispatch : uint8_t { Single, Batch, Unsupported };

template <typename T>
struct EntityBatchValue final {
  T value{};
  bool mixed = false;
  std::array<bool, 3> mixed_axes{};
};

class EVOENGINE_API EntityBatchInspector final {
 public:
  [[nodiscard]] static EntityInspectorDispatch ResolveInspectorDispatch(size_t target_count,
                                                                        bool single_inspector_available,
                                                                        bool batch_inspector_available,
                                                                        bool force_batch = false);
  [[nodiscard]] static EntityBatchInspectionContext BuildContext(const std::shared_ptr<Scene>& scene,
                                                                 const std::vector<Entity>& exact_targets,
                                                                 Entity primary);
  [[nodiscard]] static std::vector<Entity> BuildGizmoParticipants(const std::shared_ptr<Scene>& scene,
                                                                  const std::vector<Entity>& exact_targets);
  [[nodiscard]] static EntityBatchSelectionBound BuildSelectionWorldBound(const std::shared_ptr<Scene>& scene,
                                                                          const std::vector<Entity>& roots);
  [[nodiscard]] static Entity FindGizmoReference(const std::shared_ptr<Scene>& scene,
                                                 const std::vector<Entity>& participants, Entity primary);
  [[nodiscard]] static EntityBatchValue<glm::vec3> ReadLocalPosition(const std::shared_ptr<Scene>& scene,
                                                                     const std::vector<Entity>& targets);
  [[nodiscard]] static EntityBatchValue<glm::vec3> ReadLocalRotationDegrees(const std::shared_ptr<Scene>& scene,
                                                                            const std::vector<Entity>& targets);
  [[nodiscard]] static EntityBatchValue<glm::vec3> ReadLocalScale(const std::shared_ptr<Scene>& scene,
                                                                  const std::vector<Entity>& targets);
  static bool WriteLocalTransformField(const std::shared_ptr<Scene>& scene, const std::vector<Entity>& targets,
                                       int field, const glm::vec3& value, int axis = -1);
  static bool WriteRelativeLocalTransformField(const std::shared_ptr<Scene>& scene, const std::vector<Entity>& targets,
                                               const std::vector<Transform>& original_transforms, int field, int axis,
                                               float start_value, float current_value);
  [[nodiscard]] static bool TryApplyGizmoTransform(const glm::mat4& initial_handle, const glm::mat4& manipulated_handle,
                                                   const glm::mat4& participant_world,
                                                   EntityBatchGizmoOperation operation,
                                                   EntityBatchGizmoPivot pivot_mode,
                                                   EntityBatchGizmoOrientation orientation_mode,
                                                   glm::mat4& candidate_world);
};
}  // namespace evo_engine
