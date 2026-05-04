
#pragma once
#include "EntityMetadata.hpp"
#include "Transform.hpp"

namespace evo_engine {

/**
 * @class TransformGraph
 * @brief Responsible for managing and updating the transform graph of entities within the engine.
 *
 * The TransformGraph class is a singleton that provides functionalities to calculate and
 * manage global transformations of entities and their hierarchy within a scene.
 */
class TransformGraph final {
 public:
  static TransformGraph& GetInstance();

 private:
  friend class PhysicsSystem;  ///< Grants PhysicsSystem access to private and protected members of TransformGraph.
  friend class Application;    ///< Grants Application access to private and protected members of TransformGraph.

  EntityQuery transform_query_;  ///< Maintains the query responsible for fetching entities with transform components.
  bool physics_system_override_ =
      false;  ///< Indicates whether the physics system has overridden transform calculations.

  /**
   * @brief Calculates the transformation graph starting from a parent entity.
   *
   * This method calculates the global transformations of entities in the scene, starting
   * from the provided parent entity and propagating through its descendants.
   *
   * @param scene Reference to the scene containing the entities.
   * @param entity_infos A vector of metadata for all entities.
   * @param parent_global_transform The global transform of the parent entity.
   * @param parent The parent entity to start the transformation calculation from.
   */
  static void CalculateTransformGraph(const std::shared_ptr<Scene>& scene,
                                      const std::vector<EntityMetadata>& entity_infos,
                                      const GlobalTransform& parent_global_transform, const Entity& parent);

  /**
   * @brief Initializes the TransformGraph instance.
   *
   * This method is used to initialize the TransformGraph as part of the engine setup process.
   */
  static void Initialize();

 public:
  /**
   * @brief Calculates the transformation graph for all descendants of a specific entity.
   *
   * This method updates the global transformations of all descendants of the specified entity
   * within the given scene.
   *
   * @param scene Reference to the scene containing the entities.
   * @param entity The entity whose descendants' transformations need to be calculated.
   */
  static void CalculateTransformGraphForDescendants(const std::shared_ptr<Scene>& scene, const Entity& entity);

  /**
   * @brief Calculates the transformation graphs for all entities in the scene.
   *
   * Updates global transformations for all entities in the scene. It can also check only static entities if requested.
   *
   * @param scene Reference to the scene containing the entities.
   * @param check_static Optional parameter indicating whether only static entities should be checked. Default is true.
   */
  static void CalculateTransformGraphs(const std::shared_ptr<Scene>& scene, bool check_static = true);
};

}  // namespace evo_engine
