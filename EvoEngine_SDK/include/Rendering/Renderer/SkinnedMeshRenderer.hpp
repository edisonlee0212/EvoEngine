
#pragma once
#include "Animator.hpp"
#include "Material.hpp"
#include "PrivateComponentRef.hpp"
#include "SkinnedMesh.hpp"

namespace evo_engine {

/**
 * @brief Represents a component that renders a skinned mesh with bone animation.
 *
 * This class provides mechanisms for updating bone matrices, managing ragdoll physics,
 * and handling asset references for skinned meshes and materials.
 */
class SkinnedMeshRenderer : public IPrivateComponent {
  friend class Animator;
  friend class AnimationLayer;
  friend class Prefab;
  friend class RenderLayer;

  /**
   * @brief Renders the bounding box of the skinned mesh with a specified color.
   *
   * @param editor_layer The editor layer used for rendering.
   * @param color The color to use for rendering the bounding box.
   */
  void RenderBound(const std::shared_ptr<EditorLayer>& editor_layer, glm::vec4& color);

  friend class Platform;
  friend class RenderInstanceStorage;

  /// Specifies whether ragdoll physics is applied to the renderer.
  bool rag_doll_ = false;

  /// Stores the chain of transformation matrices for the ragdoll physics.
  std::vector<glm::mat4> rag_doll_transform_chain_;

  /// Stores the entities that are bound to the skinned mesh.
  std::vector<EntityRef> bound_entities_;

 public:
  /**
   * @brief Updates the bone matrices for the skinned mesh.
   */
  void UpdateBoneMatrices();

  /// A flag indicating whether the ragdoll should be frozen.
  bool rag_doll_freeze = false;

  /**
   * @brief Retrieves whether the ragdoll physics is currently enabled.
   *
   * @return True if the ragdoll is enabled, otherwise false.
   */
  [[nodiscard]] bool RagDoll() const;

  /**
   * @brief Sets the ragdoll state for the skinned mesh renderer.
   *
   * @param value True to enable ragdoll physics, false to disable it.
   */
  void SetRagDoll(bool value);

  /// Reference to the animator controlling the skinned mesh.
  PrivateComponentRef animator;

  /// Pointer to the bone matrices structure.
  std::shared_ptr<BoneMatrices> bone_matrices;

  /// Specifies whether the skinned mesh should cast shadows.
  bool cast_shadow = true;

  /// Asset reference for the skinned mesh.
  AssetRef skinned_mesh;

  /// Asset reference for the material.
  AssetRef material;

  /**
   * @brief Inspects the component in the editor.
   *
   * @param editor_layer The editor layer used for inspection.
   * @return True if the inspection was successful, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Initializes the component when it is created.
   */
  void OnCreate() override;

  /**
   * @brief Cleans up the component when it is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Serializes the component data to the specified YAML emitter.
   *
   * @param out The YAML emitter to serialize data into.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the component data from the specified YAML node.
   *
   * @param in The YAML node containing serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Relinks handles and assets after components have been cloned or moved.
   *
   * @param map A mapping of old handles to new handles.
   * @param scene A pointer to the scene containing the component.
   */
  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) override;

  /**
   * @brief Collects all asset references used by the component.
   *
   * @param list A vector to store collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list) override;

  /**
   * @brief Executes actions to finalize cloning of this component.
   *
   * @param target The component onto which this component has been cloned.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) override;

  /**
   * @brief Retrieves the number of bones used in the ragdoll physics.
   *
   * @return The number of bones in the ragdoll system.
   */
  [[nodiscard]] size_t GetRagDollBoneSize() const;

  /**
   * @brief Sets a specific entity as a bound entity for ragdoll simulation.
   *
   * @param index The index of the ragdoll bone to bind the entity to.
   * @param entity The entity to bind to the specified bone.
   * @param reset_transform Whether to reset the entity's transform to match the bone's transform.
   */
  void SetRagDollBoundEntity(int index, const Entity& entity, bool reset_transform = true);

  /**
   * @brief Sets multiple entities as bound entities for ragdoll simulation.
   *
   * @param entities A vector of entities to bind to the ragdoll bones.
   * @param reset_transform Whether to reset the entities' transforms to match the bone transforms.
   */
  void SetRagDollBoundEntities(const std::vector<Entity>& entities, bool reset_transform = true);
};

}  // namespace evo_engine
