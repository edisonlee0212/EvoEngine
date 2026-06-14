
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

  [[nodiscard]] const std::vector<glm::mat4>& PeekRagDollTransformChain() const;

  [[nodiscard]] std::vector<glm::mat4>& RefRagDollTransformChain();

  [[nodiscard]] const std::vector<EntityRef>& PeekRagDollBoundEntities() const;

  [[nodiscard]] std::vector<EntityRef>& RefRagDollBoundEntities();

  [[nodiscard]] Entity GetRagDollBoundEntity(int index);

  /**
   * @brief Sets the ragdoll state for the skinned mesh renderer.
   *
   * @param value True to enable ragdoll physics, false to disable it.
   */
  void SetRagDoll(bool value);

  void SetRagDollState(bool value);

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
   * @brief Initializes the component when it is created.
   */
  void OnCreate() override;

  /**
   * @brief Cleans up the component when it is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Relinks handles and assets after components have been cloned or moved.
   *
   * @param map A mapping of old handles to new handles.
   * @param scene A pointer to the scene containing the component.
   */
  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene);

  /**
   * @brief Collects all asset references used by the component.
   *
   * @param list A vector to store collected asset references.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);

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

  void ClearRagDollBoundEntity(int index);

  /**
   * @brief Sets multiple entities as bound entities for ragdoll simulation.
   *
   * @param entities A vector of entities to bind to the ragdoll bones.
   * @param reset_transform Whether to reset the entities' transforms to match the bone transforms.
   */
  void SetRagDollBoundEntities(const std::vector<Entity>& entities, bool reset_transform = true);
};

}  // namespace evo_engine
