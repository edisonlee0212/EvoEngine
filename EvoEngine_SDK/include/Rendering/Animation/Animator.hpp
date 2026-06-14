
#pragma once
#include "Animation.hpp"
#include "Scene.hpp"

namespace evo_engine {

/**
 * @class Animator
 * @brief A class responsible for managing bone animations and transformations.
 *
 * This class is used to set up and apply animations to a skeleton structure,
 * managing bones, transforms, and offset matrices. It is designed as a final
 * class and cannot be inherited from.
 */
class Animator final : public IPrivateComponent {
  /**
   * @brief A collection of bones used in the skeletal animation.
   */
  std::vector<std::shared_ptr<Bone>> bones_;

  /// @cond PRIVATE
  friend class SkinnedMeshRenderer;
  friend class RenderLayer;
  /// @endcond

  /**
   * @brief A collection of transformation matrices, forming a transformation chain.
   */
  std::vector<glm::mat4> transform_chain_;

  /**
   * @brief A collection of offset matrices for each bone.
   */
  std::vector<glm::mat4> offset_matrices_;

  /**
   * @brief A collection of names corresponding to the bones.
   */
  std::vector<std::string> names_;

  /**
   * @brief Reference to the animation asset.
   */
  AssetRef animation_;

  /**
   * @brief Number of bones in the skeleton.
   */
  size_t bone_size_ = 0;

  /**
   * @brief Helper function to recursively set up bones for animation.
   * @param bone_walker A shared pointer to the current bone.
   */
  void BoneSetter(const std::shared_ptr<Bone> &bone_walker);

  /**
   * @brief Internal setup function for initializing the animator.
   */
  void Setup();

  /**
   * @brief Name of the currently activated animation.
   */
  std::string current_activated_animation_;

  /**
   * @brief Current time point in the active animation.
   */
  float current_animation_time_ = 0.0f;

  /**
   * @brief Applies the calculated transformations to the skeleton.
   */
  void Apply();

 public:
  /**
   * @brief Sets up the animator for ragdoll-like behavior by only setting offset matrices.
   *
   * @param name A collection of bone names.
   * @param offset_matrices A collection of offset matrices.
   */
  void Setup(const std::vector<std::string> &name, const std::vector<glm::mat4> &offset_matrices);

  /**
   * @brief Applies the offset matrices to the skeleton.
   */
  void ApplyOffsetMatrices();

  /**
   * @brief Retrieves the reverse transform matrix for a specified bone.
   *
   * @param bone_index The index of the target bone.
   * @return The reverse transformation matrix.
   */
  [[nodiscard]] glm::mat4 GetReverseTransform(int bone_index) const;

  /**
   * @brief Gets the current animation time point.
   *
   * @return The current time point in the active animation.
   */
  [[nodiscard]] float GetCurrentAnimationTimePoint() const;

  /**
   * @brief Retrieves the name of the current active animation.
   *
   * @return The name of the currently active animation.
   */
  [[nodiscard]] std::string GetCurrentAnimationName() const;

  /**
   * @brief Gets the number of bones currently managed by this animator.
   *
   * @return The number of bones in the current animation skeleton.
   */
  [[nodiscard]] size_t GetBoneSize() const;

  [[nodiscard]] const std::vector<glm::mat4> &PeekTransformChain() const;

  [[nodiscard]] std::vector<glm::mat4> &RefTransformChain();

  [[nodiscard]] const std::vector<glm::mat4> &PeekOffsetMatrices() const;

  [[nodiscard]] std::vector<glm::mat4> &RefOffsetMatrices();

  [[nodiscard]] const std::vector<std::string> &PeekBoneNames() const;

  [[nodiscard]] std::vector<std::string> &RefBoneNames();

  /**
   * @brief Animates the skeleton using the specified animation name and time.
   *
   * @param animation_name The name of the animation to play.
   * @param time The time point in the animation to play.
   */
  void Animate(const std::string &animation_name, float time);

  /**
   * @brief Animates the skeleton using the given time point for the current animation.
   *
   * @param time The time point in the animation to play.
   */
  void Animate(float time);

  /**
   * @brief Called when the component is destroyed.
   */
  void OnDestroy() override;

  /**
   * @brief Sets up the animator using a target animation.
   *
   * @param target_animation A shared pointer to the target animation.
   */
  void Setup(const std::shared_ptr<Animation> &target_animation);

  /**
   * @brief Clears the current animation asset and derived skeleton state.
   */
  void ClearAnimation();

  /**
   * @brief Called during the post-clone process to apply actions to the cloned object.
   *
   * @param target A shared pointer to the cloned component target.
   */
  void PostCloneAction(const std::shared_ptr<IPrivateComponent> &target) override;

  /**
   * @brief Retrieves the animation associated with this animator.
   *
   * @return A shared pointer to the animation.
   */
  [[nodiscard]] std::shared_ptr<Animation> GetAnimation();

  [[nodiscard]] const AssetRef &PeekAnimationRef() const;

  [[nodiscard]] AssetRef &RefAnimationRef();

  void RestorePlaybackState(const std::string &animation_name, float time);

  void RebuildAnimationState();

  /**
   * @brief Collects asset references used by the animator.
   *
   * @param list The list to append asset references to.
   */
  void CollectAssetRef(std::vector<AssetRef> &list);
};

}  // namespace evo_engine
