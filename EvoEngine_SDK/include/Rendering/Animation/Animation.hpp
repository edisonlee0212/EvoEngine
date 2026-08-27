
#pragma once
#include "Scene.hpp"
#include "Transform.hpp"
namespace evo_engine {

#pragma region Bone

/**
 * @struct BonePosition
 * @brief Represents a position keyframe for a bone, including its timestamp.
 */
struct BonePosition {
  glm::vec3 value;   ///< The position value.
  float time_stamp;  ///< The timestamp of the keyframe.
};

/**
 * @struct BoneRotation
 * @brief Represents a rotation keyframe for a bone, including its timestamp.
 */
struct BoneRotation {
  glm::quat value;   ///< The rotation value in quaternion form.
  float time_stamp;  ///< The timestamp of the keyframe.
};

/**
 * @struct BoneScale
 * @brief Represents a scale keyframe for a bone, including its timestamp.
 */
struct BoneScale {
  glm::vec3 m_value;  ///< The scale value.
  float time_stamp;   ///< The timestamp of the keyframe.
};

/**
 * @struct BoneKeyFrames
 * @brief Encapsulates all keyframes (position, rotation, scale) for a bone
 *        and provides utilities for interpolation.
 */
struct EVOENGINE_API BoneKeyFrames {
  std::vector<BonePosition> positions;  ///< List of position keyframes.
  std::vector<BoneRotation> rotations;  ///< List of rotation keyframes.
  std::vector<BoneScale> scales;        ///< List of scaling keyframes.
  float max_time_stamp = 0.0f;          ///< Maximum timestamp in the keyframes.

  /**
   * @brief Gets the current index in positions to interpolate to
   *        based on the current animation time.
   * @param animation_time The current time within the animation.
   * @return The index of the position keyframe.
   */
  int GetPositionIndex(const float &animation_time) const;

  /**
   * @brief Gets the current index in rotations to interpolate to
   *        based on the current animation time.
   * @param animation_time The current time within the animation.
   * @return The index of the rotation keyframe.
   */
  int GetRotationIndex(const float &animation_time) const;

  /**
   * @brief Gets the current index in scales to interpolate to
   *        based on the current animation time.
   * @param animation_time The current time within the animation.
   * @return The index of the scaling keyframe.
   */
  int GetScaleIndex(const float &animation_time) const;

  /**
   * @brief Calculates the scale factor used for interpolation.
   * @param last_time_stamp The time stamp of the previous keyframe.
   * @param next_time_stamp The time stamp of the next keyframe.
   * @param animation_time The current time within the animation.
   * @return The normalized scale factor.
   */
  static float GetScaleFactor(const float &last_time_stamp, const float &next_time_stamp, const float &animation_time);

  /**
   * @brief Interpolates between position keyframes and computes the translation matrix.
   * @param animation_time The current time within the animation.
   * @return The interpolated translation matrix.
   */
  glm::mat4 InterpolatePosition(const float &animation_time) const;

  /**
   * @brief Interpolates between rotation keyframes and computes the rotation matrix.
   * @param animation_time The current time within the animation.
   * @return The interpolated rotation matrix.
   */
  glm::mat4 InterpolateRotation(const float &animation_time) const;

  /**
   * @brief Interpolates between scaling keyframes and computes the scaling matrix.
   * @param animation_time The current time within the animation.
   * @return The interpolated scaling matrix.
   */
  glm::mat4 InterpolateScaling(const float &animation_time) const;

  /**
   * @brief Serializes the BoneKeyFrames data to a YAML emitter.
   * @param out The YAML emitter to serialize the data into.
   */
  void Serialize(YAML::Emitter &out) const;

  /**
   * @brief Deserializes the BoneKeyFrames data from a YAML node.
   * @param in The YAML node to deserialize the data from.
   */
  void Deserialize(const YAML::Node &in);
};

/**
 * @struct Bone
 * @brief Defines a single bone in the skeleton hierarchy,
 *        along with its animations and transformations.
 */
struct EVOENGINE_API Bone {
  std::map<std::string, BoneKeyFrames> animations;  ///< Map of animation names to their keyframes.
  std::string name;                                 ///< Name of the bone.
  Transform offset_matrix = Transform();            ///< Offset matrix for the bone.
  size_t index;                                     ///< Index of the bone in the skeleton array.
  std::vector<std::shared_ptr<Bone>> children;      ///< Child bones in the hierarchy.

  /**
   * @brief Performs interpolation of keyframes and computes the local transformation matrix.
   * @param target_name The target animation name.
   * @param animation_time The current time within the animation.
   * @param parent_transform The transform of the parent bone.
   * @param root_transform The transform of the root bone.
   * @param results The vector to store the final transformation matrices.
   */
  void Animate(const std::string &target_name, const float &animation_time, const glm::mat4 &parent_transform,
               const glm::mat4 &root_transform, std::vector<glm::mat4> &results);

  /**
   * @brief Serializes the Bone data to a YAML emitter.
   * @param out The YAML emitter to serialize the data into.
   */
  void Serialize(YAML::Emitter &out) const;

  /**
   * @brief Deserializes the Bone data from a YAML node.
   * @param in The YAML node to deserialize the data from.
   */
  void Deserialize(const YAML::Node &in);
};

#pragma endregion

/**
 * @class Animation
 * @brief Represents an animation asset containing skeleton and animation details.
 */
class EVOENGINE_API Animation : public IAsset {
 public:
  [[nodiscard]] bool SupportsStagedLoading() const {
    return true;
  }

  std::map<std::string, float> animation_length;  ///< Map of animation names to their lengths.
  std::shared_ptr<Bone> root_bone;                ///< Root bone of the skeleton hierarchy.
  size_t bone_size = 0;                           ///< Total number of bones in the skeleton.

  /**
   * @brief Provides unsafe access to the root bone.
   * @return A reference to the root bone.
   */
  [[nodiscard]] std::shared_ptr<Bone> &UnsafeGetRootBone();

  /**
   * @brief Provides unsafe access to the animation lengths.
   * @return A reference to the map of animation lengths.
   */
  [[nodiscard]] std::map<std::string, float> &UnsafeGetAnimationLengths();

  /**
   * @brief Performs the animation by interpolating transformations.
   * @param name The name of the animation.
   * @param animation_time The current time within the animation.
   * @param root_transform The transformation of the root bone.
   * @param results The vector to store the resulting transformation matrices.
   */
  void Animate(const std::string &name, const float &animation_time, const glm::mat4 &root_transform,
               std::vector<glm::mat4> &results);

  /**
   * @brief Retrieves the name of the first available animation.
   * @return The name of the first animation.
   */
  [[nodiscard]] std::string GetFirstAvailableAnimationName() const;

  /**
   * @brief Gets the length of a specified animation.
   * @param animation_name The name of the animation.
   * @return The length of the animation.
   */
  [[nodiscard]] float GetAnimationLength(const std::string &animation_name) const;

  /**
   * @brief Checks if a specified animation exists.
   * @param animation_name The name of the animation.
   * @return True if the animation exists; otherwise false.
   */
  [[nodiscard]] bool HasAnimation(const std::string &animation_name) const;

  /**
   * @brief Checks if there are no animations available.
   * @return True if there are no animations; otherwise false.
   */
  [[nodiscard]] bool IsEmpty() const;
};

}  // namespace evo_engine
