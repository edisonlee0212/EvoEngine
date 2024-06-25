#pragma once
#include "Scene.hpp"
#include "Transform.hpp"
namespace evo_engine {
#pragma region Bone
struct BonePosition {
  glm::vec3 value;
  float time_stamp;
};

struct BoneRotation {
  glm::quat value;
  float time_stamp;
};

struct BoneScale {
  glm::vec3 m_value;
  float time_stamp;
};

struct BoneKeyFrames {
  std::vector<BonePosition> positions;
  std::vector<BoneRotation> rotations;
  std::vector<BoneScale> scales;
  float max_time_stamp = 0.0f;
  /* Gets the current index on mKeyPositions to interpolate to based on the current
  animation time */
  int GetPositionIndex(const float &animation_time) const;

  /* Gets the current index on mKeyRotations to interpolate to based on the current
  animation time */
  int GetRotationIndex(const float &animation_time) const;

  /* Gets the current index on mKeyScalings to interpolate to based on the current
  animation time */
  int GetScaleIndex(const float &animation_time) const;

  /* Gets normalized value for Lerp & Slerp*/
  static float GetScaleFactor(const float &last_time_stamp, const float &next_time_stamp, const float &animation_time);

  /* figures out which position keys to interpolate b/w and performs the interpolation
  and returns the translation matrix */
  glm::mat4 InterpolatePosition(const float &animation_time) const;

  /* figures out which rotations keys to interpolate b/w and performs the interpolation
  and returns the rotation matrix */
  glm::mat4 InterpolateRotation(const float &animation_time) const;

  /* figures out which scaling keys to interpolate b/w and performs the interpolation
  and returns the scale matrix */
  glm::mat4 InterpolateScaling(const float &animation_time) const;

  void Serialize(YAML::Emitter &out) const;
  void Deserialize(const YAML::Node &in);
};

struct Bone {
  std::map<std::string, BoneKeyFrames> animations;
  std::string name;
  Transform offset_matrix = Transform();
  size_t index;
  std::vector<std::shared_ptr<Bone>> children;
  /* Interpolates b/w positions,rotations & scaling keys based on the current time of the
  animation and prepares the local transformation matrix by combining all keys transformations */
  void Animate(const std::string &target_name, const float &animation_time, const glm::mat4 &parent_transform,
               const glm::mat4 &root_transform, std::vector<glm::mat4> &results);
  bool OnInspect();

  void Serialize(YAML::Emitter &out) const;
  void Deserialize(const YAML::Node &in);
};

#pragma endregion
class Animation : public IAsset {
 public:
  std::map<std::string, float> animation_length;
  std::shared_ptr<Bone> root_bone;
  size_t bone_size = 0;
  [[nodiscard]] std::shared_ptr<Bone> &UnsafeGetRootBone();
  [[nodiscard]] std::map<std::string, float> &UnsafeGetAnimationLengths();
  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;
  void Animate(const std::string &name, const float &animation_time, const glm::mat4 &root_transform,
               std::vector<glm::mat4> &results);
  [[nodiscard]] std::string GetFirstAvailableAnimationName() const;
  [[nodiscard]] float GetAnimationLength(const std::string &animation_name) const;
  [[nodiscard]] bool HasAnimation(const std::string &animation_name) const;
  [[nodiscard]] bool IsEmpty() const;
  void Serialize(YAML::Emitter &out) const override;
  void Deserialize(const YAML::Node &in) override;
};
}  // namespace evo_engine