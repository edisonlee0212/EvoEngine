#pragma once
#include "Curve.hpp"
#include "Plot2D.hpp"
#include "MaizeState.hpp"

namespace digital_agriculture_plugin {
using namespace evo_engine;

class MaizeDescriptor;

/**
 * @brief Represents a pair of maize growth stages.
 */
class MaizeGrowthStagePair {
  /**
   * @brief Helper function to interpolate leaf state between two stages.
   *
   * @param left The MaizeLeafState at the start of the interpolation.
   * @param right The MaizeLeafState at the end of the interpolation.
   * @param a Interpolation factor (0.0 to 1.0).
   * @param leaf_index Index of the leaf being processed.
   */
  void LeafStateHelper(MaizeLeafState& left, MaizeLeafState& right, float& a, int leaf_index) const;

 public:
  MaizeState left_stage = MaizeState();               ///< The initial state of the maize.
  MaizeState right_stage = MaizeState();              ///< The target state of the maize.
  int state_mode = static_cast<int>(MaizeStateMode::Default);  ///< The state mode of the growth stage.

  /**
   * @brief Gets the interpolated leaf size.
   *
   * @param a Interpolation factor (0.0 to 1.0).
   * @return The interpolated leaf size.
   */
  [[nodiscard]] int GetLeafSize(float a) const;

  /**
   * @brief Gets the interpolated stem length.
   *
   * @param a Interpolation factor (0.0 to 1.0).
   * @return The interpolated stem length.
   */
  [[nodiscard]] float GetStemLength(float a) const;

  /**
   * @brief Gets the interpolated stem direction.
   *
   * @param a Interpolation factor (0.0 to 1.0).
   * @return The interpolated stem direction as a 3D vector.
   */
  [[nodiscard]] glm::vec3 GetStemDirection(float a) const;

  /**
   * @brief Gets a specific point along the stem.
   *
   * @param a Interpolation factor (0.0 to 1.0).
   * @param point The location along the stem to sample.
   * @return The position of the point along the stem.
   */
  [[nodiscard]] glm::vec3 GetStemPoint(float a, float point) const;

  /**
   * @brief Applies the interpolated panicle state to the target maize descriptor.
   *
   * @param target_state Target maize descriptor to modify.
   * @param a Interpolation factor (0.0 to 1.0).
   */
  void ApplyPanicle(const std::shared_ptr<MaizeDescriptor>& target_state, float a) const;

  /**
   * @brief Applies the interpolated stem state to the target maize descriptor.
   *
   * @param target_state Target maize descriptor to modify.
   * @param a Interpolation factor (0.0 to 1.0).
   */
  void ApplyStem(const std::shared_ptr<MaizeDescriptor>& target_state, float a) const;

  /**
   * @brief Applies the complete interpolated growth stage to the target maize descriptor.
   *
   * @param target_maize_descriptor Target maize descriptor to modify.
   * @param a Interpolation factor (0.0 to 1.0).
   */
  void Apply(const std::shared_ptr<MaizeDescriptor>& target_maize_descriptor, float a) const;

  /**
   * @brief Applies the interpolated leaf states to the target maize descriptor.
   *
   * @param target_descriptor Target maize descriptor to modify.
   * @param a Interpolation factor (0.0 to 1.0).
   */
  void ApplyLeaves(const std::shared_ptr<MaizeDescriptor>& target_descriptor, float a) const;

  /**
   * @brief Applies a specific leaf's interpolated state to the target maize descriptor.
   *
   * @param target_descriptor Target maize descriptor to modify.
   * @param a Interpolation factor (0.0 to 1.0).
   * @param leaf_index The index of the leaf to apply.
   */
  void ApplyLeaf(const std::shared_ptr<MaizeDescriptor>& target_descriptor, float a, int leaf_index) const;
};

/**
 * @brief Class representing multiple growth stages of Maize.
 */
class MaizeGrowthStages : public IAsset {
 public:
  std::vector<std::pair<float, MaizeState>>
      maize_growth_stages;                              ///< Contains maize growth stages mapped to time.
  int state_mode = static_cast<int>(MaizeStateMode::Default);  ///< The state mode of the growth stages.

  /**
   * @brief Imports maize growth stages from a CSV file.
   *
   * @param file_path Path to the CSV file.
   * @return True if import is successful, false otherwise.
   */
  [[nodiscard]] bool ImportCsv(const std::filesystem::path& file_path);

  /**
   * @brief Gets the start time of the current growth stage.
   *
   * @return The start time of the current stage.
   */
  [[nodiscard]] float GetCurrentStartTime() const;

  /**
   * @brief Gets the end time of the current growth stage.
   *
   * @return The end time of the current stage.
   */
  [[nodiscard]] float GetCurrentEndTime() const;

  /**
   * @brief Adds a new growth stage at the specified time.
   *
   * @param time The time at which the growth stage occurs.
   * @param state The maize state at that time.
   */
  void Add(float time, const MaizeState& state);

  /**
   * @brief Resets the time points for growth stages.
   *
   * @param previous_time The old time value.
   * @param new_time The new time value.
   */
  void ResetTime(float previous_time, float new_time);

  /**
   * @brief Removes a growth stage at the specified time.
   *
   * @param time The time of the growth stage to remove.
   */
  void Remove(float time);

  /**
   * @brief Applies the interpolated growth state to a maize descriptor.
   *
   * @param target_maize_descriptor The maize descriptor to apply the interpolated state to.
   * @param time The time at which the maize state should be applied.
   */
  void Apply(const std::shared_ptr<MaizeDescriptor>& target_maize_descriptor, float time) const;

  /**
   * @brief Inspects this asset in the editor.
   *
   * @param editor_layer The editor layer that is inspecting this asset.
   * @return True if the asset's content was not modified.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Serializes this object to YAML.
   *
   * @param out The YAML emitter where data is serialized.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes this object from YAML.
   *
   * @param in The YAML node containing the serialized data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Generates a texture for the thumbnail representation of the maize model.
   *
   * @return A shared pointer to the generated texture.
   */
  [[nodiscard]] std::shared_ptr<Texture2D> GenerateThumbnailTexture() override;

  /**
   * @brief Creates an entity representing the maize at a specific time.
   *
   * @param time The time at which the entity should represent the maize growth.
   * @return The created entity.
   */
  [[nodiscard]] Entity CreateEntity(float time = 0.0f) const;
};

}  // namespace digital_agriculture_plugin
