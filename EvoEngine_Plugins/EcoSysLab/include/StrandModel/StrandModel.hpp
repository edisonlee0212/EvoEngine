
#pragma once
#include "TreeGrowthData.hpp"

using namespace evo_engine;
namespace eco_sys_lab_plugin {

/**
 * @brief Class representing the strand model used for procedural tree generation.
 */
class StrandModel {
  /**
   * @brief Applies a given strand model profile to a skeleton node.
   * @param strand_model_parameters Parameters defining the strand model profile.
   * @param node_handle Handle to the skeleton node where the profile is applied.
   */
  void ApplyProfile(const StrandModelParameters& strand_model_parameters, SkeletonNodeHandle node_handle);

  /**
   * @brief Calculates the strand model profile for a given skeleton node.
   * @param max_root_distance Maximum distance from the root for profile calculations.
   * @param node_handle Handle to the skeleton node.
   * @param strand_model_parameters Parameters defining the strand model profile.
   */
  void CalculateProfile(float max_root_distance, SkeletonNodeHandle node_handle,
                        const StrandModelParameters& strand_model_parameters);

  /**
   * @brief Packs a processing task for a given skeleton node.
   * @param node_handle Handle to the skeleton node.
   * @param strand_model_parameters Parameters defining the strand model.
   */
  void PackTask(SkeletonNodeHandle node_handle, const StrandModelParameters& strand_model_parameters);

  /**
   * @brief Merges the computed profile data for a given skeleton node.
   * @param max_root_distance Maximum distance from the root for profile merging.
   * @param node_handle Handle to the skeleton node.
   * @param strand_model_parameters Parameters defining the strand model profile.
   */
  void MergeTask(float max_root_distance, SkeletonNodeHandle node_handle,
                 const StrandModelParameters& strand_model_parameters);

  /**
   * @brief Copies the front buffer data to the back buffer for a given skeleton node.
   * @param node_handle Handle to the skeleton node.
   */
  void CopyFrontToBackTask(SkeletonNodeHandle node_handle);

  /// Random engine instance used for stochastic profile variations.
  std::mt19937 random_engine_;

 public:
  /// Seed value for random number generation.
  int seed = 0;

  /// Structure representing the skeleton of the strand model.
  StrandModelSkeleton strand_model_skeleton;

  /**
   * @brief Resets all strand model profiles to their default states.
   * @param strand_model_parameters Parameters defining the strand model profile.
   */
  void ResetAllProfiles(const StrandModelParameters& strand_model_parameters);

  /**
   * @brief Initializes strand model profiles based on given parameters.
   * @param strand_model_parameters Parameters defining the strand model profile.
   */
  void InitializeProfiles(const StrandModelParameters& strand_model_parameters);

  /**
   * @brief Computes profiles for the strand model asynchronously.
   * @param strand_model_parameters Parameters defining the strand model profile.
   * @return JobHandle representing the asynchronous computation task.
   */
  JobHandle CalculateProfiles(const StrandModelParameters& strand_model_parameters);

  /**
   * @brief Adjusts strand profile transformations based on computed data.
   * @param strand_model_parameters Parameters defining the strand model profile.
   */
  void CalculateStrandProfileAdjustedTransforms(const StrandModelParameters& strand_model_parameters);

  /**
   * @brief Applies strand model profiles to the skeleton structure.
   * @param strand_model_parameters Parameters defining the strand model profile.
   */
  void ApplyProfiles(const StrandModelParameters& strand_model_parameters);

  /**
   * @brief Interpolates the position of a strand segment.
   * @param strand_segment_handle Handle to the strand segment.
   * @param a Interpolation factor (typically between 0 and 1).
   * @return Interpolated position as a glm::vec3.
   */
  [[nodiscard]] glm::vec3 InterpolateStrandSegmentPosition(StrandSegmentHandle strand_segment_handle, float a) const;

  /**
   * @brief Interpolates the axis direction of a strand segment.
   * @param strand_segment_handle Handle to the strand segment.
   * @param a Interpolation factor (typically between 0 and 1).
   * @return Interpolated axis direction as a glm::vec3.
   */
  [[nodiscard]] glm::vec3 InterpolateStrandSegmentAxis(StrandSegmentHandle strand_segment_handle, float a) const;

  /**
   * @brief Interpolates the radius of a strand segment.
   * @param strand_segment_handle Handle to the strand segment.
   * @param a Interpolation factor (typically between 0 and 1).
   * @return Interpolated radius value.
   */
  [[nodiscard]] float InterpolateStrandSegmentRadius(StrandSegmentHandle strand_segment_handle, float a) const;
};

}  // namespace eco_sys_lab_plugin
