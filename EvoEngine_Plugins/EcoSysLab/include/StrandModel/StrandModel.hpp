#pragma once
#include "TreeGrowthData.hpp"

using namespace evo_engine;
namespace eco_sys_lab_plugin {
class StrandModel {
  void ApplyProfile(const StrandModelParameters& strand_model_parameters, SkeletonNodeHandle node_handle);
  void CalculateProfile(float max_root_distance, SkeletonNodeHandle node_handle,
                        const StrandModelParameters& strand_model_parameters);

  void PackTask(SkeletonNodeHandle node_handle, const StrandModelParameters& strand_model_parameters);
  void MergeTask(float max_root_distance, SkeletonNodeHandle node_handle,
                 const StrandModelParameters& strand_model_parameters);
  void CopyFrontToBackTask(SkeletonNodeHandle node_handle);
  std::mt19937 random_engine_;

 public:
  int seed = 0;
  StrandModelSkeleton strand_model_skeleton;
  void ResetAllProfiles(const StrandModelParameters& strand_model_parameters);
  void InitializeProfiles(const StrandModelParameters& strand_model_parameters);
  JobHandle CalculateProfiles(const StrandModelParameters& strand_model_parameters);
  void CalculateStrandProfileAdjustedTransforms(const StrandModelParameters& strand_model_parameters);
  void ApplyProfiles(const StrandModelParameters& strand_model_parameters);

  [[nodiscard]] glm::vec3 InterpolateStrandSegmentPosition(StrandSegmentHandle strand_segment_handle, float a) const;
  [[nodiscard]] glm::vec3 InterpolateStrandSegmentAxis(StrandSegmentHandle strand_segment_handle, float a) const;
  [[nodiscard]] float InterpolateStrandSegmentRadius(StrandSegmentHandle strand_segment_handle, float a) const;

  static glm::vec2 DiskRand(std::mt19937& random_engine, float radius);
};
}  // namespace eco_sys_lab_plugin