#pragma once
#include "StarPicking.hpp"

namespace universe_package {
struct StarBaseSample;
struct StarClusterGpuParameters;
struct StarClusterGpuResult;

StarClusterGpuResult EvaluateStar(const StarClusterGpuParameters& parameters, const StarBaseSample& sample);
glm::dmat4 StarReferenceFrame(const glm::dvec3& position, const glm::dmat4& cluster_world,
                              const glm::dmat3& previous_orientation = glm::dmat3(1));

struct StarFollowChange {
  glm::dmat4 camera_transform{1};
  bool rebase = false, look_at = false;
};

struct StarFollowCameraPose {
  glm::dvec3 position{};
  glm::dquat rotation{1, 0, 0, 0};
  bool valid = false;
};
StarFollowCameraPose CalculateStarFollowCameraPose(double radius);
StarFollowCameraPose CalculateStarOverviewCameraPose(const glm::dvec3& position, const glm::dquat& rotation,
                                                     double bounding_radius, const glm::dmat4& projection,
                                                     double near_distance);
double StarClusterBoundingRadius(const StarClusterGpuParameters& parameters, const glm::dvec3& gaussian_bound);

struct StarViewTransition {
  double radius_scale = 30, start_scale = 30, target_scale = 30;
  double fade_strength = 0, start_fade_strength = 0, target_fade_strength = 0, start_time = 0;
  void Update(double now);
  void SetLocked(bool locked, double now, double locked_fade_strength);
};

struct StarFollowState {
  bool following = false, available = false;
  uint64_t generation = 0;
  double selected_radius = 0;
  std::string status = "No star selected";
  glm::dvec3 selected_world_position{};
  glm::dmat4 selected_frame{1}, reference_to_world{1};
  StarPickSnapshot target;

  StarFollowChange Update(const StarPickSnapshot& selection, const StarClusterBatch& batch, bool toggle);
};
}  // namespace universe_package
