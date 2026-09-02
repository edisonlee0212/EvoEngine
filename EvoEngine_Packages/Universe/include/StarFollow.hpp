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
StarFollowCameraPose CalculateStarFollowCameraPose(const glm::dvec3& position, const glm::dquat& rotation,
                                                   double radius);

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
