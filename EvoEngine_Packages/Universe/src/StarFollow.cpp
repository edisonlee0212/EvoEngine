#include "StarFollow.hpp"
#include <limits>
#include "UniverseLayer.hpp"

using namespace universe_package;

namespace {
bool ValidFollowRadius(const double radius) {
  const double distance = radius * 50;
  return std::isfinite(radius) && radius > 0 && distance >= std::numeric_limits<float>::denorm_min() &&
         distance <= (std::numeric_limits<float>::max)();
}
}  // namespace

StarClusterGpuResult universe_package::EvaluateStar(const StarClusterGpuParameters& p, const StarBaseSample& sample) {
  const double proportion = sample.orbital_proportion;
  const bool disk = proportion > p.ellipse1.z;
  const double t = disk ? (proportion - p.ellipse1.z) / (1 - p.ellipse1.z) : proportion / p.ellipse1.z;
  const auto mix = [t](const double a, const double b) {
    return a + (b - a) * t;
  };
  const double a = disk ? mix(p.ellipse0.z, p.ellipse0.x) : mix(p.ellipse1.x, p.ellipse0.z);
  const double b = disk ? mix(p.ellipse0.w, p.ellipse0.y) : mix(p.ellipse1.y, p.ellipse0.w);
  const double tilt_x = disk ? mix(p.speed_tilt.w, p.speed_tilt.y) : mix(p.tilt_radius.y, p.speed_tilt.w);
  const double tilt_z = disk ? mix(p.tilt_radius.x, p.speed_tilt.z) : mix(p.tilt_radius.z, p.tilt_radius.x);
  const double speed = disk ? mix(p.spread_speed.w, p.spread_speed.z) : mix(p.speed_tilt.x, p.spread_speed.w);
  const double angle = (proportion * 360 + p.time_padding.x) / std::sqrt(a + b) * speed;
  glm::dvec3 point(std::sin(angle) * a, 0, std::cos(angle) * b);
  const auto rotate = [&](const int axis, const double degrees) {
    const double s = std::sin(glm::radians(degrees)), c = std::cos(glm::radians(degrees));
    if (axis == 0)
      point = {point.x, c * point.y - s * point.z, s * point.y + c * point.z};
    else if (axis == 1)
      point = {c * point.x + s * point.z, point.y, -s * point.x + c * point.z};
    else
      point = {c * point.x - s * point.y, s * point.x + c * point.y, point.z};
  };
  rotate(0, tilt_x);
  rotate(1, -p.ellipse1.w * proportion);
  rotate(2, tilt_z);
  point += glm::dvec3(p.center_offset) * (1 - proportion) + glm::dvec3(p.center_position);
  point += glm::dvec3(sample.gaussian_x * p.spread_speed.y, sample.gaussian_y * p.spread_speed.x,
                      sample.gaussian_z * p.spread_speed.y) *
           (p.ellipse0.x + p.ellipse0.y);
  point /= 20;
  const glm::dvec4 world = glm::dmat4(p.world0, p.world1, p.world2, p.world3) * glm::dvec4(point, 1);
  StarClusterGpuResult result;
  result.world_position_radius = glm::dvec4(glm::dvec3(world), p.tilt_radius.w);
  result.color_emission = disk ? glm::mix(p.core_color_intensity, p.disk_color_intensity, float(t))
                               : glm::mix(p.center_color_intensity, p.core_color_intensity, float(t));
  result.alpha_padding.x = static_cast<float>(p.time_padding.y);
  return result;
}

glm::dmat4 universe_package::StarReferenceFrame(const glm::dvec3& position, const glm::dmat4& cluster_world,
                                                const glm::dmat3& previous_orientation) {
  glm::dmat4 frame(previous_orientation);
  frame[3] = glm::dvec4(position, 1);
  const auto delta = glm::dvec3(cluster_world[3]) - position;
  if (glm::dot(delta, delta) < 1e-20)
    return frame;
  const auto front = glm::normalize(delta);
  auto up = glm::dvec3(cluster_world[1]);
  auto right = glm::cross(front, up);
  if (glm::dot(right, right) < 1e-12 * (std::max)(1.0, glm::dot(up, up))) {
    up = previous_orientation[1];
    right = glm::cross(front, up);
    if (glm::dot(right, right) < 1e-12)
      right = glm::cross(front, std::abs(front.y) < 0.9 ? glm::dvec3(0, 1, 0) : glm::dvec3(1, 0, 0));
  }
  right = glm::normalize(right);
  frame[0] = glm::dvec4(right, 0);
  frame[1] = glm::dvec4(glm::cross(right, front), 0);
  frame[2] = glm::dvec4(-front, 0);
  return frame;
}

StarFollowCameraPose universe_package::CalculateStarFollowCameraPose(const glm::dvec3& position,
                                                                     const glm::dquat& rotation, const double radius) {
  StarFollowCameraPose result{position, rotation, false};
  if (!ValidFollowRadius(radius))
    return result;
  const auto direction =
      glm::dot(position, position) > 0 ? glm::normalize(position) : glm::normalize(rotation * glm::dvec3(0, 0, 1));
  result.position = direction * (radius * 50);
  result.rotation = glm::quat_cast(glm::dmat3(StarReferenceFrame(direction, glm::dmat4(1), glm::mat3_cast(rotation))));
  result.valid = true;
  return result;
}

StarFollowChange StarFollowState::Update(const StarPickSnapshot& selection, const StarClusterBatch& batch,
                                         const bool toggle) {
  available = false;
  selected_radius = 0;
  const auto component = selection.cluster.lock();
  const auto clock = batch.clocks.find(component.get());
  if (selection.result.valid && component && clock != batch.clocks.end() &&
      clock->second.component.lock() == component && clock->second.identity == selection.identity) {
    for (size_t i = 0; i < batch.ranges.size(); ++i) {
      const auto& range = batch.ranges[i];
      if (range.identity != selection.identity || range.seed != selection.seed || selection.ordinal >= range.count)
        continue;
      const auto& p = batch.parameters[i];
      selected_radius = p.tilt_radius.w;
      selected_world_position =
          glm::dvec3(EvaluateStar(p, batch.samples[range.offset + selection.ordinal]).world_position_radius);
      selected_frame = StarReferenceFrame(selected_world_position, glm::dmat4(p.world0, p.world1, p.world2, p.world3),
                                          glm::dmat3(selected_frame));
      available = true;
      break;
    }
  }
  StarFollowChange change;
  const bool valid_radius = ValidFollowRadius(selected_radius);
  const bool same_target = target.identity == selection.identity && target.seed == selection.seed &&
                           target.ordinal == selection.ordinal && target.cluster.lock() == component;
  if (following && available && same_target)
    reference_to_world = selected_frame;
  if (following && (!available || !valid_radius || toggle)) {
    change = {reference_to_world, true, false};
    following = false;
    reference_to_world = glm::dmat4(1);
    ++generation;
  } else if (available && valid_radius && ((toggle && !following) || (following && !same_target))) {
    change = {glm::inverse(selected_frame) * reference_to_world, true, true};
    following = true;
    target = selection;
    reference_to_world = selected_frame;
    ++generation;
  }
  status = !selection.result.valid ? "No star selected"
           : !available            ? "Selected star unavailable"
           : !valid_radius         ? "Cannot follow: finite positive radius with representable camera distance required"
           : following             ? "Following selected star"
                                   : "Selected star ready to follow";
  return change;
}
