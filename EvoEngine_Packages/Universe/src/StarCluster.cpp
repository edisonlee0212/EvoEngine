#include "StarCluster.hpp"

#include "Serialization.hpp"

using namespace universe_package;

namespace {
template <typename T>
void Read(const YAML::Node& in, const char* name, T& value) {
  if (in[name])
    value = in[name].as<T>();
}
}  // namespace

void StarCluster::SetStarCount(const uint32_t count) {
  star_count_ = count;
}

uint32_t StarCluster::GetStarCount() const {
  return star_count_;
}

void StarCluster::ResetAuthoringState() {
  const StarCluster defaults;
  seed = defaults.seed;
  star_minimum_distance = defaults.star_minimum_distance;
  y_spread = defaults.y_spread;
  xz_spread = defaults.xz_spread;
  disk_diameter = defaults.disk_diameter;
  disk_eccentricity = defaults.disk_eccentricity;
  core_proportion = defaults.core_proportion;
  core_eccentricity = defaults.core_eccentricity;
  center_diameter = defaults.center_diameter;
  center_eccentricity = defaults.center_eccentricity;
  disk_speed = defaults.disk_speed;
  core_speed = defaults.core_speed;
  center_speed = defaults.center_speed;
  disk_tilt_x = defaults.disk_tilt_x;
  disk_tilt_z = defaults.disk_tilt_z;
  core_tilt_x = defaults.core_tilt_x;
  core_tilt_z = defaults.core_tilt_z;
  center_tilt_x = defaults.center_tilt_x;
  center_tilt_z = defaults.center_tilt_z;
  twist = defaults.twist;
  center_offset = defaults.center_offset;
  center_position = defaults.center_position;
  disk_color = defaults.disk_color;
  core_color = defaults.core_color;
  center_color = defaults.center_color;
  disk_emission_intensity = defaults.disk_emission_intensity;
  core_emission_intensity = defaults.core_emission_intensity;
  center_emission_intensity = defaults.center_emission_intensity;
  alpha = defaults.alpha;
  radius_min = defaults.radius_min;
  radius_max = defaults.radius_max;
  radius_deviation = defaults.radius_deviation;
  time_scale = defaults.time_scale;
  phase = defaults.phase;
  paused = defaults.paused;
  star_count_ = 0;
}

void StarCluster::OnCreate() {
  ResetAuthoringState();
}

void StarCluster::OnDestroy() {
  ResetAuthoringState();
}

void universe_package::SerializeStarCluster(YAML::Emitter& out, const StarCluster& cluster) {
  out << YAML::Key << "star_count" << YAML::Value << cluster.star_count_;
#define WRITE_VALUE(name) out << YAML::Key << #name << YAML::Value << cluster.name
  WRITE_VALUE(seed);
  WRITE_VALUE(star_minimum_distance);
  WRITE_VALUE(y_spread);
  WRITE_VALUE(xz_spread);
  WRITE_VALUE(disk_diameter);
  WRITE_VALUE(disk_eccentricity);
  WRITE_VALUE(core_proportion);
  WRITE_VALUE(core_eccentricity);
  WRITE_VALUE(center_diameter);
  WRITE_VALUE(center_eccentricity);
  WRITE_VALUE(disk_speed);
  WRITE_VALUE(core_speed);
  WRITE_VALUE(center_speed);
  WRITE_VALUE(disk_tilt_x);
  WRITE_VALUE(disk_tilt_z);
  WRITE_VALUE(core_tilt_x);
  WRITE_VALUE(core_tilt_z);
  WRITE_VALUE(center_tilt_x);
  WRITE_VALUE(center_tilt_z);
  WRITE_VALUE(twist);
  WRITE_VALUE(center_offset);
  WRITE_VALUE(center_position);
  WRITE_VALUE(disk_color);
  WRITE_VALUE(core_color);
  WRITE_VALUE(center_color);
  WRITE_VALUE(disk_emission_intensity);
  WRITE_VALUE(core_emission_intensity);
  WRITE_VALUE(center_emission_intensity);
  WRITE_VALUE(alpha);
  WRITE_VALUE(radius_min);
  WRITE_VALUE(radius_max);
  WRITE_VALUE(radius_deviation);
  WRITE_VALUE(time_scale);
  WRITE_VALUE(phase);
  WRITE_VALUE(paused);
#undef WRITE_VALUE
}

void universe_package::DeserializeStarCluster(const YAML::Node& in, StarCluster& cluster) {
  cluster.ResetAuthoringState();
  if (in["star_count"]) {
    cluster.star_count_ = in["star_count"].as<uint32_t>();
  } else {
    std::vector<uint64_t> legacy_ids;
    Serialization::DeserializeVector("star_ids", legacy_ids, in);
    cluster.star_count_ = static_cast<uint32_t>(legacy_ids.size());
  }
#define READ_VALUE(name) Read(in, #name, cluster.name)
  READ_VALUE(seed);
  READ_VALUE(star_minimum_distance);
  READ_VALUE(y_spread);
  READ_VALUE(xz_spread);
  READ_VALUE(disk_diameter);
  READ_VALUE(disk_eccentricity);
  READ_VALUE(core_proportion);
  READ_VALUE(core_eccentricity);
  READ_VALUE(center_diameter);
  READ_VALUE(center_eccentricity);
  READ_VALUE(disk_speed);
  READ_VALUE(core_speed);
  READ_VALUE(center_speed);
  READ_VALUE(disk_tilt_x);
  READ_VALUE(disk_tilt_z);
  READ_VALUE(core_tilt_x);
  READ_VALUE(core_tilt_z);
  READ_VALUE(center_tilt_x);
  READ_VALUE(center_tilt_z);
  READ_VALUE(twist);
  READ_VALUE(center_offset);
  READ_VALUE(center_position);
  READ_VALUE(disk_color);
  READ_VALUE(core_color);
  READ_VALUE(center_color);
  READ_VALUE(disk_emission_intensity);
  READ_VALUE(core_emission_intensity);
  READ_VALUE(center_emission_intensity);
  READ_VALUE(alpha);
  READ_VALUE(radius_min);
  READ_VALUE(radius_max);
  if (in["radius_deviation"])
    cluster.radius_deviation = in["radius_deviation"].as<double>();
  else if (in["radius_standard_deviation"]) {
    const double range = cluster.radius_max - cluster.radius_min;
    cluster.radius_deviation = range > 0 ? in["radius_standard_deviation"].as<double>() / range : 0;
  }
  READ_VALUE(time_scale);
  READ_VALUE(phase);
  READ_VALUE(paused);
#undef READ_VALUE
}
