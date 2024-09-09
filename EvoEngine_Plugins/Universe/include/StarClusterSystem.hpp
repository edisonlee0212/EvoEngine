#pragma once
#include <Application.hpp>
#include "Scene.hpp"
using namespace evo_engine;
namespace universe_plugin {
/// <summary>
/// The calculated precise position of the star.
/// </summary>
struct StarPosition : IDataComponent {
  glm::dvec3 value;
};
struct SelectionStatus : IDataComponent {
  int value;
};
/// <summary>
/// The seed of the star, use this to calculate initial position.
/// </summary>
struct StarInfo : IDataComponent {
  bool initialized = false;
};

/// <summary>
/// Original color of the star
/// </summary>
struct OriginalColor : IDataComponent {
  glm::vec3 value;
};
/// <summary>
/// The deviation of its orbit
/// </summary>
struct StarOrbitOffset : IDataComponent {
  glm::dvec3 value;
};
/// <summary>
/// This will help calculate the orbit. Smaller = close to center, bigger = close to disk
/// </summary>
struct StarOrbitProportion : IDataComponent {
  double value;
};
/// <summary>
/// This will help calculate the orbit. Smaller = close to center, bigger = close to disk
/// </summary>
struct SurfaceColor : IDataComponent {
  glm::vec3 value;
  float intensity = 1.0f;
};
/// <summary>
/// The actual display color after selection system.
/// </summary>
struct DisplayColor : IDataComponent {
  glm::vec3 value;
  float intensity = 1.0f;
};

struct StarOrbit : IDataComponent {
  double a;
  double b;
  double speed_multiplier;

  double tilt_x;
  double tilt_y;
  double tilt_z;

  glm::dvec3 m_center;
  [[nodiscard]] glm::dvec3 GetPoint(const glm::dvec3 &orbit_offset, const double &time,
                                    const bool &is_star = true) const {
    const double angle = is_star ? time / glm::sqrt(a + b) * speed_multiplier : time;

    glm::dvec3 point{glm::sin(glm::radians(angle)) * a, 0, glm::cos(glm::radians(angle)) * b};

    point = Rotate(glm::angleAxis(glm::radians(tilt_x), glm::dvec3(1, 0, 0)), point);
    point = Rotate(glm::angleAxis(glm::radians(tilt_y), glm::dvec3(0, 1, 0)), point);
    point = Rotate(glm::angleAxis(glm::radians(tilt_z), glm::dvec3(0, 0, 1)), point);

    point += m_center;
    point += orbit_offset;
    return point;
  }
  static glm::dvec3 Rotate(const glm::qua<double> &rotation, const glm::dvec3 &point) {
    const double x = rotation.x * 2.0;
    const double y = rotation.y * 2.0;
    const double z = rotation.z * 2.0;
    const double xx = rotation.x * x;
    const double yy = rotation.y * y;
    const double zz = rotation.z * z;
    const double xy = rotation.x * y;
    const double xz = rotation.x * z;
    const double yz = rotation.y * z;
    const double wx = rotation.w * x;
    const double wy = rotation.w * y;
    const double wz = rotation.w * z;
    glm::dvec3 res;
    res.x = (1.0 - (yy + zz)) * point.x + (xy - wz) * point.y + (xz + wy) * point.z;
    res.y = (1.0 - (xx + zz)) * point.y + (yz - wx) * point.z + (xy + wz) * point.x;
    res.z = (1.0 - (xx + yy)) * point.z + (xz - wy) * point.x + (yz + wx) * point.y;
    return res;
  }
};
/// <summary>
/// The star cluster it actually belongs to.
/// </summary>
struct StarClusterIndex : IDataComponent {
  int m_value = 0;
};

class StarClusterPattern {
  double disk_a_ = 0;
  double disk_b_ = 0;
  double core_a_ = 0;
  double core_b_ = 0;
  double center_a_ = 0;
  double center_b_ = 0;
  double core_diameter_ = 0;

 public:
  std::string name = "Cluster Pattern";
  StarClusterIndex star_cluster_index;
  void OnInspect();
  double y_spread = 0.05;
  double xz_spread = 0.015;

  double disk_diameter = 3000;
  double disk_eccentricity = 0.5;

  double core_proportion = 0.4;
  double core_eccentricity = 0.7;

  double center_diameter = 10;
  double center_eccentricity = 0.3;

  double disk_speed = 1;
  double core_speed = 5;
  double center_speed = 10;

  double disk_tilt_x = 0;
  double disk_tilt_z = 0;
  double core_tilt_x = 0;
  double core_tilt_z = 0;
  double center_tilt_x = 0;
  double center_tilt_z = 0;

  float disk_emission_intensity = 3.0f;
  float core_emission_intensity = 2.0f;
  float center_emission_intensity = 1.0f;
  glm::vec3 disk_color = glm::vec3(0, 0, 1);
  glm::vec3 core_color = glm::vec3(1, 1, 0);
  glm::vec3 center_color = glm::vec3(1, 1, 1);

  double twist = 360;
  glm::dvec3 center_offset = glm::dvec3(0);
  glm::dvec3 center_position = glm::dvec3(0);

  void Apply(const bool &force_update_all_stars = false, const bool &only_update_colors = false);

  void SetAb() {
    disk_a_ = disk_diameter * disk_eccentricity;
    disk_b_ = disk_diameter * (1 - disk_eccentricity);
    center_a_ = center_diameter * center_eccentricity;
    center_b_ = center_diameter * (1 - center_eccentricity);
    core_diameter_ = center_diameter / 2 + center_diameter / 2 +
                     (disk_a_ + disk_b_ - center_diameter / 2 - center_diameter / 2) * core_proportion;
    core_a_ = core_diameter_ * core_eccentricity;
    core_b_ = core_diameter_ * (1 - core_eccentricity);
  }

  /// <summary>
  /// Set the ellipse by the proportion.
  /// </summary>
  /// <param name="star_orbit_proportion">
  /// The position of the ellipse in the density waves, range is from 0 to 1
  /// </param>
  /// <param name="orbit">
  /// The ellipse will be reset by the proportion and the density wave properties.
  /// </param>
  [[nodiscard]] StarOrbit GetOrbit(const double &star_orbit_proportion) const {
    StarOrbit orbit;
    if (star_orbit_proportion > core_proportion) {
      // If the wave is outside the disk;
      const double actual_proportion = (star_orbit_proportion - core_proportion) / (1 - core_proportion);
      orbit.a = core_a_ + (disk_a_ - core_a_) * actual_proportion;
      orbit.b = core_b_ + (disk_b_ - core_b_) * actual_proportion;
      orbit.tilt_x = core_tilt_x - (core_tilt_x - disk_tilt_x) * actual_proportion;
      orbit.tilt_z = core_tilt_z - (core_tilt_z - disk_tilt_z) * actual_proportion;
      orbit.speed_multiplier = core_speed + (disk_speed - core_speed) * actual_proportion;
    } else {
      const double actual_proportion = star_orbit_proportion / core_proportion;
      orbit.a = center_a_ + (core_a_ - center_a_) * actual_proportion;
      orbit.b = center_b_ + (core_b_ - center_b_) * actual_proportion;
      orbit.tilt_x = center_tilt_x - (center_tilt_x - core_tilt_x) * actual_proportion;
      orbit.tilt_z = center_tilt_z - (center_tilt_z - core_tilt_z) * actual_proportion;
      orbit.speed_multiplier = center_speed + (core_speed - center_speed) * actual_proportion;
    }
    orbit.tilt_y = -twist * star_orbit_proportion;
    orbit.m_center = center_offset * (1 - star_orbit_proportion) + center_position;
    return orbit;
  }

  [[nodiscard]] StarOrbitOffset GetOrbitOffset(const double &proportion) const {
    double offset = glm::sqrt(1 - proportion);
    StarOrbitOffset orbit_offset;
    glm::dvec3 d3;
    d3.y = glm::gaussRand(0.0, 1.0) * (disk_a_ + disk_b_) * y_spread;
    d3.x = glm::gaussRand(0.0, 1.0) * (disk_a_ + disk_b_) * xz_spread;
    d3.z = glm::gaussRand(0.0, 1.0) * (disk_a_ + disk_b_) * xz_spread;
    orbit_offset.value = d3;
    return orbit_offset;
  }

  [[nodiscard]] glm::vec3 GetColor(const double &proportion) const {
    glm::vec3 color = glm::vec3();
    if (proportion > core_proportion) {
      // If the wave is outside the disk;
      const double actual_proportion = (proportion - core_proportion) / (1 - core_proportion);
      color =
          core_color * (1 - static_cast<float>(actual_proportion)) + disk_color * static_cast<float>(actual_proportion);
    } else {
      const double actual_proportion = proportion / core_proportion;
      color =
          core_color * static_cast<float>(actual_proportion) + center_color * (1 - static_cast<float>(actual_proportion));
    }
    return color;
  }

  [[nodiscard]] float GetIntensity(const double &proportion) const {
    float intensity = 1.0f;
    if (proportion > core_proportion) {
      // If the wave is outside the disk;
      const double actual_proportion = (proportion - core_proportion) / (1 - core_proportion);
      intensity = core_emission_intensity * (1 - static_cast<float>(actual_proportion)) +
                  disk_emission_intensity * static_cast<float>(actual_proportion);
    } else {
      const double actual_proportion = proportion / core_proportion;
      intensity = core_emission_intensity * static_cast<float>(actual_proportion) +
                  center_emission_intensity * (1 - static_cast<float>(actual_proportion));
    }
    return intensity;
  }
};

class StarClusterSystem : public ISystem {
  EntityRef renderer_front_;
  EntityRef renderer_back_;
  EntityQuery star_query_;
  EntityArchetype star_archetype_;
  std::vector<StarClusterPattern> star_cluster_patterns_;
  bool use_front_ = true;
  int counter_ = 0;

  float apply_position_timer_ = 0;
  float copy_position_timer_ = 0;
  float calc_position_timer_ = 0;
  float calc_position_result_ = 0;
  float speed_ = 0.0f;
  float size_ = 0.05f;
  float galaxy_time_ = 0.0;
  bool first_time_ = true;

 public:
  void Serialize(YAML::Emitter &out) const override;
  void Deserialize(const YAML::Node &in) override;
  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;

  void CalculateStarPositionSync();
  void ApplyPosition();
  void CopyPosition(const bool &reverse = false);
  void OnCreate() override;
  void Start() override;
  void Update() override;
  void PushStars(StarClusterPattern &pattern, const size_t &amount = 10000);
  void RandomlyRemoveStars(const size_t &amount = 10000);
  void ClearAllStars();
  void FixedUpdate() override;
  void OnEnable() override;
};
}  // namespace universe_plugin
