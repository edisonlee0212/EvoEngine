#include "StarCluster.hpp"

#include "EditorLayer.hpp"
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
  visual_radius = defaults.visual_radius;
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

bool universe_package::InspectStarCluster(InspectorContext&, StarCluster& cluster) {
  bool changed = false;
  changed |= ImGui::InputScalar("Star count", ImGuiDataType_U32, &cluster.star_count_);
  changed |= ImGui::InputScalar("Seed", ImGuiDataType_U64, &cluster.seed);
  changed |= ImGui::Checkbox("Paused", &cluster.paused);
  changed |= ImGui::DragScalar("Time scale", ImGuiDataType_Double, &cluster.time_scale, 0.1f);
  changed |= ImGui::DragScalar("Phase", ImGuiDataType_Double, &cluster.phase, 1.0f);
  changed |= ImGui::DragScalar("Visual radius", ImGuiDataType_Double, &cluster.visual_radius, 0.01f);
  if (ImGui::TreeNode("Density wave")) {
    changed |= ImGui::DragScalar("Disk diameter", ImGuiDataType_Double, &cluster.disk_diameter, 1.0f);
    changed |= ImGui::DragScalar("Disk eccentricity", ImGuiDataType_Double, &cluster.disk_eccentricity, 0.01f);
    changed |= ImGui::DragScalar("Core proportion", ImGuiDataType_Double, &cluster.core_proportion, 0.01f);
    changed |= ImGui::DragScalar("Core eccentricity", ImGuiDataType_Double, &cluster.core_eccentricity, 0.01f);
    changed |= ImGui::DragScalar("Center diameter", ImGuiDataType_Double, &cluster.center_diameter, 1.0f);
    changed |= ImGui::DragScalar("Center eccentricity", ImGuiDataType_Double, &cluster.center_eccentricity, 0.01f);
    changed |= ImGui::DragScalar("Y spread", ImGuiDataType_Double, &cluster.y_spread, 0.001f);
    changed |= ImGui::DragScalar("XZ spread", ImGuiDataType_Double, &cluster.xz_spread, 0.001f);
    changed |= ImGui::DragScalar("Twist", ImGuiDataType_Double, &cluster.twist, 1.0f);
    changed |= ImGui::DragScalarN("Center offset", ImGuiDataType_Double, &cluster.center_offset.x, 3, 1.0f);
    changed |= ImGui::DragScalarN("Center position", ImGuiDataType_Double, &cluster.center_position.x, 3, 1.0f);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Movement")) {
    changed |= ImGui::DragScalar("Disk speed", ImGuiDataType_Double, &cluster.disk_speed, 0.1f);
    changed |= ImGui::DragScalar("Core speed", ImGuiDataType_Double, &cluster.core_speed, 0.1f);
    changed |= ImGui::DragScalar("Center speed", ImGuiDataType_Double, &cluster.center_speed, 0.1f);
    changed |= ImGui::DragScalar("Disk X tilt", ImGuiDataType_Double, &cluster.disk_tilt_x, 1.0f);
    changed |= ImGui::DragScalar("Disk Z tilt", ImGuiDataType_Double, &cluster.disk_tilt_z, 1.0f);
    changed |= ImGui::DragScalar("Core X tilt", ImGuiDataType_Double, &cluster.core_tilt_x, 1.0f);
    changed |= ImGui::DragScalar("Core Z tilt", ImGuiDataType_Double, &cluster.core_tilt_z, 1.0f);
    changed |= ImGui::DragScalar("Center X tilt", ImGuiDataType_Double, &cluster.center_tilt_x, 1.0f);
    changed |= ImGui::DragScalar("Center Z tilt", ImGuiDataType_Double, &cluster.center_tilt_z, 1.0f);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Colors and emission")) {
    changed |= ImGui::ColorEdit3("Disk color", &cluster.disk_color.x);
    changed |= ImGui::ColorEdit3("Core color", &cluster.core_color.x);
    changed |= ImGui::ColorEdit3("Center color", &cluster.center_color.x);
    changed |= ImGui::DragFloat("Disk emission", &cluster.disk_emission_intensity, 0.01f);
    changed |= ImGui::DragFloat("Core emission", &cluster.core_emission_intensity, 0.01f);
    changed |= ImGui::DragFloat("Center emission", &cluster.center_emission_intensity, 0.01f);
    ImGui::TreePop();
  }
  ImGui::TextUnformatted("Simulation and rendering are managed by Universe Layer.");
  return changed;
}

void universe_package::SerializeStarCluster(YAML::Emitter& out, const StarCluster& cluster) {
  out << YAML::Key << "star_count" << YAML::Value << cluster.star_count_;
#define WRITE_VALUE(name) out << YAML::Key << #name << YAML::Value << cluster.name
  WRITE_VALUE(seed);
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
  WRITE_VALUE(visual_radius);
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
  READ_VALUE(visual_radius);
  READ_VALUE(time_scale);
  READ_VALUE(phase);
  READ_VALUE(paused);
#undef READ_VALUE
}
