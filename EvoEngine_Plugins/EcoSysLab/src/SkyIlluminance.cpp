//
// Created by lllll on 2/23/2022.
//
#include "SkyIlluminance.hpp"
#include "rapidcsv.h"
#ifdef BUILD_WITH_RAYTRACER
#  include "RayTracerLayer.hpp"
#endif

using namespace eco_sys_lab;
SkyIlluminanceSnapshot SkyIlluminanceSnapshotLerp(const SkyIlluminanceSnapshot& l, const SkyIlluminanceSnapshot& r,
                                                  float a) {
  if (a < 0.0f)
    return l;
  if (a > 1.0f)
    return r;
  SkyIlluminanceSnapshot snapshot;
  snapshot.m_ghi = l.m_ghi * a + r.m_ghi * (1.0f - a);
  snapshot.m_azimuth = l.m_azimuth * a + r.m_azimuth * (1.0f - a);
  snapshot.m_zenith = l.m_zenith * a + r.m_zenith * (1.0f - a);
  return snapshot;
}

SkyIlluminanceSnapshot SkyIlluminance::Get(float time) {
  if (m_snapshots.empty()) {
    return {};
  }
  if (time <= m_snapshots.begin()->first)
    return m_snapshots.begin()->second;
  SkyIlluminanceSnapshot last_shot = m_snapshots.begin()->second;
  float last_time = m_snapshots.begin()->first;
  for (const auto& pair : m_snapshots) {
    if (time < pair.first) {
      if (pair.first - last_time == 0)
        return last_shot;
      return SkyIlluminanceSnapshotLerp(last_shot, pair.second, (time - last_time) / (pair.first - last_time));
    }
    last_shot = pair.second;
    last_time = pair.first;
  }
  return std::prev(m_snapshots.end())->second;
}
void SkyIlluminance::ImportCsv(const std::filesystem::path& path) {
  rapidcsv::Document doc(path.string());
  const std::vector<float> time_series = doc.GetColumn<float>("Time");
  const std::vector<float> ghi_series = doc.GetColumn<float>("SunLightDensity");
  const std::vector<float> azimuth_series = doc.GetColumn<float>("Azimuth");
  const std::vector<float> zenith_series = doc.GetColumn<float>("Zenith");
  assert(time_series.size() == ghi_series.size() && azimuth_series.size() == zenith_series.size() &&
         time_series.size() == azimuth_series.size());
  m_snapshots.clear();
  m_maxTime = 0;
  m_minTime = 999999;
  for (int i = 0; i < time_series.size(); i++) {
    SkyIlluminanceSnapshot snapshot;
    snapshot.m_ghi = ghi_series[i];
    snapshot.m_azimuth = azimuth_series[i];
    snapshot.m_zenith = zenith_series[i];
    auto time = time_series[i];
    m_snapshots[time] = snapshot;
    if (m_maxTime < time) {
      m_maxTime = time;
    }
    if (m_minTime > time) {
      m_minTime = time;
    }
  }
}
bool SkyIlluminance::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  FileUtils::OpenFile(
      "Import CSV", "CSV", {".csv"},
      [&](const std::filesystem::path& path) {
        ImportCsv(path);
        changed = true;
      },
      false);
  static float time;
  static SkyIlluminanceSnapshot snapshot;
  static bool auto_apply = false;
  ImGui::Checkbox("Auto Apply", &auto_apply);
  if (ImGui::SliderFloat("Time", &time, m_minTime, m_maxTime)) {
    snapshot = Get(time);
#ifdef BUILD_WITH_RAYTRACER
    if (auto_apply) {
      auto& env_prop = Application::GetLayer<RayTracerLayer>()->environment_properties;
      env_prop.m_sunDirection = snapshot.GetSunDirection();
      env_prop.m_skylightIntensity = snapshot.GetSunIntensity();
    }
#endif
  }
  ImGui::Text("Ghi: %.3f", snapshot.m_ghi);
  ImGui::Text("Azimuth: %.3f", snapshot.m_azimuth);
  ImGui::Text("Zenith: %.3f", snapshot.m_zenith);
  return changed;
}
void SkyIlluminance::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "m_minTime" << YAML::Value << m_minTime;
  out << YAML::Key << "m_maxTime" << YAML::Value << m_maxTime;
  if (!m_snapshots.empty()) {
    out << YAML::Key << "m_snapshots" << YAML::Value << YAML::BeginSeq;
    for (const auto& pair : m_snapshots) {
      out << YAML::BeginMap;
      out << YAML::Key << "time" << YAML::Value << pair.first;
      out << YAML::Key << "m_ghi" << YAML::Value << pair.second.m_ghi;
      out << YAML::Key << "m_azimuth" << YAML::Value << pair.second.m_azimuth;
      out << YAML::Key << "m_zenith" << YAML::Value << pair.second.m_zenith;
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}
void SkyIlluminance::Deserialize(const YAML::Node& in) {
  if (in["m_minTime"])
    m_minTime = in["m_minTime"].as<float>();
  if (in["m_maxTime"])
    m_maxTime = in["m_maxTime"].as<float>();
  if (in["m_snapshots"]) {
    m_snapshots.clear();
    for (const auto& data : in["m_snapshots"]) {
      SkyIlluminanceSnapshot snapshot;
      snapshot.m_ghi = data["m_ghi"].as<float>();
      snapshot.m_azimuth = data["m_azimuth"].as<float>();
      snapshot.m_zenith = data["m_zenith"].as<float>();
      m_snapshots[data["time"].as<float>()] = snapshot;
    }
  }
}

glm::vec3 SkyIlluminanceSnapshot::GetSunDirection() {
  return glm::quat(glm::radians(glm::vec3(90.0f - m_zenith, m_azimuth, 0))) * glm::vec3(0, 0, -1);
}
float SkyIlluminanceSnapshot::GetSunIntensity() {
  return m_ghi;
}
