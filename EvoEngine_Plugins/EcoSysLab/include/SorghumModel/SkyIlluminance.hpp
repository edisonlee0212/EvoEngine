#pragma once

using namespace evo_engine;
namespace eco_sys_lab {
struct SkyIlluminanceSnapshot {
  float m_ghi = 1000;
  float m_azimuth = 0;
  float m_zenith = 0;
  [[nodiscard]] glm::vec3 GetSunDirection();
  [[nodiscard]] float GetSunIntensity();
};
class SkyIlluminance : public IAsset {
 public:
  std::map<float, SkyIlluminanceSnapshot> m_snapshots;
  float m_minTime;
  float m_maxTime;
  [[nodiscard]] SkyIlluminanceSnapshot Get(float time);
  void ImportCsv(const std::filesystem::path &path);
  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;
  void Serialize(YAML::Emitter &out) const override;
  void Deserialize(const YAML::Node &in) override;
};
}  // namespace eco_sys_lab