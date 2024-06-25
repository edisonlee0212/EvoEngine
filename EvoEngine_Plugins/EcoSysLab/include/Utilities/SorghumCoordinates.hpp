#pragma once
#include "SorghumField.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
class SorghumCoordinates : public IAsset {
  friend class SorghumLayer;

 public:
  AssetRef m_sorghumStateGenerator;
  float m_factor = 1.0f;
  std::vector<glm::dvec2> m_positions;
  glm::vec3 m_rotationVariance = glm::vec3(0.0f);

  glm::dvec2 m_sampleX = glm::dvec2(0.0);
  glm::dvec2 m_sampleY = glm::dvec2(0.0);

  glm::dvec2 m_xRange = glm::vec2(0, 0);
  glm::dvec2 m_yRange = glm::vec2(0, 0);
  void Apply(const std::shared_ptr<SorghumField>& sorghumField);
  void Apply(const std::shared_ptr<SorghumField>& sorghumField, glm::dvec2& offset, unsigned i = 0, float radius = 2.5f,
             float positionVariance = 0.0f);
  void ImportFromFile(const std::filesystem::path& path);
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};

template <typename T>
inline void SaveListAsBinary(const std::string& name, const std::vector<T>& target, YAML::Emitter& out) {
  if (!target.empty()) {
    out << YAML::Key << name << YAML::Value
        << YAML::Binary((const unsigned char*)target.data(), target.size() * sizeof(T));
  }
}
template <typename T>
inline void LoadListFromBinary(const std::string& name, std::vector<T>& target, const YAML::Node& in) {
  if (in[name]) {
    auto binaryList = in[name].as<YAML::Binary>();
    target.resize(binaryList.size() / sizeof(T));
    std::memcpy(target.data(), binaryList.data(), binaryList.size());
  }
}
}  // namespace eco_sys_lab