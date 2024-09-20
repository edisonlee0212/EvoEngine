#pragma once
#include "SorghumField.hpp"
using namespace evo_engine;
namespace digital_agriculture_plugin {
class SorghumCoordinates : public IAsset {
  friend class SorghumLayer;

 public:
  AssetRef sorghum_state_generator;
  float factor = 1.0f;
  std::vector<glm::dvec2> positions;
  glm::vec3 rotation_variance = glm::vec3(0.0f);

  glm::dvec2 sample_x = glm::dvec2(0.0);
  glm::dvec2 sample_y = glm::dvec2(0.0);

  glm::dvec2 x_range = glm::vec2(0, 0);
  glm::dvec2 y_range = glm::vec2(0, 0);
  void Apply(const std::shared_ptr<SorghumField>& sorghum_field);
  void Apply(const std::shared_ptr<SorghumField>& sorghum_field, glm::dvec2& offset, unsigned i = 0,
             float radius = 2.5f, float position_variance = 0.0f);
  void ImportFromFile(const std::filesystem::path& path);
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Serialize(YAML::Emitter& out) const override;
  void Deserialize(const YAML::Node& in) override;
  void CollectAssetRef(std::vector<AssetRef>& list) override;
};

template <typename T>
void SaveListAsBinary(const std::string& name, const std::vector<T>& target, YAML::Emitter& out) {
  if (!target.empty()) {
    out << YAML::Key << name << YAML::Value
        << YAML::Binary(static_cast<const unsigned char*>(static_cast<const void*>(target.data())),
                        target.size() * sizeof(T));
  }
}
template <typename T>
void LoadListFromBinary(const std::string& name, std::vector<T>& target, const YAML::Node& in) {
  if (in[name]) {
    const auto binary_list = in[name].as<YAML::Binary>();
    target.resize(binary_list.size() / sizeof(T));
    std::memcpy(target.data(), binary_list.data(), binary_list.size());
  }
}
}  // namespace digital_agriculture_plugin