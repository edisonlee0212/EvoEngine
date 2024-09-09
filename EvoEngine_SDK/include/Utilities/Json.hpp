#pragma once
#include "IAsset.hpp"
#include "nlohmann/json.hpp"
namespace evo_engine {
class Json : public IAsset {
 protected:
  bool SaveInternal(const std::filesystem::path& path) const override;
  bool LoadInternal(const std::filesystem::path& path) override;

 public:
  nlohmann::json m_json;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
};
}  // namespace eco_sys_lab_plugin