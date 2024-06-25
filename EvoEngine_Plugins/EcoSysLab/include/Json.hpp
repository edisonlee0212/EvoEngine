#pragma once
#include "json.h"
using namespace evo_engine;
namespace eco_sys_lab {
class Json : public IAsset {
 protected:
  bool SaveInternal(const std::filesystem::path& path) const override;
  bool LoadInternal(const std::filesystem::path& path) override;

 public:
  treeio::json m_json;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;
};
}  // namespace eco_sys_lab