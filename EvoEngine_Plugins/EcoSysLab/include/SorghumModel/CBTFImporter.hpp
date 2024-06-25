#pragma once
#include <SorghumLayer.hpp>

using namespace evo_engine;

namespace eco_sys_lab {
class CBTFImporter : public IPrivateComponent {
 public:
  bool m_processing = false;
  std::filesystem::path m_currentExportFolder;
  std::vector<std::filesystem::path> m_importFolders;
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Update() override;
};
}  // namespace eco_sys_lab