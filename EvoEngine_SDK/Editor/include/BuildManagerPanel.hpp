#pragma once

#include <array>
#include <atomic>
#include "AssetRef.hpp"
#include "BuildManagerModel.hpp"
#include "EditorPanel.hpp"
#include "EvoEngineAPI.hpp"
#include "EvoEngineEditorAPI.hpp"
#include "RuntimeExportJob.hpp"
namespace evo_engine {
class Scene;
class EVOENGINE_EDITOR_API BuildManagerPanel final : public EditorPanel {
  friend struct BuildManagerPanelTestAccess;

 public:
  BuildManagerPanel();
  ~BuildManagerPanel() override;
  void Draw(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void Tick();
  [[nodiscard]] bool ExportActive() const;

 private:
  void RefreshFromProject();
  BuildPreflightResult Preflight(std::shared_ptr<Scene>& scene, std::filesystem::path& runtime_template) const;
  void StartExport();
  ProjectBuildSettings draft_;
  AssetRef startup_scene_;
  RuntimeExportJob job_;
  std::array<char, 256> application_name_{};
  std::array<char, 1024> output_directory_{};
  std::vector<std::string> errors_;
  std::string status_;
  bool initialized_ = false;
  std::filesystem::path loaded_project_path_;
  std::filesystem::path export_output_directory_;
  std::atomic_bool export_active_ = false;
};
}  // namespace evo_engine
