#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "AssetManager.hpp"
#include "BuildManagerPanel.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>

using namespace evo_engine;
namespace evo_engine {
struct BuildManagerPanelTestAccess {
  static void Refresh(BuildManagerPanel& panel) {
    panel.RefreshFromProject();
  }
  static void Build(BuildManagerPanel& panel) {
    panel.StartExport();
  }
};
}  // namespace evo_engine
namespace {
class TempBuildProject {
 public:
  TempBuildProject() {
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineBuildStateTest_" + std::to_string(nonce));
    std::filesystem::create_directories(root_ / "Assets");
  }
  ~TempBuildProject() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }
  [[nodiscard]] std::filesystem::path ProjectPath() const {
    return root_ / "BuildState.eveproj";
  }

 private:
  std::filesystem::path root_;
};
ApplicationInitializationSettings Settings(const TempBuildProject& project) {
  ApplicationInitializationSettings settings;
  settings.project_path = project.ProjectPath();
  settings.load_default_resources = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  settings.redirect_standard_streams_to_console = false;
  return settings;
}
}  // namespace

TEST(ProjectManagerBuildState, TracksStartupSceneAndBuildSettingsAgainstLastSuccessfulSave) {
  TempBuildProject project;
  Application application;
  ApplicationContextScope scope(application);
  application.Initialize(Settings(project));

  EXPECT_TRUE(ProjectManager::ProjectMetadataSaved());
  const auto first_scene = std::make_shared<Scene>();
  ProjectManager::SetStartScene(first_scene);
  EXPECT_FALSE(ProjectManager::ProjectMetadataSaved());

  ProjectManager::SaveProject();
  ASSERT_TRUE(ProjectManager::ProjectMetadataSaved());
  EXPECT_EQ(YAML::LoadFile(project.ProjectPath().string())["start_scene_handle"].as<uint64_t>(),
            first_scene->GetHandle().GetValue());

  const auto second_scene = std::make_shared<Scene>();
  ProjectManager::SetStartScene(second_scene);
  EXPECT_FALSE(ProjectManager::ProjectMetadataSaved());
  ProjectManager::SetStartScene(first_scene);
  EXPECT_TRUE(ProjectManager::ProjectMetadataSaved());

  auto build_settings = ProjectManager::GetBuildSettings();
  build_settings.application_name = "Changed Runtime App";
  ProjectManager::SetBuildSettings(build_settings);
  EXPECT_FALSE(ProjectManager::ProjectMetadataSaved());
  ProjectManager::SaveProject();
  EXPECT_TRUE(ProjectManager::ProjectMetadataSaved());
}

TEST(ProjectManagerBuildState, NullStartupScenePersistsAndFailedWriteRemainsDirty) {
  TempBuildProject project;
  Application application;
  ApplicationContextScope scope(application);
  application.Initialize(Settings(project));

  ProjectManager::SetStartScene(std::make_shared<Scene>());
  ProjectManager::SaveProject();
  ASSERT_TRUE(ProjectManager::ProjectMetadataSaved());

  ProjectManager::SetStartScene(nullptr);
  EXPECT_FALSE(ProjectManager::ProjectMetadataSaved());
  ProjectManager::SaveProject();
  ASSERT_TRUE(ProjectManager::ProjectMetadataSaved());
  EXPECT_FALSE(YAML::LoadFile(project.ProjectPath().string())["start_scene_handle"]);

  auto build_settings = ProjectManager::GetBuildSettings();
  build_settings.application_name = "Must Remain Dirty";
  ProjectManager::SetBuildSettings(build_settings);
  ASSERT_FALSE(ProjectManager::ProjectMetadataSaved());
  std::filesystem::remove(project.ProjectPath());
  std::filesystem::create_directory(project.ProjectPath());
  ProjectManager::SaveProject();
  EXPECT_FALSE(ProjectManager::ProjectMetadataSaved());
}

TEST(ProjectManagerBuildState, UnloadedStartupSceneRemainsSavedAndIsPreserved) {
  TempBuildProject project;
  {
    std::ofstream stream(project.ProjectPath());
    stream << "start_scene_handle: 42\n";
  }
  Application application;
  ApplicationContextScope scope(application);
  application.Initialize(Settings(project));

  EXPECT_FALSE(ProjectManager::GetStartScene().lock());
  EXPECT_TRUE(ProjectManager::ProjectMetadataSaved());
  ProjectManager::SaveProject();
  EXPECT_TRUE(ProjectManager::ProjectMetadataSaved());
  EXPECT_EQ(YAML::LoadFile(project.ProjectPath().string())["start_scene_handle"].as<uint64_t>(), 42);
}

TEST(ProjectManagerBuildState, BuildDefaultsToActiveSceneAndSavesSettingsBeforeValidation) {
  TempBuildProject project;
  Application application;
  ApplicationContextScope scope(application);
  application.Initialize(Settings(project));
  const auto project_start = AssetManager::CreateTemporaryAsset<Scene>();
  ProjectManager::SetStartScene(project_start);
  for (int i = 0;
       i < 100 && (ProjectManager::GetProjectState() != ProjectState::Loaded || !ProjectManager::IsProjectIdle()); ++i)
    application.Loop();
  ASSERT_EQ(ProjectManager::GetProjectState(), ProjectState::Loaded);
  ASSERT_TRUE(ProjectManager::IsProjectIdle());
  const auto active = AssetManager::CreateTemporaryAsset<Scene>();
  application.Attach(active);
  BuildManagerPanel panel;
  BuildManagerPanelTestAccess::Refresh(panel);
  EXPECT_EQ(ProjectManager::GetBuildSettings().startup_scene_handle, active->GetHandle().GetValue());
  EXPECT_FALSE(ProjectManager::ProjectMetadataSaved());
  BuildManagerPanelTestAccess::Build(panel);
  EXPECT_TRUE(ProjectManager::ProjectMetadataSaved());
  EXPECT_FALSE(panel.ExportActive());
  const auto saved = YAML::LoadFile(project.ProjectPath().string());
  EXPECT_EQ(saved["build_settings"]["startup_scene_handle"].as<uint64_t>(), active->GetHandle().GetValue());
  EXPECT_EQ(saved["start_scene_handle"].as<uint64_t>(), project_start->GetHandle().GetValue());
}

TEST(ProjectManagerBuildState, BuildManagerPreservesExplicitStartupSceneSelection) {
  TempBuildProject project;
  Application application;
  ApplicationContextScope scope(application);
  application.Initialize(Settings(project));
  application.Attach(AssetManager::CreateTemporaryAsset<Scene>());
  const auto selected = AssetManager::CreateTemporaryAsset<Scene>();
  auto settings = ProjectManager::GetBuildSettings();
  settings.startup_scene_handle = selected->GetHandle().GetValue();
  ProjectManager::SetBuildSettings(settings);
  BuildManagerPanel panel;
  BuildManagerPanelTestAccess::Refresh(panel);
  EXPECT_EQ(ProjectManager::GetBuildSettings().startup_scene_handle, selected->GetHandle().GetValue());
}

TEST(ProjectManagerBuildState, BuildManagerWaitsForStoppedSceneBeforeSelectingDefault) {
  TempBuildProject project;
  Application application;
  ApplicationContextScope scope(application);
  application.Initialize(Settings(project));
  const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
  ProjectManager::SetStartScene(scene);
  application.Attach(scene);
  BuildManagerPanel panel;
  for (int i = 0;
       i < 100 && (ProjectManager::GetProjectState() != ProjectState::Loaded || !ProjectManager::IsProjectIdle()); ++i)
    application.Loop();
  ASSERT_EQ(ProjectManager::GetProjectState(), ProjectState::Loaded);
  ASSERT_TRUE(ProjectManager::IsProjectIdle());
  application.Play();
  ASSERT_EQ(application.GetApplicationStatus(), Application::ExecutionStatus::Playing);
  ASSERT_NE(application.GetActiveScene(), scene);
  BuildManagerPanelTestAccess::Refresh(panel);
  EXPECT_EQ(ProjectManager::GetBuildSettings().startup_scene_handle, 0);
  application.Pause();
  BuildManagerPanelTestAccess::Refresh(panel);
  EXPECT_EQ(ProjectManager::GetBuildSettings().startup_scene_handle, 0);
  application.Stop();
  BuildManagerPanelTestAccess::Refresh(panel);
  EXPECT_EQ(ProjectManager::GetBuildSettings().startup_scene_handle, scene->GetHandle().GetValue());
}
