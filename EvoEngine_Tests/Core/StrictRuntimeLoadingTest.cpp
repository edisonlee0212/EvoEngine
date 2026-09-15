#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"

#include <chrono>
#include <fstream>

using namespace evo_engine;

namespace {
constexpr uint64_t kSceneHandle = 0xE703'0000'0000'0001ull;
constexpr uint64_t kCameraEntityHandle = 0xE703'0000'0000'0002ull;
constexpr uint64_t kDormantAssetHandle = 0xE703'0000'0000'0003ull;
constexpr uint64_t kGeneratedTextureHandle = 0xE703'0000'0000'0004ull;

class StrictRuntimeProject {
 public:
  StrictRuntimeProject() {
    root_ =
        std::filesystem::temp_directory_path() /
        ("EvoEngineStrictRuntimeTest_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  }

  ~StrictRuntimeProject() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path ProjectPath() const {
    return root_ / "Runtime.eveproj";
  }

  [[nodiscard]] std::filesystem::path AssetsPath() const {
    return root_ / "Assets";
  }

  void CreateDirectories() const {
    std::filesystem::create_directories(AssetsPath());
  }

  void WriteProject(const uint64_t scene_handle = kSceneHandle) const {
    CreateDirectories();
    std::ofstream(ProjectPath()) << "start_scene_handle: " << scene_handle << "\n";
  }

  void WriteAsset(const std::string& type_name, const std::string& contents) const {
    WriteAsset("Startup.evescene", kSceneHandle, type_name, contents);
  }

  void WriteAsset(const std::string& filename, const uint64_t handle, const std::string& type_name,
                  const std::string& contents) const {
    const auto path = AssetsPath() / filename;
    std::ofstream(path) << contents;
    std::ofstream metadata(path.string() + ".evefilemeta");
    metadata << "asset_extension_: " << path.extension().string() << "\n";
    metadata << "asset_file_name_: " << path.stem().string() << "\n";
    metadata << "asset_type_name_: " << type_name << "\n";
    metadata << "asset_handle_: " << handle << "\n";
  }

 private:
  std::filesystem::path root_;
};

ApplicationInitializationSettings StrictSettings(const StrictRuntimeProject& project) {
  ApplicationInitializationSettings settings;
  settings.project_path = project.ProjectPath();
  settings.strict_runtime = true;
  settings.load_default_resources = false;
  settings.load_project_assets = false;
  settings.load_project_start_scene = true;
  settings.enable_runtime_packages = false;
  return settings;
}

void InitializeStrict(Application& application, const StrictRuntimeProject& project) {
  application.Initialize(StrictSettings(project));
}

enum class CameraState { Enabled, Missing, DisabledComponent, DisabledOwner };

std::string RuntimeScene(const bool missing_reference = false, const CameraState camera_state = CameraState::Enabled) {
  const auto main_camera_handle = camera_state == CameraState::Missing ? 0 : kCameraEntityHandle;
  const auto main_camera_type = camera_state == CameraState::Missing ? "" : "Camera";
  const bool owner_enabled = camera_state != CameraState::DisabledOwner;
  const bool camera_enabled = camera_state != CameraState::DisabledComponent;
  std::string scene =
      "main_camera:\n"
      "  entity_handle_: " +
      std::to_string(main_camera_handle) +
      "\n"
      "  private_component_type_name_: " +
      main_camera_type +
      "\n"
      "entity_metadata_list:\n"
      "  - n: Main Camera\n"
      "    h: " +
      std::to_string(kCameraEntityHandle) +
      "\n"
      "    e: " +
      (owner_enabled ? "true" : "false") +
      "\n"
      "    s: false\n"
      "    r: " +
      std::to_string(kCameraEntityHandle) +
      "\n"
      "    pc:\n"
      "      - tn: Camera\n"
      "        e: " +
      (camera_enabled ? "true" : "false") + "\n";
  if (missing_reference) {
    scene +=
        "        post_processing_stack_ref:\n"
        "          asset_handle_: 999999\n"
        "          type_name_: PostProcessingStack\n";
  }
  return scene +
         "systems_: []\n"
         "data_component_storage_list: []\n";
}
}  // namespace

TEST(StrictRuntimeLoading, MissingProjectFailsWithoutCreatingContent) {
  StrictRuntimeProject project;
  Application application;
  ApplicationContextScope scope(application);

  InitializeStrict(application, project);

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Failed);
  EXPECT_NE(ProjectManager::GetProjectFailure().find("does not exist"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(project.ProjectPath()));
  EXPECT_FALSE(std::filesystem::exists(project.AssetsPath()));
}

TEST(StrictRuntimeLoading, MissingMetadataFailsWithoutRepair) {
  StrictRuntimeProject project;
  project.WriteProject();
  const auto asset = project.AssetsPath() / "Startup.evescene";
  std::ofstream(asset) << "untracked";
  Application application;
  ApplicationContextScope scope(application);

  InitializeStrict(application, project);

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Failed);
  EXPECT_NE(ProjectManager::GetProjectFailure().find("Missing asset metadata"), std::string::npos);
  EXPECT_FALSE(std::filesystem::exists(asset.string() + ".evefilemeta"));
  EXPECT_EQ(std::filesystem::file_size(asset), std::string("untracked").size());
}

TEST(StrictRuntimeLoading, UnknownRequiredStartupTypeFails) {
  StrictRuntimeProject project;
  project.WriteProject();
  project.WriteAsset("UnavailableSceneProvider", "provider payload\n");
  Application application;
  ApplicationContextScope scope(application);

  InitializeStrict(application, project);

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Failed);
  EXPECT_NE(ProjectManager::GetProjectFailure().find("startup scene"), std::string::npos);
}

TEST(StrictRuntimeLoading, MissingRequiredSceneReferenceFailsBeforeAttach) {
  StrictRuntimeProject project;
  project.WriteProject();
  project.WriteAsset("Scene", RuntimeScene(true));
  Application application;
  ApplicationContextScope scope(application);

  InitializeStrict(application, project);

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Failed);
  EXPECT_NE(ProjectManager::GetProjectFailure().find("startup scene"), std::string::npos);
  EXPECT_FALSE(application.GetActiveScene());
}

TEST(StrictRuntimeLoading, MissingMainCameraFailsBeforeAttach) {
  StrictRuntimeProject project;
  project.WriteProject();
  project.WriteAsset("Scene", RuntimeScene(false, CameraState::Missing));
  Application application;
  ApplicationContextScope scope(application);

  InitializeStrict(application, project);

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Failed);
  EXPECT_NE(ProjectManager::GetProjectFailure().find("startup scene"), std::string::npos);
  EXPECT_FALSE(application.GetActiveScene());
}

TEST(StrictRuntimeLoading, DisabledMainCameraFailsBeforeAttach) {
  StrictRuntimeProject project;
  project.WriteProject();
  project.WriteAsset("Scene", RuntimeScene(false, CameraState::DisabledComponent));
  Application application;
  ApplicationContextScope scope(application);

  InitializeStrict(application, project);

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Failed);
  EXPECT_NE(ProjectManager::GetProjectFailure().find("startup scene"), std::string::npos);
  EXPECT_FALSE(application.GetActiveScene());
}

TEST(StrictRuntimeLoading, DisabledMainCameraOwnerFailsBeforeAttach) {
  StrictRuntimeProject project;
  project.WriteProject();
  project.WriteAsset("Scene", RuntimeScene(false, CameraState::DisabledOwner));
  Application application;
  ApplicationContextScope scope(application);

  InitializeStrict(application, project);

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Failed);
  EXPECT_NE(ProjectManager::GetProjectFailure().find("startup scene"), std::string::npos);
  EXPECT_FALSE(application.GetActiveScene());
}

TEST(StrictRuntimeLoading, ValidStartupSceneLoads) {
  StrictRuntimeProject project;
  project.WriteProject();
  project.WriteAsset("Scene", RuntimeScene());
  Application application;
  ApplicationContextScope scope(application);

  InitializeStrict(application, project);

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Loaded);
  EXPECT_TRUE(ProjectManager::GetProjectFailure().empty());
  EXPECT_TRUE(application.GetActiveScene());
}

TEST(StrictRuntimeLoading, MultipleDotsInAssetNamesPreserveMetadataIdentity) {
  StrictRuntimeProject project;
  project.WriteProject();
  project.WriteAsset("Startup.v1.evescene", kSceneHandle, "Scene", RuntimeScene());
  project.WriteAsset("SampleW1.0.unknown", kDormantAssetHandle, "UnavailableProvider", "opaque\n");
  Application application;
  ApplicationContextScope scope(application);

  InitializeStrict(application, project);

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Loaded) << ProjectManager::GetProjectFailure();
  ASSERT_TRUE(application.GetActiveScene());
  EXPECT_EQ(application.GetActiveScene()->GetHandle(), Handle(kSceneHandle));
  EXPECT_TRUE(std::filesystem::is_regular_file(project.AssetsPath() / "SampleW1.0.unknown.evefilemeta"));
  EXPECT_FALSE(std::filesystem::exists(project.AssetsPath() / "SampleW1.unknown.evefilemeta"));
}

TEST(StrictRuntimeLoading, DormantUnknownAssetRemainsOpaque) {
  StrictRuntimeProject project;
  project.WriteProject();
  project.WriteAsset("Scene", RuntimeScene());
  project.WriteAsset("Dormant.unknown", kDormantAssetHandle, "UnavailableAssetProvider", "opaque payload\n");
  Application application;
  ApplicationContextScope scope(application);

  InitializeStrict(application, project);

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Loaded);
  EXPECT_TRUE(application.GetActiveScene());
  ASSERT_TRUE(FileManager::GetFile(Handle(kDormantAssetHandle)));
  EXPECT_EQ(FileManager::GetFile(Handle(kDormantAssetHandle))->GetAssetTypeName(), "UnavailableAssetProvider");
}

TEST(StrictRuntimeLoading, ExistingAssetPathLookupDoesNotRepairMissingContent) {
  StrictRuntimeProject project;
  project.WriteProject();
  project.WriteAsset("Scene", RuntimeScene());
  project.WriteAsset("Generated.evepostprocessingstack", kGeneratedTextureHandle, "PostProcessingStack", "{}\n");
  Application application;
  ApplicationContextScope scope(application);
  InitializeStrict(application, project);
  ASSERT_EQ(ProjectManager::GetProjectState(), ProjectState::Loaded) << ProjectManager::GetProjectFailure();

  std::vector<std::pair<std::filesystem::path, uintmax_t>> before;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(project.AssetsPath())) {
    if (entry.is_regular_file()) {
      before.emplace_back(entry.path().lexically_relative(project.AssetsPath()), entry.file_size());
    }
  }
  std::sort(before.begin(), before.end());

  const auto existing = ProjectManager::GetAsset("Generated.evepostprocessingstack");
  ASSERT_TRUE(existing);
  EXPECT_EQ(existing->GetHandle(), Handle(kGeneratedTextureHandle));
  EXPECT_FALSE(ProjectManager::GetAsset("Missing.evepostprocessingstack"));

  std::vector<std::pair<std::filesystem::path, uintmax_t>> after;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(project.AssetsPath())) {
    if (entry.is_regular_file()) {
      after.emplace_back(entry.path().lexically_relative(project.AssetsPath()), entry.file_size());
    }
  }
  std::sort(after.begin(), after.end());
  EXPECT_EQ(after, before);
  EXPECT_FALSE(std::filesystem::exists(project.AssetsPath() / "Missing.evepostprocessingstack.evefilemeta"));
}
