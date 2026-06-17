#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "AssetManager.hpp"
#include "EditorLayer.hpp"
#include "IAsset.hpp"
#include "Jobs.hpp"
#include "PackageManager.hpp"
#include "PathUtils.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <thread>

using namespace evo_engine;
using namespace std::chrono_literals;

TEST(ApplicationInitializationSettings, DefaultsToFullHdWindowSize) {
  const ApplicationInitializationSettings settings;
  EXPECT_EQ(settings.default_window_size.x, 1920);
  EXPECT_EQ(settings.default_window_size.y, 1080);
}

namespace {
constexpr uint64_t kBlockingAssetHandle = 0xE701'0000'0000'0001ull;
constexpr auto kBlockingAssetTypeName = "BlockingLoadAsset";
constexpr auto kBlockingAssetExtension = ".evetestasset";
constexpr uint64_t kStagedAssetHandle = 0xE701'0000'0000'0002ull;
constexpr auto kStagedAssetTypeName = "StagedLoadAsset";
constexpr auto kStagedAssetExtension = ".evestagedasset";
constexpr uint64_t kGpuPendingAssetHandle = 0xE701'0000'0000'0003ull;
constexpr auto kGpuPendingAssetTypeName = "GpuPendingLoadAsset";
constexpr auto kGpuPendingAssetExtension = ".evegpupendingasset";

struct BlockingLoadState {
  std::mutex mutex;
  std::condition_variable cv;
  size_t load_count = 0;
  bool load_started = false;
  bool release_load = false;
  std::weak_ptr<IAsset> first_loaded_asset;
};

BlockingLoadState& GetBlockingLoadState() {
  static BlockingLoadState state;
  return state;
}

void ResetBlockingLoadState() {
  auto& state = GetBlockingLoadState();
  std::lock_guard lock(state.mutex);
  state.load_count = 0;
  state.load_started = false;
  state.release_load = false;
  state.first_loaded_asset.reset();
}

bool WaitForBlockingLoadStarted(const std::chrono::milliseconds timeout) {
  auto& state = GetBlockingLoadState();
  std::unique_lock lock(state.mutex);
  return state.cv.wait_for(lock, timeout, [&]() {
    return state.load_started;
  });
}

void ReleaseBlockingLoad() {
  auto& state = GetBlockingLoadState();
  {
    std::lock_guard lock(state.mutex);
    state.release_load = true;
  }
  state.cv.notify_all();
}

size_t GetBlockingLoadCount() {
  auto& state = GetBlockingLoadState();
  std::lock_guard lock(state.mutex);
  return state.load_count;
}

std::shared_ptr<IAsset> GetFirstLoadedAsset() {
  auto& state = GetBlockingLoadState();
  std::lock_guard lock(state.mutex);
  return state.first_loaded_asset.lock();
}

class ScopedBlockingLoadRelease {
 public:
  ~ScopedBlockingLoadRelease() {
    ReleaseBlockingLoad();
  }
};

class BlockingLoadAsset final : public IAsset {
 public:
  static bool RegisterAssetIoHandlers() {
    return Serialization::RegisterAssetIoHandler<BlockingLoadAsset>(
        {},
        [](BlockingLoadAsset& asset, const std::filesystem::path& path) {
          return asset.LoadBlockingFixture(path);
        },
        {}, {}, {}, {}, kBlockingAssetTypeName);
  }

 protected:
  bool LoadBlockingFixture(const std::filesystem::path&) {
    {
      auto& state = GetBlockingLoadState();
      std::lock_guard lock(state.mutex);
      ++state.load_count;
      if (state.first_loaded_asset.expired()) {
        state.first_loaded_asset = GetSelf();
      }
      state.load_started = true;
    }
    GetBlockingLoadState().cv.notify_all();

    auto& state = GetBlockingLoadState();
    std::unique_lock lock(state.mutex);
    state.cv.wait(lock, [&]() {
      return state.release_load;
    });
    return true;
  }
};

struct StagedLoadState {
  std::mutex mutex;
  std::condition_variable cv;
  size_t sync_load_count = 0;
  size_t payload_load_count = 0;
  size_t finalize_count = 0;
  bool payload_started = false;
  bool release_payload = false;
  bool payload_ran_on_asset_io = false;
  bool finalize_ran_on_main_thread = false;
};

struct StagedLoadSnapshot {
  size_t sync_load_count = 0;
  size_t payload_load_count = 0;
  size_t finalize_count = 0;
  bool payload_started = false;
  bool release_payload = false;
  bool payload_ran_on_asset_io = false;
  bool finalize_ran_on_main_thread = false;
};

StagedLoadState& GetStagedLoadState() {
  static StagedLoadState state;
  return state;
}

void ResetStagedLoadState() {
  auto& state = GetStagedLoadState();
  std::lock_guard lock(state.mutex);
  state.sync_load_count = 0;
  state.payload_load_count = 0;
  state.finalize_count = 0;
  state.payload_started = false;
  state.release_payload = false;
  state.payload_ran_on_asset_io = false;
  state.finalize_ran_on_main_thread = false;
}

bool WaitForStagedPayloadStarted(const std::chrono::milliseconds timeout) {
  auto& state = GetStagedLoadState();
  std::unique_lock lock(state.mutex);
  return state.cv.wait_for(lock, timeout, [&]() {
    return state.payload_started;
  });
}

bool WaitForStagedPayloadStartedWhilePumping(const std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    AssetManager::ExecuteMainThreadAssetTasks(1);
    Jobs::ExecuteMainThreadJobs(1);
    if (WaitForStagedPayloadStarted(10ms)) {
      return true;
    }
  }
  return false;
}

void ReleaseStagedPayload() {
  auto& state = GetStagedLoadState();
  {
    std::lock_guard lock(state.mutex);
    state.release_payload = true;
  }
  state.cv.notify_all();
}

StagedLoadSnapshot SnapshotStagedLoadState() {
  auto& state = GetStagedLoadState();
  std::lock_guard lock(state.mutex);
  StagedLoadSnapshot snapshot;
  snapshot.sync_load_count = state.sync_load_count;
  snapshot.payload_load_count = state.payload_load_count;
  snapshot.finalize_count = state.finalize_count;
  snapshot.payload_started = state.payload_started;
  snapshot.release_payload = state.release_payload;
  snapshot.payload_ran_on_asset_io = state.payload_ran_on_asset_io;
  snapshot.finalize_ran_on_main_thread = state.finalize_ran_on_main_thread;
  return snapshot;
}

class ScopedStagedPayloadRelease {
 public:
  ~ScopedStagedPayloadRelease() {
    ReleaseStagedPayload();
  }
};

class StagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  int value = 42;
};

class StagedLoadAsset final : public IAsset {
 public:
  static bool RegisterAssetIoHandlers() {
    return Serialization::RegisterAssetIoHandler<StagedLoadAsset>(
        {},
        [](StagedLoadAsset& asset, const std::filesystem::path& path) {
          return asset.LoadSynchronously(path);
        },
        [](const StagedLoadAsset& asset, const std::filesystem::path&) {
          return asset.SupportsFixtureStagedLoading();
        },
        [](const StagedLoadAsset& asset, const std::filesystem::path& path) {
          return asset.LoadFixtureStagedPayload(path);
        },
        [](StagedLoadAsset& asset, const std::filesystem::path& path,
           const std::shared_ptr<StagedAssetLoadPayload>& payload) {
          return asset.ApplyFixtureStagedPayload(path, payload);
        },
        {}, kStagedAssetTypeName);
  }

 protected:
  bool LoadSynchronously(const std::filesystem::path&) {
    auto& state = GetStagedLoadState();
    std::lock_guard lock(state.mutex);
    ++state.sync_load_count;
    return true;
  }

  [[nodiscard]] bool SupportsFixtureStagedLoading() const {
    return true;
  }

  [[nodiscard]] std::shared_ptr<StagedAssetLoadPayload> LoadFixtureStagedPayload(const std::filesystem::path&) const {
    {
      auto& state = GetStagedLoadState();
      std::lock_guard lock(state.mutex);
      ++state.payload_load_count;
      state.payload_started = true;
      state.payload_ran_on_asset_io = Jobs::IsExecutorThread(JobExecutorType::AssetIo);
    }
    GetStagedLoadState().cv.notify_all();

    auto& state = GetStagedLoadState();
    std::unique_lock lock(state.mutex);
    state.cv.wait(lock, [&]() {
      return state.release_payload;
    });
    return std::make_shared<StagedLoadPayload>();
  }

  bool ApplyFixtureStagedPayload(const std::filesystem::path&, const std::shared_ptr<StagedAssetLoadPayload>& payload) {
    const auto staged_payload = std::dynamic_pointer_cast<StagedLoadPayload>(payload);
    auto& state = GetStagedLoadState();
    std::lock_guard lock(state.mutex);
    ++state.finalize_count;
    state.finalize_ran_on_main_thread = Jobs::IsMainThread();
    return staged_payload && staged_payload->value == 42;
  }
};

struct GpuPendingLoadState {
  std::mutex mutex;
  std::condition_variable cv;
  size_t payload_load_count = 0;
  size_t finalize_count = 0;
  size_t gpu_work_start_count = 0;
  size_t gpu_work_complete_count = 0;
  bool gpu_work_started = false;
  bool release_gpu_work = false;
  bool fail_gpu_work = false;
  bool gpu_work_ran_on_background = false;
};

GpuPendingLoadState& GetGpuPendingLoadState() {
  static GpuPendingLoadState state;
  return state;
}

void ResetGpuPendingLoadState(const bool fail_gpu_work = false) {
  auto& state = GetGpuPendingLoadState();
  std::lock_guard lock(state.mutex);
  state.payload_load_count = 0;
  state.finalize_count = 0;
  state.gpu_work_start_count = 0;
  state.gpu_work_complete_count = 0;
  state.gpu_work_started = false;
  state.release_gpu_work = false;
  state.fail_gpu_work = fail_gpu_work;
  state.gpu_work_ran_on_background = false;
}

bool WaitForGpuPendingWorkStartedWhilePumping(const std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    AssetManager::ExecuteMainThreadAssetTasks(1);
    Jobs::ExecuteMainThreadJobs(1);
    auto& state = GetGpuPendingLoadState();
    std::unique_lock lock(state.mutex);
    if (state.cv.wait_for(lock, 10ms, [&]() {
          return state.gpu_work_started;
        })) {
      return true;
    }
  }
  return false;
}

void ReleaseGpuPendingWork() {
  auto& state = GetGpuPendingLoadState();
  {
    std::lock_guard lock(state.mutex);
    state.release_gpu_work = true;
  }
  state.cv.notify_all();
}

bool WaitForAssetFutureReadyWhilePumping(const std::shared_future<std::shared_ptr<IAsset>>& future,
                                         const std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    AssetManager::ExecuteMainThreadAssetTasks(1);
    Jobs::ExecuteMainThreadJobs(1);
    if (future.wait_for(10ms) == std::future_status::ready) {
      return true;
    }
  }
  return false;
}

class ScopedGpuPendingWorkRelease {
 public:
  ~ScopedGpuPendingWorkRelease() {
    ReleaseGpuPendingWork();
  }
};

class GpuPendingLoadPayload final : public StagedAssetLoadPayload {};

class GpuPendingLoadAsset final : public IAsset {
 public:
  static bool RegisterAssetIoHandlers() {
    return Serialization::RegisterAssetIoHandler<GpuPendingLoadAsset>(
        {}, {},
        [](const GpuPendingLoadAsset& asset, const std::filesystem::path&) {
          return asset.SupportsFixtureStagedLoading();
        },
        [](const GpuPendingLoadAsset& asset, const std::filesystem::path& path) {
          return asset.LoadFixtureStagedPayload(path);
        },
        [](GpuPendingLoadAsset& asset, const std::filesystem::path& path,
           const std::shared_ptr<StagedAssetLoadPayload>& payload) {
          return asset.ApplyFixtureStagedPayload(path, payload);
        },
        {}, kGpuPendingAssetTypeName);
  }

 protected:
  [[nodiscard]] bool SupportsFixtureStagedLoading() const {
    return true;
  }

  [[nodiscard]] std::shared_ptr<StagedAssetLoadPayload> LoadFixtureStagedPayload(const std::filesystem::path&) const {
    auto& state = GetGpuPendingLoadState();
    std::lock_guard lock(state.mutex);
    ++state.payload_load_count;
    return std::make_shared<GpuPendingLoadPayload>();
  }

  bool ApplyFixtureStagedPayload(const std::filesystem::path&, const std::shared_ptr<StagedAssetLoadPayload>& payload) {
    if (!std::dynamic_pointer_cast<GpuPendingLoadPayload>(payload)) {
      return false;
    }
    {
      auto& state = GetGpuPendingLoadState();
      std::lock_guard lock(state.mutex);
      ++state.finalize_count;
    }

    const auto handle = Jobs::RunOnBackgroundThread([]() {
      {
        auto& state = GetGpuPendingLoadState();
        std::lock_guard lock(state.mutex);
        ++state.gpu_work_start_count;
        state.gpu_work_started = true;
        state.gpu_work_ran_on_background = Jobs::IsExecutorThread(JobExecutorType::Background);
      }
      GetGpuPendingLoadState().cv.notify_all();

      auto& state = GetGpuPendingLoadState();
      std::unique_lock lock(state.mutex);
      state.cv.wait(lock, [&]() {
        return state.release_gpu_work;
      });
      const bool should_fail = state.fail_gpu_work;
      if (should_fail) {
        throw std::runtime_error("gpu readiness failure");
      }
      ++state.gpu_work_complete_count;
    });
    Jobs::Execute(handle);
    TrackPendingGpuWork(handle);
    return true;
  }
};

class TempProject {
 public:
  TempProject() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineAssetManagerTest_" + std::to_string(now));
    std::filesystem::create_directories(AssetsPath());
  }

  ~TempProject() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path ProjectPath() const {
    return root_ / "AssetManagerTest.eveproj";
  }

  [[nodiscard]] std::filesystem::path AssetsPath() const {
    return root_ / "Assets";
  }

 private:
  std::filesystem::path root_;
};

class TempPackageDirectory {
 public:
  TempPackageDirectory() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEnginePackageManagerTest_" + std::to_string(now));
    std::filesystem::create_directories(root_);
  }

  ~TempPackageDirectory() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path RootPath() const {
    return root_;
  }

  void WritePackageManifest(const std::string& package_name, const std::string& library_name,
                            const bool create_library) const {
    std::ofstream manifest_file(root_ / (package_name + ".evepackage"));
    manifest_file << "name: " << package_name << "\n";
    manifest_file << "library: " << library_name << "\n";
    manifest_file << "version: 0.1.0\n";
    manifest_file << "description: Test package.\n";
    manifest_file.close();
    if (create_library) {
      std::ofstream library_file(root_ / library_name);
      library_file << "test package library placeholder";
    }
  }

 private:
  std::filesystem::path root_;
};

void WriteBlockingAssetFixture(const TempProject& project) {
  const auto asset_path = project.AssetsPath() / ("Coalesced" + std::string(kBlockingAssetExtension));
  {
    std::ofstream asset_file(asset_path);
    asset_file << "test asset payload";
  }

  const auto metadata_path = asset_path.string() + ".evefilemeta";
  std::ofstream metadata_file(metadata_path);
  metadata_file << "asset_extension_: " << kBlockingAssetExtension << "\n";
  metadata_file << "asset_file_name_: Coalesced\n";
  metadata_file << "asset_type_name_: " << kBlockingAssetTypeName << "\n";
  metadata_file << "asset_handle_: " << kBlockingAssetHandle << "\n";
}

void WriteStagedAssetFixture(const TempProject& project) {
  const auto asset_path = project.AssetsPath() / ("Staged" + std::string(kStagedAssetExtension));
  {
    std::ofstream asset_file(asset_path);
    asset_file << "staged asset payload";
  }

  const auto metadata_path = asset_path.string() + ".evefilemeta";
  std::ofstream metadata_file(metadata_path);
  metadata_file << "asset_extension_: " << kStagedAssetExtension << "\n";
  metadata_file << "asset_file_name_: Staged\n";
  metadata_file << "asset_type_name_: " << kStagedAssetTypeName << "\n";
  metadata_file << "asset_handle_: " << kStagedAssetHandle << "\n";
}

void WriteGpuPendingAssetFixture(const TempProject& project) {
  const auto asset_path = project.AssetsPath() / ("GpuPending" + std::string(kGpuPendingAssetExtension));
  {
    std::ofstream asset_file(asset_path);
    asset_file << "gpu pending asset payload";
  }

  const auto metadata_path = asset_path.string() + ".evefilemeta";
  std::ofstream metadata_file(metadata_path);
  metadata_file << "asset_extension_: " << kGpuPendingAssetExtension << "\n";
  metadata_file << "asset_file_name_: GpuPending\n";
  metadata_file << "asset_type_name_: " << kGpuPendingAssetTypeName << "\n";
  metadata_file << "asset_handle_: " << kGpuPendingAssetHandle << "\n";
}

ApplicationInitializationSettings TestApplicationSettings(const TempProject& project) {
  ApplicationInitializationSettings settings;
  settings.project_path = project.ProjectPath();
  settings.load_default_resources = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  settings.redirect_standard_streams_to_console = false;
  return settings;
}
}  // namespace

TEST(ProjectManager, ReportsNoProjectBeforeProjectSelection) {
  Application app;
  ApplicationContextScope scope(app);

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::NoProject);
  EXPECT_FALSE(ProjectManager::HasProject());
  EXPECT_FALSE(ProjectManager::IsProjectLoaded());
  EXPECT_FALSE(ProjectManager::IsProjectIdle());
}

TEST(ProjectManager, ReportsLoadingAfterProjectSelectionWithoutStartScene) {
  TempProject project;

  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(TestApplicationSettings(project));

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Loading);
  EXPECT_TRUE(ProjectManager::HasProject());
  EXPECT_FALSE(ProjectManager::IsProjectLoaded());
  EXPECT_FALSE(ProjectManager::IsProjectIdle());
}

TEST(ProjectManager, ReportsLoadedProjectAndResetsOnTerminate) {
  TempProject project;

  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(TestApplicationSettings(project));
  ProjectManager::SetStartScene(std::make_shared<Scene>());

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Loaded);
  EXPECT_TRUE(ProjectManager::HasProject());
  EXPECT_TRUE(ProjectManager::IsProjectLoaded());
  EXPECT_TRUE(ProjectManager::IsProjectIdle());

  app.Terminate();

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::NoProject);
  EXPECT_FALSE(ProjectManager::HasProject());
  EXPECT_FALSE(ProjectManager::IsProjectLoaded());
  EXPECT_FALSE(ProjectManager::IsProjectIdle());
}

TEST(ProjectManager, AssetsFolderContainmentUsesPathBoundary) {
  TempProject project;
  const auto assets_file = project.AssetsPath() / "Texture.bin";
  const auto sibling_folder = project.ProjectPath().parent_path() / "AssetsBackup";
  const auto sibling_file = sibling_folder / "Texture.bin";
  std::ofstream(assets_file).close();
  std::filesystem::create_directories(sibling_folder);
  std::ofstream(sibling_file).close();

  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(TestApplicationSettings(project));

  EXPECT_TRUE(ProjectManager::IsInAssetsFolder(std::filesystem::absolute(assets_file)));
  EXPECT_FALSE(ProjectManager::IsInAssetsFolder(std::filesystem::absolute(sibling_file)));
  EXPECT_EQ(ProjectManager::GetAssetsRelativePath(std::filesystem::absolute(assets_file)),
            std::filesystem::path("Texture.bin"));
  EXPECT_TRUE(ProjectManager::GetAssetsRelativePath(std::filesystem::absolute(sibling_file)).empty());
}

TEST(ProjectManager, LoadsDefaultLaunchMetadataForLegacyProjectFile) {
  TempProject project;
  std::ofstream project_file(project.ProjectPath());
  project_file << "start_scene_handle: 42\n";
  project_file.close();

  const auto metadata = ProjectManager::LoadProjectLaunchMetadata(project.ProjectPath());

  EXPECT_EQ(metadata.application_name, "EvoEngine Editor");
  EXPECT_EQ(metadata.preferred_editor, "EvoEngineEditor");
  EXPECT_TRUE(metadata.startup_runtime_packages.empty());
}

TEST(ProjectManager, LoadsLaunchMetadataFromProjectFile) {
  TempProject project;
  std::ofstream project_file(project.ProjectPath());
  project_file << "application_name: Metadata Test\n";
  project_file << "preferred_editor: EvoEngineEditor\n";
  project_file << "startup_runtime_packages:\n";
  project_file << "  - PackageA\n";
  project_file << "  - PackageB\n";
  project_file << "start_scene_handle: 42\n";
  project_file.close();

  const auto metadata = ProjectManager::LoadProjectLaunchMetadata(project.ProjectPath());

  EXPECT_EQ(metadata.application_name, "Metadata Test");
  EXPECT_EQ(metadata.preferred_editor, "EvoEngineEditor");
  ASSERT_EQ(metadata.startup_runtime_packages.size(), 2);
  EXPECT_EQ(metadata.startup_runtime_packages[0], "PackageA");
  EXPECT_EQ(metadata.startup_runtime_packages[1], "PackageB");
}

TEST(ProjectManager, SaveLaunchMetadataCreatesProjectManifestWithoutOpeningProject) {
  TempProject project;
  ProjectLaunchMetadata metadata;
  metadata.application_name = "Template Project";
  metadata.preferred_editor = "EvoEngineEditor";
  metadata.startup_runtime_packages = {"EcoSysLab", "DigitalAgriculture"};

  ProjectManager::SaveProjectLaunchMetadata(project.ProjectPath(), metadata);

  const auto loaded_metadata = ProjectManager::LoadProjectLaunchMetadata(project.ProjectPath());
  EXPECT_EQ(loaded_metadata.application_name, "Template Project");
  ASSERT_EQ(loaded_metadata.startup_runtime_packages.size(), 2);
  EXPECT_EQ(loaded_metadata.startup_runtime_packages[0], "EcoSysLab");
  EXPECT_EQ(loaded_metadata.startup_runtime_packages[1], "DigitalAgriculture");
  Application app;
  ApplicationContextScope scope(app);
  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::NoProject);
  EXPECT_FALSE(YAML::LoadFile(project.ProjectPath().string())["start_scene_handle"]);
}

TEST(ProjectManager, OpensMetadataOnlyProjectByCreatingDefaultStartScene) {
  TempProject project;
  ProjectLaunchMetadata metadata;
  metadata.application_name = "Template Project";
  metadata.preferred_editor = "EvoEngineEditor";

  ProjectManager::SaveProjectLaunchMetadata(project.ProjectPath(), metadata);
  ASSERT_FALSE(YAML::LoadFile(project.ProjectPath().string())["start_scene_handle"]);

  Application app;
  ApplicationContextScope scope(app);
  auto settings = TestApplicationSettings(project);
  settings.load_project_start_scene = true;

  ASSERT_NO_THROW(app.Initialize(settings));

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Loaded);
  EXPECT_TRUE(ProjectManager::IsProjectLoaded());
  EXPECT_TRUE(ProjectManager::GetStartScene().lock());

  const auto project_yaml = YAML::LoadFile(project.ProjectPath().string());
  ASSERT_TRUE(project_yaml["start_scene_handle"]);
  EXPECT_NE(project_yaml["start_scene_handle"].as<uint64_t>(), 0);
}

TEST(ProjectManager, LoadedStateDoesNotWaitForBackgroundAssetLoad) {
  ResetStagedLoadState();
  TempProject project;
  WriteStagedAssetFixture(project);

  Application app;
  ApplicationContextScope scope(app);
  app.RegisterAsset<StagedLoadAsset>(kStagedAssetTypeName, {kStagedAssetExtension});
  ASSERT_TRUE(StagedLoadAsset::RegisterAssetIoHandlers());
  auto settings = TestApplicationSettings(project);
  settings.load_project_assets = false;
  app.Initialize(settings);
  ProjectManager::SetStartScene(std::make_shared<Scene>());

  auto asset_future = AssetManager::RequestAssetLoad(Handle(kStagedAssetHandle));
  const bool payload_started = WaitForStagedPayloadStartedWhilePumping(5s);
  if (!payload_started) {
    ReleaseStagedPayload();
    FAIL() << "Timed out waiting for staged asset payload loading to start.";
  }
  ScopedStagedPayloadRelease release_on_exit;

  EXPECT_EQ(ProjectManager::GetProjectState(), ProjectState::Loaded);
  EXPECT_TRUE(ProjectManager::IsProjectLoaded());
  EXPECT_FALSE(ProjectManager::IsProjectIdle());
  EXPECT_EQ(asset_future.wait_for(100ms), std::future_status::timeout);

  ReleaseStagedPayload();
  ASSERT_TRUE(WaitForAssetFutureReadyWhilePumping(asset_future, 5s));
  EXPECT_NE(asset_future.get(), nullptr);
  EXPECT_TRUE(ProjectManager::IsProjectIdle());
}

TEST(ProjectManager, SaveLaunchMetadataPreservesExistingStartSceneHandle) {
  TempProject project;
  std::ofstream project_file(project.ProjectPath());
  project_file << "start_scene_handle: 42\n";
  project_file.close();

  ProjectLaunchMetadata metadata;
  metadata.application_name = "Template Project";
  metadata.preferred_editor = "EvoEngineEditor";
  metadata.startup_runtime_packages = {"LogGrading"};
  ProjectManager::SaveProjectLaunchMetadata(project.ProjectPath(), metadata);

  const auto project_yaml = YAML::LoadFile(project.ProjectPath().string());
  EXPECT_EQ(project_yaml["start_scene_handle"].as<uint64_t>(), 42);
  const auto loaded_metadata = ProjectManager::LoadProjectLaunchMetadata(project.ProjectPath());
  EXPECT_EQ(loaded_metadata.application_name, "Template Project");
  ASSERT_EQ(loaded_metadata.startup_runtime_packages.size(), 1);
  EXPECT_EQ(loaded_metadata.startup_runtime_packages[0], "LogGrading");
}

TEST(ProjectManager, MergesProjectLaunchMetadataIntoApplicationSettings) {
  TempProject project;
  std::ofstream project_file(project.ProjectPath());
  project_file << "application_name: Metadata Test\n";
  project_file << "preferred_editor: EvoEngineEditor\n";
  project_file << "startup_runtime_packages:\n";
  project_file << "  - MissingPackageForMetadataTest\n";
  project_file << "start_scene_handle: 42\n";
  project_file.close();

  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(TestApplicationSettings(project));

  const auto& settings = app.GetApplicationInfo();
  EXPECT_TRUE(settings.enable_runtime_packages);
  ASSERT_EQ(settings.startup_runtime_packages.size(), 1);
  EXPECT_EQ(settings.startup_runtime_packages[0], "MissingPackageForMetadataTest");

  const auto metadata = ProjectManager::GetProjectLaunchMetadata();
  EXPECT_EQ(metadata.application_name, "Metadata Test");
  ASSERT_EQ(metadata.startup_runtime_packages.size(), 1);
  EXPECT_EQ(metadata.startup_runtime_packages[0], "MissingPackageForMetadataTest");
}

TEST(ProjectManager, SaveProjectPersistsLaunchMetadata) {
  TempProject project;
  std::ofstream project_file(project.ProjectPath());
  project_file << "application_name: Metadata Test\n";
  project_file << "preferred_editor: EvoEngineEditor\n";
  project_file << "startup_runtime_packages:\n";
  project_file << "  - MissingPackageForMetadataTest\n";
  project_file << "start_scene_handle: 42\n";
  project_file.close();

  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(TestApplicationSettings(project));
  ProjectManager::SetStartScene(std::make_shared<Scene>());
  ProjectManager::SaveProject();

  const auto metadata = ProjectManager::LoadProjectLaunchMetadata(project.ProjectPath());
  EXPECT_EQ(metadata.application_name, "Metadata Test");
  ASSERT_EQ(metadata.startup_runtime_packages.size(), 1);
  EXPECT_EQ(metadata.startup_runtime_packages[0], "MissingPackageForMetadataTest");
}

TEST(ProjectManager, ExistingProjectLoadDoesNotRewriteProjectManifestBeforeSave) {
  TempProject project;
  {
    Application app;
    ApplicationContextScope scope(app);
    app.Initialize(TestApplicationSettings(project));
    const auto scene = AssetManager::CreateTemporaryAsset<Scene>();
    ASSERT_TRUE(scene->SetPathAndSave("Existing Scene.evescene"));
    ProjectManager::SetStartScene(scene);
    ProjectManager::SaveProject();
    app.Terminate();
  }

  {
    std::ofstream project_file(project.ProjectPath(), std::ios::app);
    project_file << "\nlegacy_marker: keep\n";
  }

  Application app;
  ApplicationContextScope scope(app);
  auto settings = TestApplicationSettings(project);
  settings.load_project_start_scene = true;

  ASSERT_NO_THROW(app.Initialize(settings));

  const auto project_yaml = YAML::LoadFile(project.ProjectPath().string());
  ASSERT_TRUE(project_yaml["legacy_marker"]);
  EXPECT_EQ(project_yaml["legacy_marker"].as<std::string>(), "keep");
}

TEST(EditorLayer, MissingEditorStateKeepsDefaultsAndRequestsDefaultLayout) {
  EditorLayer editor_layer;

  editor_layer.Deserialize(YAML::Node());

  EXPECT_TRUE(editor_layer.show_scene_window);
  EXPECT_TRUE(editor_layer.show_camera_window);
  EXPECT_FLOAT_EQ(editor_layer.velocity, 10.0f);
  EXPECT_FLOAT_EQ(editor_layer.sensitivity, 0.1f);
  EXPECT_TRUE(editor_layer.DefaultEditorLayoutPending());
}

TEST(EditorLayer, PartialEditorStateChangesOnlyPresentFields) {
  EditorLayer editor_layer;

  editor_layer.Deserialize(YAML::Load(R"(
show_scene_window: false
velocity: 3.5
)"));

  EXPECT_FALSE(editor_layer.show_scene_window);
  EXPECT_TRUE(editor_layer.show_camera_window);
  EXPECT_FLOAT_EQ(editor_layer.velocity, 3.5f);
  EXPECT_FLOAT_EQ(editor_layer.sensitivity, 0.1f);
  EXPECT_TRUE(editor_layer.DefaultEditorLayoutPending());
}

TEST(EditorLayer, MissingEmptyOrNonDockingImGuiIniRequestsDefaultLayout) {
  EditorLayer missing_layout;
  missing_layout.Deserialize(YAML::Load("{show_scene_window: true}"));
  EXPECT_TRUE(missing_layout.DefaultEditorLayoutPending());

  EditorLayer empty_layout;
  empty_layout.Deserialize(YAML::Load("{ImGuiIni: ''}"));
  EXPECT_TRUE(empty_layout.DefaultEditorLayoutPending());

  EditorLayer non_docking_layout;
  non_docking_layout.Deserialize(YAML::Load(R"(
ImGuiIni: |
  [Window][Scene]
  Pos=0,0
)"));
  EXPECT_TRUE(non_docking_layout.DefaultEditorLayoutPending());
}

TEST(EditorLayer, ValidDockingImGuiIniDefersRestoreWithoutDefaultLayout) {
  EditorLayer editor_layer;
  editor_layer.Deserialize(YAML::Node());
  ASSERT_TRUE(editor_layer.DefaultEditorLayoutPending());

  editor_layer.Deserialize(YAML::Load(R"(
ImGuiIni: |
  [Docking][Data]
  DockSpace ID=0x00000001 Window=0x00000002
)"));

  EXPECT_FALSE(editor_layer.DefaultEditorLayoutPending());
}

TEST(PackageManager, ReportsManifestLibraryAvailability) {
  TempPackageDirectory package_directory;
  package_directory.WritePackageManifest("AvailablePackage", "AvailablePackage.dll", true);
  package_directory.WritePackageManifest("MissingLibraryPackage", "MissingLibraryPackage.dll", false);

  Application app;
  ApplicationContextScope scope(app);
  PackageManager::Initialize({package_directory.RootPath()}, {});

  const auto packages = PackageManager::GetAvailablePackages();
  auto find_package = [&](const std::string& package_name) {
    return std::find_if(packages.begin(), packages.end(), [&](const AvailablePackageInfo& package) {
      return package.name == package_name;
    });
  };

  const auto available_package = find_package("AvailablePackage");
  ASSERT_NE(available_package, packages.end());
  EXPECT_TRUE(available_package->library_exists);

  const auto missing_library_package = find_package("MissingLibraryPackage");
  ASSERT_NE(missing_library_package, packages.end());
  EXPECT_FALSE(missing_library_package->library_exists);
}

TEST(PackageManager, InitializeDeduplicatesSearchPaths) {
  TempPackageDirectory package_directory;

  Application app;
  ApplicationContextScope scope(app);
  PackageManager::Initialize({package_directory.RootPath(), package_directory.RootPath() / "."}, {});

  const auto expected_path = path_utils::NormalizeAbsolutePath(package_directory.RootPath());
  const auto search_paths = PackageManager::GetSearchPaths();
  const auto count = std::count(search_paths.begin(), search_paths.end(), expected_path);
  EXPECT_EQ(count, 1);
}

TEST(PackageManager, ModificationIsBlockedWhilePlayingPausedOrStepping) {
  TempProject project;

  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(TestApplicationSettings(project));
  const auto scene = std::make_shared<Scene>();
  ProjectManager::SetStartScene(scene);
  app.Attach(scene);

  EXPECT_EQ(app.GetApplicationStatus(), Application::ExecutionStatus::NotPlaying);
  EXPECT_TRUE(PackageManager::CanModifyPackages());

  app.Play();
  EXPECT_EQ(app.GetApplicationStatus(), Application::ExecutionStatus::Playing);
  EXPECT_FALSE(PackageManager::CanModifyPackages());

  app.Pause();
  EXPECT_EQ(app.GetApplicationStatus(), Application::ExecutionStatus::Pause);
  EXPECT_FALSE(PackageManager::CanModifyPackages());

  app.Step();
  EXPECT_EQ(app.GetApplicationStatus(), Application::ExecutionStatus::Step);
  EXPECT_FALSE(PackageManager::CanModifyPackages());

  app.Stop();
  EXPECT_EQ(app.GetApplicationStatus(), Application::ExecutionStatus::NotPlaying);
  EXPECT_TRUE(PackageManager::CanModifyPackages());
}

TEST(PackageManager, LoadAllIsRejectedWhileRuntimeIsBusy) {
  TempProject project;
  TempPackageDirectory package_directory;

  Application app;
  ApplicationContextScope scope(app);
  app.Initialize(TestApplicationSettings(project));
  PackageManager::Initialize({package_directory.RootPath()}, {});
  const auto scene = std::make_shared<Scene>();
  ProjectManager::SetStartScene(scene);
  app.Attach(scene);

  ASSERT_TRUE(PackageManager::LoadAll());

  app.Play();
  EXPECT_FALSE(PackageManager::LoadAll());

  app.Pause();
  EXPECT_FALSE(PackageManager::LoadAll());

  app.Step();
  EXPECT_FALSE(PackageManager::LoadAll());
}

TEST(AssetManager, BlockingAccessJoinsInFlightSynchronousProjectLoad) {
  ResetBlockingLoadState();
  TempProject project;
  WriteBlockingAssetFixture(project);

  Application app;
  app.RegisterAsset<BlockingLoadAsset>(kBlockingAssetTypeName, {kBlockingAssetExtension});
  ASSERT_TRUE(BlockingLoadAsset::RegisterAssetIoHandlers());

  std::exception_ptr project_load_exception;
  std::thread project_load_thread([&]() {
    try {
      ApplicationContextScope scope(app);
      app.Initialize(TestApplicationSettings(project));
    } catch (...) {
      project_load_exception = std::current_exception();
      ReleaseBlockingLoad();
    }
  });

  const bool load_started = WaitForBlockingLoadStarted(5s);
  if (!load_started) {
    ReleaseBlockingLoad();
    if (project_load_thread.joinable()) {
      project_load_thread.join();
    }
    if (project_load_exception) {
      std::rethrow_exception(project_load_exception);
    }
    FAIL() << "Timed out waiting for the fixture asset to enter its registered load handler.";
  }
  ScopedBlockingLoadRelease release_on_exit;

  auto instant_access = std::async(std::launch::async, [&]() {
    ApplicationContextScope scope(app);
    return AssetManager::GetAsset<BlockingLoadAsset>(Handle(kBlockingAssetHandle));
  });

  EXPECT_EQ(instant_access.wait_for(100ms), std::future_status::timeout);

  ReleaseBlockingLoad();
  std::shared_ptr<BlockingLoadAsset> instant_asset;
  std::exception_ptr instant_access_exception;
  try {
    instant_asset = instant_access.get();
  } catch (...) {
    instant_access_exception = std::current_exception();
  }

  if (project_load_thread.joinable()) {
    project_load_thread.join();
  }
  if (project_load_exception) {
    std::rethrow_exception(project_load_exception);
  }
  if (instant_access_exception) {
    std::rethrow_exception(instant_access_exception);
  }

  ASSERT_NE(instant_asset, nullptr);
  EXPECT_EQ(GetBlockingLoadCount(), 1);
  EXPECT_EQ(instant_asset, GetFirstLoadedAsset());

  const auto later_asset = AssetManager::GetAsset<BlockingLoadAsset>(Handle(kBlockingAssetHandle));
  EXPECT_EQ(later_asset, instant_asset);
}

TEST(AssetManager, AsyncStagedLoadSeparatesAssetIoFromMainThreadFinalization) {
  ResetStagedLoadState();
  TempProject project;
  WriteStagedAssetFixture(project);

  Application app;
  ApplicationContextScope scope(app);
  app.RegisterAsset<StagedLoadAsset>(kStagedAssetTypeName, {kStagedAssetExtension});
  ASSERT_TRUE(StagedLoadAsset::RegisterAssetIoHandlers());
  auto settings = TestApplicationSettings(project);
  settings.load_project_assets = false;
  app.Initialize(settings);

  auto asset_future = AssetManager::GetAssetFuture<StagedLoadAsset>(Handle(kStagedAssetHandle));
  const auto initial_snapshot = AssetManager::GetAssetLoadSnapshot();
  EXPECT_EQ(initial_snapshot.total, 1);
  EXPECT_TRUE(initial_snapshot.Active());

  const bool payload_started = WaitForStagedPayloadStartedWhilePumping(5s);
  if (!payload_started) {
    ReleaseStagedPayload();
    FAIL() << "Timed out waiting for staged asset payload loading to start.";
  }
  ScopedStagedPayloadRelease release_on_exit;

  auto state_before_finalize = SnapshotStagedLoadState();
  EXPECT_EQ(state_before_finalize.sync_load_count, 0);
  EXPECT_EQ(state_before_finalize.payload_load_count, 1);
  EXPECT_EQ(state_before_finalize.finalize_count, 0);

  ReleaseStagedPayload();
  const auto asset = asset_future.get();
  ASSERT_NE(asset, nullptr);

  const auto state_after_finalize = SnapshotStagedLoadState();
  const auto final_snapshot = AssetManager::GetAssetLoadSnapshot();
  EXPECT_FALSE(final_snapshot.Active());
  EXPECT_EQ(final_snapshot.total, 1);
  EXPECT_EQ(final_snapshot.completed, 1);
  EXPECT_EQ(state_after_finalize.sync_load_count, 0);
  EXPECT_EQ(state_after_finalize.payload_load_count, 1);
  EXPECT_EQ(state_after_finalize.finalize_count, 1);
  EXPECT_TRUE(state_after_finalize.payload_ran_on_asset_io);
  EXPECT_TRUE(state_after_finalize.finalize_ran_on_main_thread);

  const auto later_asset = AssetManager::GetAsset<StagedLoadAsset>(Handle(kStagedAssetHandle));
  EXPECT_EQ(later_asset, asset);
}

TEST(AssetManager, AsyncStagedLoadWaitsForGpuReadinessBeforePublishing) {
  ResetGpuPendingLoadState();
  TempProject project;
  WriteGpuPendingAssetFixture(project);

  Application app;
  ApplicationContextScope scope(app);
  app.RegisterAsset<GpuPendingLoadAsset>(kGpuPendingAssetTypeName, {kGpuPendingAssetExtension});
  ASSERT_TRUE(GpuPendingLoadAsset::RegisterAssetIoHandlers());
  auto settings = TestApplicationSettings(project);
  settings.load_project_assets = false;
  app.Initialize(settings);

  auto asset_future = AssetManager::RequestAssetLoad(Handle(kGpuPendingAssetHandle));
  const bool gpu_work_started = WaitForGpuPendingWorkStartedWhilePumping(5s);
  if (!gpu_work_started) {
    ReleaseGpuPendingWork();
    FAIL() << "Timed out waiting for simulated GPU readiness work to start.";
  }
  ScopedGpuPendingWorkRelease release_on_exit;

  const auto gpu_pending_snapshot = AssetManager::GetAssetLoadSnapshot();
  EXPECT_TRUE(gpu_pending_snapshot.Active());
  EXPECT_EQ(gpu_pending_snapshot.active_state, AssetManager::AssetLoadState::GpuPending);
  EXPECT_EQ(gpu_pending_snapshot.gpu_pending, 1);
  EXPECT_EQ(asset_future.wait_for(100ms), std::future_status::timeout);

  auto sync_access = std::async(std::launch::async, [&]() {
    ApplicationContextScope thread_scope(app);
    return AssetManager::GetAsset<GpuPendingLoadAsset>(Handle(kGpuPendingAssetHandle));
  });
  EXPECT_EQ(sync_access.wait_for(100ms), std::future_status::timeout);

  ReleaseGpuPendingWork();
  ASSERT_TRUE(WaitForAssetFutureReadyWhilePumping(asset_future, 5s));
  const auto asset = std::dynamic_pointer_cast<GpuPendingLoadAsset>(asset_future.get());
  ASSERT_NE(asset, nullptr);
  EXPECT_EQ(sync_access.get(), asset);

  const auto final_snapshot = AssetManager::GetAssetLoadSnapshot();
  EXPECT_FALSE(final_snapshot.Active());
  EXPECT_EQ(final_snapshot.completed, 1);

  auto& state = GetGpuPendingLoadState();
  std::lock_guard lock(state.mutex);
  EXPECT_EQ(state.payload_load_count, 1);
  EXPECT_EQ(state.finalize_count, 1);
  EXPECT_EQ(state.gpu_work_start_count, 1);
  EXPECT_EQ(state.gpu_work_complete_count, 1);
  EXPECT_TRUE(state.gpu_work_ran_on_background);
}

TEST(AssetManager, AsyncStagedLoadPropagatesGpuReadinessFailure) {
  ResetGpuPendingLoadState(true);
  TempProject project;
  WriteGpuPendingAssetFixture(project);

  Application app;
  ApplicationContextScope scope(app);
  app.RegisterAsset<GpuPendingLoadAsset>(kGpuPendingAssetTypeName, {kGpuPendingAssetExtension});
  ASSERT_TRUE(GpuPendingLoadAsset::RegisterAssetIoHandlers());
  auto settings = TestApplicationSettings(project);
  settings.load_project_assets = false;
  app.Initialize(settings);

  auto asset_future = AssetManager::RequestAssetLoad(Handle(kGpuPendingAssetHandle));
  const bool gpu_work_started = WaitForGpuPendingWorkStartedWhilePumping(5s);
  if (!gpu_work_started) {
    ReleaseGpuPendingWork();
    FAIL() << "Timed out waiting for simulated GPU readiness work to start.";
  }

  ReleaseGpuPendingWork();
  ASSERT_TRUE(WaitForAssetFutureReadyWhilePumping(asset_future, 5s));
  EXPECT_THROW((void)asset_future.get(), std::runtime_error);

  const auto final_snapshot = AssetManager::GetAssetLoadSnapshot();
  EXPECT_FALSE(final_snapshot.Active());
  EXPECT_EQ(final_snapshot.failed, 1);
}
