#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "AssetManager.hpp"
#include "IAsset.hpp"
#include "Jobs.hpp"

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <thread>

using namespace evo_engine;
using namespace std::chrono_literals;

namespace {
constexpr uint64_t kBlockingAssetHandle = 0xE701'0000'0000'0001ull;
constexpr auto kBlockingAssetTypeName = "BlockingLoadAsset";
constexpr auto kBlockingAssetExtension = ".evetestasset";
constexpr uint64_t kStagedAssetHandle = 0xE701'0000'0000'0002ull;
constexpr auto kStagedAssetTypeName = "StagedLoadAsset";
constexpr auto kStagedAssetExtension = ".evestagedasset";

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
 protected:
  bool LoadInternal(const std::filesystem::path&) override {
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
 protected:
  bool LoadInternal(const std::filesystem::path&) override {
    auto& state = GetStagedLoadState();
    std::lock_guard lock(state.mutex);
    ++state.sync_load_count;
    return true;
  }

  [[nodiscard]] bool SupportsStagedLoading() const override {
    return true;
  }

  [[nodiscard]] std::shared_ptr<StagedAssetLoadPayload> LoadStagedPayloadInternal(
      const std::filesystem::path&) const override {
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

  bool ApplyStagedPayloadInternal(const std::filesystem::path&,
                                  const std::shared_ptr<StagedAssetLoadPayload>& payload) override {
    const auto staged_payload = std::dynamic_pointer_cast<StagedLoadPayload>(payload);
    auto& state = GetStagedLoadState();
    std::lock_guard lock(state.mutex);
    ++state.finalize_count;
    state.finalize_ran_on_main_thread = Jobs::IsMainThread();
    return staged_payload && staged_payload->value == 42;
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

ApplicationInitializationSettings TestApplicationSettings(const TempProject& project) {
  ApplicationInitializationSettings settings;
  settings.project_path = project.ProjectPath();
  settings.load_default_resources = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  return settings;
}
}  // namespace

TEST(AssetManager, BlockingAccessJoinsInFlightSynchronousProjectLoad) {
  ResetBlockingLoadState();
  TempProject project;
  WriteBlockingAssetFixture(project);

  Application app;
  app.RegisterAsset<BlockingLoadAsset>(kBlockingAssetTypeName, {kBlockingAssetExtension});

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
    FAIL() << "Timed out waiting for the fixture asset to enter LoadInternal().";
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
