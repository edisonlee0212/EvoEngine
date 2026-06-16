#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "AssetManager.hpp"
#include "FileManager.hpp"
#include "IAsset.hpp"
#include "Jobs.hpp"
#include "ProjectManager.hpp"
#include "Serialization.hpp"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

using namespace evo_engine;

namespace {
constexpr uint64_t kBinaryAssetHandle = 0xE702'0000'0000'0001ull;
constexpr uint64_t kThumbnailProbeAssetHandle = 0xE702'0000'0000'0002ull;
constexpr uint64_t kNativePrefabAssetHandle = 0xE702'0000'0000'0003ull;
constexpr uint64_t kSourceModelPrefabAssetHandle = 0xE702'0000'0000'0004ull;
constexpr auto kThumbnailProbeAssetTypeName = "ThumbnailProbeAsset";
constexpr auto kThumbnailProbeAssetExtension = ".evethumbnailprobe";

std::atomic<size_t>& ThumbnailProbeLoadCount() {
  static std::atomic<size_t> count{0};
  return count;
}

class ThumbnailProbeAsset final : public IAsset {
 public:
  static bool RegisterAssetIoHandlers() {
    return Serialization::RegisterAssetIoHandler<ThumbnailProbeAsset>(
        {},
        [](ThumbnailProbeAsset&, const std::filesystem::path&) {
          ++ThumbnailProbeLoadCount();
          return true;
        },
        {}, {}, {}, {}, kThumbnailProbeAssetTypeName);
  }
};

class TempFileManagerProject {
 public:
  TempFileManagerProject() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineFileManagerTest_" + std::to_string(now));
    std::filesystem::create_directories(AssetsPath());
  }

  ~TempFileManagerProject() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path ProjectPath() const {
    return root_ / "FileManagerTest.eveproj";
  }

  [[nodiscard]] std::filesystem::path AssetsPath() const {
    return root_ / "Assets";
  }

  [[nodiscard]] std::filesystem::path BinaryAssetPath() const {
    return AssetsPath() / "Source.bin";
  }

 private:
  std::filesystem::path root_;
};

ApplicationInitializationSettings TestApplicationSettings(const TempFileManagerProject& project) {
  ApplicationInitializationSettings settings;
  settings.project_path = project.ProjectPath();
  settings.load_default_resources = false;
  settings.load_project_start_scene = false;
  settings.enable_runtime_packages = false;
  return settings;
}

void WriteBinaryAssetFixture(const TempFileManagerProject& project) {
  std::ofstream asset_file(project.BinaryAssetPath());
  asset_file << "binary payload";
  asset_file.close();

  std::ofstream metadata_file(project.BinaryAssetPath().string() + ".evefilemeta");
  metadata_file << "asset_extension_: .bin\n";
  metadata_file << "asset_file_name_: Source\n";
  metadata_file << "asset_type_name_: Binary\n";
  metadata_file << "asset_handle_: " << kBinaryAssetHandle << "\n";
}

void WriteThumbnailProbeAssetFixture(const TempFileManagerProject& project) {
  const auto asset_path = project.AssetsPath() / ("ThumbnailProbe" + std::string(kThumbnailProbeAssetExtension));
  std::ofstream asset_file(asset_path);
  asset_file << "thumbnail probe payload";
  asset_file.close();

  std::ofstream metadata_file(asset_path.string() + ".evefilemeta");
  metadata_file << "asset_extension_: " << kThumbnailProbeAssetExtension << "\n";
  metadata_file << "asset_file_name_: ThumbnailProbe\n";
  metadata_file << "asset_type_name_: " << kThumbnailProbeAssetTypeName << "\n";
  metadata_file << "asset_handle_: " << kThumbnailProbeAssetHandle << "\n";
}

void WritePrefabFixture(const TempFileManagerProject& project, const std::string& file_name,
                        const std::string& extension, const uint64_t handle) {
  const auto asset_path = project.AssetsPath() / (file_name + extension);
  std::ofstream asset_file(asset_path);
  if (extension == ".eveprefab") {
    asset_file << "in: " << file_name << "\n";
    asset_file << "e: true\n";
    asset_file << "eh: 0\n";
  } else {
    asset_file << "o " << file_name << "\n";
    asset_file << "v 0 0 0\n";
    asset_file << "v 1 0 0\n";
    asset_file << "v 0 1 0\n";
    asset_file << "f 1 2 3\n";
  }
  asset_file.close();

  std::ofstream metadata_file(asset_path.string() + ".evefilemeta");
  metadata_file << "asset_extension_: " << extension << "\n";
  metadata_file << "asset_file_name_: " << file_name << "\n";
  metadata_file << "asset_type_name_: Prefab\n";
  metadata_file << "asset_handle_: " << handle << "\n";
}

bool WaitForThumbnailProbeLoad(const std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    AssetManager::ExecuteMainThreadAssetTasks(1);
    Jobs::ExecuteMainThreadJobs(1);
    if (ThumbnailProbeLoadCount().load() != 0) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

void OpenProject(Application& app, const TempFileManagerProject& project) {
  app.Initialize(TestApplicationSettings(project));
  ASSERT_TRUE(ProjectManager::GetAssetsFolder());
}
}  // namespace

TEST(FileManager, MissingMutationHandlesAreNoOps) {
  TempFileManagerProject project;
  Application app;
  ApplicationContextScope scope(app);
  OpenProject(app, project);

  const auto root_folder = ProjectManager::GetAssetsFolder();
  const Handle missing_handle(0xE702'0000'0000'00FFull);

  EXPECT_NO_THROW(root_folder->RemoveFile(missing_handle));
  EXPECT_NO_THROW(root_folder->DeleteChild(missing_handle));
  EXPECT_NO_THROW({
    const auto duplicate = root_folder->Duplicate(missing_handle);
    EXPECT_FALSE(duplicate);
  });
  EXPECT_FALSE(FileManager::GetFile(missing_handle));
  EXPECT_FALSE(FileManager::GetFolder(missing_handle));
}

TEST(FileManager, DeleteChildRemovesFolderFromDiskAndRegistry) {
  TempFileManagerProject project;
  Application app;
  ApplicationContextScope scope(app);
  OpenProject(app, project);

  const auto root_folder = ProjectManager::GetAssetsFolder();
  const auto child_folder = ProjectManager::GetOrCreateFolder("Child").lock();
  ASSERT_TRUE(child_folder);
  const auto child_handle = child_folder->GetHandle();
  ASSERT_TRUE(FileManager::GetFolder(child_handle));

  root_folder->DeleteChild(child_handle);

  EXPECT_FALSE(std::filesystem::exists(project.AssetsPath() / "Child"));
  EXPECT_FALSE(std::filesystem::exists(project.AssetsPath() / "Child.evefoldermeta"));
  EXPECT_FALSE(FileManager::GetFolder(child_handle));
}

TEST(FileManager, DeleteChildRemovesDottedFolderMetadata) {
  TempFileManagerProject project;
  Application app;
  ApplicationContextScope scope(app);
  OpenProject(app, project);

  const auto root_folder = ProjectManager::GetAssetsFolder();
  const auto child_folder = ProjectManager::GetOrCreateFolder("Child.v1").lock();
  ASSERT_TRUE(child_folder);
  const auto child_handle = child_folder->GetHandle();
  ASSERT_TRUE(std::filesystem::exists(project.AssetsPath() / "Child.v1.evefoldermeta"));

  root_folder->DeleteChild(child_handle);

  EXPECT_FALSE(std::filesystem::exists(project.AssetsPath() / "Child.v1.evefoldermeta"));
  EXPECT_FALSE(std::filesystem::exists(project.AssetsPath() / "Child.evefoldermeta"));
}

TEST(FileManager, RemoveFileDeletesAssetAndMetadataFromDiskAndRegistry) {
  TempFileManagerProject project;
  WriteBinaryAssetFixture(project);
  Application app;
  ApplicationContextScope scope(app);
  OpenProject(app, project);

  const Handle asset_handle(kBinaryAssetHandle);
  const auto root_folder = ProjectManager::GetAssetsFolder();
  ASSERT_TRUE(FileManager::GetFile(asset_handle));

  root_folder->RemoveFile(asset_handle);

  EXPECT_FALSE(std::filesystem::exists(project.BinaryAssetPath()));
  EXPECT_FALSE(std::filesystem::exists(project.BinaryAssetPath().string() + ".evefilemeta"));
  EXPECT_FALSE(FileManager::GetFile(asset_handle));
}

TEST(FileManager, DuplicateBinaryAssetRegistersCopiedFile) {
  TempFileManagerProject project;
  WriteBinaryAssetFixture(project);
  Application app;
  ApplicationContextScope scope(app);
  OpenProject(app, project);

  const auto root_folder = ProjectManager::GetAssetsFolder();
  ASSERT_TRUE(FileManager::GetFile(Handle(kBinaryAssetHandle)));

  const auto duplicate = root_folder->Duplicate(Handle(kBinaryAssetHandle));

  EXPECT_FALSE(duplicate);
  const auto copied_asset_path = project.AssetsPath() / "Source (1).bin";
  const auto copied_metadata_path = copied_asset_path.string() + ".evefilemeta";
  ASSERT_TRUE(std::filesystem::exists(copied_asset_path));
  ASSERT_TRUE(std::filesystem::exists(copied_metadata_path));
  const auto copied_metadata = YAML::LoadFile(copied_metadata_path);
  const auto copied_handle = Handle(copied_metadata["asset_handle_"].as<uint64_t>());
  const auto copied_file = FileManager::GetFile(copied_handle);
  ASSERT_TRUE(copied_file);
  EXPECT_EQ(copied_file->GetAssetFileName(), "Source (1)");
  EXPECT_EQ(copied_file->GetAssetExtension(), ".bin");
}

TEST(FileManager, ProjectScanDefersSourceModelPrefabAutoLoad) {
  TempFileManagerProject project;
  WritePrefabFixture(project, "NativePrefab", ".eveprefab", kNativePrefabAssetHandle);
  WritePrefabFixture(project, "SourceModel", ".obj", kSourceModelPrefabAssetHandle);

  Application app;
  ApplicationContextScope scope(app);
  OpenProject(app, project);

  ASSERT_TRUE(FileManager::GetFile(Handle(kNativePrefabAssetHandle)));
  ASSERT_TRUE(FileManager::GetFile(Handle(kSourceModelPrefabAssetHandle)));
  const auto snapshot = AssetManager::GetAssetLoadSnapshot();
  EXPECT_EQ(snapshot.total, 1);
  EXPECT_EQ(snapshot.completed + snapshot.failed + snapshot.cancelled, 1);
}

TEST(FileManager, ThumbnailLookupCanAvoidStartingAssetLoad) {
  ThumbnailProbeLoadCount() = 0;
  TempFileManagerProject project;
  WriteThumbnailProbeAssetFixture(project);

  Application app;
  ApplicationContextScope scope(app);
  app.RegisterAsset<ThumbnailProbeAsset>(kThumbnailProbeAssetTypeName, {kThumbnailProbeAssetExtension});
  ASSERT_TRUE(ThumbnailProbeAsset::RegisterAssetIoHandlers());
  ASSERT_TRUE(Serialization::RegisterAssetPreviewHandler<ThumbnailProbeAsset>(
      [](const std::shared_ptr<ThumbnailProbeAsset>&, const OffscreenPreviewSettings&) {
        return std::shared_ptr<Texture2D>();
      },
      {}, kThumbnailProbeAssetTypeName));
  auto settings = TestApplicationSettings(project);
  settings.load_project_assets = false;
  app.Initialize(settings);

  const auto file = FileManager::GetFile(Handle(kThumbnailProbeAssetHandle));
  ASSERT_TRUE(file);

  (void)file->GetThumbnail(false);
  for (size_t i = 0; i < 8; ++i) {
    AssetManager::ExecuteMainThreadAssetTasks(1);
    Jobs::ExecuteMainThreadJobs(1);
  }
  EXPECT_EQ(ThumbnailProbeLoadCount().load(), 0);

  (void)file->GetThumbnail(true);
  EXPECT_TRUE(WaitForThumbnailProbeLoad(std::chrono::seconds(5)));
}

TEST(ProjectManager, CreateFolderGeneratesUniqueChildFolders) {
  TempFileManagerProject project;
  Application app;
  ApplicationContextScope scope(app);
  OpenProject(app, project);

  const auto root_folder = ProjectManager::GetAssetsFolder();
  const auto first_folder = ProjectManager::CreateFolder(root_folder, "New Folder");
  const auto second_folder = ProjectManager::CreateFolder(root_folder, "New Folder");

  ASSERT_TRUE(first_folder);
  ASSERT_TRUE(second_folder);
  EXPECT_EQ(first_folder->GetName(), "New Folder");
  EXPECT_EQ(second_folder->GetName(), "New Folder (1)");
  EXPECT_TRUE(std::filesystem::exists(project.AssetsPath() / "New Folder"));
  EXPECT_TRUE(std::filesystem::exists(project.AssetsPath() / "New Folder (1)"));
}

TEST(ProjectManager, MoveFolderMovesChildFolderAndRejectsDescendantDestination) {
  TempFileManagerProject project;
  Application app;
  ApplicationContextScope scope(app);
  OpenProject(app, project);

  const auto root_folder = ProjectManager::GetAssetsFolder();
  const auto source_folder = ProjectManager::CreateFolder(root_folder, "Source");
  const auto child_folder = ProjectManager::CreateFolder(source_folder, "Child");
  const auto destination_folder = ProjectManager::CreateFolder(root_folder, "Destination");
  ASSERT_TRUE(source_folder);
  ASSERT_TRUE(child_folder);
  ASSERT_TRUE(destination_folder);

  EXPECT_FALSE(ProjectManager::MoveFolder(source_folder->GetHandle(), child_folder));
  ASSERT_TRUE(ProjectManager::MoveFolder(source_folder->GetHandle(), destination_folder));

  EXPECT_FALSE(std::filesystem::exists(project.AssetsPath() / "Source"));
  EXPECT_TRUE(std::filesystem::exists(project.AssetsPath() / "Destination" / "Source"));
  EXPECT_TRUE(FileManager::GetFolder(source_folder->GetHandle()));
}

TEST(ProjectManager, MoveAssetMovesBinaryFileToDestinationFolder) {
  TempFileManagerProject project;
  WriteBinaryAssetFixture(project);
  Application app;
  ApplicationContextScope scope(app);
  OpenProject(app, project);

  const auto destination_folder = ProjectManager::CreateFolder(ProjectManager::GetAssetsFolder(), "Destination");
  ASSERT_TRUE(destination_folder);
  const Handle asset_handle(kBinaryAssetHandle);

  ASSERT_TRUE(ProjectManager::MoveAsset(asset_handle, destination_folder));

  EXPECT_FALSE(std::filesystem::exists(project.BinaryAssetPath()));
  EXPECT_TRUE(std::filesystem::exists(project.AssetsPath() / "Destination" / "Source.bin"));
  const auto moved_file = FileManager::GetFile(asset_handle);
  ASSERT_TRUE(moved_file);
  EXPECT_EQ(moved_file->GetFolder().lock(), destination_folder);
}

TEST(ProjectManager, DeleteAssetAndFolderRemoveProjectItems) {
  TempFileManagerProject project;
  WriteBinaryAssetFixture(project);
  Application app;
  ApplicationContextScope scope(app);
  OpenProject(app, project);

  const auto folder = ProjectManager::CreateFolder(ProjectManager::GetAssetsFolder(), "DeleteMe");
  ASSERT_TRUE(folder);
  const auto folder_handle = folder->GetHandle();
  const Handle asset_handle(kBinaryAssetHandle);

  EXPECT_TRUE(ProjectManager::DeleteAsset(asset_handle));
  EXPECT_FALSE(std::filesystem::exists(project.BinaryAssetPath()));
  EXPECT_FALSE(FileManager::GetFile(asset_handle));

  EXPECT_TRUE(ProjectManager::DeleteFolder(folder_handle));
  EXPECT_FALSE(std::filesystem::exists(project.AssetsPath() / "DeleteMe"));
  EXPECT_FALSE(FileManager::GetFolder(folder_handle));
}
