#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "AppBootstrap.hpp"
#include "DemoProfiles.hpp"
#include "LauncherUtils.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>

using namespace evo_engine;

namespace {
class TempLauncherDirectory {
 public:
  TempLauncherDirectory() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineLauncherTest_" + std::to_string(now));
    std::filesystem::create_directories(root_);
  }

  ~TempLauncherDirectory() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path RootPath() const {
    return root_;
  }

  [[nodiscard]] std::filesystem::path WriteProject(const std::string& name) const {
    const auto path = root_ / (name + ".eveproj");
    std::ofstream project_file(path);
    project_file << "application_name: " << name << "\n";
    return path;
  }

 private:
  std::filesystem::path root_;
};

AvailablePackageInfo PackageInfo(const std::string& name, const bool library_exists) {
  AvailablePackageInfo package;
  package.name = name;
  package.library_exists = library_exists;
  return package;
}
}  // namespace

TEST(LauncherUtils, PackageAvailabilityTracksManifestAndLibraryState) {
  const auto availability =
      launcher::BuildPackageAvailability({PackageInfo("LSystem", true), PackageInfo("DigitalAgriculture", false)});

  EXPECT_TRUE(launcher::IsPackageAvailable(availability, "LSystem"));
  EXPECT_FALSE(launcher::IsPackageAvailable(availability, "DigitalAgriculture"));
  EXPECT_FALSE(launcher::IsPackageAvailable(availability, "MissingPackage"));
}

TEST(LauncherUtils, SelectedRuntimePackagesRequireAllPackages) {
  const std::vector<std::string> selected_packages{"LSystem", "DigitalAgriculture"};
  const auto full_availability =
      launcher::BuildPackageAvailability({PackageInfo("LSystem", true), PackageInfo("DigitalAgriculture", true)});
  const auto missing_manifest = launcher::BuildPackageAvailability({PackageInfo("LSystem", true)});
  const auto missing_library =
      launcher::BuildPackageAvailability({PackageInfo("LSystem", true), PackageInfo("DigitalAgriculture", false)});

  EXPECT_TRUE(launcher::ArePackagesAvailable(full_availability, selected_packages));
  EXPECT_FALSE(launcher::ArePackagesAvailable(missing_manifest, selected_packages));
  EXPECT_FALSE(launcher::ArePackagesAvailable(missing_library, selected_packages));
  EXPECT_EQ(launcher::MissingPackages(missing_library, selected_packages),
            std::vector<std::string>{"DigitalAgriculture"});
}

TEST(LauncherUtils, BuildsProjectMetadataFromSelectedPackages) {
  const std::vector<std::string> selected_packages{"LSystem", "DigitalAgriculture"};
  const auto metadata = launcher::BuildProjectLaunchMetadata("NewProject", selected_packages);

  EXPECT_EQ(metadata.application_name, "NewProject");
  EXPECT_EQ(metadata.preferred_editor, "EvoEngineEditor");
  EXPECT_EQ(metadata.startup_runtime_packages, selected_packages);
}

TEST(LauncherUtils, DemoProfilesExposeStableIdsAndPackageRequirements) {
  const auto& profiles = GetDemoProfiles();
  ASSERT_EQ(profiles.size(), 5);

  EXPECT_EQ(profiles[0].id, DemoProfileId::Rendering);
  EXPECT_STREQ(profiles[0].id_name, "rendering");
  EXPECT_EQ(profiles[0].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[0].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[0].id, ApplicationMode::Player));
  EXPECT_TRUE(profiles[0].startup_runtime_packages.empty());
  EXPECT_STREQ(profiles[0].preview_image_path, "Launcher/DemoPreviews/rendering.png");
  EXPECT_EQ(profiles[1].id, DemoProfileId::Ddgi);
  EXPECT_STREQ(profiles[1].id_name, "ddgi");
  EXPECT_EQ(profiles[1].default_application_mode, ApplicationMode::Player);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[1].id, ApplicationMode::Player));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[1].id, ApplicationMode::Editor));
  EXPECT_TRUE(profiles[1].startup_runtime_packages.empty());
  EXPECT_STREQ(profiles[1].preview_image_path, "Launcher/DemoPreviews/ddgi.png");
  EXPECT_EQ(profiles[2].id, DemoProfileId::EcoSysLab);
  EXPECT_STREQ(profiles[2].id_name, "ecosyslab");
  EXPECT_EQ(profiles[2].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[2].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[2].id, ApplicationMode::Player));
  EXPECT_EQ(profiles[2].startup_runtime_packages, std::vector<std::string>{"EcoSysLab"});
  EXPECT_STREQ(profiles[2].preview_image_path, "Launcher/DemoPreviews/ecosyslab.png");
  EXPECT_EQ(profiles[3].id, DemoProfileId::DigitalAgriculture);
  EXPECT_STREQ(profiles[3].id_name, "digital-agriculture");
  EXPECT_EQ(profiles[3].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[3].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[3].id, ApplicationMode::Player));
  EXPECT_EQ(profiles[3].startup_runtime_packages, std::vector<std::string>{"DigitalAgriculture"});
  EXPECT_STREQ(profiles[3].preview_image_path, "Launcher/DemoPreviews/digital-agriculture.png");
  EXPECT_EQ(profiles[4].id, DemoProfileId::LSystem);
  EXPECT_STREQ(profiles[4].id_name, "lsystem");
  EXPECT_EQ(profiles[4].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[4].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[4].id, ApplicationMode::Player));
  EXPECT_EQ(profiles[4].startup_runtime_packages, (std::vector<std::string>{"LSystem", "DigitalAgriculture"}));
  EXPECT_STREQ(profiles[4].preview_image_path, "Launcher/DemoPreviews/lsystem.png");

  ASSERT_NE(FindDemoProfile("rendering"), nullptr);
  EXPECT_EQ(FindDemoProfile("rendering")->id, DemoProfileId::Rendering);
  EXPECT_EQ(FindDemoProfile("missing"), nullptr);
  EXPECT_STREQ(GetDemoProfileIdName(DemoProfileId::LSystem), "lsystem");
}

TEST(LauncherUtils, DemoProfileProjectPathsResolveFromResourcesRoot) {
  TempLauncherDirectory temp;
  const auto resource_root = temp.RootPath() / "Resources";
  std::filesystem::create_directories(resource_root / "LSystemProjectAssets");
  const auto lsystem_fallback = resource_root / "LSystemProjectAssets" / "test.eveproj";
  std::ofstream project_file(lsystem_fallback);
  project_file << "application_name: LSystem\n";
  project_file.close();

  EXPECT_EQ(
      ResolveDemoProfileProjectPath(DemoProfileId::Rendering, resource_root),
      launcher::NormalizeProjectPath(resource_root / "EvoEngine-DemoProjects" / "Rendering" / "Rendering.eveproj"));
  EXPECT_EQ(
      ResolveDemoProfileProjectPath(DemoProfileId::Ddgi, resource_root),
      launcher::NormalizeProjectPath(resource_root / "EvoEngine-DemoProjects" / "CornellBox" / "CornellBox.eveproj"));
  EXPECT_EQ(ResolveDemoProfileProjectPath(DemoProfileId::EcoSysLab, resource_root),
            launcher::NormalizeProjectPath(resource_root / "EcoSysLabProject" / "test.eveproj"));
  EXPECT_EQ(ResolveDemoProfileProjectPath(DemoProfileId::DigitalAgriculture, resource_root),
            launcher::NormalizeProjectPath(resource_root / "DigitalAgricultureProject" / "test.eveproj"));
  EXPECT_EQ(ResolveDemoProfileProjectPath(DemoProfileId::LSystem, resource_root),
            launcher::NormalizeProjectPath(lsystem_fallback));
}

TEST(LauncherUtils, DemoResourceChecksRequireProjectsOnlyForPackageProfiles) {
  TempLauncherDirectory temp;
  const auto resource_root = temp.RootPath() / "Resources";
  std::filesystem::create_directories(resource_root);

  EXPECT_TRUE(MissingDemoProfileResourceRequirements(DemoProfileId::Rendering, resource_root).empty());
  EXPECT_TRUE(MissingDemoProfileResourceRequirements(DemoProfileId::Ddgi, resource_root).empty());
  EXPECT_FALSE(MissingDemoProfileResourceRequirements(DemoProfileId::EcoSysLab, resource_root).empty());

  std::filesystem::create_directories(resource_root / "EcoSysLabProject");
  std::ofstream project_file(resource_root / "EcoSysLabProject" / "test.eveproj");
  project_file << "application_name: EcoSysLab\n";
  project_file.close();
  EXPECT_TRUE(MissingDemoProfileResourceRequirements(DemoProfileId::EcoSysLab, resource_root).empty());
}

TEST(LauncherUtils, ApplicationModeNamesAndArgumentsAreStable) {
  EXPECT_EQ(ParseApplicationModeName("Editor"), ApplicationMode::Editor);
  EXPECT_EQ(ParseApplicationModeName("player"), ApplicationMode::Player);
  EXPECT_EQ(ParseApplicationModeName("Headless"), ApplicationMode::Headless);
  EXPECT_STREQ(GetApplicationModeName(ApplicationMode::Editor), "Editor");
  EXPECT_STREQ(GetApplicationModeName(ApplicationMode::Player), "Player");
  EXPECT_STREQ(GetApplicationModeName(ApplicationMode::Headless), "Headless");
  EXPECT_STREQ(GetApplicationModeArgument(ApplicationMode::Editor), "--editor");
  EXPECT_STREQ(GetApplicationModeArgument(ApplicationMode::Player), "--player");
  EXPECT_STREQ(GetApplicationModeArgument(ApplicationMode::Headless), "--headless");
}

TEST(LauncherUtils, ValidatesProjectNamesAndCreatePaths) {
  TempLauncherDirectory temp;
  const launcher::PackageAvailability availability;
  const auto metadata = launcher::BuildProjectLaunchMetadata("NewProject", {});
  const auto derived_path = launcher::BuildDerivedProjectPath(temp.RootPath(), "NewProject");

  EXPECT_EQ(launcher::Trim("  NewProject\t"), "NewProject");
  EXPECT_TRUE(launcher::ValidateCreateProjectRequest("NewProject", temp.RootPath(), derived_path.folder,
                                                     derived_path.project_file, metadata, availability)
                  .empty());
  EXPECT_FALSE(launcher::ValidateCreateProjectRequest("", temp.RootPath(), derived_path.folder,
                                                      derived_path.project_file, metadata, availability)
                   .empty());
  EXPECT_FALSE(launcher::ValidateCreateProjectRequest("Bad/Name", temp.RootPath(), derived_path.folder,
                                                      derived_path.project_file, metadata, availability)
                   .empty());
  EXPECT_FALSE(launcher::ValidateCreateProjectRequest("Bad*Name", temp.RootPath(), derived_path.folder,
                                                      derived_path.project_file, metadata, availability)
                   .empty());
  EXPECT_FALSE(launcher::ValidateCreateProjectRequest("NewProject", temp.RootPath() / "missing", derived_path.folder,
                                                      derived_path.project_file, metadata, availability)
                   .empty());

  std::filesystem::create_directories(derived_path.folder);
  EXPECT_EQ(launcher::ValidateCreateProjectRequest("NewProject", temp.RootPath(), derived_path.folder,
                                                   derived_path.project_file, metadata, availability),
            "Project folder already exists.");

  const auto existing_project_file = temp.WriteProject("ExistingProject");
  EXPECT_EQ(launcher::ValidateCreateProjectRequest("ExistingProject", temp.RootPath(), temp.RootPath() / "OtherFolder",
                                                   existing_project_file, metadata, availability),
            "Project file already exists.");
}

TEST(LauncherUtils, RecentProjectsAreNormalizedDeduplicatedPrunedAndCapped) {
  TempLauncherDirectory temp;
  std::vector<std::filesystem::path> valid_projects;
  for (int i = 0; i < 9; ++i) {
    valid_projects.emplace_back(temp.WriteProject("Project" + std::to_string(i)));
  }
  const auto settings_path = temp.RootPath() / "settings.yaml";
  {
    std::ofstream settings(settings_path);
    settings << "recent_projects:\n";
    settings << "  - " << valid_projects[0].string() << "\n";
    settings << "  - " << valid_projects[0].string() << "\n";
    settings << "  - " << (temp.RootPath() / "Missing.eveproj").string() << "\n";
    settings << "  - " << (temp.RootPath() / "Wrong.txt").string() << "\n";
    for (int i = 1; i < 9; ++i) {
      settings << "  - " << valid_projects[static_cast<size_t>(i)].string() << "\n";
    }
  }

  bool pruned = false;
  auto recent_projects = launcher::LoadRecentProjects(settings_path, pruned);
  EXPECT_TRUE(pruned);
  ASSERT_EQ(recent_projects.size(), launcher::kMaxRecentProjectCount);
  EXPECT_EQ(recent_projects.front(), launcher::NormalizeProjectPath(valid_projects[0]));
  EXPECT_EQ(
      std::count(recent_projects.begin(), recent_projects.end(), launcher::NormalizeProjectPath(valid_projects[0])), 1);

  launcher::AddRecentProject(recent_projects, valid_projects[8]);
  EXPECT_EQ(recent_projects.front(), launcher::NormalizeProjectPath(valid_projects[8]));
  EXPECT_EQ(recent_projects.size(), launcher::kMaxRecentProjectCount);

  launcher::SaveRecentProjects(settings_path, recent_projects);
  bool saved_pruned = false;
  const auto reloaded_projects = launcher::LoadRecentProjects(settings_path, saved_pruned);
  EXPECT_FALSE(saved_pruned);
  EXPECT_EQ(reloaded_projects, recent_projects);
}
