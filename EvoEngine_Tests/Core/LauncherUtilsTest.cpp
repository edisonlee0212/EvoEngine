#include "EvoEngine_SDK_PCH.hpp"

#include <gtest/gtest.h>

#include "AppBootstrap.hpp"
#include "DemoProfiles.hpp"
#include "LauncherUtils.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>

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

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}
}  // namespace

TEST(LauncherUtils, PackageAvailabilityTracksManifestAndLibraryState) {
  const auto availability =
      launcher::BuildPackageAvailability({PackageInfo("LSystem", true), PackageInfo("DigitalAgriculture", false)});

  EXPECT_TRUE(launcher::IsPackageAvailable(availability, "LSystem"));
  EXPECT_FALSE(launcher::IsPackageAvailable(availability, "DigitalAgriculture"));
  EXPECT_FALSE(launcher::IsPackageAvailable(availability, "MissingPackage"));
}

TEST(LauncherUtils, EcoSysLabDemoAnimatesEightYearAcaciaGrowth) {
  const auto source = ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) /
                                   "EvoEngine_Packages/EcoSysLab/src/EcoSysLabLayer.cpp");
  const auto growth_start = source.find("void EcoSysLabLayer::UpdateDemoTreeGrowth()");
  const auto growth_end = source.find("void EcoSysLabLayer::Update()", growth_start);
  ASSERT_NE(growth_start, std::string::npos);
  ASSERT_NE(growth_end, std::string::npos);
  const auto growth_source = source.substr(growth_start, growth_end - growth_start);

  EXPECT_NE(source.find("project_path.parent_path().filename() != \"EcoSysLabProject\""), std::string::npos);
  EXPECT_NE(source.find("TreeDescriptors/Basic/Acacia.tree"), std::string::npos);
  EXPECT_NE(growth_source.find("IsPlantVisualSplitLayoutReady()"), std::string::npos);
  EXPECT_NE(growth_source.find("8.0f * 365.0f"), std::string::npos);
  const auto simulate_call = growth_source.find("Simulate(growth_settings, simulation_stats)");
  ASSERT_NE(simulate_call, std::string::npos);
  EXPECT_EQ(growth_source.find("Simulate(growth_settings, simulation_stats)", simulate_call + 1), std::string::npos);
  EXPECT_EQ(growth_source.find("for (float elapsed_time"), std::string::npos);
  EXPECT_NE(growth_source.find("demo_mesh_generator_settings.foliage_instancing = false"), std::string::npos);
  EXPECT_NE(growth_source.find("GenerateGeometryEntities(demo_mesh_generator_settings)"), std::string::npos);
  EXPECT_NE(source.find("UpdateDemoTreeGrowth();"), std::string::npos);
}

TEST(LauncherUtils, EcoSysLabDemoSplitsSceneAndPlantVisual) {
  const auto app_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App/src/DemoProfiles.cpp");
  const auto editor_header =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK/include/Layers/EditorLayer.hpp");
  const auto editor_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK/src/EditorLayer.cpp");

  EXPECT_NE(editor_header.find("std::optional<float> plant_visual_fraction"), std::string::npos);
  EXPECT_NE(editor_header.find("bool IsPlantVisualSplitLayoutReady() const"), std::string::npos);
  EXPECT_NE(app_source.find("dock_layout.plant_visual_fraction = 0.50f"), std::string::npos);
  EXPECT_NE(app_source.find("RequestEditorLayout(CreateEcoSysLabDemoEditorLayout())"), std::string::npos);
  EXPECT_NE(editor_source.find("DockBuilderDockWindow(\"Plant Visual\", plant_visual_node)"), std::string::npos);
  EXPECT_NE(editor_source.find("plant_visual_fraction.has_value()"), std::string::npos);
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
  ASSERT_EQ(profiles.size(), 10);

  EXPECT_EQ(profiles[0].id, DemoProfileId::Rendering);
  EXPECT_STREQ(profiles[0].id_name, "rendering");
  EXPECT_EQ(profiles[0].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[0].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[0].id, ApplicationMode::Player));
  EXPECT_TRUE(profiles[0].startup_runtime_packages.empty());
  EXPECT_STREQ(profiles[0].preview_image_path, "Launcher/DemoPreviews/rendering.png");
  EXPECT_EQ(profiles[1].id, DemoProfileId::ProceduralGalaxy);
  EXPECT_STREQ(profiles[1].id_name, "procedural-galaxy");
  EXPECT_EQ(profiles[1].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[1].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[1].id, ApplicationMode::Player));
  EXPECT_EQ(profiles[1].startup_runtime_packages, std::vector<std::string>{"Universe"});
  EXPECT_STREQ(profiles[1].preview_image_path, "Launcher/DemoPreviews/procedural-galaxy.png");
  EXPECT_EQ(profiles[2].id, DemoProfileId::GaussianSplat);
  EXPECT_STREQ(profiles[2].id_name, "3dgs");
  EXPECT_EQ(profiles[2].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[2].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[2].id, ApplicationMode::Player));
  EXPECT_TRUE(profiles[2].startup_runtime_packages.empty());
  EXPECT_STREQ(profiles[2].preview_image_path, "Launcher/DemoPreviews/3dgs.png");
  EXPECT_EQ(profiles[3].id, DemoProfileId::Bicycle);
  EXPECT_STREQ(profiles[3].id_name, "bicycle");
  EXPECT_EQ(profiles[3].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[3].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[3].id, ApplicationMode::Player));
  EXPECT_TRUE(profiles[3].startup_runtime_packages.empty());
  EXPECT_STREQ(profiles[3].preview_image_path, "Launcher/DemoPreviews/bicycle.png");
  EXPECT_EQ(profiles[4].id, DemoProfileId::Bistro);
  EXPECT_STREQ(profiles[4].id_name, "bistro");
  EXPECT_EQ(profiles[4].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[4].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[4].id, ApplicationMode::Player));
  EXPECT_TRUE(profiles[4].startup_runtime_packages.empty());
  EXPECT_STREQ(profiles[4].preview_image_path, "Launcher/DemoPreviews/bistro.png");
  EXPECT_EQ(profiles[5].id, DemoProfileId::EcoSysLab);
  EXPECT_STREQ(profiles[5].id_name, "ecosyslab");
  EXPECT_EQ(profiles[5].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[5].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[5].id, ApplicationMode::Player));
  EXPECT_EQ(profiles[5].startup_runtime_packages, std::vector<std::string>{"EcoSysLab"});
  EXPECT_STREQ(profiles[5].preview_image_path, "Launcher/DemoPreviews/ecosyslab.png");
  EXPECT_EQ(profiles[6].id, DemoProfileId::LSystem);
  EXPECT_STREQ(profiles[6].id_name, "lsystem");
  EXPECT_EQ(profiles[6].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[6].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[6].id, ApplicationMode::Player));
  EXPECT_EQ(profiles[6].startup_runtime_packages, (std::vector<std::string>{"LSystem", "DigitalAgriculture"}));
  EXPECT_STREQ(profiles[6].preview_image_path, "Launcher/DemoPreviews/lsystem.png");
  EXPECT_EQ(profiles[7].id, DemoProfileId::DigitalAgriculture);
  EXPECT_STREQ(profiles[7].id_name, "digital-agriculture");
  EXPECT_EQ(profiles[7].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[7].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[7].id, ApplicationMode::Player));
  EXPECT_EQ(profiles[7].startup_runtime_packages, std::vector<std::string>{"DigitalAgriculture"});
  EXPECT_STREQ(profiles[7].preview_image_path, "Launcher/DemoPreviews/digital-agriculture.png");
  EXPECT_EQ(profiles[8].id, DemoProfileId::Ddgi);
  EXPECT_STREQ(profiles[8].id_name, "ddgi");
  EXPECT_EQ(profiles[8].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[8].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[8].id, ApplicationMode::Player));
  EXPECT_TRUE(profiles[8].startup_runtime_packages.empty());
  EXPECT_STREQ(profiles[8].preview_image_path, "Launcher/DemoPreviews/ddgi.png");
  EXPECT_EQ(profiles[9].id, DemoProfileId::RenderingRegression);
  EXPECT_STREQ(profiles[9].id_name, "rendering-regression");
  EXPECT_EQ(profiles[9].default_application_mode, ApplicationMode::Editor);
  EXPECT_TRUE(IsDemoProfileApplicationModeSupported(profiles[9].id, ApplicationMode::Editor));
  EXPECT_FALSE(IsDemoProfileApplicationModeSupported(profiles[9].id, ApplicationMode::Player));
  EXPECT_TRUE(profiles[9].startup_runtime_packages.empty());
  EXPECT_STREQ(profiles[9].preview_image_path, "Launcher/DemoPreviews/rendering-regression.png");

  ASSERT_NE(FindDemoProfile("rendering"), nullptr);
  EXPECT_EQ(FindDemoProfile("rendering")->id, DemoProfileId::Rendering);
  ASSERT_NE(FindDemoProfile("rendering-regression"), nullptr);
  EXPECT_EQ(FindDemoProfile("rendering-regression")->id, DemoProfileId::RenderingRegression);
  ASSERT_NE(FindDemoProfile("procedural-galaxy"), nullptr);
  EXPECT_EQ(FindDemoProfile("procedural-galaxy")->id, DemoProfileId::ProceduralGalaxy);
  ASSERT_NE(FindDemoProfile("3dgs"), nullptr);
  EXPECT_EQ(FindDemoProfile("3dgs")->id, DemoProfileId::GaussianSplat);
  ASSERT_NE(FindDemoProfile("bicycle"), nullptr);
  EXPECT_EQ(FindDemoProfile("bicycle")->id, DemoProfileId::Bicycle);
  ASSERT_NE(FindDemoProfile("bistro"), nullptr);
  EXPECT_EQ(FindDemoProfile("bistro")->id, DemoProfileId::Bistro);
  EXPECT_EQ(FindDemoProfile("missing"), nullptr);
  EXPECT_STREQ(GetDemoProfileIdName(DemoProfileId::RenderingRegression), "rendering-regression");
  EXPECT_STREQ(GetDemoProfileIdName(DemoProfileId::ProceduralGalaxy), "procedural-galaxy");
  EXPECT_STREQ(GetDemoProfileIdName(DemoProfileId::GaussianSplat), "3dgs");
  EXPECT_STREQ(GetDemoProfileIdName(DemoProfileId::Bicycle), "bicycle");
  EXPECT_STREQ(GetDemoProfileIdName(DemoProfileId::Bistro), "bistro");
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
  EXPECT_EQ(ResolveDemoProfileProjectPath(DemoProfileId::RenderingRegression, resource_root),
            launcher::NormalizeProjectPath(resource_root / ".generated" / "EvoEngine-DemoProjects" /
                                           "RenderingRegression" / "RenderingRegression.eveproj"));
  EXPECT_EQ(
      ResolveDemoProfileProjectPath(DemoProfileId::Ddgi, resource_root),
      launcher::NormalizeProjectPath(resource_root / "EvoEngine-DemoProjects" / "CornellBox" / "CornellBox.eveproj"));
  EXPECT_EQ(ResolveDemoProfileProjectPath(DemoProfileId::EcoSysLab, resource_root),
            launcher::NormalizeProjectPath(resource_root / "EcoSysLabProject" / "test.eveproj"));
  EXPECT_EQ(ResolveDemoProfileProjectPath(DemoProfileId::DigitalAgriculture, resource_root),
            launcher::NormalizeProjectPath(resource_root / "DigitalAgricultureProject" / "test.eveproj"));
  EXPECT_EQ(ResolveDemoProfileProjectPath(DemoProfileId::LSystem, resource_root),
            launcher::NormalizeProjectPath(lsystem_fallback));
  EXPECT_EQ(ResolveDemoProfileProjectPath(DemoProfileId::ProceduralGalaxy, resource_root),
            launcher::NormalizeProjectPath(resource_root / "EvoEngine-DemoProjects" / "Universe" /
                                           "ProceduralGalaxy.eveproj"));
  EXPECT_EQ(ResolveDemoProfileProjectPath(DemoProfileId::GaussianSplat, resource_root),
            launcher::NormalizeProjectPath(resource_root / "EvoEngine-DemoProjects" / "3DGS" / "3DGS.eveproj"));
  EXPECT_EQ(ResolveDemoProfileProjectPath(DemoProfileId::Bicycle, resource_root),
            launcher::NormalizeProjectPath(resource_root / "EvoEngine-DemoProjects" / "Bicycle" / "Bicycle.eveproj"));
  EXPECT_EQ(ResolveDemoProfileProjectPath(DemoProfileId::Bistro, resource_root),
            launcher::NormalizeProjectPath(resource_root / ".generated" / "EvoEngine-DemoProjects" / "Bistro" /
                                           "Bistro.eveproj"));
}

TEST(LauncherUtils, DemoResourceChecksIncludeRenderingLightingAssets) {
  TempLauncherDirectory temp;
  const auto resource_root = temp.RootPath() / "Resources";
  std::filesystem::create_directories(resource_root);

  const std::vector<std::string> missing_sponza_lighting{
      "SponzaEnvironment.eveenvironmentalmap", "SponzaGlobal.evereflectionprobe", "SponzaLocal.evereflectionprobepack"};
  EXPECT_EQ(MissingDemoProfileResourceRequirements(DemoProfileId::Rendering, resource_root), missing_sponza_lighting);
  EXPECT_EQ(MissingDemoProfileResourceRequirements(DemoProfileId::RenderingRegression, resource_root),
            missing_sponza_lighting);
  EXPECT_TRUE(MissingDemoProfileResourceRequirements(DemoProfileId::Ddgi, resource_root).empty());
  EXPECT_TRUE(MissingDemoProfileResourceRequirements(DemoProfileId::ProceduralGalaxy, resource_root).empty());
  const auto missing_gaussian_resources =
      MissingDemoProfileResourceRequirements(DemoProfileId::GaussianSplat, resource_root);
  EXPECT_EQ(missing_gaussian_resources,
            (std::vector<std::string>{"spatial_dragon.ply", "spatial_dragon.ply.evefilemeta"}));
  const auto missing_bicycle_resources = MissingDemoProfileResourceRequirements(DemoProfileId::Bicycle, resource_root);
  EXPECT_EQ(missing_bicycle_resources, (std::vector<std::string>{"bicycle.ply", "bicycle.ply.evefilemeta"}));
  const auto missing_bistro_resources = MissingDemoProfileResourceRequirements(DemoProfileId::Bistro, resource_root);
  EXPECT_EQ(missing_bistro_resources, (std::vector<std::string>{"Bistro.eveproj", "bistro.gltf", "bistro.bin",
                                                                "textures", "objects", "bistro.gltf.evefilemeta"}));
  EXPECT_FALSE(MissingDemoProfileResourceRequirements(DemoProfileId::EcoSysLab, resource_root).empty());

  const auto sponza_lighting =
      resource_root / "EvoEngine-DemoProjects" / "Rendering" / "Assets" / "Lighting" / "Sponza";
  std::filesystem::create_directories(sponza_lighting);
  std::ofstream(sponza_lighting / missing_sponza_lighting.front()) << "environment_source_type: 3\n";
  for (size_t index = 1; index < missing_sponza_lighting.size(); ++index) {
    std::ofstream(sponza_lighting / missing_sponza_lighting[index]) << "{}\n";
  }
  const std::vector<std::string> missing_probe_payloads(missing_sponza_lighting.begin() + 1,
                                                        missing_sponza_lighting.end());
  EXPECT_EQ(MissingDemoProfileResourceRequirements(DemoProfileId::Rendering, resource_root), missing_probe_payloads);
  EXPECT_EQ(MissingDemoProfileResourceRequirements(DemoProfileId::RenderingRegression, resource_root),
            missing_probe_payloads);

  const auto gaussian_asset_folder = resource_root / "EvoEngine-DemoProjects" / "3DGS" / "Assets" / "GaussianSplats";
  std::filesystem::create_directories(gaussian_asset_folder);
  std::ofstream gaussian_asset(gaussian_asset_folder / "spatial_dragon.ply");
  gaussian_asset << "ply\n";
  gaussian_asset.close();
  std::ofstream gaussian_metadata(gaussian_asset_folder / "spatial_dragon.ply.evefilemeta");
  gaussian_metadata << "asset_type_name_: GaussianSplat\n";
  gaussian_metadata.close();
  EXPECT_TRUE(MissingDemoProfileResourceRequirements(DemoProfileId::GaussianSplat, resource_root).empty());

  const auto bicycle_assets_folder = resource_root / "EvoEngine-DemoProjects" / "Bicycle" / "Assets";
  const auto bicycle_splats_folder = bicycle_assets_folder / "GaussianSplats";
  std::filesystem::create_directories(bicycle_splats_folder);
  std::ofstream bicycle_asset(bicycle_splats_folder / "bicycle.ply");
  bicycle_asset << "ply\n";
  bicycle_asset.close();
  std::ofstream bicycle_metadata(bicycle_splats_folder / "bicycle.ply.evefilemeta");
  bicycle_metadata << "asset_type_name_: GaussianSplat\n";
  bicycle_metadata.close();
  EXPECT_TRUE(MissingDemoProfileResourceRequirements(DemoProfileId::Bicycle, resource_root).empty());

  const auto bistro_root = resource_root / ".generated" / "EvoEngine-DemoProjects" / "Bistro";
  const auto bistro_asset_folder = bistro_root / "Assets" / "Models" / "Bistro";
  std::filesystem::create_directories(bistro_asset_folder / "textures");
  std::filesystem::create_directories(bistro_asset_folder / "objects");
  std::ofstream bistro_project(bistro_root / "Bistro.eveproj");
  bistro_project << "application_name: Bistro\n";
  bistro_project.close();
  std::ofstream bistro_asset(bistro_asset_folder / "bistro.gltf");
  bistro_asset << "{}\n";
  bistro_asset.close();
  std::ofstream bistro_bin(bistro_asset_folder / "bistro.bin");
  bistro_bin << "bin\n";
  bistro_bin.close();
  std::ofstream bistro_metadata(bistro_asset_folder / "bistro.gltf.evefilemeta");
  bistro_metadata << "asset_type_name_: Prefab\n";
  bistro_metadata.close();
  EXPECT_TRUE(MissingDemoProfileResourceRequirements(DemoProfileId::Bistro, resource_root).empty());

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

TEST(LauncherUtils, LauncherDefersRenderPipelinePrewarm) {
  const auto launcher_source =
      ReadTextFile(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App/src/EvoEngineLauncher.cpp");
  EXPECT_NE(launcher_source.find("application_info.prewarm_render_pipelines = false"), std::string::npos);
}

TEST(LauncherUtils, ShadowCascadeFitNamesExcludeLegacyStable) {
  EXPECT_EQ(ParseShadowCascadeFitModeName("sphere"), RenderSettings::ShadowCascadeFitMode::StableSphere);
  EXPECT_EQ(ParseShadowCascadeFitModeName("stable-sphere"), RenderSettings::ShadowCascadeFitMode::StableSphere);
  EXPECT_EQ(ParseShadowCascadeFitModeName("tight"), RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb);
  EXPECT_EQ(ParseShadowCascadeFitModeName("aabb"), RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb);
  EXPECT_EQ(ParseShadowCascadeFitModeName("tight-aabb"), RenderSettings::ShadowCascadeFitMode::TightLightSpaceAabb);
  EXPECT_THROW(ParseShadowCascadeFitModeName("legacy"), std::invalid_argument);
  EXPECT_THROW(ParseShadowCascadeFitModeName("legacy-stable"), std::invalid_argument);
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
