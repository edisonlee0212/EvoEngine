#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace evo_engine {

enum class ApplicationMode;
struct ApplicationInitializationSettings;
class Scene;

enum class DemoProfileId {
  Rendering,
  RenderingRegression,
  Ddgi,
  EcoSysLab,
  DigitalAgriculture,
  LSystem,
  ProceduralGalaxy,
  GaussianSplat,
  Bicycle,
  Bistro
};

struct DemoProfileDescriptor {
  DemoProfileId id;
  const char* id_name;
  const char* title;
  const char* source_app_name;
  const char* description;
  const char* preview_image_path;
  ApplicationMode default_application_mode;
  std::vector<ApplicationMode> supported_application_modes;
  std::vector<std::string> startup_runtime_packages;
};

struct DdgiCornellBoxDemoSettings {
  float point_light_brightness = 2.0f;
  float indirect_lighting_intensity = 1.0f;
  bool enable_probe_relocation = true;
  bool enable_probe_classification = true;
};

[[nodiscard]] const std::vector<DemoProfileDescriptor>& GetDemoProfiles();
[[nodiscard]] const DemoProfileDescriptor* FindDemoProfile(std::string_view id_name);
[[nodiscard]] const DemoProfileDescriptor& GetDemoProfile(DemoProfileId id);
[[nodiscard]] const char* GetDemoProfileIdName(DemoProfileId id);
[[nodiscard]] bool IsDemoProfileApplicationModeSupported(DemoProfileId id, ApplicationMode mode);
[[nodiscard]] std::filesystem::path FindDemoProfileResourcesRoot(const std::filesystem::path& preferred_root = {});
[[nodiscard]] std::filesystem::path ResolveDemoProfileProjectPath(
    DemoProfileId id, const std::filesystem::path& preferred_resource_root = {});
[[nodiscard]] std::vector<std::string> MissingDemoProfileResourceRequirements(
    DemoProfileId id, const std::filesystem::path& preferred_resource_root = {});
void NormalizeLegacyResourceExtensions(const std::filesystem::path& resource_root);
void ApplyRenderingDemoEditorSetup();
void ApplyEcoSysLabDemoEditorSetup();
void ConfigureDdgiCornellBoxApplication(ApplicationInitializationSettings& application_info,
                                        ApplicationMode application_mode);
void ConfigureDdgiCornellBoxScene(const std::shared_ptr<Scene>& scene, const DdgiCornellBoxDemoSettings& settings = {});

}  // namespace evo_engine
