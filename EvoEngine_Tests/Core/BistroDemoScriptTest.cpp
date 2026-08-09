#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace {
class TempBistroScriptDirectory {
 public:
  TempBistroScriptDirectory() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineBistroScriptTest_" + std::to_string(now));
    std::filesystem::create_directories(root_);
  }

  ~TempBistroScriptDirectory() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path RootPath() const {
    return root_;
  }

 private:
  std::filesystem::path root_;
};

std::string Quote(const std::filesystem::path& path) {
  std::string quoted = "\"";
  for (const char character : path.string()) {
    if (character == '"') {
      quoted += "\\\"";
    } else {
      quoted += character;
    }
  }
  quoted += "\"";
  return quoted;
}

std::string SystemCommand(const std::string& command) {
#ifdef _WIN32
  return "\"" + command + "\"";
#else
  return command;
#endif
}

void WriteText(const std::filesystem::path& path, const std::string& text) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream stream(path);
  stream << text;
}

std::string ReadText(const std::filesystem::path& path) {
  std::ifstream stream(path);
  return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
}

std::string ExtractTextRange(const std::string& source, const std::string& begin, const std::string& end) {
  const auto begin_offset = source.find(begin);
  if (begin_offset == std::string::npos) {
    return {};
  }
  const auto end_offset = source.find(end, begin_offset + begin.size());
  return end_offset == std::string::npos ? std::string{} : source.substr(begin_offset, end_offset - begin_offset);
}

void WriteMinimalBistroSource(const std::filesystem::path& source_root) {
  WriteText(source_root / "LICENSE", "Bistro fixture license\n");
  WriteText(source_root / "README.md", "Bistro fixture readme\n");
  WriteText(source_root / "bistro.bin", "binary\n");
  WriteText(source_root / "textures" / "albedo.png", "png\n");
  WriteText(source_root / "textures" / "albedo.dds", "dds\n");
  WriteText(source_root / "objects" / "mesh.bin", "mesh\n");
  WriteText(source_root / "bistro.gltf", R"({
  "extensionsUsed": ["KHR_lights_punctual", "MSFT_texture_dds"],
  "images": [
    {"uri": "textures/albedo.png"},
    {"uri": "textures/albedo.dds"}
  ],
  "textures": [
    {"source": 0, "extensions": {"MSFT_texture_dds": {"source": 1}}}
  ],
  "materials": [
    {"pbrMetallicRoughness": {"baseColorTexture": {"index": 0}}}
  ],
  "nodes": [
    {"name": "Sun", "extensions": {"KHR_lights_punctual": {"light": 0}}}
  ],
  "scenes": [
    {"nodes": [0]}
  ],
  "scene": 0,
  "extensions": {
    "KHR_lights_punctual": {
      "lights": [
        {"name": "Sun", "type": "directional", "intensity": 6830, "color": [1, 1, 1]}
      ]
    }
  }
}
)");
}
}  // namespace

TEST(BistroDemoScript, GeneratesIgnoredProjectFromExistingSourceRoot) {
  TempBistroScriptDirectory temp;
  const auto source_root = temp.RootPath() / "source";
  const auto resource_root = temp.RootPath() / "Resources";
  WriteMinimalBistroSource(source_root);

  const auto script_path = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "Scripts" / "prepare_demos.py";
  const std::string command = Quote(EVOENGINE_TEST_PYTHON_EXECUTABLE) + " " + Quote(script_path) +
                              " --demo bistro --validate --prepare --resource-root " + Quote(resource_root) +
                              " --bistro-source-root " + Quote(source_root) +
                              " --no-download --bistro-asset-mode copy --no-previews";
  ASSERT_EQ(std::system(SystemCommand(command).c_str()), 0);

  const auto demo_root = resource_root / ".generated" / "EvoEngine-DemoProjects" / "Bistro";
  EXPECT_TRUE(std::filesystem::exists(demo_root / "Bistro.eveproj"));
  EXPECT_TRUE(std::filesystem::exists(demo_root / "Assets" / "Models" / "Bistro" / "bistro.gltf"));
  EXPECT_TRUE(std::filesystem::exists(demo_root / "Assets" / "Models" / "Bistro" / "bistro.bin"));
  EXPECT_TRUE(std::filesystem::exists(demo_root / "Assets" / "Models" / "Bistro" / "textures" / "albedo.png"));
  EXPECT_TRUE(std::filesystem::exists(demo_root / "Assets" / "Models" / "Bistro" / "textures" / "albedo.dds"));
  EXPECT_TRUE(std::filesystem::exists(demo_root / "Assets" / "Models" / "Bistro" / "objects" / "mesh.bin"));

  const auto gltf = ReadText(demo_root / "Assets" / "Models" / "Bistro" / "bistro.gltf");
  EXPECT_NE(gltf.find("\"name\": \"Sun\""), std::string::npos);
  EXPECT_NE(gltf.find("\"light\": 0"), std::string::npos);
  EXPECT_NE(gltf.find("\"intensity\": 6830"), std::string::npos);

  const auto metadata = ReadText(demo_root / "Assets" / "Models" / "Bistro" / "bistro.gltf.evefilemeta");
  EXPECT_NE(metadata.find("asset_type_name_: Prefab"), std::string::npos);
  EXPECT_NE(metadata.find("asset_extension_: .gltf"), std::string::npos);

  const auto readme = ReadText(demo_root / "README.md");
  EXPECT_NE(readme.find("https://github.com/zeux/niagara_bistro.git"), std::string::npos);
  EXPECT_NE(readme.find("MSFT_texture_dds textures: 1"), std::string::npos);
}

TEST(BistroDemoScript, PrefabImporterKeepsDdsFallbackAndResolvedFlipPolicy) {
  const auto prefab_source =
      ReadText(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "Prefab.cpp");

  EXPECT_NE(prefab_source.find("CollectTextureImportCandidates"), std::string::npos);
  EXPECT_NE(prefab_source.find("extension == \".dds\""), std::string::npos);
  EXPECT_NE(prefab_source.find("\".png\", \".tga\", \".jpg\", \".jpeg\""), std::string::npos);
  EXPECT_NE(prefab_source.find("!texture_2d->Import(full_path)"), std::string::npos);
  EXPECT_NE(prefab_source.find("TexturePathNeedsYFlip(full_path)"), std::string::npos);
  EXPECT_NE(prefab_source.find("resolved_texture_source_needs_y_flip"), std::string::npos);
  EXPECT_EQ(prefab_source.find("TextureUriUsesDds"), std::string::npos);
  const auto add_candidate_begin = prefab_source.find("void AddTextureImportCandidate");
  const auto collect_candidates_begin = prefab_source.find("CollectTextureImportCandidates", add_candidate_begin);
  ASSERT_NE(add_candidate_begin, std::string::npos);
  ASSERT_NE(collect_candidates_begin, std::string::npos);
  const auto candidate_function =
      prefab_source.substr(add_candidate_begin, collect_candidates_begin - add_candidate_begin);
  EXPECT_LT(candidate_function.find("candidates.emplace_back(absolute_path)"),
            candidate_function.find("for (const auto* fallback_extension"));
  EXPECT_NE(prefab_source.find("resolved_texture_indices"), std::string::npos);
  EXPECT_NE(prefab_source.find("target_material->SetGltfMaterialData(imported_material_data->material_data)"),
            std::string::npos);
  EXPECT_NE(prefab_source.find("target_material->RefTextureRefs() = imported_material_data->texture_refs"),
            std::string::npos);
  EXPECT_NE(prefab_source.find("if (importer_material && !imported_material_data)"), std::string::npos);
  EXPECT_NE(prefab_source.find("Direct glTF material data owns extension texture/factor semantics"), std::string::npos);
}

TEST(BistroDemoScript, TexturePipelineRegistersDdsAndBc7Upload) {
  const auto texture_source =
      ReadText(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "Texture2D.cpp");
  const auto storage_source =
      ReadText(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "TextureStorage.cpp");
  const auto application_source =
      ReadText(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "Application.cpp");

  EXPECT_NE(texture_source.find("VK_FORMAT_BC7_UNORM_BLOCK"), std::string::npos);
  EXPECT_NE(texture_source.find("VK_FORMAT_BC7_SRGB_BLOCK"), std::string::npos);
  EXPECT_NE(texture_source.find("compressed_mip_levels"), std::string::npos);
  EXPECT_NE(storage_source.find("SetCompressedDataAsync"), std::string::npos);
  EXPECT_NE(storage_source.find("BuildCompressedMipCopyRegions"), std::string::npos);
  EXPECT_NE(storage_source.find("new_compressed_mip_levels_"), std::string::npos);
  EXPECT_NE(application_source.find("\".dds\""), std::string::npos);
  EXPECT_NE(application_source.find("\".DDS\""), std::string::npos);
}

TEST(BistroDemoScript, Texture2DStorageSamplerMatchesReferenceLodFiltering) {
  const auto storage_source =
      ReadText(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "TextureStorage.cpp");
  const auto sampler_start = storage_source.find("VkSamplerCreateInfo DefaultTextureSamplerCreateInfo()");
  ASSERT_NE(sampler_start, std::string::npos);
  const auto sampler_end = storage_source.find("return sampler_info;", sampler_start);
  ASSERT_NE(sampler_end, std::string::npos);
  const auto initialize_sampler = storage_source.substr(sampler_start, sampler_end - sampler_start);

  EXPECT_NE(initialize_sampler.find("sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR"), std::string::npos);
  EXPECT_NE(initialize_sampler.find("sampler_info.anisotropyEnable = VK_FALSE"), std::string::npos);
  EXPECT_NE(initialize_sampler.find("sampler_info.maxAnisotropy = 1.0f"), std::string::npos);
  EXPECT_NE(initialize_sampler.find("sampler_info.maxLod = VK_LOD_CLAMP_NONE"), std::string::npos);
  EXPECT_EQ(initialize_sampler.find("sampler_info.anisotropyEnable = VK_TRUE"), std::string::npos);
}

TEST(BistroDemoScript, DemoSceneAlignsRootToReferenceCamera) {
  const auto demo_scene_source =
      ReadText(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "DemoScene.cpp");
  const auto bistro_source = ExtractTextRange(demo_scene_source, "void evo_engine::ConfigureBistroDemoScene",
                                              "std::filesystem::path evo_engine::FindDemoResourcesRoot");
  ASSERT_FALSE(bistro_source.empty());

  EXPECT_NE(demo_scene_source.find("--gltfCamera 0"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("kBistroReferenceCameraPosition"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("kBistroReferenceCameraNodeRotation"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("kBistroReferenceCameraFront"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("kBistroReferenceCameraRotation"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("kBistroReferenceCameraYFov"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("kBistroReferenceCameraEvoEngineFov = kBistroReferenceCameraYFov * 2.0f"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("ConfigureBistroCameraPostProcessing"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("ConfigureBistroRasterizationPostProcessing"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("ConfigureBistroReferenceToneMapping"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("ConfigureBistroRayTracingPostProcessing"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("tone_mapping.method = ToneMapping::ToneMapMethod::Filmic"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("tone_mapping.exposure = 1.0f"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("tone_mapping.brightness = 1.0f"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("tone_mapping.auto_exposure = true"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("tone_mapping.average_mode = 1"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("kBistroDirectionalLightSize = 0.01f"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("light->light_size = kBistroDirectionalLightSize"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("glm::quatLookAt(kBistroReferenceCameraFront"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("CalculateBistroRootTransformForCameraFrame"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("frame.position - root_rotation * kBistroReferenceCameraPosition"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("CalculateBistroCameraFrame(bistro->GetBoundingBox())"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("RemoveDefaultDirectionalLight(scene)"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("FindEntityNamed(scene, \"Directional Light\")"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("scene->HasPrivateComponent<DirectionalLight>(*default_light_entity)"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("scene->DeleteEntity(*default_light_entity)"), std::string::npos);
  EXPECT_EQ(demo_scene_source.find("FindEntityNamed(scene, \"Sun\")"), std::string::npos);
  EXPECT_NE(bistro_source.find("ConfigureEnvironmentalLightingMapSource(*lighting, "
                               "Resources::GetInstance().GetDefaultEnvironmentalMap(), 1.0f"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("SetEnvironmentalLightingFallbackIntensities(*lighting, 0.0f, 0.0f)"),
            std::string::npos);
  EXPECT_EQ(bistro_source.find("scene->environment.environment_type"), std::string::npos);
  EXPECT_EQ(bistro_source.find("scene->environment.indirect_lighting_intensity"), std::string::npos);
  EXPECT_EQ(bistro_source.find("scene->environment.background_intensity"), std::string::npos);
  EXPECT_NE(bistro_source.find("scene_camera->camera_settings.background_source = "
                               "Camera::BackgroundSource::InheritEnvironmentalLighting"),
            std::string::npos);
  EXPECT_NE(bistro_source.find("scene_camera->camera_settings.background_intensity = 1.0f"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("ConfigureBistroReferenceToneMapping(scene_camera)"), std::string::npos);

  const auto scene_source =
      ReadText(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "Scene.cpp");
  EXPECT_EQ(scene_source.find("CreateEntity(\"Directional Light\")"), std::string::npos);

  const auto editor_source =
      ReadText(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_App" / "src" / "EvoEngineEditor.cpp");
  EXPECT_NE(editor_source.find("demo_profile_id == DemoProfileId::Bistro"), std::string::npos);
  EXPECT_NE(editor_source.find("Camera::IsRayCameraRenderMode(resolved_render_mode)"), std::string::npos);
  EXPECT_NE(editor_source.find("ConfigureBistroRayTracingPostProcessing(scene_camera)"), std::string::npos);

  const auto post_processing_source =
      ReadText(std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / "EvoEngine_SDK" / "src" / "PostProcessingStack.cpp");
  EXPECT_NE(post_processing_source.find("enable_ambient_occlusion = true;\n  enable_bloom = false;\n  "
                                        "enable_screen_space_reflection = false;"),
            std::string::npos);
  EXPECT_NE(demo_scene_source.find("auto& ddgi = RequireEnvironmentalLightingDdgiSettings(scene);"), std::string::npos);
  EXPECT_NE(demo_scene_source.find("camera->post_processing_stack_ref = "
                                   "AssetManager::CreateTemporaryAsset<PostProcessingStack>();"),
            std::string::npos);
}

TEST(BistroDemoScript, SmokeRunnerEnforcesValidatedInstalledLifecycle) {
  const auto source_root = std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR);
  const auto runner_source = ReadText(source_root / "Scripts" / "run_bistro_smoke.py");
  const auto editor_source = ReadText(source_root / "EvoEngine_App" / "src" / "EvoEngineEditor.cpp");
  const auto application_source = ReadText(source_root / "EvoEngine_SDK" / "src" / "Application.cpp");

  EXPECT_NE(runner_source.find("default=30"), std::string::npos);
  EXPECT_NE(runner_source.find("default=1920"), std::string::npos);
  EXPECT_NE(runner_source.find("default=1080"), std::string::npos);
  EXPECT_NE(runner_source.find("out/install/vs2026-x64/bin/EvoEngineEditor.exe"), std::string::npos);
  EXPECT_NE(runner_source.find("EVOENGINE_VULKAN_VALIDATION enabled"), std::string::npos);
  EXPECT_NE(runner_source.find("EVOENGINE_BISTRO_DDGI_READY"), std::string::npos);
  EXPECT_NE(runner_source.find("EVOENGINE_BISTRO_FRAME_READY resolution="), std::string::npos);
  EXPECT_NE(runner_source.find("EVOENGINE_BISTRO_SMOKE_HEARTBEAT"), std::string::npos);
  EXPECT_NE(runner_source.find("EVOENGINE_BISTRO_SMOKE_SHUTDOWN_COMPLETE"), std::string::npos);
  EXPECT_NE(runner_source.find("PostMessageW(hwnd, WM_CLOSE, 0, 0)"), std::string::npos);
  EXPECT_NE(runner_source.find("ready_time = time.monotonic()"), std::string::npos);
  EXPECT_NE(runner_source.find("environment[\"EVOENGINE_IMGUI_INI_PATH\"] = str(isolated_imgui_path)"),
            std::string::npos);
  EXPECT_NE(runner_source.find("env=environment"), std::string::npos);
  EXPECT_NE(runner_source.find("installed_imgui_after != installed_imgui_before"), std::string::npos);
  EXPECT_NE(runner_source.find("Installed editor ImGui layout changed during Bistro smoke"), std::string::npos);

  EXPECT_NE(editor_source.find("--bistro-smoke requires --preview-width 1920 --preview-height 1080."),
            std::string::npos);
  EXPECT_NE(editor_source.find("EVOENGINE_BISTRO_DDGI_READY active_probes="), std::string::npos);
  EXPECT_NE(editor_source.find("stats.recorded_probe_update_count > 0"), std::string::npos);
  EXPECT_NE(editor_source.find("stats.recorded_ray_sample_count > 0"), std::string::npos);
  EXPECT_NE(editor_source.find("stats.lighting_descriptors_bound"), std::string::npos);
  EXPECT_NE(editor_source.find("EVOENGINE_BISTRO_FRAME_READY resolution="), std::string::npos);
  EXPECT_NE(editor_source.find("EVOENGINE_BISTRO_SMOKE_SHUTDOWN_COMPLETE"), std::string::npos);

  const auto terminate = application_source.find("void Application::Terminate()");
  const auto gpu_drain = application_source.find("Platform::DrainGpuResourceWork()", terminate);
  const auto destroy_layers = application_source.find("for (auto i = this->layers_.rbegin()", terminate);
  ASSERT_NE(terminate, std::string::npos);
  ASSERT_NE(gpu_drain, std::string::npos);
  ASSERT_NE(destroy_layers, std::string::npos);
  EXPECT_LT(gpu_drain, destroy_layers);
}
