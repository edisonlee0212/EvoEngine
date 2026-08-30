#pragma once
#include "AnimationPlayer.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "DemoScene.hpp"
#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"
#include "PlayerController.hpp"
#include "PostProcessingStack.hpp"
#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
#include "Times.hpp"
#include "WindowLayer.hpp"
#include "pybind11/pybind11.h"
#include "pybind11/stl/filesystem.h"

namespace py_evo_engine {
using namespace evo_engine;

class PyEvoEngine {
 public:
  PyEvoEngine();
  ~PyEvoEngine();

  Application application;
  std::unordered_map<Handle, std::shared_ptr<IAsset>> runtime_assets;

  static PyEvoEngine& GetRuntime();
  [[nodiscard]] Application& GetApplication();
  /**
   * @brief Create a runtime asset, the asset's ownership is kept by PyEvoEngine.
   * @param asset_type The type of the asset.
   * @return The handle to the asset.
   */
  static Handle CreateRuntimeAsset(const std::string& asset_type);
  /**
   * @brief Remove a runtime asset created by PyEvoEngine.
   * @param asset_handle Target handle of the asset.
   */
  static void DeleteRuntimeAsset(const Handle& asset_handle);

  /**
   * @brief Retrieves an asset of type T corresponding to the given handle.
   * @param asset_handle The handle associated with the asset.
   * @return A shared pointer to the requested asset of type T.
   */
  [[nodiscard]] static std::shared_ptr<IAsset> GetAsset(const Handle& asset_handle);
  /**
   * \brief Check if target asset is runtime asset.
   * \param asset_handle Handle of the target asset.
   * \return If this asset is a runtime asset.
   */
  [[nodiscard]] static bool IsRuntimeAsset(const Handle& asset_handle);

  /**
   * \brief Get asset stored within project's asset folder.
   * \param asset_relative_path Path of asset relative to the project folder.
   * \return The handle of the asset.
   */
  static Handle GetAssetHandle(const std::filesystem::path& asset_relative_path);

  /**
   * \brief Create a runtime asset, and import from file outside project's asset folder.
   * \param asset_type The type of the asset.
   * \param asset_absolute_path The absolute path to the asset. Must be outside project's asset folder.
   * \return The handle of the runtime asset.
   */
  static Handle ImportRuntimeAsset(const std::string& asset_type, const std::filesystem::path& asset_absolute_path);

  /**
   * \brief Export and save runtime asset to the disk outside project's asset folder.
   * \param asset_handle The handle of the runtime asset.
   * \param asset_absolute_path The absolute path to the runtime asset. Must be outside project's asset folder.
   * \return The handle of the runtime asset.
   */
  static bool ExportAsset(const Handle& asset_handle, const std::filesystem::path& asset_absolute_path);
  /**
   * \brief Save target asset.
   * \param asset_handle Handle to the target asset, must not be runtime asset.
   * \return True on successfully saved asset, false otherwise.
   */
  static bool AssetSave(const Handle& asset_handle);
  /**
   * \brief (Re)load target asset.
   * \param asset_handle Handle to the target asset, must not be runtime asset.
   * \return True on successfully loaded asset, false otherwise.
   */
  static bool AssetLoad(const Handle& asset_handle);

  /**
   * @brief Register for python binding.
   * @param m The target python binding module to register functions and classes.
   */
  static void Initialize(pybind11::module& m);
  /**
   * @brief Add RenderLayer to the framework.
   */
  static void PushRenderLayer();
  /**
   * @rief Add WindowLayer to the framework.
   */
  static void PushWindowLayer();
  /**
   * @brief Add EditorLayer to the framework.
   */
  static void PushEditorLayer();
  /**
   * @brief Start the framework with RenderLayer but no WindowLayer.
   * @param project_path The path to the target project to load.
   * @return True when initialization was requested successfully.
   */
  static bool RunWindowless(const std::filesystem::path& project_path);
  /**
   * @brief Start a built-in DemoApp scene with RenderLayer but no WindowLayer.
   * @param demo_setup_name Name of the demo setup. Currently supports "Rendering".
   * @param resource_folder_path Path to the Resources folder to use.
   * @param clear_generated_project_files Whether generated project metadata should be removed before setup.
   * @return True when initialization was requested successfully.
   */
  static bool RunDemoWindowless(const std::string& demo_setup_name, const std::filesystem::path& resource_folder_path,
                                bool clear_generated_project_files = true);
  /**
   * @brief Configure the active scene's main camera for a deterministic capture.
   * @param render_mode Exact camera render mode name.
   * @param samples_per_frame Ray samples accumulated per rendered frame.
   * @param bounces Maximum ray-path bounce count.
   * @return True when the requested render mode is available and the camera was configured.
   */
  static bool ConfigureCurrentSceneCameraForCapture(const std::string& render_mode, int samples_per_frame, int bounces);
  /**
   * @brief Configure the raster submission paths used by a deterministic capture.
   * @param meshlet_enabled Whether raster geometry uses the mesh-shader path.
   * @param indirect_enabled Whether raster geometry uses indirect submission.
   * @return True when the requested path is available and was configured.
   */
  static bool ConfigureRasterPathForCapture(bool meshlet_enabled, bool indirect_enabled);
  /**
   * @brief Render and save the active scene's main camera.
   * @param resolution_x Capture width.
   * @param resolution_y Capture height.
   * @param output_path Output image path.
   * @param warmup_frames Number of application loops before saving.
   * @param require_accumulated_frames When true, keep rendering through history resets until the camera reaches the
   * requested frame count.
   * @return True when a non-empty image was written.
   */
  static bool CaptureCurrentScene(int resolution_x, int resolution_y, const std::filesystem::path& output_path,
                                  int warmup_frames = 1, bool require_accumulated_frames = false);
  /**
   * @brief Check whether the active scene is configured to render with DDGI.
   * @return True when DDGI runtime is enabled, indirect rendering is enabled, and an enabled DDGI volume exists.
   */
  static bool IsCurrentSceneDdgiEnabled();
  /**
   * @brief Start application with a project.
   * @param project_path The path to the target project to load.
   */
  static void Run(const std::filesystem::path& project_path);
  /**
   * \brief Load a scene under the project.
   * \param project_path The path to the target project to load.
   * \param project_relative_path Path to scene asset relative to project path.
   */
  static void RunWithScene(const std::filesystem::path& project_path,
                           const std::filesystem::path& project_relative_path);

  /**
   * @brief Trigger main application loop.
   * @return Whether end application signal has been triggered.
   */
  static bool Loop();
  /**
   * @brief End application.
   */
  static void Terminate();

  static Entity CreateEntity(const std::string& name);
  static void DeleteEntity(const Entity& entity);
  static bool IsEntityValid(const Entity& entity);
};
}  // namespace py_evo_engine
