#pragma once

#ifdef DIGITAL_AGRICULTURE_PLUGIN

#  include "AnimationPlayer.hpp"
#  include "Application.hpp"
#  include "ClassRegistry.hpp"
#  include "Climate.hpp"
#  include "EditorLayer.hpp"
#  include "HeightField.hpp"
#  include "MeshRenderer.hpp"
#  include "ObjectRotator.hpp"
#  include "PlayerController.hpp"
#  include "PostProcessingStack.hpp"
#  include "Prefab.hpp"
#  include "ProjectManager.hpp"
#  include "RadialBoundingVolume.hpp"
#  include "RenderLayer.hpp"
#  include "Scene.hpp"
#  include "Soil.hpp"
#  include "SorghumLayer.hpp"
#  include "Times.hpp"
#  include "Tree.hpp"
#  include "TreeModel.hpp"
#  include "TreeStructor.hpp"
#  include "WindowLayer.hpp"
#  include "pybind11/pybind11.h"
#  include "pybind11/stl/filesystem.h"
#  ifdef CUDA_MODULE_PLUGIN
#    include <CUDAModule.hpp>
#    include <RayTracerLayer.hpp>
#  endif

#  if DATASET_GENERATION_PLUGIN
#    include <SorghumPointCloudScanner.hpp>
#    include "DatasetGenerator.hpp"
using namespace dataset_generation_plugin;
#  endif
#  include "PyEvoEngine.hpp"

using namespace evo_engine;
using namespace py_evo_engine;
using namespace digital_agriculture_plugin;

namespace py_digital_agriculture_plugin {
class PyDigitalAgriculture {
  EVOENGINE_SINGLETON_INSTANCE(PyDigitalAgriculture)
 public:
  /**
   * @brief Add SorghumLayer to the framework.
   */
  static void PushSorghumLayer();
  static void RegisterClasses();
  static Entity CreateEntityFromSorghumState(const Handle& sorghum_handle);
  static Entity CreateEntityFromSorghumDescriptor(const Handle& sorghum_handle);
  static Entity CreateEntityFromSorghumGenerator(const Handle& sorghum_generator_handle, int seed);
  static Entity CreateEntityFromSorghumField(const Handle& sorghum_generator_handle, int seed);
  static void ApplySorghumGrid(const Handle& sorghum_field_handle, const Handle& sorghum_generator_handle,
                               const SorghumGrid& sorghum_grid);

  /**
   * @brief Register for python binding.
   * @param m The target python binding module to register functions and classes.
   */
  static void Initialize(pybind11::module& m);

  static void EnableBTF();

  static void SetCBTFGroup(const Handle& cbtf_group_handle);

  static bool CheckBTFComponentsExist();

  static void SetSkyDome();

  /**
   * @brief Set the number of ray samples and bounces used by all illumination estimation calls.
   *
   * The default is RayProperties{4, 4} (bounces=4, samples=4). Under Skydome (Nishita) mode
   * the solar disk subtends ~6.8e-5 sr, so 4 samples almost never directly hit it — the
   * estimator receives diffuse-sky-only flux and underestimates PARa by ~5×. Set samples=256
   * (or at minimum 64) to ensure the direct solar beam is adequately sampled.
   *
   * Call this after PushSorghumLayer() and before any IlluminationEstimation call.
   *
   * @param samples  Number of ray samples per probe (default 4; use 64–256 for Skydome).
   * @param bounces  Number of ray bounces (default 4).
   */
  static void SetIlluminationSamples(int samples, int bounces);

  /**
   * @brief Create a single upward-facing reference sensor at (x, y, z) for open-sky flux measurement.
   *
   * Creates a PARSensorGroup with one degenerate-triangle probe (normal = up) placed at the
   * supplied world position. Intended for use outside the canopy footprint to measure the
   * unobstructed Nishita sky flux, which is then used to normalise plant results to physical
   * PAR units via: PARa = (plant_flux / ref_flux) * PAR_incident.
   *
   * Caller must DeleteRuntimeAsset(handle) when done.
   *
   * @param x  World X position (m). Place outside field footprint.
   * @param y  World Y position (m). Recommend >= 3.0 (above canopy).
   * @param z  World Z position (m).
   * @return   Handle to the PARSensorGroup asset.
   */
  static Handle CreateReferenceSensor(float x, float y, float z);

  /**
   * @brief Set SingleLightSource environment for physically calibrated illumination estimation.
   *
   * Replaces the Nishita sky dome with a uniform directional light whose intensity is set
   * to the supplied PAR values (µmol photons m⁻² s⁻¹). For a probe in fully open sky:
   *   probe.energy ≈ par_direct_umol × 0.5   (hemisphere-average cosine factor)
   * Multiply probe.energy by 2.0 to recover absorbed PARa in µmol m⁻² s⁻¹.
   *
   * Must be called after SetSunDirection() for each new solar position.
   *
   * @param par_direct_umol  Direct beam PAR (µmol m⁻² s⁻¹) from pvlib clear-sky model.
   * @param par_diffuse_umol Diffuse sky PAR (µmol m⁻² s⁻¹).
   */
  static void SetDirectLightSource(float par_direct_umol, float par_diffuse_umol);

  static void PushRayTracerLayer();

  static void SetSunDirection(glm::vec3 angles);

  static void IlluminationEstimationOnSorghum();

  static void CheckTriangleEstimator(const Entity& sorghum_entity);

  static Entity InstantiateSorghumField(const Handle& sorghum_field_handle, const Handle& sorghum_coordinates,
                                       const int seed, const int index=200, const float radius=2000.0f);

  static std::vector<std::vector<glm::vec3>> GetAllIlluminationEstimationResultsOnSorghum();

  static Handle SetPARSensors(const Entity& sorghum_field);

  static void IlluminationEstimationOnSensors(const Handle& sensor_group_handle);

  static std::vector<std::vector<glm::vec3>> GetAllIlluminationEstimationResultsFromSensors(
      const Handle& sensor_group_handle);
};

}  // namespace py_digital_agriculture_plugin

#endif