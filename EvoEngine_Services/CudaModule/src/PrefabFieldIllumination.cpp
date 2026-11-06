#include "PrefabFieldIllumination.hpp"

#include "ApplicationContext.hpp"
#include "RayTracerLayer.hpp"
#include "Scene.hpp"
#include "TransformGraph.hpp"
#include "TriangleIlluminationEstimator.hpp"

using namespace evo_engine;

namespace {
const std::unordered_set<std::string> kDoubleSidedFieldMeshes = {"leaf"};
}

PrefabFieldIlluminationSummary evo_engine::EstimatePrefabFieldIllumination(
    PrefabField& field, const PrefabFieldIlluminationSettings& settings) {
  const auto scene = field.GetScene();
  const auto layer = ApplicationContext::Get().GetLayer<RayTracerLayer>();
  if (!scene || !layer || settings.samples <= 0 || settings.bounces < 0 || settings.push_normal_distance < 0.0f) {
    return {};
  }
  // Persist Python overrides too, so saved field scenes show the settings actually measured.
  field.illumination_settings = settings;

  // Synchronize transforms and OptiX geometry once before sampling individual plants.
  TransformGraph::CalculateTransformGraphs(scene, false);
  layer->UpdateScene(scene);
  RayProperties ray_properties;
  ray_properties.samples = settings.samples;
  ray_properties.bounces = settings.bounces;

  const auto models = field.GetGeneratedEntities();
  const auto placements = field.BuildPlacements(field.CollectAssetPaths().size());
  const auto region = field.GetMeasurementRegion();
  std::vector<PrefabFieldIlluminationResult> results;
  results.reserve(models.size());
  for (size_t index = 0; index < models.size(); ++index) {
    const int row = field.columns > 0 ? static_cast<int>(index) / field.columns : 0;
    const int column = field.columns > 0 ? static_cast<int>(index) % field.columns : 0;
    // Filter receivers only: UpdateScene above includes the complete field in the OptiX scene.
    if (!region.Contains(row, column))
      continue;
    const auto estimator = scene->GetOrSetPrivateComponent<TriangleIlluminationEstimator>(models[index]).lock();
    // Helios exports the thin leaf sheet as an object named "leaf"; closed stem and petiole meshes stay one-sided.
    estimator->PrepareLightProbeGroup(kDoubleSidedFieldMeshes);
    if (estimator->total_area > 0.0f) {
      estimator->SampleLightProbeGroup(ray_properties, settings.seed + static_cast<int>(index),
                                       settings.push_normal_distance);
    }
    results.push_back({index, index < placements.size() ? placements[index].asset_index : 0, row, column,
                       estimator->total_area, estimator->total_area > 0.0f ? estimator->total_flux : glm::vec3(0.0f),
                       estimator->total_area > 0.0f ? estimator->average_flux : glm::vec3(0.0f)});
    // Helios names stems "shoot" and leaflet stalks "petiolule"; retain other mesh names verbatim.
    auto& organs = results.back().organs;
    for (const auto& [mesh_name, value] : estimator->PeekMeshIllumination()) {
      const auto organ_name = mesh_name == "shoot" ? "stem" : mesh_name == "petiolule" ? "petiole" : mesh_name;
      auto& organ = organs[organ_name];
      organ.total_area += value.total_area;
      organ.total_flux += value.total_flux;
      organ.average_flux += value.average_flux * value.total_area;
    }
    for (auto& [name, organ] : organs)
      organ.average_flux = organ.total_area > 0.0f ? organ.average_flux / organ.total_area : glm::vec3(0.0f);
  }
  field.last_illumination = PrefabField::SummarizeIllumination(std::move(results));
  return field.last_illumination;
}
