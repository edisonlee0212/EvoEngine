#include "PrefabField.hpp"

#include "Prefab.hpp"
#include "ProjectManager.hpp"
#include "Scene.hpp"
#include "TransformGraph.hpp"

#include <cmath>
#include <numeric>

using namespace evo_engine;

namespace {
constexpr auto kGeneratedRootName = "Generated Prefab Field";

bool IsSupportedPrefabExtension(std::string extension) {
  std::transform(extension.begin(), extension.end(), extension.begin(), [](const unsigned char value) {
    return std::tolower(value);
  });
  static const std::unordered_set<std::string> extensions = {".eveprefab", ".obj", ".gltf", ".glb", ".blend",
                                                             ".ply",       ".fbx", ".dae",  ".x3d"};
  return extensions.find(extension) != extensions.end();
}
}  // namespace

std::vector<std::filesystem::path> PrefabField::CollectAssetPaths() const {
  std::vector<std::filesystem::path> paths;
  if (asset_folder.empty() || asset_folder.is_absolute()) {
    return paths;
  }

  const auto assets_root = ProjectManager::GetAssetsFolderPath();
  const auto folder = (assets_root / asset_folder).lexically_normal();
  if (!std::filesystem::is_directory(folder)) {
    return paths;
  }

  // Keep paths relative to Assets because ProjectManager resolves assets in that namespace.
  const auto append_if_supported = [&](const std::filesystem::directory_entry& entry) {
    if (entry.is_regular_file() && IsSupportedPrefabExtension(entry.path().extension().string())) {
      paths.emplace_back(std::filesystem::relative(entry.path(), assets_root));
    }
  };
  if (recursive) {
    for (const auto& entry : std::filesystem::recursive_directory_iterator(folder)) {
      append_if_supported(entry);
    }
  } else {
    for (const auto& entry : std::filesystem::directory_iterator(folder)) {
      append_if_supported(entry);
    }
  }
  std::sort(paths.begin(), paths.end());
  return paths;
}

std::vector<PrefabFieldPlacement> PrefabField::BuildPlacements(const size_t asset_count) const {
  std::vector<PrefabFieldPlacement> placements;
  if (rows <= 0 || columns <= 0 || asset_count == 0) {
    return placements;
  }

  placements.reserve(static_cast<size_t>(rows) * static_cast<size_t>(columns));
  std::mt19937 generator(seed);
  std::uniform_real_distribution<float> yaw_distribution(-random_yaw_degrees, random_yaw_degrees);
  std::vector<size_t> shuffled_assets(asset_count);
  std::iota(shuffled_assets.begin(), shuffled_assets.end(), 0);
  std::shuffle(shuffled_assets.begin(), shuffled_assets.end(), generator);
  const glm::vec2 origin = -0.5f * glm::vec2((columns - 1) * spacing.x, (rows - 1) * spacing.y);
  size_t placement_index = 0;
  for (int row = 0; row < rows; ++row) {
    for (int column = 0; column < columns; ++column) {
      if (placement_index > 0 && placement_index % asset_count == 0)
        std::shuffle(shuffled_assets.begin(), shuffled_assets.end(), generator);
      placements.push_back({shuffled_assets[placement_index % asset_count],
                            glm::vec3(origin.x + column * spacing.x, 0.0f, origin.y + row * spacing.y),
                            random_yaw_degrees > 0.0f ? yaw_distribution(generator) : 0.0f});
      ++placement_index;
    }
  }
  return placements;
}

PrefabFieldMeasurementRegion PrefabField::GetMeasurementRegion() const {
  if (rows <= 0 || columns <= 0 || !std::isfinite(illumination_roi) || illumination_roi <= 0.0f ||
      illumination_roi > 1.0f)
    return {};
  const int selected_rows = std::clamp(static_cast<int>(std::round(rows * illumination_roi)), 1, rows);
  const int selected_columns = std::clamp(static_cast<int>(std::round(columns * illumination_roi)), 1, columns);
  // An odd padding count leaves the extra buffer cell on the larger-index side.
  return {(rows - selected_rows) / 2, (columns - selected_columns) / 2, selected_rows, selected_columns};
}

void PrefabField::ClearField() const {
  last_illumination = {};
  const auto scene = GetScene();
  if (!scene || !scene->IsEntityValid(GetOwner())) {
    return;
  }
  for (const auto child : scene->GetChildren(GetOwner())) {
    if (scene->GetEntityName(child) == kGeneratedRootName) {
      scene->DeleteEntity(child);
      return;
    }
  }
}

std::vector<Entity> PrefabField::GenerateField() const {
  const auto scene = GetScene();
  if (!scene || !scene->IsEntityValid(GetOwner())) {
    return {};
  }

  const auto paths = CollectAssetPaths();
  const auto placements = BuildPlacements(paths.size());
  if (placements.empty()) {
    EVOENGINE_WARNING("PrefabField found no supported models in Assets/" + asset_folder.generic_string())
    return {};
  }

  // A large library may contain hundreds of variants; resolve only the subset selected for this field.
  std::vector<std::shared_ptr<Prefab>> prefabs(paths.size());
  for (const auto& placement : placements) {
    auto& prefab = prefabs[placement.asset_index];
    if (!prefab)
      prefab = std::dynamic_pointer_cast<Prefab>(ProjectManager::GetOrCreateAsset(paths[placement.asset_index]));
    if (!prefab) {
      EVOENGINE_ERROR("PrefabField failed to import " + paths[placement.asset_index].generic_string())
      return {};
    }
  }

  ClearField();
  const auto generated_root = scene->CreateEntity(kGeneratedRootName);
  scene->SetParent(generated_root, GetOwner());

  std::vector<Entity> entities;
  entities.reserve(placements.size());
  for (size_t index = 0; index < placements.size(); ++index) {
    const auto& placement = placements[index];
    const auto entity = prefabs[placement.asset_index]->ToEntity(scene, false, false);
    scene->SetEntityName(entity, "Field model " + std::to_string(index + 1));
    Transform transform;
    const auto rotation = model_rotation_degrees + glm::vec3(0.0f, placement.yaw_degrees, 0.0f);
    transform.value = glm::translate(placement.position) * glm::mat4_cast(glm::quat(glm::radians(rotation))) *
                      glm::scale(model_scale);
    scene->SetDataComponent(entity, transform);
    scene->SetParent(entity, generated_root);
    entities.emplace_back(entity);
  }
  TransformGraph::CalculateTransformGraphForDescendants(scene, generated_root);
  return entities;
}

std::vector<Entity> PrefabField::GetGeneratedEntities() const {
  const auto scene = GetScene();
  if (!scene || !scene->IsEntityValid(GetOwner())) {
    return {};
  }
  for (const auto child : scene->GetChildren(GetOwner())) {
    if (scene->GetEntityName(child) == kGeneratedRootName) {
      return scene->GetChildren(child);
    }
  }
  return {};
}

PrefabFieldIlluminationSummary PrefabField::SummarizeIllumination(std::vector<PrefabFieldIlluminationResult> models) {
  PrefabFieldIlluminationSummary summary;
  summary.models = std::move(models);
  for (const auto& model : summary.models) {
    summary.total_area += model.total_area;
    summary.total_flux += model.total_flux;
    summary.area_weighted_average_flux += model.average_flux * model.total_area;
    summary.mean_model_average_flux += model.average_flux;
    for (const auto& [name, value] : model.organs) {
      auto& organ = summary.organs[name];
      organ.total_area += value.total_area;
      organ.total_flux += value.total_flux;
      organ.average_flux += value.average_flux * value.total_area;
    }
  }
  if (summary.total_area > 0.0f) {
    summary.area_weighted_average_flux /= summary.total_area;
  }
  if (!summary.models.empty()) {
    summary.mean_model_average_flux /= static_cast<float>(summary.models.size());
  }
  for (auto& [name, organ] : summary.organs)
    organ.average_flux = organ.total_area > 0.0f ? organ.average_flux / organ.total_area : glm::vec3(0.0f);
  return summary;
}
