#include "LSystemSimulation.hpp"
#include "Scene.hpp"
#include "ScotsPine.hpp"
namespace l_system_package::simulation {
using namespace evo_engine;
float NormalizeDayOfYear(float day) {
  if (!std::isfinite(day)) {
    return 0.0f;
  }
  day = std::fmod(day, 365.0f);
  if (day < 0.0f) {
    day += 365.0f;
  }
  return day;
}

bool IsInActiveSeason(const float simulation_day_of_year, const int season_start_day, const int season_end_day) {
  const int day = static_cast<int>(std::floor(NormalizeDayOfYear(simulation_day_of_year)));
  const int start = std::clamp(season_start_day, 0, 364);
  const int end = std::clamp(season_end_day, 0, 364);
  if (start <= end) {
    return day >= start && day <= end;
  }
  return day >= start || day <= end;
}

int ClampColorModeIndex(const int mode) {
  return std::clamp(mode, 0, 6);
}

int ResolveEffectiveColorMode(const int selected_mode, const bool scene_plant_view_tint_enabled) {
  return scene_plant_view_tint_enabled ? ClampColorModeIndex(selected_mode) : 0;
}

void ApplyGlobalPlantColorMode(const int selected_mode, const bool scene_plant_view_tint_enabled) {
  const int effective_mode = ResolveEffectiveColorMode(selected_mode, scene_plant_view_tint_enabled);
  ScotsPine::SetGlobalColorMode(static_cast<ScotsPine::ColorMode>(effective_mode));
}

void ApplyPineStemOnlyMode(const std::shared_ptr<Scene>& scene, const bool stem_only_mode,
                           const bool regenerate_existing_pines) {
  ScotsPine::SetGenerateNeedleTopologyEnabled(!stem_only_mode);

  if (!regenerate_existing_pines || !scene) {
    return;
  }

  if (const auto* pine_entities_ptr = scene->UnsafeGetPrivateComponentOwnersList<ScotsPine>()) {
    const std::vector<Entity> pine_entities = *pine_entities_ptr;
    for (const auto& entity : pine_entities) {
      if (!scene->IsEntityValid(entity)) {
        continue;
      }
      if (const auto pine = scene->GetOrSetPrivateComponent<ScotsPine>(entity).lock()) {
        pine->GrowToTargetGDD();
      }
    }
  }
}
}  // namespace l_system_package::simulation
