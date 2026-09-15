#pragma once
#include <memory>
namespace evo_engine {
class Scene;
}
namespace l_system_package::simulation {
using evo_engine::Scene;
float NormalizeDayOfYear(float day);
bool IsInActiveSeason(const float simulation_day_of_year, const int season_start_day, const int season_end_day);
int ClampColorModeIndex(const int mode);
int ResolveEffectiveColorMode(const int selected_mode, const bool scene_plant_view_tint_enabled);
void ApplyGlobalPlantColorMode(const int selected_mode, const bool scene_plant_view_tint_enabled);
void ApplyPineStemOnlyMode(const std::shared_ptr<Scene>& scene, const bool stem_only_mode,
                           const bool regenerate_existing_pines);
}  // namespace l_system_package::simulation
