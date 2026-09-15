#pragma once

#include "EvoEngineEditorAPI.hpp"

namespace evo_engine::editor_theme {
enum class Theme { Dark, Light };

EVOENGINE_EDITOR_API void Apply(Theme theme);
EVOENGINE_EDITOR_API void ApplyDefault();
EVOENGINE_EDITOR_API Theme GetCurrentTheme();
EVOENGINE_EDITOR_API void ApplyEvoEngineDark();
EVOENGINE_EDITOR_API void ApplyEvoEngineLight();
}  // namespace evo_engine::editor_theme
