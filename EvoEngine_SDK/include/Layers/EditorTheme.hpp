#pragma once

namespace evo_engine::editor_theme {
enum class Theme { Dark, Light };

EVOENGINE_API void Apply(Theme theme);
EVOENGINE_API void ApplyDefault();
EVOENGINE_API Theme GetCurrentTheme();
EVOENGINE_API void ApplyEvoEngineDark();
EVOENGINE_API void ApplyEvoEngineLight();
}  // namespace evo_engine::editor_theme
