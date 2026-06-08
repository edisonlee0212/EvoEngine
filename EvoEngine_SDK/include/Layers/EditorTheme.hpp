#pragma once

namespace evo_engine::editor_theme {
enum class Theme { Dark, Light };

void Apply(Theme theme);
void ApplyDefault();
Theme GetCurrentTheme();
void ApplyEvoEngineDark();
void ApplyEvoEngineLight();
}  // namespace evo_engine::editor_theme
