#pragma once

#include "EvoEngineAPI.hpp"
#include "EvoEngineEditorAPI.hpp"

#include <memory>
#include <string>

namespace evo_engine {
class EVOENGINE_API Camera;
class EVOENGINE_EDITOR_API EditorLayer;
struct EVOENGINE_API DrawSettings;
namespace procedural_noise {
class IProceduralNoise;
}

EVOENGINE_EDITOR_API void DrawCameraDebugViews(const Camera& camera, float debug_scale);
EVOENGINE_EDITOR_API bool DrawProceduralNoiseGraph(procedural_noise::IProceduralNoise& noise,
                                                   const std::string& window_title,
                                                   const std::shared_ptr<EditorLayer>& editor_layer);
EVOENGINE_EDITOR_API bool DrawSettingsGui(DrawSettings& draw_settings);
EVOENGINE_EDITOR_API void RegisterSdkInspectionAdapters();
}  // namespace evo_engine
