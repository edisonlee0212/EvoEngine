#pragma once
#include "EvoEngineAPI.hpp"

#include <memory>
#include <string>

namespace evo_engine {
class Camera;
class EditorLayer;
struct DrawSettings;
namespace procedural_noise {
class IProceduralNoise;
}

EVOENGINE_API void DrawCameraDebugViews(const Camera& camera, float debug_scale);
EVOENGINE_API bool DrawProceduralNoiseGraph(procedural_noise::IProceduralNoise& noise, const std::string& window_title,
                                            const std::shared_ptr<EditorLayer>& editor_layer);
EVOENGINE_API bool DrawSettingsGui(DrawSettings& draw_settings);
void RegisterSdkInspectionAdapters();
}  // namespace evo_engine
