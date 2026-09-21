#include "RuntimeDebugGui.hpp"
#include "Application.hpp"
#include "Camera.hpp"
#include "GuiTextureRegistry.hpp"
#include "RuntimeGuiContext.hpp"
#include "Texture2D.hpp"
#include "Times.hpp"
using namespace evo_engine;

void RuntimeDebugGui::OnGui(RuntimeGuiContext& context) {
  const auto origin = context.GetOrigin();
  const auto size = context.GetSize();
  context.GetBackgroundDrawList()->AddRect({origin.x + 2, origin.y + 2}, {origin.x + size.x - 2, origin.y + size.y - 2},
                                           IM_COL32(80, 180, 220, 160));
  const ImVec2 marker{origin.x + size.x - 16, origin.y + 16};
  auto* foreground = context.GetForegroundDrawList();
  foreground->AddLine({marker.x - 6, marker.y}, {marker.x + 6, marker.y}, IM_COL32_WHITE);
  foreground->AddLine({marker.x, marker.y - 6}, {marker.x, marker.y + 6}, IM_COL32_WHITE);
  if (context.BeginWindow("Runtime debug")) {
    auto& times = ApplicationContext::Get().GetTimes();
    ImGui::Text("Frame %.2f ms", times.DeltaTime() * 1000.0);
    float time_step = static_cast<float>(times.TimeStep());
    if (ImGui::SliderFloat("Time step", &time_step, 0.001f, 0.1f, "%.3f s"))
      times.SetTimeStep(time_step);
    if (ImGui::Checkbox("Camera preview (shared asset setting)", &show_camera))
      SetUnsaved();
    if (context.BeginChild("Images", {0, 120}, ImGuiChildFlags_Borders | ImGuiChildFlags_ResizeY)) {
      if (const auto image = texture.Get<Texture2D>())
        ImGui::Image(GuiTextureRegistry::GetTextureId(*image), {128, 96}, {0, 1}, {1, 0});
      if (show_camera)
        if (const auto camera = context.GetCamera())
          ImGui::Image(GuiTextureRegistry::GetColorTextureId(*camera->GetRenderTexture()), {128, 96}, {0, 1}, {1, 0});
    }
    context.EndChild();
  }
  context.EndWindow();
}
void RuntimeDebugGui::Serialize(YAML::Emitter& out) const {
  texture.Save("texture", out);
  out << YAML::Key << "show_camera" << YAML::Value << show_camera;
}
void RuntimeDebugGui::Deserialize(const YAML::Node& in) {
  texture.Load("texture", in);
  if (in["show_camera"])
    show_camera = in["show_camera"].as<bool>();
}
void RuntimeDebugGui::CollectAssetRef(std::vector<AssetRef>& refs) {
  refs.push_back(texture);
}
