#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "EditorFileDialogs.hpp"
#include "SorghumGenerator.hpp"
#include "SorghumLayer.hpp"
#include "TransformGraph.hpp"
using namespace digital_agriculture_package;
bool digital_agriculture_package::InspectSorghumCoordinates(InspectorContext& context,
                                                            SorghumCoordinates& coordinates) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (editor_layer->DragAndDropButton<SorghumGenerator>(coordinates.sorghum_generator, "SorghumGenerator")) {
    changed = true;
  }
  ImGui::Text("Available count: %d", coordinates.positions.size());
  ImGui::DragFloat("Distance factor", &coordinates.factor, 0.01f, 0.0f, 20.0f);
  ImGui::DragFloat3("Rotation variance", &coordinates.rotation_variance.x, 0.01f, 0.0f, 180.0f);

  ImGui::Text("X range: [%.3f, %.3f]", coordinates.x_range.x, coordinates.x_range.y);
  ImGui::Text("Y Range: [%.3f, %.3f]", coordinates.y_range.x, coordinates.y_range.y);

  if (ImGui::DragScalarN("Width range", ImGuiDataType_Double, &coordinates.sample_x.x, 2, 0.1f)) {
    coordinates.sample_x.x = glm::min(coordinates.sample_x.x, coordinates.sample_x.y);
    coordinates.sample_x.y = glm::max(coordinates.sample_x.x, coordinates.sample_x.y);
  }
  if (ImGui::DragScalarN("Length Range", ImGuiDataType_Double, &coordinates.sample_y.x, 2, 0.1f)) {
    coordinates.sample_y.x = glm::min(coordinates.sample_y.x, coordinates.sample_y.y);
    coordinates.sample_y.y = glm::max(coordinates.sample_y.x, coordinates.sample_y.y);
  }

  EditorFileDialogs::OpenFile(
      "Load Positions", "Position list", {".txt"},
      [&coordinates](const std::filesystem::path& path) {
        coordinates.ImportFromFile(path);
      },
      false);

  return changed;
}
