#include "DigitalAgricultureInspectionAdapters.hpp"
#include "DigitalAgricultureInspectorStates.hpp"
#include "DigitalAgricultureSerializationAdapters.hpp"
#include "EditorFileDialogs.hpp"
#include "rapidcsv.h"
using namespace digital_agriculture_package;
bool SkyIlluminanceInspector::Inspect(InspectorContext& context, SkyIlluminance& illuminance) {
  (void)context;
  bool changed = false;
  EditorFileDialogs::OpenFile(
      "Import CSV", "CSV", {".csv"},
      [&illuminance, &changed](const std::filesystem::path& path) {
        illuminance.ImportCsv(path);
        changed = true;
      },
      false);
  auto& time = ui_time;
  auto& snapshot = ui_snapshot;
  if (ImGui::SliderFloat("Time", &time, illuminance.min_time, illuminance.max_time)) {
    snapshot = illuminance.Get(time);
  }
  ImGui::Text("Ghi: %.3f", snapshot.m_ghi);
  ImGui::Text("Azimuth: %.3f", snapshot.m_azimuth);
  ImGui::Text("Zenith: %.3f", snapshot.m_zenith);
  return changed;
}
