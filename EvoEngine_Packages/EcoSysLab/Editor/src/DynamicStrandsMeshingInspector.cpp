#include "DynamicStrandsMeshingInspector.hpp"
#include "BufferExporter.hpp"
#include "DynamicStrands.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorWidgets.hpp"
using namespace eco_sys_lab_package;
using namespace evo_engine;
bool DynamicStrandsMeshingInspector::Inspect(InspectorContext& context, DsAlphaShapeMeshing& target) {
  return target.OnInspect(context.editor_layer);
}
void DynamicStrandsMeshingInspector::DrawStats(const DsAlphaShapeMeshing& target) {
  ImGui::Text((std::string("Uniform particles count: ") + std::to_string(target.uniform_particles.size())).c_str());
  ImGui::Text((std::string("Meshlet count: ") + std::to_string(target.delaunay_tetrahedrons.size())).c_str());
}
void DynamicStrandsMeshingInspector::DrawDsAlphaShapeMeshingSettings(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Render branches", &DsAlphaShapeMeshing::RefRenderSettings().branches_render_parameters.enabled);
  if (DsAlphaShapeMeshing::RefRenderSettings().branches_render_parameters.enabled) {
    if (ImGui::TreeNodeEx("Branch render settings")) {
      if (ImGui::Button("Rebuild branches pipelines")) {
        DsAlphaShapeMeshing::BuildBranchesRenderingPipelines();
      }
      InspectSettings(DsAlphaShapeMeshing::RefRenderSettings().branches_render_parameters, editor_layer);
      ImGui::TreePop();
    }
  }
  ImGui::Checkbox("GPU volume measure → CSV", &DsAlphaShapeMeshing::RefRenderSettings().enable_volume_measure);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "While the application is Playing: compute cumulative tet volume on GPU and append absolute + %% of initial "
        "to Metadata/alpha_volume_measure_<timestamp>.csv. Paused/stopped skips measure and CSV.");
  }
  ImGui::Checkbox("Render Visualization", &DsAlphaShapeMeshing::RefRenderSettings().visualization_rendering);
  if (DsAlphaShapeMeshing::RefRenderSettings().visualization_rendering) {
    ImGui::Checkbox("Render strands",
                    &DsAlphaShapeMeshing::RefRenderSettings().small_segments_visualization_render_parameters.enabled);
    if (DsAlphaShapeMeshing::RefRenderSettings().small_segments_visualization_render_parameters.enabled) {
      if (ImGui::TreeNodeEx("Strands render settings")) {
        InspectSettings(DsAlphaShapeMeshing::RefRenderSettings().small_segments_visualization_render_parameters,
                        editor_layer);
        ImGui::TreePop();
      }
    }
  } else {
    ImGui::Checkbox("Render splinters",
                    &DsAlphaShapeMeshing::RefRenderSettings().small_segments_render_parameters.enabled);
    if (DsAlphaShapeMeshing::RefRenderSettings().small_segments_render_parameters.enabled) {
      if (ImGui::TreeNodeEx("Splinter render settings")) {
        InspectSettings(DsAlphaShapeMeshing::RefRenderSettings().small_segments_render_parameters, editor_layer);
        ImGui::TreePop();
      }
    }
  }
}
bool DynamicStrandsMeshingInspector::Inspect(InspectorContext& context, DsAlphaShapeVisualizationParameters& target) {
  bool changed = false;
  if (ImGui::Checkbox("Uniform Particle", &target.render_uniform_particles))
    changed = true;
  if (target.render_uniform_particles) {
    if (ImGui::Combo("Uniform particle mode", {"Default", "Segment color", "Single Particles"},
                     target.uniform_particle_render_mode))
      changed = true;
    switch (target.uniform_particle_render_mode) {
      case 0: {
        if (ImGui::ColorEdit4("Uniform particle color", &target.uniform_particle_main.x))
          changed = true;
        break;
      }
    }
    if (ImGui::DragFloat("Uniform Particle multiplier", &target.uniform_particle_radius_multiplier, 0.1f, 0.1f, 1000.f))
      changed = true;
  }

  return changed;
}
bool DynamicStrandsMeshingInspector::Inspect(InspectorContext& context, DsKineticVoronoiMeshing& target) {
  return target.OnInspect(context.editor_layer);
}
void DynamicStrandsMeshingInspector::DrawStats(const DsKineticVoronoiMeshing& target) {
  ImGui::Text(
      (std::string("Segment Meshlets Vertices: ") + std::to_string(target.segment_meshlet_vertices.size())).c_str());
  ImGui::Text(
      (std::string("Segment Meshlets Triangles: ") + std::to_string(target.segment_meshlet_triangles.size())).c_str());
}
void DynamicStrandsMeshingInspector::DrawDsKineticVoronoiMeshingSettings(
    const std::shared_ptr<EditorLayer>& editor_layer) {
  auto& render_settings = DsKineticVoronoiMeshing::RefRenderSettings();
  ImGui::Checkbox("Render Segment Meshlets", &render_settings.segment_meshlet_render_parameters.enabled);
  ImGui::Checkbox("Volume measure → CSV (download+smooth)", &render_settings.enable_volume_measure);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "While Playing: every N frames, download meshlets, re-apply bark smooth, measure cumulative volume on CPU, "
        "and append absolute + %% of initial to Metadata/kinetic_volume_measure_<timestamp>.csv. "
        "GPU skinned volume is nearly rigid; this path captures bark-smooth volume change. "
        "Paused/stopped skips measure and CSV.");
  }
  if (ImGui::DragInt("Volume measure interval (frames)", &render_settings.volume_measure_interval_frames, 1, 1, 1000)) {
    render_settings.volume_measure_interval_frames = glm::max(1, render_settings.volume_measure_interval_frames);
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Download + bark smooth + CPU volume measure every N physics frames (default 10). "
        "Also drives volume-change heatmap buffer uploads while that color mode is active.");
  }
  if (render_settings.segment_meshlet_render_parameters.enabled) {
    if (ImGui::Button("Rebuild segment meshlet pipelines")) {
      DsKineticVoronoiMeshing::BuildSegmentMeshletsRenderingPipelines();
    }

    ImGui::Combo("Color mode",
                 {"Standard", "Normals", "UVs", "Pair", "Neighbor connectivity", "Neighbor tags",
                  "Volume change heatmap"},
                 render_settings.segment_meshlet_render_parameters.color_mode);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Neighbor tags: brown = -2 (bark), blue = -1 (interior/open), green = >=0 (lateral), "
          "magenta = <-2 (out of range).\n"
          "Volume change heatmap: white = 0%%, red = loss, blue = gain (clamped to +-30%%); "
          "CPU volumes after download+smooth, updated every Volume measure interval frames while Playing.");
    }

    ImGui::Checkbox("Debug neighbor connectivity",
                    &render_settings.segment_meshlet_render_parameters.debug_neighbor_connectivity);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Color meshlet faces by lateral-neighbor state (viewport + OBJ export): grey = never had a neighbor or "
          "neighbor removed by compact, brown = bark, red = pair disconnected, green = pair still connected.");
    }

    ImGui::DragFloat("UV height factor", &render_settings.segment_meshlet_render_parameters.uv_height_factor, 0.001f,
                     0.001f, 1.0f);
    ImGui::DragFloat("UV circum factor", &render_settings.segment_meshlet_render_parameters.uv_circum_factor, 1.0f,
                     1.0f, 50.0f, "%.0f");

    ImGui::DragFloat("Fracture distance", &render_settings.segment_meshlet_render_parameters.fracture_distance, 0.0001f,
                     0.0f, 2.0f, "%.4f");
  }
  (void)editor_layer;
}
void DynamicStrandsMeshingInspector::DrawStats(const DsMeshing& target) {
  if (const auto* alpha = dynamic_cast<const DsAlphaShapeMeshing*>(&target))
    DrawStats(*alpha);
  else if (const auto* kinetic = dynamic_cast<const DsKineticVoronoiMeshing*>(&target))
    DrawStats(*kinetic);
  else
    ImGui::TextUnformatted("no stats available");
}
