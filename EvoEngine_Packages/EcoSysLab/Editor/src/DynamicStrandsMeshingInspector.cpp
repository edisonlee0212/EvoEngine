#include "DynamicStrandsMeshingInspector.hpp"
#include "BufferExporter.hpp"
#include "DynamicStrands.hpp"
#include "EcoSysLabSettingsEditor.hpp"
#include "EditorFileDialogs.hpp"
#include "EditorWidgets.hpp"
#include "ObjExporter.hpp"
using namespace eco_sys_lab_package;
using namespace evo_engine;
bool DynamicStrandsMeshingInspector::Inspect(InspectorContext& context, DsAlphaShapeMeshing& target) {
  return false;
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
  EditorFileDialogs::SaveFile(
      "Download and export PLY", "PLY", {".ply"},
      [&](const std::filesystem::path& path) {
        target.dynamic_strands->Download();
        EVOENGINE_LOG("Downloaded data from GPU");
        PlyExporter::ExportAscii(
            path, target.segment_meshlet_vertices, target.segment_meshlet_triangles,
            DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.uv_height_factor,
            DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.uv_circum_factor);
      },
      false);
  ImGui::SameLine();
  EditorFileDialogs::SaveFile(
      "Export PLY", "PLY", {".ply"},
      [&](const std::filesystem::path& path) {
        PlyExporter::ExportAscii(
            path, target.segment_meshlet_vertices, target.segment_meshlet_triangles,
            DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.uv_height_factor,
            DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.uv_circum_factor);
      },
      false);

  EditorFileDialogs::SaveFile(
      "Download and export OBJ", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        target.dynamic_strands->Download();
        EVOENGINE_LOG("Downloaded data from GPU");
        ObjExporter::ExportObj(
            path, target.segment_meshlet_vertices, target.segment_meshlet_triangles, target.dynamic_strands->segments,
            DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.uv_height_factor,
            DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.uv_circum_factor,
            DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.fracture_distance);
      },
      false);
  ImGui::SameLine();
  EditorFileDialogs::SaveFile(
      "Export OBJ", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        ObjExporter::ExportObj(
            path, target.segment_meshlet_vertices, target.segment_meshlet_triangles, target.dynamic_strands->segments,
            DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.uv_height_factor,
            DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.uv_circum_factor,
            DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.fracture_distance);
      },
      false);

  return false;
}
void DynamicStrandsMeshingInspector::DrawStats(const DsKineticVoronoiMeshing& target) {
  ImGui::Text(
      (std::string("Segment Meshlets Vertices: ") + std::to_string(target.segment_meshlet_vertices.size())).c_str());
  ImGui::Text(
      (std::string("Segment Meshlets Triangles: ") + std::to_string(target.segment_meshlet_triangles.size())).c_str());
}
void DynamicStrandsMeshingInspector::DrawDsKineticVoronoiMeshingSettings(
    const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::Checkbox("Render Segment Meshlets",
                  &DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.enabled);
  if (DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.enabled) {
    if (ImGui::Button("Rebuild segment meshlet pipelines")) {
      DsKineticVoronoiMeshing::BuildSegmentMeshletsRenderingPipelines();
    }

    ImGui::Combo("Color mode", {"Standard", "Normals", "UVs", "Pair"},
                 DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.color_mode);

    // uv factors
    ImGui::DragFloat("UV height factor",
                     &DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.uv_height_factor,
                     0.001f, 0.001f, 1.0f);
    ImGui::DragFloat("UV circum factor",
                     &DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.uv_circum_factor,
                     1.0f, 1.0f, 50.0f, "%.0f");

    ImGui::DragFloat("Fracture distance",
                     &DsKineticVoronoiMeshing::RefRenderSettings().segment_meshlet_render_parameters.fracture_distance,
                     0.0001f, 0.0f, 2.0f, "%.4f");
  }
}
void DynamicStrandsMeshingInspector::DrawStats(const DsMeshing& target) {
  if (const auto* alpha = dynamic_cast<const DsAlphaShapeMeshing*>(&target))
    DrawStats(*alpha);
  else if (const auto* kinetic = dynamic_cast<const DsKineticVoronoiMeshing*>(&target))
    DrawStats(*kinetic);
  else
    ImGui::TextUnformatted("no stats available");
}
