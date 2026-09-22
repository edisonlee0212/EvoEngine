#include "DsIntersectionBoundaryMeshGroup.hpp"
#include <fstream>
#include "BufferExporter.hpp"
#include "DsIntersectionBoundaryMesh.hpp"
#include "DsKineticVoronoiMeshing.hpp"
#include "DynamicTreeStrands.hpp"
#include "EditorLayer.hpp"
#include "Transform.hpp"
#include "Utilities.hpp"
#include "kinDS/kinDS/ObjExporter.hpp"

using namespace eco_sys_lab_plugin;

namespace {

struct IntersectAllResult {
  std::vector<MeshletObjExport::MeshGroup> export_groups;
  std::vector<std::pair<std::string, DsKineticVoronoiMeshing::IntersectionRunStats>> intersection_stats_rows;
};

IntersectAllResult CollectIntersectedMeshGroups(DsKineticVoronoiMeshing* dskvm, const std::shared_ptr<Scene>& scene,
                                                const Entity& group, const Entity& dts_owner) {
  IntersectAllResult result;
  if (!dskvm || !scene || !scene->IsEntityValid(dts_owner)) {
    return result;
  }

  const auto tree_gt = scene->GetDataComponent<GlobalTransform>(dts_owner);
  const bool collect_intersection_stats = DsKineticVoronoiMeshing::meshing_settings.collect_meshing_statistics;
  for (const auto& child : scene->GetChildren(group)) {
    if (!scene->HasPrivateComponent<DsIntersectionBoundaryMesh>(child)) {
      continue;
    }
    const auto ibm = scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMesh>(child).lock();
    if (!ibm || ibm->GetMesh().getTriangleCount() == 0) {
      continue;
    }
    const auto boundary_gt = scene->GetDataComponent<GlobalTransform>(child);
    DsKineticVoronoiMeshing::IntersectionRunStats intersection_stats;
    if (!dskvm->IntersectMeshletsWithBoundary(ibm->GetMesh(), boundary_gt, tree_gt,
                                              collect_intersection_stats ? &intersection_stats : nullptr, false)) {
      EVOENGINE_ERROR("Intersect and export all: intersection failed for entity " << child.GetIndex() << ".");
      continue;
    }
    MeshletObjExport::MeshGroup mesh_group;
    mesh_group.name = ibm->GetPath().stem().string();
    if (mesh_group.name.empty()) {
      mesh_group.name = "entity_" + std::to_string(child.GetIndex());
    }
    mesh_group.vertices = dskvm->segment_meshlet_vertices;
    mesh_group.triangles = dskvm->segment_meshlet_triangles;
    if (collect_intersection_stats) {
      result.intersection_stats_rows.emplace_back(mesh_group.name, intersection_stats);
    }
    result.export_groups.push_back(std::move(mesh_group));
  }
  dskvm->ResetMeshletsToGpu();
  return result;
}

void WriteIntersectionStatsIfNeeded(const std::filesystem::path& out_path,
                                    const std::vector<std::pair<std::string, DsKineticVoronoiMeshing::IntersectionRunStats>>&
                                        intersection_stats_rows) {
  if (!DsKineticVoronoiMeshing::meshing_settings.collect_meshing_statistics || intersection_stats_rows.empty()) {
    return;
  }
  const std::filesystem::path stats_base =
      out_path.parent_path() / (out_path.stem().string() + "_intersection_stats.csv");
  DsKineticVoronoiMeshing::WriteIntersectionStatisticsCsv(stats_base, intersection_stats_rows);
}

}  // namespace

bool DsIntersectionBoundaryMeshGroup::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  const auto scene = GetScene();
  const Entity group = GetOwner();
  if (!scene || !scene->IsEntityValid(group)) {
    return false;
  }
  const auto owner_meshing = DsKineticVoronoiMeshing::FindForEntity(scene, group);
  auto* dskvm = owner_meshing.meshing;
  const Entity dts_owner = owner_meshing.dts_owner;

  FileUtils::OpenFile(
      "Add intersection boundary mesh", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        try {
          kinDS::VoronoiMesh loaded_mesh = kinDS::ObjExporter::readMesh(path);
          const auto child = scene->CreateEntity("Intersection Mesh (" + path.stem().string() + ")");
          scene->SetParent(child, group);
          GlobalTransform child_gt{};
          child_gt.value = glm::mat4(1.0f);
          scene->SetDataComponent(child, child_gt);
          const auto ibm = scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMesh>(child).lock();
          if (ibm) {
            ibm->LoadMesh(std::move(loaded_mesh), path);
          }
          EVOENGINE_LOG("Added intersection boundary mesh from " << path.string() << ".");
          changed = true;
        } catch (const std::exception& ex) {
          EVOENGINE_ERROR("Failed to load intersection boundary OBJ: " << ex.what());
        }
      },
      false);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Add an OBJ boundary mesh as a child of this Intersection Meshes group.");
  }

  ImGui::SameLine();
  FileUtils::SaveFile(
      "Save intersection setup", "YAML", {".yml"},
      [&](const std::filesystem::path& save_path) {
        YAML::Emitter out;
        out << YAML::BeginMap;
        const auto group_gt = scene->GetDataComponent<GlobalTransform>(group);
        out << YAML::Key << "group_transform" << YAML::Value << group_gt.value;
        out << YAML::Key << "intersection_meshes" << YAML::Value << YAML::BeginSeq;
        for (const auto& child : scene->GetChildren(group)) {
          if (!scene->HasPrivateComponent<DsIntersectionBoundaryMesh>(child)) {
            continue;
          }
          const auto ibm = scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMesh>(child).lock();
          if (!ibm || ibm->GetMesh().getTriangleCount() == 0) {
            continue;
          }
          const auto gt = scene->GetDataComponent<GlobalTransform>(child);
          out << YAML::BeginMap;
          out << YAML::Key << "obj_path" << YAML::Value << ibm->GetPath().string();
          out << YAML::Key << "transform" << YAML::Value << gt.value;
          out << YAML::EndMap;
        }
        out << YAML::EndSeq;
        out << YAML::EndMap;
        std::ofstream ofs(save_path.string(), std::ofstream::out | std::ofstream::trunc);
        ofs << out.c_str();
        EVOENGINE_LOG("Saved intersection setup to " << save_path.string() << ".");
      },
      false);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Save this group's transform and its boundary meshes to a YAML file.");
  }

  const bool can_intersect_all = dskvm && dskvm->HasMeshedSegmentMeshlets();
  const auto disabled_tooltip = [&]() {
    if (!dskvm || !scene->IsEntityValid(dts_owner)) {
      ImGui::SetTooltip(
          "Could not find DynamicTreeStrands with Kinetic Voronoi meshing on this group, its parents, "
          "or elsewhere in the scene.");
    } else if (!dskvm->HasMeshedSegmentMeshlets()) {
      ImGui::SetTooltip(
          "No meshlets available on the resolved DynamicTreeStrands (including Tree children used by "
          "PhysicsDemo trunk experiments). Run meshing first.");
    }
  };

  if (!can_intersect_all) {
    ImGui::BeginDisabled();
  }
  FileUtils::SaveFile(
      "Intersect and export all", "OBJ", {".obj"},
      [&](const std::filesystem::path& out_path) {
        if (!dskvm) {
          EVOENGINE_ERROR("Intersect and export all: parent does not use DsKineticVoronoiMeshing.");
          return;
        }
        if (!scene->IsEntityValid(dts_owner)) {
          EVOENGINE_ERROR("Intersect and export all: could not find DynamicTreeStrands owner.");
          return;
        }
        auto result = CollectIntersectedMeshGroups(dskvm, scene, group, dts_owner);
        if (result.export_groups.empty()) {
          EVOENGINE_ERROR("Intersect and export all: no intersection meshes exported.");
          return;
        }
        const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(dts_owner).lock();
        if (!dts || !dts->dynamic_strands) {
          EVOENGINE_ERROR("Intersect and export all: DynamicTreeStrands is missing.");
          return;
        }
        MeshletObjExport::ExportObjCombined(
            out_path, result.export_groups, dts->dynamic_strands->segments,
            DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.uv_height_factor,
            DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.uv_circum_factor,
            DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance,
            dts->dynamic_strands->segment_pairs, dts->dynamic_strands->segment_data_list);
        EVOENGINE_LOG("Intersect and export all: exported " << result.export_groups.size() << " object(s) to "
                                                            << out_path.string() << ".");
        WriteIntersectionStatsIfNeeded(out_path, result.intersection_stats_rows);
      },
      false);
  if (!can_intersect_all) {
    ImGui::EndDisabled();
  }
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    if (!can_intersect_all) {
      disabled_tooltip();
    } else {
      ImGui::SetTooltip(
          "For each boundary mesh in this group, compute the intersection and export all results as a single OBJ "
          "(one object per boundary mesh, bark/interior materials). Does not modify simulation physics; restores "
          "pristine meshlets afterward.");
    }
  }

  if (ImGui::TreeNodeEx("Visualization Export", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextUnformatted("Color");
    ImGui::RadioButton("Strands##viz_color", reinterpret_cast<int*>(&MeshletObjExport::visualization_color_mode),
                       static_cast<int>(MeshletObjExport::VisualizationColorMode::Strands));
    ImGui::SameLine();
    ImGui::RadioButton("Segments##viz_color", reinterpret_cast<int*>(&MeshletObjExport::visualization_color_mode),
                       static_cast<int>(MeshletObjExport::VisualizationColorMode::Segments));
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Solid materials match Visualization Segment mode (Strand color / Segment color); "
          "duplicate colors share one material.");
    }

    ImGui::TextUnformatted("Object grouping");
    ImGui::RadioButton("Intersection meshes##intersection_viz_grouping",
                       reinterpret_cast<int*>(&MeshletObjExport::intersection_visualization_object_grouping),
                       static_cast<int>(MeshletObjExport::VisualizationObjectGrouping::IntersectionMeshes));
    ImGui::SameLine();
    ImGui::RadioButton("By highlight##intersection_viz_grouping",
                       reinterpret_cast<int*>(&MeshletObjExport::intersection_visualization_object_grouping),
                       static_cast<int>(MeshletObjExport::VisualizationObjectGrouping::ByHighlight));
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Intersection meshes: one object per boundary result (faces colored by Color above).\n"
          "By highlight: one object per strand or segment matching Color "
          "(prefixed by each intersection mesh name).");
    }

    if (!can_intersect_all) {
      ImGui::BeginDisabled();
    }
    ImGui::PushID("intersection_visualization_export_obj");
    FileUtils::SaveFile(
        "Intersect and export visualization OBJ", "OBJ", {".obj"},
        [&](const std::filesystem::path& out_path) {
          if (!dskvm) {
            EVOENGINE_ERROR("Intersect and export visualization: parent does not use DsKineticVoronoiMeshing.");
            return;
          }
          if (!scene->IsEntityValid(dts_owner)) {
            EVOENGINE_ERROR("Intersect and export visualization: could not find DynamicTreeStrands owner.");
            return;
          }
          auto result = CollectIntersectedMeshGroups(dskvm, scene, group, dts_owner);
          if (result.export_groups.empty()) {
            EVOENGINE_ERROR("Intersect and export visualization: no intersection meshes exported.");
            return;
          }
          const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(dts_owner).lock();
          if (!dts || !dts->dynamic_strands) {
            EVOENGINE_ERROR("Intersect and export visualization: DynamicTreeStrands is missing.");
            return;
          }
          dts->dynamic_strands->Download();
          MeshletObjExport::ExportVisualizationObjCombined(
              out_path, result.export_groups, dts->dynamic_strands->segments,
              MeshletObjExport::visualization_color_mode,
              MeshletObjExport::intersection_visualization_object_grouping,
              DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.uv_height_factor,
              DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.uv_circum_factor,
              DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance,
              dts->dynamic_strands->segment_pairs, dts->dynamic_strands->segment_data_list);
          EVOENGINE_LOG("Intersect and export visualization: exported " << result.export_groups.size()
                                                                        << " intersection mesh(es) to "
                                                                        << out_path.string() << ".");
          WriteIntersectionStatsIfNeeded(out_path, result.intersection_stats_rows);
        },
        false);
    ImGui::PopID();
    if (!can_intersect_all) {
      ImGui::EndDisabled();
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      if (!can_intersect_all) {
        disabled_tooltip();
      } else {
        ImGui::SetTooltip(
            "Same intersection pass as Intersect and export all, but writes Visualization solid-color materials "
            "with the object grouping selected above.");
      }
    }
    ImGui::TreePop();
  }

  return changed;
}
