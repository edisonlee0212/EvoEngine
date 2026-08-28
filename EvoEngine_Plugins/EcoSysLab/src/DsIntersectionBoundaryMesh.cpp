#include "DsIntersectionBoundaryMesh.hpp"
#include "AssetManager.hpp"
#include "DsKineticVoronoiMeshing.hpp"
#include "DynamicStrands.hpp"
#include "DynamicTreeStrands.hpp"
#include "EditorLayer.hpp"
#include "MeshRenderer.hpp"
#include "Transform.hpp"

using namespace eco_sys_lab_plugin;

void DsIntersectionBoundaryMesh::OnCreate() {
}

void DsIntersectionBoundaryMesh::OnDestroy() {
  preview_mesh_.reset();
  preview_material_.reset();
}

void DsIntersectionBoundaryMesh::ClearMesh() {
  mesh_ = kinDS::VoronoiMesh();
  path_.clear();
  // Remove the MeshRenderer so the preview disappears.
  const auto scene = GetScene();
  if (scene) {
    const auto owner = GetOwner();
    if (scene->HasPrivateComponent<MeshRenderer>(owner)) {
      scene->RemovePrivateComponent<MeshRenderer>(owner);
    }
  }
  preview_mesh_.reset();
  preview_material_.reset();
}

void DsIntersectionBoundaryMesh::UpdatePreviewMeshRenderer() {
  const auto scene = GetScene();
  if (!scene) {
    return;
  }

  std::vector<Vertex> vertices;
  std::vector<unsigned> indices;
  vertices.reserve(mesh_.getTriangleCount() * 3);
  indices.reserve(mesh_.getTriangleCount() * 3);

  // Unique corners with non-degenerate UVs to avoid NaN tangents in deferred shading.
  static constexpr glm::vec2 kCornerUvs[3] = {{0.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}};
  const auto& mesh_vertices = mesh_.getVertices();
  const auto& triangles = mesh_.getTriangles();
  const bool has_normals = mesh_.getNormalMode() == kinDS::PerTriangleCorner && !mesh_.getNormals().empty();
  for (size_t tri = 0; tri < mesh_.getTriangleCount(); ++tri) {
    for (size_t corner = 0; corner < 3; ++corner) {
      const size_t src = triangles[tri * 3 + corner];
      Vertex v{};
      const glm::dvec3& p = mesh_vertices[src];
      v.position = glm::vec3(p);
      v.tex_coord = kCornerUvs[corner];
      if (has_normals) {
        const glm::dvec3& n = mesh_.getNormal(tri * 3 + corner);
        v.normal = glm::vec3(n);
      }
      vertices.push_back(v);
      indices.push_back(static_cast<unsigned>(indices.size()));
    }
  }

  if (!preview_mesh_) {
    preview_mesh_ = AssetManager::CreateTemporaryAsset<Mesh>();
  }
  VertexAttributes attrs{};
  attrs.normal = has_normals;
  attrs.tex_coord = true;
  preview_mesh_->SetVertices(attrs, vertices, indices);

  if (!preview_material_) {
    preview_material_ = AssetManager::CreateTemporaryAsset<Material>();
    preview_material_->material_properties.albedo_color = glm::vec3(0.25f, 0.7f, 1.0f);
    preview_material_->material_properties.roughness = 0.5f;
    preview_material_->material_properties.metallic = 0.0f;
  }

  const auto mr = scene->GetOrSetPrivateComponent<MeshRenderer>(GetOwner()).lock();
  mr->mesh = preview_mesh_;
  mr->material = preview_material_;
}

void DsIntersectionBoundaryMesh::LoadMesh(kinDS::VoronoiMesh mesh, std::filesystem::path path) {
  mesh_ = std::move(mesh);
  path_ = std::move(path);
  UpdatePreviewMeshRenderer();
}

bool DsIntersectionBoundaryMesh::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;

  if (mesh_.getTriangleCount() > 0) {
    ImGui::TextWrapped("File: %s", path_.string().c_str());
    ImGui::Text("Triangles: %zu", mesh_.getTriangleCount());
  }

  // --- Duplicate button ---
  if (mesh_.getTriangleCount() > 0) {
    if (ImGui::Button("Duplicate")) {
      const auto scene = GetScene();
      const Entity owner = GetOwner();
      if (scene && scene->IsEntityValid(owner)) {
        const Entity parent = scene->GetParent(owner);
        if (scene->IsEntityValid(parent)) {
          const auto src_gt = scene->GetDataComponent<GlobalTransform>(owner);
          const auto sibling = scene->CreateEntity(scene->GetEntityName(owner));
          scene->SetParent(sibling, parent);
          scene->SetDataComponent(sibling, src_gt);
          const auto ibm = scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMesh>(sibling).lock();
          if (ibm) {
            ibm->LoadMesh(kinDS::VoronoiMesh(mesh_), path_);
          }
          changed = true;
        }
      }
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Create a sibling entity with the same mesh and transform.");
    }
  }

  // --- Intersect button ---
  const bool has_mesh = mesh_.getTriangleCount() > 0;
  if (!has_mesh) {
    ImGui::BeginDisabled();
  }
  if (ImGui::Button("Intersect")) {
    // Walk up through group → DynamicTreeStrands entity.
    const auto scene = GetScene();
    const Entity owner = GetOwner();
    if (scene && scene->IsEntityValid(owner)) {
      // owner → group → DTS entity
      const Entity group = scene->GetParent(owner);
      const Entity parent = scene->IsEntityValid(group) ? scene->GetParent(group) : Entity{};
      if (scene->IsEntityValid(parent) && scene->HasPrivateComponent<DynamicTreeStrands>(parent)) {
        const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(parent).lock();
        if (dts && dts->dynamic_strands && dts->dynamic_strands->meshing) {
          auto* dskvm = dynamic_cast<DsKineticVoronoiMeshing*>(dts->dynamic_strands->meshing.get());
          if (dskvm) {
            const auto boundary_gt = scene->GetDataComponent<GlobalTransform>(owner);
            const auto tree_gt = scene->GetDataComponent<GlobalTransform>(parent);
            DsKineticVoronoiMeshing::IntersectionRunStats intersection_stats;
            const bool collect_stats = DsKineticVoronoiMeshing::meshing_settings.collect_meshing_statistics;
            if (dskvm->IntersectMeshletsWithBoundary(mesh_, boundary_gt, tree_gt,
                                                     collect_stats ? &intersection_stats : nullptr)) {
              if (collect_stats) {
                std::string name = path_.stem().string();
                if (name.empty()) {
                  name = "entity_" + std::to_string(owner.GetIndex());
                }
                std::filesystem::path stats_base = path_.empty()
                                                       ? std::filesystem::path(name + "_intersection_stats.csv")
                                                       : path_.parent_path() / (name + "_intersection_stats.csv");
                DsKineticVoronoiMeshing::WriteIntersectionStatisticsCsv(stats_base,
                                                                        {{std::move(name), intersection_stats}});
              }
              // Hide the preview after intersection (re-enable to reposition and intersect again).
              scene->SetEnable(owner, false);
              changed = true;
            }
          } else {
            EVOENGINE_ERROR(
                "DsIntersectionBoundaryMesh: parent DynamicTreeStrands does not use DsKineticVoronoiMeshing.");
          }
        }
      } else {
        EVOENGINE_ERROR("DsIntersectionBoundaryMesh: parent entity does not have a DynamicTreeStrands component.");
      }
    }
  }
  if (!has_mesh) {
    ImGui::EndDisabled();
  }
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip(
        "Clip the strand meshlets against this boundary mesh and rebuild GPU buffers. "
        "Requires a loaded OBJ. The parent entity must own a DynamicTreeStrands component. "
        "When Collect meshing statistics is enabled, writes a timestamped intersection CSV.");
  }

  return changed;
}
