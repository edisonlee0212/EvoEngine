#pragma once
#include <filesystem>
#include "IPrivateComponent.hpp"
#include "kinDS/kinDS/VoronoiMesh.hpp"

namespace evo_engine {
class Mesh;
class Material;
class EditorLayer;
}  // namespace evo_engine

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @class DsIntersectionBoundaryMesh
 * @brief A child-entity component that holds one OBJ boundary mesh for intersecting with DsKineticVoronoiMeshing.
 *
 * Attach any number of these under a @c DsIntersectionBoundaryMeshGroup (itself under DynamicTreeStrands).
 * Each one shows Duplicate / Intersect in its inspector so the user can trigger independent clips.
 */
class DsIntersectionBoundaryMesh : public IPrivateComponent {
 public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
  void OnCreate() override;
  void OnDestroy() override;

  [[nodiscard]] const kinDS::VoronoiMesh& GetMesh() const {
    return mesh_;
  }
  [[nodiscard]] const std::filesystem::path& GetPath() const {
    return path_;
  }

  void LoadMesh(kinDS::VoronoiMesh mesh, std::filesystem::path path);

 private:
  kinDS::VoronoiMesh mesh_;
  std::filesystem::path path_;

  std::shared_ptr<Mesh> preview_mesh_;
  std::shared_ptr<Material> preview_material_;

  void UpdatePreviewMeshRenderer();
  void ClearMesh();
};

}  // namespace eco_sys_lab_plugin
