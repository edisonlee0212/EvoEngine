#pragma once
#include "IPrivateComponent.hpp"

namespace evo_engine {
class EditorLayer;
}

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @class DsIntersectionBoundaryMeshGroup
 * @brief Marker/group component that parents @c DsIntersectionBoundaryMesh children under a
 *        DynamicTreeStrands owner. Multiple groups may exist under the same owner.
 *
 * Moving/rotating the group applies a shared offset to every boundary mesh in that group.
 * Save / Intersect-and-export-all / Add-mesh operate only on this group.
 */
class DsIntersectionBoundaryMeshGroup : public IPrivateComponent {
 public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;
};

}  // namespace eco_sys_lab_plugin
