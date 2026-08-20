#pragma once
#include "IPrivateComponent.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @class DsIntersectionBoundaryMeshGroup
 * @brief Marker component that identifies the group entity which parents all
 *        DsIntersectionBoundaryMesh child entities under a DynamicTreeStrands owner.
 *
 * Attach exactly one of these to a direct child of the DynamicTreeStrands entity.
 * All DsIntersectionBoundaryMesh entities are then created as children of that group entity.
 * Moving/rotating the group entity applies a shared offset to every boundary mesh at once.
 */
class DsIntersectionBoundaryMeshGroup : public IPrivateComponent {
 public:
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override {
    return false;
  }
};

}  // namespace eco_sys_lab_plugin
