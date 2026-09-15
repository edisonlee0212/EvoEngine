#include "DynamicStrandsComponentInspectors.hpp"
using namespace evo_engine;
using namespace eco_sys_lab_package;
void eco_sys_lab_package::DrawColliderBound(const IDsCollider& target, const std::shared_ptr<EditorLayer>& editor_layer,
                                            const std::shared_ptr<Camera>& editor_camera, const glm::vec4& color) {
  if (const auto* box = dynamic_cast<const DsBoxCollider*>(&target))
    DrawColliderBound(*box, editor_layer, editor_camera, color);
  else if (const auto* cylinder = dynamic_cast<const DsCylinderCollider*>(&target))
    DrawColliderBound(*cylinder, editor_layer, editor_camera, color);
  else if (const auto* sphere = dynamic_cast<const DsSphereCollider*>(&target))
    DrawColliderBound(*sphere, editor_layer, editor_camera, color);
}
