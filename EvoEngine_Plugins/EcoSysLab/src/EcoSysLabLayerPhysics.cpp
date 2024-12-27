//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"
#ifdef CUDA_MODULE_PLUGIN
#  include <RayTracerLayer.hpp>
#endif

#include "ClassRegistry.hpp"
#include "DsColliders.hpp"
#include "DynamicTreeStrands.hpp"
#include "RenderLayer.hpp"
#include "Soil.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

void EcoSysLabLayer::StrandPhysics() const {
  if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
    const auto scene = GetScene();
    const std::vector<Entity>* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
    const auto for_each_dts_entity =
        [&](const std::function<void(const std::shared_ptr<DynamicTreeStrands>& dts)>& action) {
          if (dts_entities && !dts_entities->empty()) {
            for (const auto& i : *dts_entities) {
              const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(i).lock();
              action(dts);
            }
          }
        };
    for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
      if (!dts->dynamic_strands->WaitForUpload())
        dts->dynamic_strands->UpdateBindings();
    });
    if (dynamic_strands_settings_.enable_physics) {
      for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
        dts->InteractionStep();
        if (dts->enable_physics)
          dts->PhysicsStep(dynamic_strands_settings_.physics_parameters);
      });
    }
  }
}

void EcoSysLabLayer::StrandVisualization() const {
  if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
    const auto scene = GetScene();
    const std::vector<Entity>* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
    const auto for_each_dts_entity =
        [&](const std::function<void(const std::shared_ptr<DynamicTreeStrands>& dts)>& action) {
          if (dts_entities && !dts_entities->empty()) {
            for (const auto& i : *dts_entities) {
              const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(i).lock();
              action(dts);
            }
          }
        };
    const auto* box_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsBoxCollider>();
    const auto* sphere_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsSphereCollider>();
    const auto* cylinder_collider_entities = scene->UnsafeGetPrivateComponentOwnersList<DsCylinderCollider>();
    const auto for_each_collider_entity =
        [&](const std::function<void(const std::shared_ptr<IDsCollider>& dts)>& action) {
          if (box_collider_entities && !box_collider_entities->empty()) {
            for (const auto& i : *box_collider_entities) {
              const auto box_collider = scene->GetOrSetPrivateComponent<DsBoxCollider>(i).lock();
              action(std::dynamic_pointer_cast<IDsCollider>(box_collider));
            }
          }
          if (sphere_collider_entities && !sphere_collider_entities->empty()) {
            for (const auto& i : *sphere_collider_entities) {
              const auto sphere_collider = scene->GetOrSetPrivateComponent<DsSphereCollider>(i).lock();
              action(std::dynamic_pointer_cast<IDsCollider>(sphere_collider));
            }
          }
          if (cylinder_collider_entities && !cylinder_collider_entities->empty()) {
            for (const auto& i : *cylinder_collider_entities) {
              const auto cylinder_collider = scene->GetOrSetPrivateComponent<DsCylinderCollider>(i).lock();
              action(std::dynamic_pointer_cast<IDsCollider>(cylinder_collider));
            }
          }
        };
    if (dynamic_strands_settings_.enable) {
      for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
        dts->Visualization(visualization_camera_, dynamic_strands_settings_.visualization_parameters);
      });
    }
    if (const auto editor_layer = Application::GetLayer<EditorLayer>()) {
      for_each_collider_entity([&](const std::shared_ptr<IDsCollider>& collider) {
        collider->RenderBound(editor_layer, visualization_camera_, collider->bound_color);
      });
    }
  }
}

void EcoSysLabLayer::RegisterStrandRenderingProcedure() const {
  if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
    const auto scene = GetScene();
    const std::vector<Entity>* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeStrands>();
    const auto for_each_dts_entity =
        [&](const std::function<void(const std::shared_ptr<DynamicTreeStrands>& dts)>& action) {
          if (dts_entities && !dts_entities->empty()) {
            for (const auto& i : *dts_entities) {
              const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(i).lock();
              action(dts);
            }
          }
        };
    if (dynamic_strands_settings_.enable_rendering) {
      const auto editor_layer = Application::GetLayer<EditorLayer>();
      for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
        if (scene->IsEntityEnabled(dts->GetOwner()) && dts->IsEnabled()) {
          dts->RegisterRenderInstance(dynamic_strands_settings_.render_parameters);
          dts->RegisterFoliageRenderInstance(dynamic_strands_settings_.foliage_render_parameters);
        }
      });
    }
  }
}
