//
// Created by lllll on 11/1/2022.
//

#include "EcoSysLabLayer.hpp"

#include "ClassRegistry.hpp"
#include "DsColliders.hpp"
#include "DynamicTreeSkeleton.hpp"
#include "DynamicTreeStrands.hpp"
#include "RenderLayer.hpp"
#include "Soil.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

void EcoSysLabLayer::DynamicStrandSimulation() {
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
      dts->dynamic_strands->UpdateBindings();
    });
    for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
      dts->InteractionStep();
    });
    if (dynamic_strands_settings_.enable_physics || dynamic_strands_settings_.remaining_step > 0) {
      for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
        if (scene->IsEntityEnabled(dts->GetOwner()) && dts->IsEnabled() && dts->enable_physics)
          dts->PhysicsStep(dynamic_strands_settings_.physics_parameters);
      });
      if (dynamic_strands_settings_.remaining_step > 0)
        dynamic_strands_settings_.remaining_step--;
    }
    for_each_dts_entity([&](const std::shared_ptr<DynamicTreeStrands>& dts) {
      if (scene->IsEntityEnabled(dts->GetOwner()) && dts->IsEnabled()) {
        dts->dynamic_strands->RenderCompute();
      }
    });
  }
}

void EcoSysLabLayer::DynamicSkeletonPhysics() const {
  const auto scene = GetScene();
  const std::vector<Entity>* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeSkeleton>();
  const auto for_each_dts_entity =
      [&](const std::function<void(const std::shared_ptr<DynamicTreeSkeleton>& dts)>& action) {
        if (dts_entities && !dts_entities->empty()) {
          for (const auto& i : *dts_entities) {
            const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(i).lock();
            action(dts);
          }
        }
      };
  if (dynamic_skeleton_settings_.enable_physics) {
    for_each_dts_entity([&](const std::shared_ptr<DynamicTreeSkeleton>& dts) {
      if (dts->simulate) {
        dts->PhysicsStep(dynamic_skeleton_settings_.physics_parameters);
      }
    });
  }
}

void EcoSysLabLayer::DynamicSkeletonVisualization() const {
  if (const auto render_layer = Application::GetLayer<RenderLayer>()) {
    const auto scene = GetScene();
    const std::vector<Entity>* dts_entities = scene->UnsafeGetPrivateComponentOwnersList<DynamicTreeSkeleton>();
    const auto for_each_dts_entity =
        [&](const std::function<void(const std::shared_ptr<DynamicTreeSkeleton>& dts)>& action) {
          if (dts_entities && !dts_entities->empty()) {
            for (const auto& i : *dts_entities) {
              const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeSkeleton>(i).lock();
              action(dts);
            }
          }
        };
    if (dynamic_skeleton_settings_.enable_visualization) {
      for_each_dts_entity([&](const std::shared_ptr<DynamicTreeSkeleton>& dts) {
        dts->Visualization(visualization_camera_, dynamic_skeleton_settings_.visualization_parameters);
      });
    }
  }
}

void EcoSysLabLayer::DynamicStrandVisualization() const {
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
              if (scene->IsEntityEnabled(i) && box_collider->IsEnabled())
                action(std::dynamic_pointer_cast<IDsCollider>(box_collider));
            }
          }
          if (sphere_collider_entities && !sphere_collider_entities->empty()) {
            for (const auto& i : *sphere_collider_entities) {
              const auto sphere_collider = scene->GetOrSetPrivateComponent<DsSphereCollider>(i).lock();
              if (scene->IsEntityEnabled(i) && sphere_collider->IsEnabled())
                action(std::dynamic_pointer_cast<IDsCollider>(sphere_collider));
            }
          }
          if (cylinder_collider_entities && !cylinder_collider_entities->empty()) {
            for (const auto& i : *cylinder_collider_entities) {
              const auto cylinder_collider = scene->GetOrSetPrivateComponent<DsCylinderCollider>(i).lock();
              if (scene->IsEntityEnabled(i) && cylinder_collider->IsEnabled())
                action(std::dynamic_pointer_cast<IDsCollider>(cylinder_collider));
            }
          }
        };
    if (dynamic_strands_settings_.enable_visualization) {
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

    auto bound = scene->GetBound();
    bound.min = glm::min(bound.min, glm::vec3(-5.f, -1.f, -5.f));
    bound.max = glm::max(bound.max, glm::vec3(5.f, 5.f, 5.f));

    scene->SetBound(bound);

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
          Handle handle = dts->GetHandle();
          auto scene = dts->GetScene();
          auto owner = dts->GetOwner();

          dts->dynamic_strands->meshing->RegisterRenderInstances(handle, scene, owner);

          dts->RegisterFoliageRenderInstance(dynamic_strands_settings_.foliage_render_parameters);

          dts->RegisterSegmentPairRenderInstance(dynamic_strands_settings_.segment_pairs_render_parameters);
        }
      });
    }
  }
}