#include "PhysXEditorLayer.hpp"
#include "EditorLayer.hpp"
#include "PhysicsLayer.hpp"
#include "Resources.hpp"
#include "Scene.hpp"

using namespace evo_engine;

void PhysXEditorLayer::RegisterTypes(Application&) {
  InspectorRegistry::GetInstance().RegisterInspector<Collider>(
      [](InspectorContext& context, Collider& target) {
        return Inspect(context, target);
      },
      {}, "Collider");
  InspectorRegistry::GetInstance().RegisterInspector<RigidBody>(
      [](InspectorContext& context, RigidBody& target) {
        return Inspect(context, target);
      },
      {}, "RigidBody");
  InspectorRegistry::GetInstance().RegisterInspector<Joint>(
      [](InspectorContext& context, Joint& target) {
        return Inspect(context, target);
      },
      {}, "Joint");
  InspectorRegistry::GetInstance().RegisterInspector<PhysicsMaterial>(
      [](InspectorContext& context, PhysicsMaterial& target) {
        return Inspect(context, target);
      },
      {}, "PhysicsMaterial");
}

bool PhysXEditorLayer::Inspect(InspectorContext& context, Collider& target) {
  const auto& editor_layer = context.editor_layer;
  const char* rigid_body_shape[]{"Sphere", "Box", "Capsule"};
  bool status_changed = false;
  if (ImGui::Combo("Shape", reinterpret_cast<int*>(&target.shape_type_), rigid_body_shape,
                   IM_ARRAYSIZE(rigid_body_shape))) {
    status_changed = true;
  }
  editor_layer->DragAndDropButton<PhysicsMaterial>(target.physics_material_, "Physics Mat");
  if (const auto physics_material = target.physics_material_.Get<PhysicsMaterial>()) {
    if (ImGui::TreeNode("Material")) {
      Inspect(context, *physics_material);
      ImGui::TreePop();
    }
  }
  glm::vec3 new_param = target.shape_param_;
  switch (target.shape_type_) {
    case ShapeType::Sphere:
      if (ImGui::DragFloat("Radius", &new_param.x, 0.01f, 0.0001f))
        status_changed = true;
      break;
    case ShapeType::Box:
      if (ImGui::DragFloat3("XYZ Size", &new_param.x, 0.01f, 0.0f))
        status_changed = true;
      break;
    case ShapeType::Capsule:
      if (ImGui::DragFloat2("R/HalfH", &new_param.x, 0.01f, 0.0001f))
        status_changed = true;
      break;
  }
  if (status_changed) {
    target.SetShapeParam(new_param);
  }
  return status_changed;
}

bool PhysXEditorLayer::Inspect(InspectorContext& context, RigidBody& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  if (ImGui::TreeNodeEx("Colliders")) {
    int index = 0;
    for (auto& i : target.colliders_) {
      editor_layer->DragAndDropButton<Collider>(i, ("Collider " + std::to_string(index++)));
    }
    ImGui::TreePop();
  }

  ImGui::Checkbox("Draw bounds", &target.draw_bounds_);
  static auto display_bound_color = glm::vec4(0.0f, 1.0f, 0.0f, 0.2f);
  if (target.draw_bounds_)
    ImGui::ColorEdit4("Color:##SkinnedMeshRenderer", (float*)(void*)&display_bound_color);
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();
  glm::ivec2 iterations = glm::ivec2(target.min_position_iterations_, target.min_velocity_iterations_);
  if (ImGui::DragInt2("Solver iterations (P/V)", &iterations.x, 1, 0, 128)) {
    target.SetSolverIterations(iterations.x, iterations.y);
  }
  if (!target.static_) {
    const auto rigid_dynamic = static_cast<PxRigidDynamic*>(target.rigid_actor_);
    if (ImGui::Checkbox("Kinematic", &target.kinematic_)) {
      const bool new_val = target.kinematic_;
      target.kinematic_ = !target.kinematic_;
      target.SetKinematic(new_val);
    }
    if (ImGui::DragFloat("Density", &target.density_, 0.1f, 0.001f)) {
      target.density_ = glm::max(0.001f, target.density_);
      PxRigidBodyExt::updateMassAndInertia(*rigid_dynamic, target.density_, &target.mass_center_);
    }
    if (ImGui::DragFloat3("Center", &target.mass_center_.x, 0.1f, 0.001f)) {
      PxRigidBodyExt::updateMassAndInertia(*rigid_dynamic, target.density_, &target.mass_center_);
    }
    if (!target.kinematic_) {
      if (ApplicationContext::Get().IsPlaying()) {
        target.linear_velocity_ = rigid_dynamic->getLinearVelocity();
        target.angular_velocity_ = rigid_dynamic->getAngularVelocity();
      }
      if (ImGui::DragFloat3("Linear Velocity", &target.linear_velocity_.x, 0.01f)) {
        rigid_dynamic->setLinearVelocity(target.linear_velocity_);
      }
      if (ImGui::DragFloat("Linear Damping", &target.linear_damping_, 0.01f)) {
        rigid_dynamic->setLinearDamping(target.linear_damping_);
      }
      if (ImGui::DragFloat3("Angular Velocity", &target.angular_velocity_.x, 0.01f)) {
        rigid_dynamic->setAngularVelocity(target.angular_velocity_);
      }
      if (ImGui::DragFloat("Angular Damping", &target.angular_damping_, 0.01f)) {
        rigid_dynamic->setAngularDamping(target.angular_damping_);
      }

      static auto apply_value = glm::vec3(0.0f);
      ImGui::DragFloat3("Value", &apply_value.x, 0.01f);
      if (ImGui::Button("Apply force")) {
        AddForce(apply_value);
      }
      if (ImGui::Button("Apply torque")) {
        AddForce(apply_value);
      }
    }
  }
  bool static_changed = false;
  const bool saved_val = target.static_;
  if (!target.kinematic_) {
    ImGui::Checkbox("Static", &target.static_);
    if (target.static_ != saved_val) {
      static_changed = true;
    }
  }
  {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(target.shape_transform_, scale, rotation, trans, skew, perspective);
    skew = glm::degrees(glm::eulerAngles(rotation));
    bool shape_trans_changed = false;
    if (ImGui::DragFloat3("Center Position", &trans.x, 0.01f))
      shape_trans_changed = true;
    if (ImGui::DragFloat3("Rotation", &skew.x, 0.01f))
      shape_trans_changed = true;
    if (shape_trans_changed) {
      const auto new_value =
          glm::translate(trans) * glm::mat4_cast(glm::quat(glm::radians(skew))) * glm::scale(glm::vec3(1.0f));
      target.SetShapeTransform(new_value);
    }
    auto scene = target.GetScene();
    auto ltw = scene->GetDataComponent<GlobalTransform>(target.GetOwner());
    ltw.SetScale(glm::vec3(1.0f));
    for (auto& collider : target.colliders_) {
      switch (collider.Get<Collider>()->shape_type_) {
        case ShapeType::Sphere:
          if (target.draw_bounds_)
            editor_layer->DrawGizmoMesh(
                Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"), editor_layer->GetSceneCamera(), display_bound_color,
                ltw.value * (target.shape_transform_ * glm::scale(glm::vec3(collider.Get<Collider>()->shape_param_.x))),
                1);
          break;
        case ShapeType::Box:
          if (target.draw_bounds_)
            editor_layer->DrawGizmoMesh(
                Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), editor_layer->GetSceneCamera(), display_bound_color,
                ltw.value *
                    (target.shape_transform_ * glm::scale(glm::vec3(collider.Get<Collider>()->shape_param_) * 2.0f)),
                1);
          break;
        case ShapeType::Capsule:
          if (target.draw_bounds_)
            editor_layer->DrawGizmoMesh(
                Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), editor_layer->GetSceneCamera(), display_bound_color,
                ltw.value * (target.shape_transform_ * glm::scale(glm::vec3(collider.Get<Collider>()->shape_param_))),
                1);
          break;
      }
    }

    if (static_changed) {
      target.RecreateBody();
    }
  }
  return changed || static_changed;
}

bool PhysXEditorLayer::Inspect(InspectorContext& context, Joint& target) {
  const auto& editor_layer = context.editor_layer;
  bool changed = false;
  int type = static_cast<int>(target.joint_type_);
  const char* joint_type_names[]{"Fixed", "D6"};
  if (ImGui::Combo("Joint Type", &type, joint_type_names, IM_ARRAYSIZE(joint_type_names))) {
    target.SetType((JointType)type);
    changed = true;
  }
  const auto stored_rigid_body1 = target.rigid_body1.Get<RigidBody>();
  const auto stored_rigid_body2 = target.rigid_body2.Get<RigidBody>();
  if (editor_layer->DragAndDropButton<RigidBody>(target.rigid_body1, "Link 1"))
    changed = true;
  if (editor_layer->DragAndDropButton<RigidBody>(target.rigid_body2, "Link 2"))
    changed = true;
  if (target.rigid_body1.Get<RigidBody>() != stored_rigid_body1 ||
      target.rigid_body2.Get<RigidBody>() != stored_rigid_body2) {
    target.Unlink();
  }
  return changed;
}

bool PhysXEditorLayer::Inspect(InspectorContext& context, PhysicsMaterial& target) {
  bool changed = false;
  if (ImGui::DragFloat("Dynamic Friction", &target.dynamic_friction_)) {
    target.SetDynamicFriction(target.dynamic_friction_);
    changed = true;
  }
  if (ImGui::DragFloat("Static Friction", &target.static_friction_)) {
    target.SetStaticFriction(target.static_friction_);
    changed = true;
  }
  if (ImGui::DragFloat("Restitution", &target.restitution_)) {
    target.SetRestitution(target.restitution_);
    changed = true;
  }

  return changed;
}
