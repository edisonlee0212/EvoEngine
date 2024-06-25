#include "RigidBody.hpp"
#include "Application.hpp"
#include "EditorLayer.hpp"
#include "PhysicsLayer.hpp"
#include "Scene.hpp"
#include "Transform.hpp"
using namespace evo_engine;

void RigidBody::SetStatic(bool value) {
  if (static_ == value)
    return;
  if (value) {
    SetKinematic(false);
  }
  static_ = value;
  RecreateBody();
}

void RigidBody::SetShapeTransform(const glm::mat4& value) {
  GlobalTransform ltw;
  ltw.value = value;
  ltw.SetScale(glm::vec3(1.0f));
  shape_transform_ = ltw.value;
}

void RigidBody::OnDestroy() {
  while (!colliders_.empty())
    DetachCollider(0);
  if (rigid_actor_) {
    rigid_actor_->release();
    rigid_actor_ = nullptr;
  }
}

void RigidBody::RecreateBody() {
  const auto physics_layer = Application::GetLayer<PhysicsLayer>();
  if (!physics_layer)
    return;
  if (rigid_actor_)
    rigid_actor_->release();
  auto scene = GetScene();
  auto owner = GetOwner();
  GlobalTransform global_transform;
  if (owner.GetIndex() != 0) {
    global_transform = scene->GetDataComponent<GlobalTransform>(owner);
    global_transform.value = global_transform.value * shape_transform_;
    global_transform.SetScale(glm::vec3(1.0f));
  }
  if (static_)
    rigid_actor_ = physics_layer->physics_->createRigidStatic(
        PxTransform(*static_cast<PxMat44*>(static_cast<void*>(&global_transform.value))));
  else
    rigid_actor_ = physics_layer->physics_->createRigidDynamic(
        PxTransform(*static_cast<PxMat44*>(static_cast<void*>(&global_transform.value))));

  if (!static_) {
    const auto rigid_dynamic = static_cast<PxRigidDynamic*>(rigid_actor_);
    rigid_dynamic->setSolverIterationCounts(min_position_iterations_, min_velocity_iterations_);
    PxRigidBodyExt::updateMassAndInertia(*rigid_dynamic, density_, &mass_center_);
    rigid_dynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, kinematic_);
    if (!kinematic_) {
      rigid_dynamic->setLinearDamping(linear_damping_);
      rigid_dynamic->setAngularDamping(angular_damping_);
      rigid_dynamic->setLinearVelocity(linear_velocity_);
      rigid_dynamic->setAngularVelocity(angular_velocity_);
      rigid_actor_->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, !gravity_);
    }
  }
  current_registered_ = false;
}

void RigidBody::OnCreate() {
  RecreateBody();
}

bool RigidBody::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNodeEx("Colliders")) {
    int index = 0;
    for (auto& i : colliders_) {
      editor_layer->DragAndDropButton<Collider>(i, ("Collider " + std::to_string(index++)));
    }
    ImGui::TreePop();
  }

  ImGui::Checkbox("Draw bounds", &draw_bounds_);
  static auto display_bound_color = glm::vec4(0.0f, 1.0f, 0.0f, 0.2f);
  if (draw_bounds_)
    ImGui::ColorEdit4("Color:##SkinnedMeshRenderer", (float*)(void*)&display_bound_color);
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();
  glm::ivec2 iterations = glm::ivec2(min_position_iterations_, min_velocity_iterations_);
  if (ImGui::DragInt2("Solver iterations (P/V)", &iterations.x, 1, 0, 128)) {
    SetSolverIterations(iterations.x, iterations.y);
  }
  if (!static_) {
    const auto rigid_dynamic = static_cast<PxRigidDynamic*>(rigid_actor_);
    if (ImGui::Checkbox("Kinematic", &kinematic_)) {
      const bool new_val = kinematic_;
      kinematic_ = !kinematic_;
      SetKinematic(new_val);
    }
    if (ImGui::DragFloat("Density", &density_, 0.1f, 0.001f)) {
      density_ = glm::max(0.001f, density_);
      PxRigidBodyExt::updateMassAndInertia(*rigid_dynamic, density_, &mass_center_);
    }
    if (ImGui::DragFloat3("Center", &mass_center_.x, 0.1f, 0.001f)) {
      PxRigidBodyExt::updateMassAndInertia(*rigid_dynamic, density_, &mass_center_);
    }
    if (!kinematic_) {
      if (Application::IsPlaying()) {
        linear_velocity_ = rigid_dynamic->getLinearVelocity();
        angular_velocity_ = rigid_dynamic->getAngularVelocity();
      }
      if (ImGui::DragFloat3("Linear Velocity", &linear_velocity_.x, 0.01f)) {
        rigid_dynamic->setLinearVelocity(linear_velocity_);
      }
      if (ImGui::DragFloat("Linear Damping", &linear_damping_, 0.01f)) {
        rigid_dynamic->setLinearDamping(linear_damping_);
      }
      if (ImGui::DragFloat3("Angular Velocity", &angular_velocity_.x, 0.01f)) {
        rigid_dynamic->setAngularVelocity(angular_velocity_);
      }
      if (ImGui::DragFloat("Angular Damping", &angular_damping_, 0.01f)) {
        rigid_dynamic->setAngularDamping(angular_damping_);
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
  const bool saved_val = static_;
  if (!kinematic_) {
    ImGui::Checkbox("Static", &static_);
    if (static_ != saved_val) {
      static_changed = true;
    }
  }
  {
    glm::vec3 scale;
    glm::vec3 trans;
    glm::quat rotation;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(shape_transform_, scale, rotation, trans, skew, perspective);
    skew = glm::degrees(glm::eulerAngles(rotation));
    bool shape_trans_changed = false;
    if (ImGui::DragFloat3("Center Position", &trans.x, 0.01f))
      shape_trans_changed = true;
    if (ImGui::DragFloat3("Rotation", &skew.x, 0.01f))
      shape_trans_changed = true;
    if (shape_trans_changed) {
      const auto new_value =
          glm::translate(trans) * glm::mat4_cast(glm::quat(glm::radians(skew))) * glm::scale(glm::vec3(1.0f));
      SetShapeTransform(new_value);
    }
    auto scene = GetScene();
    auto ltw = scene->GetDataComponent<GlobalTransform>(GetOwner());
    ltw.SetScale(glm::vec3(1.0f));
    for (auto& collider : colliders_) {
      switch (collider.Get<Collider>()->shape_type_) {
        case ShapeType::Sphere:
          if (draw_bounds_)
            editor_layer->DrawGizmoMesh(
                Resources::GetResource<Mesh>("PRIMITIVE_SPHERE"), display_bound_color,
                ltw.value * (shape_transform_ * glm::scale(glm::vec3(collider.Get<Collider>()->shape_param_.x))), 1);
          break;
        case ShapeType::Box:
          if (draw_bounds_)
            editor_layer->DrawGizmoMesh(
                Resources::GetResource<Mesh>("PRIMITIVE_CUBE"), display_bound_color,
                ltw.value * (shape_transform_ * glm::scale(glm::vec3(collider.Get<Collider>()->shape_param_) * 2.0f)),
                1);
          break;
        case ShapeType::Capsule:
          if (draw_bounds_)
            editor_layer->DrawGizmoMesh(
                Resources::GetResource<Mesh>("PRIMITIVE_CYLINDER"), display_bound_color,
                ltw.value * (shape_transform_ * glm::scale(glm::vec3(collider.Get<Collider>()->shape_param_))), 1);
          break;
      }
    }

    if (static_changed) {
      RecreateBody();
    }
  }
  return changed || static_changed;
}
void RigidBody::SetDensityAndMassCenter(float value, const glm::vec3& center) {
  if (static_) {
    EVOENGINE_ERROR("RigidBody is static!");
    return;
  }
  density_ = value;
  mass_center_ = PxVec3(center.x, center.y, center.z);
  PxRigidBodyExt::updateMassAndInertia(*reinterpret_cast<PxRigidDynamic*>(rigid_actor_), density_, &mass_center_);
}

void RigidBody::SetAngularVelocity(const glm::vec3& velocity) {
  if (static_) {
    EVOENGINE_ERROR("RigidBody is static!");
    return;
  }
  const auto rigid_dynamic = static_cast<PxRigidDynamic*>(rigid_actor_);
  angular_velocity_ = PxVec3(velocity.x, velocity.y, velocity.z);
  rigid_dynamic->setAngularVelocity(angular_velocity_);
}
void RigidBody::SetLinearVelocity(const glm::vec3& velocity) {
  if (static_) {
    EVOENGINE_ERROR("RigidBody is static!");
    return;
  }
  const auto rigid_dynamic = static_cast<PxRigidDynamic*>(rigid_actor_);
  linear_velocity_ = PxVec3(velocity.x, velocity.y, velocity.z);
  rigid_dynamic->setLinearVelocity(linear_velocity_);
}

bool RigidBody::IsKinematic() const {
  return kinematic_;
}

void RigidBody::SetKinematic(bool value) {
  if (static_) {
    EVOENGINE_ERROR("RigidBody is static!");
    return;
  }
  kinematic_ = value;
  static_cast<PxRigidBody*>(rigid_actor_)->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, kinematic_);
}
bool RigidBody::IsStatic() const {
  return static_;
}
void RigidBody::SetLinearDamping(float value) {
  if (static_) {
    EVOENGINE_ERROR("RigidBody is static!");
    return;
  }
  PxRigidDynamic* rigid_body = static_cast<PxRigidDynamic*>(rigid_actor_);
  linear_damping_ = value;
  rigid_body->setLinearDamping(linear_damping_);
}
void RigidBody::SetAngularDamping(float value) {
  if (static_) {
    EVOENGINE_ERROR("RigidBody is static!");
    return;
  }
  PxRigidDynamic* rigid_body = static_cast<PxRigidDynamic*>(rigid_actor_);
  angular_damping_ = value;
  rigid_body->setAngularDamping(angular_damping_);
}
void RigidBody::SetSolverIterations(unsigned position, unsigned velocity) {
  if (static_) {
    EVOENGINE_ERROR("RigidBody is static!");
    return;
  }
  PxRigidDynamic* rigid_body = static_cast<PxRigidDynamic*>(rigid_actor_);
  min_position_iterations_ = position;
  min_velocity_iterations_ = velocity;
  rigid_body->setSolverIterationCounts(min_position_iterations_, min_velocity_iterations_);
}
void RigidBody::SetEnableGravity(bool value) {
  gravity_ = value;
  rigid_actor_->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, !value);
}
void RigidBody::AttachCollider(std::shared_ptr<Collider>& collider) {
  collider->attach_count_++;
  colliders_.emplace_back(collider);
  if (rigid_actor_)
    rigid_actor_->attachShape(*collider->shape_);
}
void RigidBody::DetachCollider(size_t index) {
  if (rigid_actor_)
    rigid_actor_->detachShape(*colliders_[index].Get<Collider>()->shape_);
  colliders_[index].Get<Collider>()->attach_count_--;
  colliders_.erase(colliders_.begin() + index);
}
void RigidBody::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "shape_transform_" << YAML::Value << shape_transform_;
  out << YAML::Key << "draw_bounds_" << YAML::Value << draw_bounds_;
  out << YAML::Key << "static_" << YAML::Value << static_;
  out << YAML::Key << "density_" << YAML::Value << density_;
  out << YAML::Key << "mass_center_" << YAML::Value << mass_center_;
  out << YAML::Key << "linear_velocity_" << YAML::Value << linear_velocity_;
  out << YAML::Key << "angular_velocity_" << YAML::Value << angular_velocity_;
  out << YAML::Key << "kinematic_" << YAML::Value << kinematic_;
  out << YAML::Key << "linear_damping_" << YAML::Value << linear_damping_;
  out << YAML::Key << "angular_damping_" << YAML::Value << angular_damping_;
  out << YAML::Key << "min_position_iterations_" << YAML::Value << min_position_iterations_;
  out << YAML::Key << "min_velocity_iterations_" << YAML::Value << min_velocity_iterations_;
  out << YAML::Key << "gravity_" << YAML::Value << gravity_;

  if (!colliders_.empty()) {
    out << YAML::Key << "colliders_" << YAML::Value << YAML::BeginSeq;
    for (int i = 0; i < colliders_.size(); i++) {
      out << YAML::BeginMap;
      colliders_[i].Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
  }
}
void RigidBody::Deserialize(const YAML::Node& in) {
  shape_transform_ = in["shape_transform_"].as<glm::mat4>();
  draw_bounds_ = in["draw_bounds_"].as<bool>();
  static_ = in["static_"].as<bool>();
  density_ = in["density_"].as<float>();
  mass_center_ = in["mass_center_"].as<PxVec3>();
  linear_velocity_ = in["linear_velocity_"].as<PxVec3>();
  angular_velocity_ = in["angular_velocity_"].as<PxVec3>();
  kinematic_ = in["kinematic_"].as<bool>();
  linear_damping_ = in["linear_damping_"].as<float>();
  angular_damping_ = in["angular_damping_"].as<float>();
  min_position_iterations_ = in["min_position_iterations_"].as<unsigned>();
  min_velocity_iterations_ = in["min_velocity_iterations_"].as<unsigned>();
  gravity_ = in["gravity_"].as<bool>();
  RecreateBody();
  if (auto in_colliders = in["colliders_"]) {
    for (const auto& i : in_colliders) {
      AssetRef ref;
      ref.Deserialize(i);
      auto collider = ref.Get<Collider>();
      AttachCollider(collider);
    }
  }
}
void RigidBody::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
  auto ptr = std::static_pointer_cast<RigidBody>(target);
  colliders_.clear();
  shape_transform_ = ptr->shape_transform_;
  draw_bounds_ = ptr->draw_bounds_;
  density_ = ptr->density_;
  mass_center_ = ptr->mass_center_;
  static_ = ptr->static_;
  density_ = ptr->density_;
  mass_center_ = ptr->mass_center_;
  linear_velocity_ = ptr->linear_velocity_;
  angular_velocity_ = ptr->angular_velocity_;
  kinematic_ = ptr->kinematic_;
  linear_damping_ = ptr->linear_damping_;
  angular_damping_ = ptr->angular_damping_;
  min_position_iterations_ = ptr->min_position_iterations_;
  min_velocity_iterations_ = ptr->min_velocity_iterations_;
  gravity_ = ptr->gravity_;
  RecreateBody();
  for (auto& i : ptr->colliders_) {
    auto collider = i.Get<Collider>();
    AttachCollider(collider);
  }
}
void RigidBody::CollectAssetRef(std::vector<AssetRef>& list) {
  for (const auto& i : colliders_)
    list.push_back(i);
}
bool RigidBody::Registered() const {
  return current_registered_;
}
void RigidBody::AddForce(const glm::vec3& force) const {
  if (static_) {
    EVOENGINE_ERROR("RigidBody is static!");
    return;
  }
  if (kinematic_) {
    EVOENGINE_ERROR("RigidBody is kinematic!");
    return;
  }
  auto* rigid_body = static_cast<PxRigidBody*>(rigid_actor_);
  const auto px_force = PxVec3(force.x, force.y, force.z);
  rigid_body->addForce(px_force, PxForceMode::eFORCE);
}
void RigidBody::AddTorque(const glm::vec3& torque) const {
  if (static_) {
    EVOENGINE_ERROR("RigidBody is static!");
    return;
  }
  if (kinematic_) {
    EVOENGINE_ERROR("RigidBody is kinematic!");
    return;
  }
  auto* rigid_body = static_cast<PxRigidBody*>(rigid_actor_);
  const auto px_torque = PxVec3(torque.x, torque.y, torque.z);
  rigid_body->addTorque(px_torque, PxForceMode::eFORCE);
}