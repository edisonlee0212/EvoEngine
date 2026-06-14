#include "Application.hpp"
#include "PhysXSerializationAdapters.hpp"
#include "PhysicsLayer.hpp"
void evo_engine::PhysicsMaterial::OnCreate() {
  const auto physics_layer = ApplicationContext::Get().GetLayer<PhysicsLayer>();
  if (!physics_layer)
    return;
  value_ = physics_layer->physics_->createMaterial(static_friction_, dynamic_friction_, restitution_);
}
evo_engine::PhysicsMaterial::~PhysicsMaterial() {
  if (value_)
    value_->release();
}
void evo_engine::PhysicsMaterial::SetDynamicFriction(const float &value) {
  dynamic_friction_ = value;
  value_->setDynamicFriction(dynamic_friction_);
  SetUnsaved();
}
void evo_engine::PhysicsMaterial::SetStaticFriction(const float &value) {
  static_friction_ = value;
  value_->setStaticFriction(static_friction_);
  SetUnsaved();
}
void evo_engine::PhysicsMaterial::SetRestitution(const float &value) {
  restitution_ = value;
  value_->setRestitution(restitution_);
  SetUnsaved();
}
void evo_engine::PhysicsMaterial::OnGui() {
  if (ImGui::DragFloat("Dynamic Friction", &dynamic_friction_)) {
    SetDynamicFriction(dynamic_friction_);
  }
  if (ImGui::DragFloat("Static Friction", &static_friction_)) {
    SetStaticFriction(static_friction_);
  }
  if (ImGui::DragFloat("Restitution", &restitution_)) {
    SetRestitution(restitution_);
  }
}
void evo_engine::SerializePhysicsMaterial(YAML::Emitter &out, const PhysicsMaterial &target) {
  out << YAML::Key << "static_friction_" << YAML::Value << target.static_friction_;
  out << YAML::Key << "dynamic_friction_" << YAML::Value << target.dynamic_friction_;
  out << YAML::Key << "restitution_" << YAML::Value << target.restitution_;
}
void evo_engine::DeserializePhysicsMaterial(const YAML::Node &in, PhysicsMaterial &target) {
  target.static_friction_ = in["static_friction_"].as<float>();
  target.restitution_ = in["restitution_"].as<float>();
  target.dynamic_friction_ = in["dynamic_friction_"].as<float>();
  target.SetStaticFriction(target.static_friction_);
  target.SetRestitution(target.restitution_);
  target.SetDynamicFriction(target.dynamic_friction_);
}
