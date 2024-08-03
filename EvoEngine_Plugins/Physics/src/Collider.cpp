#include "Collider.hpp"
#include "Application.hpp"
#include "ClassRegistry.hpp"
#include "PhysicsLayer.hpp"
#include "EditorLayer.hpp"
using namespace evo_engine;

const char* rigid_body_shape[]{"Sphere", "Box", "Capsule"};

bool Collider::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool status_changed = false;
  if (ImGui::Combo("Shape", reinterpret_cast<int*>(&shape_type_), rigid_body_shape, IM_ARRAYSIZE(rigid_body_shape))) {
    status_changed = true;
  }
  EditorLayer::DragAndDropButton<PhysicsMaterial>(physics_material_, "Physics Mat");
  if (const auto physics_material = physics_material_.Get<PhysicsMaterial>()) {
    if (ImGui::TreeNode("Material")) {
      physics_material->OnGui();
      ImGui::TreePop();
    }
  }
  glm::vec3 new_param = shape_param_;
  switch (shape_type_) {
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
    SetShapeParam(new_param);
  }
  return status_changed;
}

void Collider::OnCreate() {
  const auto physics_layer = Application::GetLayer<PhysicsLayer>();
  if (!physics_layer)
    return;
  if (!physics_material_.Get<PhysicsMaterial>())
    physics_material_ = physics_layer->default_physics_material;
  shape_ = physics_layer->physics_->createShape(PxBoxGeometry(shape_param_.x, shape_param_.y, shape_param_.z),
                                                *physics_material_.Get<PhysicsMaterial>()->value_);
}

Collider::~Collider() {
  if (shape_ != nullptr) {
    shape_->release();
  }
}
void Collider::SetShapeType(const ShapeType& type) {
  const auto physics_layer = Application::GetLayer<PhysicsLayer>();
  if (!physics_layer)
    return;
  if (attach_count_ != 0) {
    EVOENGINE_ERROR("Unable to modify collider, attached to rigidbody!");
  }
  shape_type_ = type;
  switch (shape_type_) {
    case ShapeType::Sphere:
      shape_ = physics_layer->physics_->createShape(PxSphereGeometry(shape_param_.x),
                                                    *physics_material_.Get<PhysicsMaterial>()->value_, false);
      break;
    case ShapeType::Box:
      shape_ = physics_layer->physics_->createShape(PxBoxGeometry(shape_param_.x, shape_param_.y, shape_param_.z),
                                                    *physics_material_.Get<PhysicsMaterial>()->value_, false);
      break;
    case ShapeType::Capsule:
      shape_ = physics_layer->physics_->createShape(PxCapsuleGeometry(shape_param_.x, shape_param_.y),
                                                    *physics_material_.Get<PhysicsMaterial>()->value_, false);
      break;
  }
}
void Collider::SetMaterial(const std::shared_ptr<PhysicsMaterial>& material) {
  if (attach_count_ != 0) {
    EVOENGINE_ERROR("Unable to modify collider, attached to rigidbody!");
  }
  physics_material_ = material;
  PxMaterial* materials[1];
  materials[0] = material->value_;
  shape_->setMaterials(materials, 1);
}
void Collider::SetShapeParam(const glm::vec3& param) {
  if (attach_count_ != 0) {
    EVOENGINE_ERROR("Unable to modify collider, attached to rigidbody!");
  }
  shape_param_ = param;
  shape_param_ = glm::max(glm::vec3(0.001f), shape_param_);
  switch (shape_type_) {
    case ShapeType::Sphere:
      shape_->setGeometry(PxSphereGeometry(shape_param_.x));
      break;
    case ShapeType::Box:
      shape_->setGeometry(PxBoxGeometry(shape_param_.x, shape_param_.y, shape_param_.z));
      break;
    case ShapeType::Capsule:
      shape_->setGeometry(PxCapsuleGeometry(shape_param_.x, shape_param_.y));
      break;
  }
}
void Collider::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(physics_material_);
}
void Collider::Serialize(YAML::Emitter& out) const {
  physics_material_.Save("physics_material_", out);
  out << YAML::Key << "shape_param_" << YAML::Value << shape_param_;
  out << YAML::Key << "attach_count_" << YAML::Value << attach_count_;
  out << YAML::Key << "shape_type_" << YAML::Value << static_cast<unsigned>(shape_type_);
}
void Collider::Deserialize(const YAML::Node& in) {
  physics_material_.Load("physics_material_", in);
  shape_param_ = in["shape_param_"].as<glm::vec3>();
  shape_type_ = static_cast<ShapeType>(in["shape_type_"].as<unsigned>());
  SetShapeType(shape_type_);
  SetShapeParam(shape_param_);
  auto mat = physics_material_.Get<PhysicsMaterial>();
  if (!mat) {
    const auto physics_layer = Application::GetLayer<PhysicsLayer>();
    if (!physics_layer)
      return;
    mat = physics_layer->default_physics_material;
  }
  SetMaterial(mat);
  attach_count_ = in["attach_count_"].as<size_t>();
}