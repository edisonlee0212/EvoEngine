//
// Created by Bosheng Li on 10/10/2021.
//

#include "PrivateComponentRef.hpp"
// #include "ProjectManager.hpp"
#include "Entities.hpp"
#include "Scene.hpp"
using namespace evo_engine;
bool PrivateComponentRef::Update() {
  if (entity_handle_.GetValue() == 0 || scene_.expired()) {
    Clear();
    return false;
  }
  if (value_.expired()) {
    const auto scene = scene_.lock();
    if (const auto entity = scene->GetEntity(entity_handle_); entity.GetIndex() != 0) {
      if (scene->HasPrivateComponent(entity, private_component_type_name_)) {
        value_ = scene->GetPrivateComponent(entity, private_component_type_name_);
        return true;
      }
    }
    Clear();
    return false;
  }
  return true;
}
void PrivateComponentRef::Clear() {
  value_.reset();
  entity_handle_ = handle_ = Handle(0);
  scene_.reset();
  private_component_type_name_ = {};
}
