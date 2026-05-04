//
// Created by Bosheng Li on 4/22/2022.
//
#include "ILayer.hpp"
#include "Application.hpp"
#include "Scene.hpp"

using namespace evo_engine;

Application& ILayer::GetApplication() const {
  return *application_;
}

void ILayer::OnInputEvent(const Input::InputEvent& input_event) {
  if (!subsequent_layer_.expired()) {
    subsequent_layer_.lock()->OnInputEvent(input_event);
  }
}
std::shared_ptr<ILayer> ILayer::GetSelf() const {
  return self_.lock();
}

std::string ILayer::GetLayerName() const {
  return layer_name_;
}

std::shared_ptr<Scene> ILayer::GetScene() const {
  return scene_.lock();
}
