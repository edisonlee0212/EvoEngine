//
// Created by Bosheng Li on 4/22/2022.
//
#include "ILayer.hpp"
#include "Scene.hpp"

using namespace evo_engine;

void ILayer::OnInputEvent(const Input::InputEvent& input_event) {
  if (!subsequent_layer_.expired()) {
    subsequent_layer_.lock()->OnInputEvent(input_event);
  }
}

std::shared_ptr<Scene> ILayer::GetScene() const {
  return scene_.lock();
}
