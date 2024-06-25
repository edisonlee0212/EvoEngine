#include "ISystem.hpp"
#include "Application.hpp"
#include "Scene.hpp"
using namespace evo_engine;

ISystem::ISystem() {
  enabled_ = false;
}

void ISystem::Enable() {
  if (!enabled_) {
    enabled_ = true;
    OnEnable();
  }
}

void ISystem::Disable() {
  if (enabled_) {
    enabled_ = false;
    OnDisable();
  }
}
std::shared_ptr<Scene> ISystem::GetScene() const {
  return scene_.lock();
}
bool ISystem::Enabled() const {
  return enabled_;
}

float ISystem::GetRank() const {
  return rank_;
}

bool SystemRef::Update() {
  if (system_handle_.GetValue() == 0) {
    value_.reset();
    return false;
  }
  if (!value_.has_value() || value_->expired()) {
    auto current_scene = Application::GetActiveScene();
    auto system = current_scene->mapped_systems_.find(system_handle_);
    if (system != current_scene->mapped_systems_.end()) {
      value_ = system->second;
      return true;
    }
    Clear();
    return false;
  }
  return true;
}