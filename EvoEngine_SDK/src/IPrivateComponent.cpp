//
// Created by Bosheng Li on 8/13/2021.
//
#include "IPrivateComponent.hpp"
//#include <ProjectManager.hpp>
#include "Entities.hpp"

using namespace evo_engine;

PrivateComponentElement::PrivateComponentElement(
        size_t id, const std::shared_ptr<IPrivateComponent> &data, const Entity &owner,
        const std::shared_ptr<Scene> &scene) {
    type_index = id;
    private_component_data = data;
    private_component_data->owner_ = owner;
    private_component_data->scene_ = scene;
    private_component_data->OnCreate();
}

void PrivateComponentElement::ResetOwner(const Entity &new_owner, const std::shared_ptr<Scene> &scene) const {
    private_component_data->owner_ = new_owner;
    private_component_data->scene_ = scene;
}

std::shared_ptr<Scene> IPrivateComponent::GetScene() const {
    return scene_.lock();
}

bool IPrivateComponent::Started() const
{
    return started_;
}

bool IPrivateComponent::IsEnabled() const {
    return enabled_;
}

size_t IPrivateComponent::GetVersion() const {
    return version_;
}

Entity IPrivateComponent::GetOwner() const {
    return owner_;
}

void IPrivateComponent::SetEnabled(const bool &value) {
    if (enabled_ != value) {
        if (value) {
            OnEnable();
        } else {
            OnDisable();
        }
        enabled_ = value;
    }
}
