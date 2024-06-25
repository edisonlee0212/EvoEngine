//
// Created by Bosheng Li on 8/13/2021.
//

#include "EntityMetadata.hpp"
#include "Scene.hpp"
#include "Serialization.hpp"
using namespace evo_engine;

void EntityMetadata::Deserialize(const YAML::Node &in, const std::shared_ptr<Scene> &scene) {
  entity_name = in["n"].as<std::string>();
  entity_version = 1;
  entity_enabled = in["e"].as<bool>();
  entity_static = in["s"].as<bool>();
  entity_handle.value_ = in["h"].as<uint64_t>();
  ancestor_selected = false;
}

void EntityMetadata::Serialize(YAML::Emitter &out, const std::shared_ptr<Scene> &scene) const {
  out << YAML::BeginMap;
  {
    out << YAML::Key << "n" << YAML::Value << entity_name;
    out << YAML::Key << "h" << YAML::Value << entity_handle.value_;
    out << YAML::Key << "e" << YAML::Value << entity_enabled;
    out << YAML::Key << "s" << YAML::Value << entity_static;
    if (parent.GetIndex() != 0)
      out << YAML::Key << "p" << YAML::Value << scene->GetEntityHandle(parent);
    if (root.GetIndex() != 0)
      out << YAML::Key << "r" << YAML::Value << scene->GetEntityHandle(root);

#pragma region Private Components
    out << YAML::Key << "pc" << YAML::Value << YAML::BeginSeq;
    for (const auto &element : private_component_elements) {
      out << YAML::BeginMap;
      out << YAML::Key << "tn" << YAML::Value << element.private_component_data->type_name_;
      out << YAML::Key << "e" << YAML::Value << element.private_component_data->enabled_;
      element.private_component_data->Serialize(out);
      out << YAML::EndMap;
    }
    out << YAML::EndSeq;
#pragma endregion
  }
  out << YAML::EndMap;
}

void EntityMetadata::Clone(const std::unordered_map<Handle, Handle> &entity_map, const EntityMetadata &source,
                           const std::shared_ptr<Scene> &scene) {
  entity_handle = source.entity_handle;
  entity_name = source.entity_name;
  entity_version = source.entity_version;
  entity_enabled = source.entity_enabled;
  parent = source.parent;
  root = source.root;
  entity_static = source.entity_static;
  data_component_storage_index = source.data_component_storage_index;
  chunk_array_index = source.chunk_array_index;
  children = source.children;
  private_component_elements.resize(source.private_component_elements.size());
  for (int i = 0; i < private_component_elements.size(); i++) {
    private_component_elements[i].private_component_data = std::dynamic_pointer_cast<IPrivateComponent>(
        Serialization::ProduceSerializable(source.private_component_elements[i].private_component_data->GetTypeName(),
                                           private_component_elements[i].type_index));
    private_component_elements[i].private_component_data->scene_ = scene;
    private_component_elements[i].private_component_data->owner_ =
        source.private_component_elements[i].private_component_data->owner_;
    private_component_elements[i].private_component_data->OnCreate();
    Serialization::ClonePrivateComponent(private_component_elements[i].private_component_data,
                                         source.private_component_elements[i].private_component_data);
    private_component_elements[i].private_component_data->scene_ = scene;
    private_component_elements[i].private_component_data->Relink(entity_map, scene);
  }
  ancestor_selected = false;
}
