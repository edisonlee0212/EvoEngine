
#pragma once
#include "SorghumDescriptor.hpp"

using namespace evo_engine;
namespace digital_agriculture_plugin {

class SorghumLeafTrait {
public:
  float leaf_length;
  float leaf_width;
  int leaf_index;
  float leaf_area;

  // the internode below the leaf
  float internode_length;
  /**
  * @brief Inspects the stem descriptor in the editor.
  * @param editor_layer Shared pointer to the editor layer.
  * @return True if content is not modified, false otherwise.
  */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);

  /**
  * @brief Serializes the stem descriptor to YAML.
  * @param out YAML emitter.
  */
  void Serialize(YAML::Emitter& out) const;

  /**
  * @brief Deserializes the stem descriptor from YAML.
  * @param in YAML node.
  */
  void Deserialize(const YAML::Node& in);
};

class SorghumTraitDescriptor : public IAsset {

 public:
  
  
  std::vector<SorghumLeafTrait> leaf_traits;

  // todo: make traits for stems/other organs
  float stem_length;


  /// @brief Default constructor.
  SorghumTraitDescriptor() = default;


  /**
   * @brief Inspects the current state in an editor environment.
   * @param editor_layer Editor layer reference.
   * @return True if the asset's content is not modified, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;


  /**
   * @brief Serializes the state to a YAML emitter.
   * @param out The YAML emitter to serialize to.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the state from a YAML node.
   * @param in The YAML node to deserialize from.
   */
  void Deserialize(const YAML::Node& in) override;
};
}  // namespace digital_agriculture_plugin