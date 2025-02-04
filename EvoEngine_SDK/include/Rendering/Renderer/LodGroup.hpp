
#pragma once

#include "IPrivateComponent.hpp"
#include "Mesh.hpp"
#include "PrivateComponentRef.hpp"

namespace evo_engine {

/**
 * @brief Represents a Level of Detail (LOD) with associated parameters for a renderer.
 */
class Lod {
 public:
  /**
   * @brief The index of the LOD in the sequence.
   */
  int index = 0;

  /**
   * @brief A list of references to private components that are part of this LOD.
   */
  std::vector<PrivateComponentRef> renderers;

  /**
   * @brief The LOD offset value used for distance calculations.
   */
  float lod_offset = 0.f;

  /**
   * @brief The transition width, determining the smoothness of LOD transitions.
   */
  float transition_width = 0.f;

  /**
   * @brief Inspects the LOD properties in the editor layer.
   *
   * @param editor_layer The editor layer to inspect the properties in.
   * @return Returns true if the inspection modified any properties, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer);
};

/**
 * @brief Represents a group of Level of Detail (LOD) configurations for a component.
 */
class LodGroup : public IPrivateComponent {
 public:
  /**
   * @brief A collection of LODs that belong to this group.
   */
  std::vector<Lod> lods;

  /**
   * @brief Determines whether the LOD factor is overridden.
   */
  bool override_lod_factor = false;

  /**
   * @brief The LOD factor that impacts transition calculations.
   */
  float lod_factor = 0.f;

  /**
   * @brief Serializes the LodGroup data to YAML format.
   *
   * @param out The YAML emitter to write the serialized data to.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the LodGroup data from a YAML node.
   *
   * @param in The YAML node containing the serialized LodGroup data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Renders the LodGroup's properties in the editor layer for inspection.
   *
   * @param editor_layer The editor layer used for inspecting the properties.
   * @return Returns true if the inspection modified any properties, otherwise false.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) override;

  /**
   * @brief Relinks handles and scene references after deserialization.
   *
   * @param map The mapping of old to new handles.
   * @param scene The current scene that the component belongs to.
   */
  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) override;
};
}  // namespace evo_engine
