
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
   * @brief Relinks handles and scene references after deserialization.
   *
   * @param map The mapping of old to new handles.
   * @param scene The current scene that the component belongs to.
   */
  void Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene);
};
}  // namespace evo_engine
