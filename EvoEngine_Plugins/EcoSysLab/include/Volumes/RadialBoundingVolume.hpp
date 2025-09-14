
#pragma once

#include <Mesh.hpp>

#include "IVolume.hpp"
#include "ShootModel.hpp"
#include "Skeleton.hpp"

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @struct RadialBoundingVolumeSlice
 * @brief Represents a slice of a radial bounding volume.
 */
struct RadialBoundingVolumeSlice {
  float m_maxDistance;  ///< The maximum distance for this slice.
};

/**
 * @class RadialBoundingVolume
 * @brief A class representing a radial bounding volume used for spatial calculations.
 */
class RadialBoundingVolume : public IVolume {
  std::vector<std::shared_ptr<Mesh>> m_boundMeshes;  ///< Mesh representations of the bounding volume.
  bool m_meshGenerated = false;                      ///< Flag indicating if the bounding mesh has been generated.

  /**
   * @brief Calculates the size-related parameters for the bounding volume.
   */
  void CalculateSizes();

 public:
  glm::vec4 m_displayColor = glm::vec4(0.0f, 0.0f, 1.0f, 0.5f);  ///< Display color of the bounding volume.
  float m_offset = 0.1f;                                         ///< Offset value for bounding volume calculations.

  /**
   * @brief Generates a random point within the bounding volume.
   * @return A random point within the volume.
   */
  [[nodiscard]] glm::vec3 GetRandomPoint() override;

  /**
   * @brief Selects a slice given a specific position.
   * @param position The 3D position to evaluate.
   * @return The slice index as an integer vector.
   */
  [[nodiscard]] glm::ivec2 SelectSlice(const glm::vec3& position) const;

  /**
   * @brief Computes the tip position of a given slice.
   * @param layer The layer index.
   * @param slice The slice index.
   * @return The computed tip position.
   */
  [[nodiscard]] glm::vec3 TipPosition(int layer, int slice) const;

  float m_maxHeight = 0.0f;  ///< Maximum height of the bounding volume.
  float m_maxRadius = 0.0f;  ///< Maximum radius of the bounding volume.

  /**
   * @brief Generates the mesh representation of the bounding volume.
   */
  void GenerateMesh();

  /**
   * @brief Forms the entity representation of the radial bounding volume.
   */
  void FormEntity();

  /**
   * @brief Converts the bounding volume data into a string representation.
   * @return A string representing the bounding volume.
   */
  std::string AsString();

  /**
   * @brief Populates the bounding volume from a given string representation.
   * @param string The string containing serialized bounding volume data.
   */
  void FromString(const std::string& string);

  /**
   * @brief Exports the bounding volume as an OBJ file.
   * @param filename The name of the output OBJ file.
   */
  void ExportAsObj(const std::string& filename);

  float m_displayScale = 0.2f;  ///< Display scale factor.
  int m_layerAmount = 8;        ///< Number of layers in the bounding volume.
  int m_sectorAmount = 8;       ///< Number of sectors in each layer.

  std::vector<std::vector<RadialBoundingVolumeSlice>> m_layers;  ///< Layers storing the radial bounding volume slices.
  std::vector<std::pair<float, std::vector<float>>> m_sizes;     ///< Size-related data for the bounding volume.
  float m_totalSize = 0;                                         ///< Total calculated size of the bounding volume.

  /**
   * @brief Computes the bounding volume based on a set of points.
   * @param points A vector of points to define the volume.
   */
  void CalculateVolume(const std::vector<glm::vec3>& points);

  /**
   * @brief Inspects the bounding volume in the editor.
   * @param editorLayer Shared pointer to the editor layer.
   * @return True if the asset content is unmodified during inspection, false otherwise.
   */
  bool OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) override;

  /**
   * @brief Resizes the bounding volumes based on new parameters.
   */
  void ResizeVolumes();

  /**
   * @brief Checks whether a given position is inside the bounding volume.
   * @param globalTransform The global transformation applied to the position.
   * @param position The position to check.
   * @return True if the position is within the volume, false otherwise.
   */
  bool InVolume(const GlobalTransform& globalTransform, const glm::vec3& position) override;

  /**
   * @brief Checks whether a given position is inside the bounding volume.
   * @param position The position to check.
   * @return True if the position is within the volume, false otherwise.
   */
  bool InVolume(const glm::vec3& position) override;

  /**
   * @brief Serializes the bounding volume to a YAML emitter.
   * @param out The YAML emitter where the volume data is stored.
   */
  void Serialize(YAML::Emitter& out) const override;

  /**
   * @brief Deserializes the bounding volume from a YAML node.
   * @param in The YAML node containing volume data.
   */
  void Deserialize(const YAML::Node& in) override;

  /**
   * @brief Applies augmentation based on a given value.
   * @param value The augmentation factor.
   */
  void Augmentation(float value);
};

}  // namespace eco_sys_lab_plugin
