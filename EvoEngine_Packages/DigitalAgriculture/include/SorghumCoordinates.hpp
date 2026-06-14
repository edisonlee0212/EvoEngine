
#pragma once
#include "SorghumField.hpp"

namespace digital_agriculture_package {
using namespace evo_engine;

/**
 * @class SorghumCoordinates
 * @brief Represents the coordinates for placing sorghum plants in a field.
 *
 * This class handles the positioning, rotation variance, sampling, and import/export
 * of sorghum plant coordinates within the Digital Agriculture package.
 */
class SorghumCoordinates : public IAsset {
  friend class SorghumLayer;

 public:
  /// Reference to the asset responsible for generating sorghum models.
  AssetRef sorghum_generator;

  /// Scaling factor for sorghum placement.
  float factor = 1.0f;

  /// List of 2D positions representing sorghum placements.
  std::vector<glm::dvec2> positions;

  /// Variance in rotation applied to sorghum models.
  glm::vec3 rotation_variance = glm::vec3(0.0f);

  /// Sampling range in the X direction.
  glm::dvec2 sample_x = glm::dvec2(0.0);

  /// Sampling range in the Y direction.
  glm::dvec2 sample_y = glm::dvec2(0.0);

  /// Range of values in the X direction.
  glm::dvec2 x_range = glm::vec2(0, 0);

  /// Range of values in the Y direction.
  glm::dvec2 y_range = glm::vec2(0, 0);

  /**
   * @brief Applies the coordinate data to a SorghumField.
   * @param sorghum_field The sorghum field to modify.
   */
  void Apply(const std::shared_ptr<SorghumField>& sorghum_field);

  void Apply(SorghumField& sorghum_field);

  /**
   * @brief Applies coordinate transformations and placements to a SorghumField.
   * @param sorghum_field The sorghum field to modify.
   * @param offset The offset used for positioning.
   * @param i Index for differentiating placements.
   * @param radius The radius of influence for placement adjustments.
   * @param position_variance Additional variance in position.
   */
  void Apply(const std::shared_ptr<SorghumField>& sorghum_field, glm::dvec2& offset, unsigned i = 0,
             float radius = 2.5f, float position_variance = 0.0f);

  void Apply(SorghumField& sorghum_field, glm::dvec2& offset, unsigned i = 0, float radius = 2.5f,
             float position_variance = 0.0f);

  /**
   * @brief Imports coordinate data from a file.
   * @param path The file path to import from.
   */
  void ImportFromFile(const std::filesystem::path& path);

  /**
   * @brief Collects references to other assets used by this instance.
   * @param list The list where asset references should be added.
   */
  void CollectAssetRef(std::vector<AssetRef>& list);
};

/**
 * @brief Saves a list of data as a binary entry in a YAML file.
 * @tparam T The type of data to store.
 * @param name The key under which the binary data will be saved.
 * @param target The list of data to store.
 * @param out The YAML emitter to write to.
 */
template <typename T>
void SaveListAsBinary(const std::string& name, const std::vector<T>& target, YAML::Emitter& out) {
  if (!target.empty()) {
    out << YAML::Key << name << YAML::Value
        << YAML::Binary(static_cast<const unsigned char*>(static_cast<const void*>(target.data())),
                        target.size() * sizeof(T));
  }
}

/**
 * @brief Loads a list of data from a binary entry in a YAML file.
 * @tparam T The type of data to load.
 * @param name The key associated with the binary data.
 * @param target The list to populate with loaded data.
 * @param in The YAML node containing the data.
 */
template <typename T>
void LoadListFromBinary(const std::string& name, std::vector<T>& target, const YAML::Node& in) {
  if (in[name]) {
    const auto binary_list = in[name].as<YAML::Binary>();
    target.resize(binary_list.size() / sizeof(T));
    std::memcpy(target.data(), binary_list.data(), binary_list.size());
  }
}

}  // namespace digital_agriculture_package
