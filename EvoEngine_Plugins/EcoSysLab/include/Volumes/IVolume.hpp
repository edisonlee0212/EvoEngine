
#pragma once

#include <Transform.hpp>

using namespace evo_engine;

namespace eco_sys_lab_plugin {

/**
 * @brief Interface representing a volume in 3D space.
 *
 * This abstract class provides methods to determine whether a point is inside the volume
 * and to retrieve a random point within the volume.
 */
class IVolume : public IAsset {
 public:
  /**
   * @brief Retrieves a random point inside the volume.
   *
   * @return A random point inside the volume as a glm::vec3.
   */
  virtual glm::vec3 GetRandomPoint() {
    return glm::vec3(0.0f);
  }

  /**
   * @brief Checks whether a given position is inside the volume, considering a global transform.
   *
   * @param global_transform The global transform used to position the volume.
   * @param position The position to check.
   * @return True if the position is inside the volume, false otherwise.
   */
  virtual bool InVolume(const GlobalTransform& global_transform, const glm::vec3& position);

  /**
   * @brief Checks whether a given position is inside the volume.
   *
   * @param position The position to check.
   * @return True if the position is inside the volume, false otherwise.
   */
  virtual bool InVolume(const glm::vec3& position);

  /**
   * @brief Checks whether a set of positions are inside the volume, considering a global transform.
   *
   * @param global_transform The global transform used to position the volume.
   * @param positions A vector of positions to check.
   * @param results A vector of boolean values indicating whether each corresponding position is inside the volume.
   */
  virtual void InVolume(const GlobalTransform& global_transform, const std::vector<glm::vec3>& positions,
                        std::vector<bool>& results);

  /**
   * @brief Checks whether a set of positions are inside the volume.
   *
   * @param positions A vector of positions to check.
   * @param results A vector of boolean values indicating whether each corresponding position is inside the volume.
   */
  virtual void InVolume(const std::vector<glm::vec3>& positions, std::vector<bool>& results);
};

/**
 * @brief A spherical volume implementation of the IVolume interface.
 *
 * This class defines a spherical volume where points can be randomly sampled and checked for inclusion.
 */
class SphericalVolume : public IVolume {
 public:
  /// The radius of the spherical volume.
  glm::vec3 radius = glm::vec3(1.0f);

  /**
   * @brief Retrieves a random point inside the spherical volume.
   *
   * @return A random point inside the volume as a glm::vec3.
   */
  glm::vec3 GetRandomPoint() override;

  /**
   * @brief Checks whether a given position is inside the spherical volume, considering a global transform.
   *
   * @param global_transform The global transform used to position the volume.
   * @param position The position to check.
   * @return True if the position is inside the volume, false otherwise.
   */
  bool InVolume(const GlobalTransform& global_transform, const glm::vec3& position) override;

  /**
   * @brief Checks whether a given position is inside the spherical volume.
   *
   * @param position The position to check.
   * @return True if the position is inside the volume, false otherwise.
   */
  bool InVolume(const glm::vec3& position) override;
};

}  // namespace eco_sys_lab_plugin
