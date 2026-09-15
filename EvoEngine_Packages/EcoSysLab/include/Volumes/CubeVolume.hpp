#pragma once
#include "IVolume.hpp"
#include "Mesh.hpp"

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @class CubeVolume
 * @brief Represents a cubic volume used in EcoSysLab for procedural modeling.
 *
 * This class provides methods to check if points are inside the volume,
 * retrieve random points within the volume, and serialize/deserialize its data.
 */
class CubeVolume : public IVolume {
 public:
  /**
   * @brief Applies the bounding box of a given mesh to the volume.
   * @param mesh A shared pointer to the Mesh whose bounds will be applied.
   */
  void ApplyMeshBounds(const std::shared_ptr<Mesh>& mesh);

  /**
   * @brief The minimum and maximum bounds of the cubic volume.
   */
  Bound min_max_bound;

  /**
   * @brief Checks if a given position is inside the volume.
   * @param globalTransform The global transformation applied to the volume.
   * @param position A 3D position to check.
   * @return True if the position is inside the volume, otherwise false.
   */
  bool InVolume(const GlobalTransform& globalTransform, const glm::vec3& position) override;

  /**
   * @brief Checks if a given position is inside the volume.
   * @param position A 3D position to check.
   * @return True if the position is inside the volume, otherwise false.
   */
  bool InVolume(const glm::vec3& position) override;

  /**
   * @brief Retrieves a random point within the volume.
   * @return A random 3D point inside the cubic volume.
   */
  glm::vec3 GetRandomPoint() override;
};
}  // namespace eco_sys_lab_package
