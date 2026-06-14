
#pragma once

namespace digital_agriculture_package {
using namespace evo_engine;

/**
 * @struct SkyIlluminanceSnapshot
 * @brief Represents a snapshot of sky illuminance at a specific moment.
 */
struct SkyIlluminanceSnapshot {
  float m_ghi = 1000;   ///< Global horizontal irradiance (GHI) in watts per square meter.
  float m_azimuth = 0;  ///< Azimuth angle of the sun in degrees.
  float m_zenith = 0;   ///< Zenith angle of the sun in degrees.

  /**
   * @brief Computes the sun direction vector based on the azimuth and zenith angles.
   * @return A normalized 3D vector representing the sun's direction.
   */
  [[nodiscard]] glm::vec3 GetSunDirection();

  /**
   * @brief Computes the sun's intensity based on GHI.
   * @return The sun's intensity in watts per square meter.
   */
  [[nodiscard]] float GetSunIntensity();
};

/**
 * @class SkyIlluminance
 * @brief Represents sky illuminance data and provides functionality to manage it.
 */
class SkyIlluminance : public IAsset {
 public:
  std::map<float, SkyIlluminanceSnapshot> snapshots;  ///< Map of time values to sky illuminance snapshots.
  float min_time;                                     ///< Minimum recorded time in the dataset.
  float max_time;                                     ///< Maximum recorded time in the dataset.

  /**
   * @brief Retrieves a sky illuminance snapshot for the specified time.
   * @param time The time value for which to retrieve the snapshot.
   * @return A SkyIlluminanceSnapshot corresponding to the given time.
   */
  [[nodiscard]] SkyIlluminanceSnapshot Get(float time);

  /**
   * @brief Imports sky illuminance data from a CSV file.
   * @param path The file path of the CSV containing sky illuminance data.
   */
  void ImportCsv(const std::filesystem::path &path);
};

}  // namespace digital_agriculture_package
