
#pragma once

namespace evo_engine {

/**
 * @brief Represents the physical and optical characteristics of the Earth's atmosphere.
 */
struct Atmosphere {
  float earth_radius = 6360;       ///< Earth radius in kilometers (usually Rg or Re in literature).
  float atmosphere_radius = 6420;  ///< Radius of the atmosphere in kilometers (usually R or Ra in literature).
  float hr = 7994;                 ///< Atmospheric thickness for uniform density (Rayleigh scattering, Hr).
  float hm = 1200;                 ///< Atmospheric thickness for uniform density (Mie scattering, Hm).

  float g = 0.76f;            ///< Mean cosine value for Mie scattering phase function.
  int num_samples = 16;       ///< Number of samples used for atmospheric calculations.
  int num_samples_light = 8;  ///< Number of samples used for light scattering calculations.
  float intensity = 1.0f;     ///< Intensity of the atmosphere, representing energy or brightness.
};

}  // namespace evo_engine
