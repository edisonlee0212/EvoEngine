#pragma once

namespace evo_engine {
struct Atmosphere {
  float earth_radius = 6360;       // In the paper this is usually Rg or Re (radius ground, eart)
  float atmosphere_radius = 6420;  // In the paper this is usually R or Ra (radius atmosphere)
  float hr = 7994;                 // Thickness of the atmosphere if density was uniform (Hr) for Rayleigh scattering
  float hm = 1200;                 // Same as above but for Mie scattering (Hm)

  float g = 0.76f;  // Mean cosine for Mie scattering
  int num_samples = 16;
  int num_samples_light = 8;
  float intensity = 1.0f;
};
}  // namespace evo_engine