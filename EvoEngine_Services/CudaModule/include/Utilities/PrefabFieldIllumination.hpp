#pragma once

#include "EvoEngine_SDK_PCH.hpp"
#include "PrefabField.hpp"

namespace evo_engine {

/** Measures every generated model in a PrefabField with the OptiX triangle estimator. */
PrefabFieldIlluminationSummary EstimatePrefabFieldIllumination(PrefabField& field,
                                                               const PrefabFieldIlluminationSettings& settings);

}  // namespace evo_engine
