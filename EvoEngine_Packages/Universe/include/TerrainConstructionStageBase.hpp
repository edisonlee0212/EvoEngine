#pragma once
#include "Application.hpp"

namespace universe_package {
using namespace evo_engine;

class TerrainConstructionStageBase {
 public:
  virtual ~TerrainConstructionStageBase() = default;
  virtual void Process(glm::dvec3 point, double previous_result, double &elevation) = 0;
};
}  // namespace universe_package
