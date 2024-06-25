#pragma once
#include "Application.hpp"
using namespace evo_engine;
namespace planet {
class TerrainConstructionStageBase {
 public:
  virtual ~TerrainConstructionStageBase() = default;
  virtual void Process(glm::dvec3 point, double previous_result, double &elevation) = 0;
};
}  // namespace planet
