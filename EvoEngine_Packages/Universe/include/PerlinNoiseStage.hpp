#pragma once
#include <Application.hpp>
#include <TerrainConstructionStageBase.hpp>

namespace universe_package {
using namespace evo_engine;
class PerlinNoiseStage : public TerrainConstructionStageBase {
 public:
  void Process(glm::dvec3 point, double previous_result, double& elevation) override;
};

}  // namespace universe_package
