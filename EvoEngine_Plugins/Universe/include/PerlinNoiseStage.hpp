#pragma once
#include <Application.hpp>
#include <TerrainConstructionStageBase.hpp>
using namespace evo_engine;
namespace Universe {
class PerlinNoiseStage : public TerrainConstructionStageBase {
 public:
  void Process(glm::dvec3 point, double previous_result, double& elevation) override;
};

}  // namespace Universe