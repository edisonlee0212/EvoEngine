#pragma once
#include "Application.hpp"
#include "Camera.hpp"
#include "ISystem.hpp"
#include "Material.hpp"
#include "PlanetTerrain.hpp"
using namespace evo_engine;
namespace planet {
class PlanetTerrainSystem : public ISystem {
  friend class PlanetTerrain;

 public:
  void Update() override;
  void FixedUpdate() override;
  static void CheckLod(std::mutex &mutex, const std::shared_ptr<TerrainChunk> &chunk, const PlanetInfo &info,
                       const GlobalTransform &planet_transform, const GlobalTransform &camera_transform);
  static void RenderChunk(const std::shared_ptr<TerrainChunk> &chunk, const std::shared_ptr<Material> &material,
                          glm::mat4 &matrix, bool receive_shadow);
};
}  // namespace planet