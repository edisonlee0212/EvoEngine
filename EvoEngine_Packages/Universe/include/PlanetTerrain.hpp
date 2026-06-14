#pragma once
#include "Application.hpp"
#include "IPrivateComponent.hpp"
#include "TerrainChunk.hpp"
#include "TerrainConstructionStageBase.hpp"

namespace universe_package {
using namespace evo_engine;
struct PlanetInfo {
  unsigned max_lod_level;
  double lod_distance;
  double radius;
  unsigned index;
  unsigned resolution;
};

struct MeshInfo {
  unsigned index;
  bool enabled;
  MeshInfo(const unsigned index, const bool enabled = true) : index(index), enabled(enabled) {
  }
};

class PlanetTerrain : public IPrivateComponent {
  friend class TerrainChunk;
  friend class UniverseLayer;
  std::vector<std::shared_ptr<TerrainChunk>> chunks_;
  PlanetInfo info_;
  // Used for fast mesh generation;
  std::vector<Vertex> shared_vertices_;
  std::vector<unsigned> shared_triangles_;
  bool initialized_ = false;

 public:
  void SetPlanetInfo(const PlanetInfo &planet_info);
  const PlanetInfo &GetPlanetInfo() const;
  void CollectAssetRef(std::vector<AssetRef> &list);
  AssetRef surface_material;
  std::vector<std::shared_ptr<TerrainConstructionStageBase>> terrain_construction_stages;
  void Init();

  void Start() override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent> &target) override;
};
}  // namespace universe_package
