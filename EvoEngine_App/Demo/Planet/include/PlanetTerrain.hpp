#pragma once
#include "Application.hpp"
#include "IPrivateComponent.hpp"
#include "TerrainChunk.hpp"
#include "TerrainConstructionStageBase.hpp"
using namespace evo_engine;
namespace planet {
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
  MeshInfo(const unsigned index, const bool enabled = true) : index(index), enabled(enabled){};
};

class PlanetTerrain : public IPrivateComponent {
  friend class TerrainChunk;
  friend class PlanetTerrainSystem;
  std::vector<std::shared_ptr<TerrainChunk>> chunks_;
  PlanetInfo info_;
  // Used for fast mesh generation;
  std::vector<Vertex> shared_vertices_;
  std::vector<unsigned> shared_triangles_;
  bool initialized_ = false;

 public:
  void SetPlanetInfo(const PlanetInfo &planet_info);
  void Deserialize(const YAML::Node &in) override;

  void Serialize(YAML::Emitter &out) const override;
  void CollectAssetRef(std::vector<AssetRef> &list) override;
  AssetRef surface_material;
  std::vector<std::shared_ptr<TerrainConstructionStageBase>> terrain_construction_stages;
  void Init();
  bool OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) override;

  void Start() override;
  void PostCloneAction(const std::shared_ptr<IPrivateComponent> &target) override;
};
}  // namespace planet
