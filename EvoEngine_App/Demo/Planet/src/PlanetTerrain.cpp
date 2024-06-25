#include "PlanetTerrain.hpp"
#include "EditorLayer.hpp"
#include "PlanetTerrainSystem.hpp"
#include "yaml-cpp/yaml.h"
void planet::PlanetTerrain::Serialize(YAML::Emitter &out) const {
  out << YAML::Key << "planet_info";
  out << YAML::BeginMap;
  out << YAML::Key << "max_lod_level" << YAML::Value << info_.max_lod_level;
  out << YAML::Key << "lod_distance" << YAML::Value << info_.lod_distance;
  out << YAML::Key << "radius" << YAML::Value << info_.radius;
  out << YAML::Key << "index" << YAML::Value << info_.index;
  out << YAML::Key << "resolution" << YAML::Value << info_.resolution;
  out << YAML::EndMap;

  surface_material.Save("surface_material", out);
}

void planet::PlanetTerrain::Deserialize(const YAML::Node &in) {
  auto info = in["planet_info"];
  PlanetInfo planet_info;
  planet_info.max_lod_level = info["max_lod_level"].as<unsigned>();
  planet_info.lod_distance = info["lod_distance"].as<double>();
  planet_info.radius = info["radius"].as<double>();
  planet_info.index = info["index"].as<unsigned>();
  planet_info.resolution = info["resolution"].as<unsigned>();
  SetPlanetInfo(planet_info);

  surface_material.Load("surface_material", in);
}
void planet::PlanetTerrain::Init() {
  if (initialized_)
    return;
  shared_vertices_ = std::vector<Vertex>();
  size_t resolution = info_.resolution;
  shared_vertices_.resize(resolution * resolution);
  shared_triangles_ = std::vector<unsigned>();
  shared_triangles_.resize((resolution - 1) * (resolution - 1) * 6);

  size_t tri_index = 0;
  for (size_t y = 0; y < resolution; y++) {
    for (size_t x = 0; x < resolution; x++) {
      size_t i = x + y * resolution;
      shared_vertices_[i].tex_coord =
          glm::vec2(static_cast<float>(x) / (resolution - 1), static_cast<float>(y) / (resolution - 1));
      if (x != resolution - 1 && y != resolution - 1) {
        shared_triangles_[tri_index] = i;
        shared_triangles_[tri_index + 1] = i + resolution + 1;
        shared_triangles_[tri_index + 2] = i + resolution;

        shared_triangles_[tri_index + 3] = i;
        shared_triangles_[tri_index + 4] = i + 1;
        shared_triangles_[tri_index + 5] = i + resolution + 1;
        tri_index += 6;
      }
    }
  }
  const auto scene = Application::GetActiveScene();
  auto self = scene->GetOrSetPrivateComponent<PlanetTerrain>(GetOwner()).lock();
  chunks_.clear();
  chunks_.push_back(
      std::make_shared<TerrainChunk>(self, nullptr, 0, glm::ivec2(0), ChunkDirection::Root, glm::dvec3(1, 0, 0)));
  chunks_.push_back(
      std::make_shared<TerrainChunk>(self, nullptr, 0, glm::ivec2(0), ChunkDirection::Root, glm::dvec3(0, 1, 0)));
  chunks_.push_back(
      std::make_shared<TerrainChunk>(self, nullptr, 0, glm::ivec2(0), ChunkDirection::Root, glm::dvec3(0, 0, 1)));
  chunks_.push_back(
      std::make_shared<TerrainChunk>(self, nullptr, 0, glm::ivec2(0), ChunkDirection::Root, glm::dvec3(-1, 0, 0)));
  chunks_.push_back(
      std::make_shared<TerrainChunk>(self, nullptr, 0, glm::ivec2(0), ChunkDirection::Root, glm::dvec3(0, -1, 0)));
  chunks_.push_back(
      std::make_shared<TerrainChunk>(self, nullptr, 0, glm::ivec2(0), ChunkDirection::Root, glm::dvec3(0, 0, -1)));

  std::mutex m;
  for (auto &chunk : chunks_) {
    chunk->GenerateTerrain(m, chunk);
    chunk->active = true;
  }
  initialized_ = true;
}

bool planet::PlanetTerrain::OnInspect(const std::shared_ptr<EditorLayer> &editor_layer) {
  return editor_layer->DragAndDropButton<Material>(surface_material, "Material");
}
void planet::PlanetTerrain::PostCloneAction(const std::shared_ptr<IPrivateComponent> &target) {
  info_ = std::static_pointer_cast<planet::PlanetTerrain>(target)->info_;
  initialized_ = false;
}
void planet::PlanetTerrain::Start() {
  Init();
}
void planet::PlanetTerrain::SetPlanetInfo(const PlanetInfo &planet_info) {
  info_ = planet_info;
  initialized_ = false;
  Init();
}
void planet::PlanetTerrain::CollectAssetRef(std::vector<AssetRef> &list) {
  list.push_back(surface_material);
}
