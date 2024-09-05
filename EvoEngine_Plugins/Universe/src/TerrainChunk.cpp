#include "ProjectManager.hpp"

#include "PlanetTerrainSystem.hpp"
#include "TerrainChunk.hpp"

glm::dvec3 Universe::TerrainChunk::ChunkCenterPosition(const glm::dvec3 &planet_position, const double radius,
                                                     const glm::quat rotation) const {
  const int actual_detail_level = (int)glm::pow(2, detail_level);
  glm::dvec2 percent = glm::dvec2(0.5, 0.5) / (double)actual_detail_level;
  glm::dvec3 point = local_up +
                     (percent.x + (double)chunk_coordinate.x / (double)actual_detail_level - 0.5) * 2 * axis_a +
                     (percent.y + (double)chunk_coordinate.y / (double)actual_detail_level - 0.5) * 2 * axis_b;
  double x = rotation.x * 2.0f;
  double y = rotation.y * 2.0f;
  double z = rotation.z * 2.0f;
  double xx = rotation.x * x;
  double yy = rotation.y * y;
  double zz = rotation.z * z;
  double xy = rotation.x * y;
  double xz = rotation.x * z;
  double yz = rotation.y * z;
  double wx = rotation.w * x;
  double wy = rotation.w * y;
  double wz = rotation.w * z;

  glm::dvec3 res;
  res.x = (1.0f - (yy + zz)) * point.x + (xy - wz) * point.y + (xz + wy) * point.z;
  res.y = (xy + wz) * point.x + (1.0f - (xx + zz)) * point.y + (yz - wx) * point.z;
  res.z = (xz - wy) * point.x + (yz + wx) * point.y + (1.0f - (xx + yy)) * point.z;
  res = glm::normalize(res);
  glm::dvec3 ret = res * radius + planet_position;
  return ret;
}

Universe::TerrainChunk::TerrainChunk(const std::shared_ptr<PlanetTerrain> &planet_terrain,
                                   const std::shared_ptr<TerrainChunk> &parent, unsigned detail_level,
                                   glm::ivec2 chunk_coordinate, ChunkDirection direction, glm::dvec3 local_up) {
  this->planet_terrain_ = planet_terrain;
  this->chunk_coordinate = chunk_coordinate;
  this->detail_level = detail_level;
  this->parent = parent;
  this->local_up = local_up;
  this->axis_a = glm::dvec3(local_up.y, local_up.z, local_up.x);
  this->axis_b = glm::cross(local_up, axis_a);
  this->local_up = glm::normalize(this->local_up);
}

void Universe::TerrainChunk::Expand(std::mutex &mutex) {
  if (!active)
    return;
  if (!c0) {
    auto chunk0 = std::make_shared<TerrainChunk>(planet_terrain_.lock(), self_.lock(), detail_level + 1,
                                                 glm::ivec2(chunk_coordinate.x * 2, chunk_coordinate.y * 2 + 1),
                                                 ChunkDirection::UpperLeft, local_up);
    chunk0->self_ = chunk0;
    GenerateTerrain(mutex, chunk0);
    c0 = std::move(chunk0);
  }
  if (!c1) {
    auto chunk1 = std::make_shared<TerrainChunk>(planet_terrain_.lock(), self_.lock(), detail_level + 1,
                                                 glm::ivec2(chunk_coordinate.x * 2 + 1, chunk_coordinate.y * 2 + 1),
                                                 ChunkDirection::UpperRight, local_up);
    chunk1->self_ = chunk1;
    GenerateTerrain(mutex, chunk1);
    c1 = std::move(chunk1);
  }
  if (!c2) {
    auto chunk2 = std::make_shared<TerrainChunk>(planet_terrain_.lock(), self_.lock(), detail_level + 1,
                                                 glm::ivec2(chunk_coordinate.x * 2, chunk_coordinate.y * 2),
                                                 ChunkDirection::LowerLeft, local_up);
    chunk2->self_ = chunk2;
    GenerateTerrain(mutex, chunk2);
    c2 = std::move(chunk2);
  }
  if (!c3) {
    auto chunk3 = std::make_shared<TerrainChunk>(planet_terrain_.lock(), self_.lock(), detail_level + 1,
                                                 glm::ivec2(chunk_coordinate.x * 2 + 1, chunk_coordinate.y * 2),
                                                 ChunkDirection::LowerRight, local_up);
    chunk3->self_ = chunk3;
    GenerateTerrain(mutex, chunk3);
    c3 = std::move(chunk3);
  }
  c0->active = true;
  c1->active = true;
  c2->active = true;
  c3->active = true;
  active = false;
  children_active = true;
}

void Universe::TerrainChunk::GenerateTerrain(std::mutex &mutex, std::shared_ptr<TerrainChunk> &target_chunk) const {
  if (target_chunk->mesh) {
    Console::Error("Mesh Exist!");
  }
  const auto planet_terrain = planet_terrain_.lock();
  std::vector<Vertex> &vertices = planet_terrain->shared_vertices_;
  auto size = vertices.size();
  auto resolution = planet_terrain->info_.resolution;
  for (auto index = 0; index < size; index++) {
    const int actual_detail_level = (int)glm::pow(2, target_chunk->detail_level);
    int x = index % resolution;
    int y = index / resolution;
    glm::dvec2 percent = glm::dvec2(x, y) / (double)(resolution - 1) / (double)actual_detail_level;
    const glm::dvec2 global_percent =
        45.0 * glm::dvec2((percent.x + (double)target_chunk->chunk_coordinate.x / actual_detail_level - 0.5) * 2.0,
                          (percent.y + (double)target_chunk->chunk_coordinate.y / actual_detail_level - 0.5) * 2.0);
    const glm::dvec2 actual_percent =
        glm::dvec2(glm::tan(glm::radians(global_percent.x)), glm::tan(glm::radians(global_percent.y)));
    glm::dvec3 point_on_unit_cube =
        target_chunk->local_up + actual_percent.x * target_chunk->axis_a + actual_percent.y * target_chunk->axis_b;
    point_on_unit_cube = glm::normalize(point_on_unit_cube);
    double elevation = 1.0;

    double previous_result = 1.0;
    for (const auto &stage : planet_terrain->terrain_construction_stages) {
      stage->Process(point_on_unit_cube, previous_result, elevation);
      previous_result = elevation;
    }
    vertices.at(index).position = glm::vec3(point_on_unit_cube * planet_terrain->info_.radius * elevation);
  }
  std::lock_guard<std::mutex> lock(mutex);
  auto mesh = ProjectManager::CreateTemporaryAsset<Mesh>();
  VertexAttributes attributes{};
  attributes.tex_coord = true;

  mesh->SetVertices(attributes, planet_terrain->shared_vertices_, planet_terrain->shared_triangles_);
  target_chunk->mesh = std::move(mesh);
}

void Universe::TerrainChunk::Collapse() {
  if (!c0 || !c1 || !c2 || !c3)
    return;
  if (!c0->active || !c1->active || !c2->active || !c3->active)
    return;

  c0->active = false;
  c1->active = false;
  c2->active = false;
  c3->active = false;
  active = true;
  children_active = false;

  c0.reset();
  c1.reset();
  c2.reset();
  c3.reset();
}
