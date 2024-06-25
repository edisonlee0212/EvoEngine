#pragma once
#include "Application.hpp"
#include "Mesh.hpp"
using namespace evo_engine;
namespace planet
{
enum class ChunkDirection
{
    Root,
    UpperLeft,
    UpperRight,
    LowerLeft,
    LowerRight
};
class PlanetTerrain;
class TerrainChunk
{
    std::weak_ptr<PlanetTerrain> planet_terrain_;
    std::weak_ptr<TerrainChunk> self_;
  public:
    std::shared_ptr<Mesh> mesh;
    // The level of detail, the larger the detail, the smaller the chunk will be.
    unsigned detail_level;
    // The chunk coordinate in which a chunk belongs to the face
    glm::ivec2 chunk_coordinate;
    ChunkDirection direction;
    std::weak_ptr<TerrainChunk> parent;

    bool children_active = false;
    bool active = false;
    // The index of four children, upperleft = 0, upperright = 1, lower left = 2, lower right = 3.
    std::shared_ptr<TerrainChunk> c0;
    std::shared_ptr<TerrainChunk> c1;
    std::shared_ptr<TerrainChunk> c2;
    std::shared_ptr<TerrainChunk> c3;
    glm::dvec3 local_up;
    glm::dvec3 axis_a;
    glm::dvec3 axis_b;
    glm::dvec3 ChunkCenterPosition(const glm::dvec3& planet_position, double radius, glm::quat rotation) const;
    TerrainChunk(
        const std::shared_ptr<PlanetTerrain>& planet_terrain,
        const std::shared_ptr<TerrainChunk>& parent,
        unsigned detail_level,
        glm::ivec2 chunk_coordinate,
        ChunkDirection direction,
        glm::dvec3 local_up);
    void Expand(std::mutex &mutex);
    void GenerateTerrain(std::mutex &mutex, std::shared_ptr<TerrainChunk> &target_chunk) const;
    void Collapse();
};
} // namespace Planet
