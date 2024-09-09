#include "PlanetTerrainSystem.hpp"
#include "ClassRegistry.hpp"
#include "RenderLayer.hpp"
#include "Scene.hpp"
using namespace universe_plugin;

void PlanetTerrainSystem::Update() {
  const auto scene = GetScene();
  const std::vector<Entity> *const planet_terrain_list = scene->UnsafeGetPrivateComponentOwnersList<PlanetTerrain>();
  if (planet_terrain_list == nullptr)
    return;

  if (const auto main_camera = scene->main_camera.Get<Camera>()) {
    std::mutex mesh_gen_lock;
    const auto camera_ltw = scene->GetDataComponent<GlobalTransform>(main_camera->GetOwner());
    for (auto i = 0; i < planet_terrain_list->size(); i++) {
      const auto planet_terrain = scene->GetOrSetPrivateComponent<PlanetTerrain>(planet_terrain_list->at(i)).lock();
      if (!planet_terrain->IsEnabled())
        continue;
      auto &planet_info = planet_terrain->info_;
      auto planet_transform = scene->GetDataComponent<GlobalTransform>(planet_terrain->GetOwner());
      auto &planet_chunks = planet_terrain->chunks_;
      // 1. Scan and expand.
      for (auto &chunk : planet_chunks) {
        // futures.push_back(_PrimaryWorkers->Share([&, this](int id) { CheckLod(meshGenLock, chunk, planetInfo,
        // planetTransform, cameraLtw); }).share());
        CheckLod(mesh_gen_lock, chunk, planet_info, planet_transform, camera_ltw);
      }

      glm::mat4 matrix = glm::scale(
          glm::translate(glm::mat4_cast(planet_transform.GetRotation()), glm::vec3(planet_transform.GetPosition())),
          glm::vec3(1.0f));
      auto material = planet_terrain->surface_material.Get<Material>();
      if (material) {
        for (auto j = 0; j < planet_chunks.size(); j++) {
          RenderChunk(planet_chunks[j], material, matrix, true);
        }
      }
    }
  }
}

void PlanetTerrainSystem::FixedUpdate() {
}

void PlanetTerrainSystem::CheckLod(std::mutex &mutex, const std::shared_ptr<TerrainChunk> &chunk,
                                   const PlanetInfo &info, const GlobalTransform &planet_transform,
                                   const GlobalTransform &camera_transform) {
  if (glm::distance(glm::dvec3(chunk->ChunkCenterPosition(planet_transform.GetPosition(), info.radius,
                                                          planet_transform.GetRotation())),
                    glm::dvec3(camera_transform.GetPosition())) <
      info.lod_distance * info.radius / glm::pow(2, chunk->detail_level + 1)) {
    if (chunk->detail_level < info.max_lod_level) {
      chunk->Expand(mutex);
    }
  }
  if (chunk->c0)
    CheckLod(mutex, chunk->c0, info, planet_transform, camera_transform);
  if (chunk->c1)
    CheckLod(mutex, chunk->c1, info, planet_transform, camera_transform);
  if (chunk->c2)
    CheckLod(mutex, chunk->c2, info, planet_transform, camera_transform);
  if (chunk->c3)
    CheckLod(mutex, chunk->c3, info, planet_transform, camera_transform);
  if (glm::distance(glm::dvec3(chunk->ChunkCenterPosition(planet_transform.GetPosition(), info.radius,
                                                          planet_transform.GetRotation())),
                    glm::dvec3(camera_transform.GetPosition())) >
      info.lod_distance * info.radius / glm::pow(2, chunk->detail_level + 1)) {
    chunk->Collapse();
  }
}

void PlanetTerrainSystem::RenderChunk(const std::shared_ptr<TerrainChunk> &chunk,
                                      const std::shared_ptr<Material> &material, glm::mat4 &matrix,
                                      bool receive_shadow) {
  if (chunk->active) {
    const auto render_layer = Application::GetLayer<RenderLayer>();
    render_layer->DrawMesh(chunk->mesh, material, matrix, true);
  }
  if (chunk->children_active) {
    RenderChunk(chunk->c0, material, matrix, receive_shadow);
    RenderChunk(chunk->c1, material, matrix, receive_shadow);
    RenderChunk(chunk->c2, material, matrix, receive_shadow);
    RenderChunk(chunk->c3, material, matrix, receive_shadow);
  }
}
