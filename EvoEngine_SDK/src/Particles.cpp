#include "Particles.hpp"
#include "AssetManager.hpp"

using namespace evo_engine;

void Particles::OnCreate() {
  particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  bounding_box = Bound();
  SetEnabled(true);
}

void Particles::RecalculateBoundingBox() {
  const auto pil = particle_info_list.Get<ParticleInfoList>();
  if (!pil)
    return;
  if (pil->PeekParticleInfoList().empty()) {
    bounding_box.min = glm::vec3(0.0f);
    bounding_box.max = glm::vec3(0.0f);
    return;
  }
  auto min_bound = glm::vec3(FLT_MAX);
  auto max_bound = glm::vec3(FLT_MAX);
  const auto mesh_bound = mesh.Get<Mesh>()->GetBound();
  for (const auto& i : pil->PeekParticleInfoList()) {
    const glm::vec3 center = i.instance_matrix.value * glm::vec4(mesh_bound.Center(), 1.0f);
    const glm::vec3 size = glm::vec4(mesh_bound.Size(), 0) * i.instance_matrix.value / 2.0f;
    min_bound = glm::vec3((glm::min)(min_bound.x, center.x - size.x), (glm::min)(min_bound.y, center.y - size.y),
                          (glm::min)(min_bound.z, center.z - size.z));

    max_bound = glm::vec3((glm::max)(max_bound.x, center.x + size.x), (glm::max)(max_bound.y, center.y + size.y),
                          (glm::max)(max_bound.z, center.z + size.z));
  }
  bounding_box.max = max_bound;
  bounding_box.min = min_bound;
}

void Particles::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}

void Particles::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(mesh);
  list.push_back(material);
  list.push_back(particle_info_list);
}
void Particles::OnDestroy() {
  mesh.Clear();
  material.Clear();
  particle_info_list.Clear();

  material.Clear();
  cast_shadow = true;
}
