#include "LodGroup.hpp"
using namespace evo_engine;

void LodGroup::Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
  for (auto& lod : lods) {
    for (auto& i : lod.renderers) {
      i.Relink(map, scene);
    }
  }
}
