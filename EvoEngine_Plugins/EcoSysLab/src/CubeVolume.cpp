#include "CubeVolume.hpp"

using namespace eco_sys_lab;

void CubeVolume::ApplyMeshBounds(const std::shared_ptr<Mesh>& mesh) {
  if (!mesh)
    return;
  min_max_bound = mesh->GetBound();
}

bool CubeVolume::OnInspect(const std::shared_ptr<EditorLayer>& editorLayer) {
  bool changed = false;
  if (IVolume::OnInspect(editorLayer))
    changed = true;
  if (ImGui::DragFloat3("Min", &min_max_bound.min.x, 0.1f))
    changed = true;
  if (ImGui::DragFloat3("Max", &min_max_bound.max.x, 0.1f))
    changed = true;
  static PrivateComponentRef privateComponentRef{};

  if (editorLayer->DragAndDropButton<MeshRenderer>(privateComponentRef, "Target MeshRenderer")) {
    if (const auto mmr = privateComponentRef.Get<MeshRenderer>()) {
      ApplyMeshBounds(mmr->mesh.Get<Mesh>());
      privateComponentRef.Clear();
      changed = true;
    }
  }
  return changed;
}

bool CubeVolume::InVolume(const glm::vec3& position) {
  return min_max_bound.InBound(position);
}

glm::vec3 CubeVolume::GetRandomPoint() {
  return glm::linearRand(min_max_bound.min, min_max_bound.max);
}

bool CubeVolume::InVolume(const GlobalTransform& globalTransform, const glm::vec3& position) {
  const auto finalPos = glm::vec3((glm::inverse(globalTransform.value) * glm::translate(position))[3]);
  return min_max_bound.InBound(finalPos);
}

void CubeVolume::Serialize(YAML::Emitter& out) const {
  IVolume::Serialize(out);
  out << YAML::Key << "min_max_bound.min" << YAML::Value << min_max_bound.min;
  out << YAML::Key << "min_max_bound.max" << YAML::Value << min_max_bound.max;
}

void CubeVolume::Deserialize(const YAML::Node& in) {
  IVolume::Deserialize(in);
  min_max_bound.min = in["min_max_bound.min"].as<glm::vec3>();
  min_max_bound.max = in["min_max_bound.max"].as<glm::vec3>();
}
