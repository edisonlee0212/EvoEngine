#include "BrokenBranch.hpp"

#include "Tree.hpp"

using namespace eco_sys_lab_plugin;

void BrokenBranch::Serialize(YAML::Emitter& out) const {
  
}

void BrokenBranch::Deserialize(const YAML::Node& in) {
  
}

bool BrokenBranch::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  EditorLayer::DragAndDropButton<Tree>(tree_ref, "Tree");
  if (const auto tree = tree_ref.Get<Tree>()) {
    if (ImGui::Button("Download Strand group")) {
      if (tree->strand_model.strand_model_skeleton.data.strand_group.PeekStrands().empty()) {
        tree->BuildStrandModel();
      }
      strand_group = tree->strand_model.strand_model_skeleton.data.strand_group;
      strand_group.RandomAssignColor();
    }
  }
  
  static float segment_length = 0.005f;
  if (ImGui::Button("Subdivide strands")) {
    strand_group.UniformlySubdivide(subdivided_strand_group, segment_length, segment_length * .01f);
    subdivided_strand_group.RandomAssignColor();
  }
  static float trunk_length = 0.3f;
  if (ImGui::Button("Cut trunk")) {
    const auto strand_size = subdivided_strand_group.PeekStrands().size();
    std::vector<StrandHandle> new_strand_handles;
    for (StrandHandle strand_handle = 0; strand_handle < strand_size; strand_handle++) {
      StrandSegmentHandle segment_handle;
      float t;
      subdivided_strand_group.FindStrandT(strand_handle, segment_handle, t, trunk_length);
      const auto new_strand_handle = subdivided_strand_group.Cut(segment_handle, t);
      auto &new_strand = subdivided_strand_group.RefStrand(new_strand_handle);
      new_strand.start_position += glm::vec3(0, 0.3f, 0);
      for (const auto& i : new_strand.PeekStrandSegmentHandles()) {
        subdivided_strand_group.RefStrandSegment(i).end_position += glm::vec3(0, 0.05f, 0);
      }
      new_strand_handles.emplace_back(new_strand_handle);
    }
    for (const auto& i : new_strand_handles) {
      subdivided_strand_group.RemoveStrand(i);
    }
  }
  if (ImGui::Button("Build particles for original")) {
    InitializeStrandParticles(strand_group);
  }
  ImGui::SameLine();
  if (ImGui::Button("Build particles for subdivided")) {
    InitializeStrandParticles(subdivided_strand_group);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear particles")) {
    ClearStrandParticles();
  }

  return false;
}

void BrokenBranch::Update() {
}

void BrokenBranch::OnCreate() {
  
}

void BrokenBranch::OnDestroy() {
  
}

void BrokenBranch::CollectAssetRef(std::vector<AssetRef>& list) {
  
}

void BrokenBranch::InitializeStrandParticles(const StrandModelStrandGroup& strand_group) const {
  const auto scene = GetScene();
  const auto owner = GetOwner();

  ClearStrandParticles();

  const auto strands_entity = scene->CreateEntity("Branch Strand Particles");
  scene->SetParent(strands_entity, owner);

  const auto renderer = scene->GetOrSetPrivateComponent<Particles>(strands_entity).lock();

  const auto particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  std::vector<ParticleInfo> particle_infos;
  strand_group.BuildParticles(particle_infos);
  particle_info_list->SetParticleInfos(particle_infos);

  renderer->particle_info_list = particle_info_list;
  renderer->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CUBE");
  const auto material = ProjectManager::CreateTemporaryAsset<Material>();

  renderer->material = material;
  material->vertex_color_only = true;
  material->material_properties.albedo_color = glm::vec3(0.6f, 0.3f, 0.0f);
}

void BrokenBranch::ClearStrandParticles() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Branch Strand Particles") {
      scene->DeleteEntity(child);
    }
  }
}
