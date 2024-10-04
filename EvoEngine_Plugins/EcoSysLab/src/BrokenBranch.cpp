
#include "BrokenBranch.hpp"

#include "Delaunay.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

void BrokenBranch::UploadStrands() {
  DynamicStrands::InitializeParameters initialize_parameters{};
  const auto owner = GetOwner();
  const auto scene = GetScene();
  initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(owner);
  dynamic_strands.UpdateData(initialize_parameters, subdivided_strand_group.PeekStrands(),
                             subdivided_strand_group.PeekStrandSegments());
}

void BrokenBranch::Serialize(YAML::Emitter& out) const {
}

void BrokenBranch::Deserialize(const YAML::Node& in) {
}

bool BrokenBranch::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  static bool auto_subdivide = true;
  static float segment_length = 0.01f;
  ImGui::DragFloat("Subdivision length", &segment_length, .0001f, 0.0001f, 1.f, "%.5f");
  ImGui::Checkbox("Auto subdivide", &auto_subdivide);
  if (EditorLayer::DragAndDropButton<Tree>(tree_ref, "Download Strands from Tree...")) {
    if (const auto tree = tree_ref.Get<Tree>()) {
      if (tree->strand_model.strand_model_skeleton.data.strand_group.PeekStrands().empty()) {
        tree->BuildStrandModel();
      }
      strand_group = tree->strand_model.strand_model_skeleton.data.strand_group;
      if (auto_subdivide) {
        Subdivide(segment_length, strand_group, subdivided_strand_group);
        dynamic_strands.UpdateData({}, subdivided_strand_group.PeekStrands(),
                                subdivided_strand_group.PeekStrandSegments());
      }
      tree_ref.Clear();
    }
  }
  if (ImGui::Button("Subdivide strands")) {
    Subdivide(segment_length, strand_group, subdivided_strand_group);
    dynamic_strands.UpdateData({}, subdivided_strand_group.PeekStrands(), subdivided_strand_group.PeekStrandSegments());
  }
  if (ImGui::Button("Experiment")) {
    ExperimentSetup();
  }

  ImGui::Text((std::string("Original strand count: ") + std::to_string(strand_group.PeekStrands().size())).c_str());
  ImGui::Text(
      (std::string("Original strand segment count: ") + std::to_string(strand_group.PeekStrandSegments().size()))
          .c_str());
  ImGui::Text((std::string("Subdivided strand count: ") + std::to_string(subdivided_strand_group.PeekStrands().size()))
                  .c_str());
  ImGui::Text((std::string("Subdivided strand segment count: ") +
               std::to_string(subdivided_strand_group.PeekStrandSegments().size()))
                  .c_str());

  static float min_trunk_length = 0.3f;
  static float max_trunk_length = 0.4f;
  ImGui::DragFloat("Min trunk length", &min_trunk_length, .01f, 0.01f, max_trunk_length, "%.5f");
  ImGui::DragFloat("Max trunk length", &max_trunk_length, .01f, min_trunk_length, 1.f, "%.5f");

  static float noise_frequency = 100.f;
  ImGui::DragFloat("Noise frequency", &noise_frequency, .1f, 0.1, 100.f, "%.1f");
  static bool keep_upper = true;
  static glm::vec3 upper_offset = glm::vec3(0, 0.1f, 0.0);
  ImGui::Checkbox("Keep upper", &keep_upper);
  if (keep_upper) {
    ImGui::DragFloat3("Upper offset", &upper_offset.x, 0.01, 0.0f, 1.0f);
  }
  if (ImGui::Button("Cut trunk")) {
    auto temp_strand_group = strand_group;
    const auto size = temp_strand_group.PeekStrands().size();
    for (StrandHandle strand_handle = 0; strand_handle < temp_strand_group.PeekStrands().size(); strand_handle++) {
      const auto& strand = temp_strand_group.PeekStrand(strand_handle);
      StrandSegmentHandle segment_handle;
      float t;
      temp_strand_group.FindStrandT(strand_handle, segment_handle, t,
                                    min_trunk_length + glm::perlin(strand.start_position * noise_frequency) *
                                                           (max_trunk_length - min_trunk_length));
      if (t <= 0.f)
        continue;
      const auto new_strand_handle = temp_strand_group.Cut(segment_handle, t);
      if (new_strand_handle == -1)
        continue;
      auto& new_strand = temp_strand_group.RefStrand(new_strand_handle);
      new_strand.start_position += upper_offset;
      for (const auto& i : new_strand.PeekStrandSegmentHandles()) {
        temp_strand_group.RefStrandSegment(i).end_position += upper_offset;
      }
      if (!keep_upper && new_strand_handle >= size) {
        temp_strand_group.RemoveStrand(new_strand_handle);
      }
    }
    Subdivide(segment_length, temp_strand_group, subdivided_strand_group);
    
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
  static float limit = 0.02f;
  ImGui::DragFloat("Limit", &limit);
  if (ImGui::Button("Build concave mesh")) {
    InitializeStrandConcaveMesh(subdivided_strand_group, limit);
  }
  if (ImGui::Button("Clear concave mesh")) {
    ClearStrandConcaveMesh();
  }
  ImGui::Checkbox("Render strands", &enable_simulation);

  return false;
}

void BrokenBranch::Update() {
}

void BrokenBranch::LateUpdate() {
  if (enable_simulation) {
    const auto editor_layer = Application::GetLayer<EditorLayer>();
    dynamic_strands.step_parameters.render_parameters.target_camera = editor_layer->GetSceneCamera();
    dynamic_strands.Step();
  }
}

void BrokenBranch::OnCreate() {
}

void BrokenBranch::OnDestroy() {
}

void BrokenBranch::CollectAssetRef(std::vector<AssetRef>& list) {
}

void BrokenBranch::ExperimentSetup() {
  strand_group.Clear();
  const auto strand1_handle = strand_group.AllocateStrand();
  auto& segment1 = strand_group.RefStrandSegment(strand_group.Extend(strand1_handle));
  auto& strand1 = strand_group.RefStrand(strand1_handle);
  segment1.end_position = glm::vec3(0, 0.5f, 0.);
  strand1.start_color = segment1.end_color = glm::vec4(1, 0, 0, 0);
  strand1.start_thickness = segment1.end_thickness = 0.01f;

  strand_group.CalculateRotations();
}

void BrokenBranch::Subdivide(const float segment_length, const StrandModelStrandGroup& src,
                             StrandModelStrandGroup& dst) {
  src.UniformlySubdivide(dst, segment_length, segment_length * .01f);
  dst.RandomAssignColor();
}

void BrokenBranch::InitializeStrandParticles(const StrandModelStrandGroup& target_strand_group) const {
  const auto scene = GetScene();
  const auto owner = GetOwner();

  ClearStrandParticles();

  const auto strands_entity = scene->CreateEntity("Branch Strand Particles");
  scene->SetParent(strands_entity, owner);

  const auto renderer = scene->GetOrSetPrivateComponent<Particles>(strands_entity).lock();

  const auto particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  std::vector<ParticleInfo> particle_infos;
  target_strand_group.BuildParticles(particle_infos);
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

void BrokenBranch::InitializeStrandConcaveMesh(const StrandModelStrandGroup& strand_group,
                                               const float max_edge_length) const {
  const auto scene = GetScene();
  const auto owner = GetOwner();

  ClearStrandConcaveMesh();

  const auto strands_entity = scene->CreateEntity("Branch Strand Concave Mesh");
  scene->SetParent(strands_entity, owner);
  const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(strands_entity).lock();

  std::vector<glm::vec3> points(strand_group.PeekStrandSegments().size());
  Jobs::RunParallelFor(strand_group.PeekStrandSegments().size(), [&](const auto& i) {
    points[i] = strand_group.GetStrandSegmentCenter(i);
  });

  renderer->mesh = Delaunay3D::GenerateConcaveHullMesh(points, max_edge_length);
  const auto material = ProjectManager::CreateTemporaryAsset<Material>();

  renderer->material = material;
  material->vertex_color_only = true;
  material->material_properties.albedo_color = glm::vec3(0.6f, 0.3f, 0.0f);
}

void BrokenBranch::ClearStrandConcaveMesh() const {
  const auto scene = GetScene();
  const auto self = GetOwner();
  const auto children = scene->GetChildren(self);
  for (const auto& child : children) {
    auto name = scene->GetEntityName(child);
    if (name == "Branch Strand Concave Mesh") {
      scene->DeleteEntity(child);
    }
  }
}
