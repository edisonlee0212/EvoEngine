
#include "BrokenBranch.hpp"
#include "Delaunay.hpp"
#include "DynamicStrandsPhysics.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

void BrokenBranch::UpdateDynamicStrands() {
  if (step_parameters.physics || step_parameters.render) {
    EVOENGINE_WARNING("Auto turned physics and render off.");
  }
  step_parameters.physics = false;
  step_parameters.render = false;
  const auto owner = GetOwner();
  const auto scene = GetScene();
  initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(owner);
  
  
  dynamic_strands.InitializeStrandsGroup(initialize_parameters, subdivided_strand_group);
  const auto& external_force = std::dynamic_pointer_cast<DsExternalForce>(dynamic_strands.operators.front());
  external_force->commands.resize(dynamic_strands.strand_segments.size());
  Jobs::RunParallelFor(external_force->commands.size(), [&](const size_t i) {
    external_force->commands[i].strand_segment_index = static_cast<uint32_t>(i);
    external_force->commands[i].force = glm::vec3(0, -1, 0) * dynamic_strands.strand_segments[i].mass;
  });
  const auto& position_update = std::dynamic_pointer_cast<DsPositionUpdate>(dynamic_strands.operators.back());
  position_update->commands.resize(dynamic_strands.strands.size());
  Jobs::RunParallelFor(position_update->commands.size(), [&](const size_t i) {
    position_update->commands[i].strand_segment_index = dynamic_strands.strands[i].begin_segment_handle;
    position_update->commands[i].new_position =
        dynamic_strands.strand_segments[position_update->commands[i].strand_segment_index].x0;
  });
}

void BrokenBranch::Serialize(YAML::Emitter& out) const {
}

void BrokenBranch::Deserialize(const YAML::Node& in) {
}

bool BrokenBranch::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  static bool auto_subdivide = true;
  static float segment_length = 0.05f;
  ImGui::DragFloat("Subdivision length", &segment_length, .0001f, 0.0001f, 1.f, "%.5f");
  ImGui::Checkbox("Auto subdivide", &auto_subdivide);
  if (EditorLayer::DragAndDropButton<Tree>(tree_ref, "Download Strands from Tree...")) {
    if (const auto tree = tree_ref.Get<Tree>()) {
      tree->BuildStrandModel();
      strand_group = tree->strand_model.strand_model_skeleton.data.strand_group;
      if (auto_subdivide) {
        Subdivide(segment_length, strand_group);
      }
      tree_ref.Clear();
    }
  }
  if (ImGui::Button("Subdivide strands")) {
    Subdivide(segment_length, strand_group);
  }
  if (ImGui::Button("Experiment")) {
    ExperimentSetup();
  }
  if (ImGui::TreeNode("Stats")) {
    ImGui::Text((std::string("Original strand count: ") + std::to_string(strand_group.PeekStrands().size())).c_str());
    ImGui::Text(
        (std::string("Original strand segment count: ") + std::to_string(strand_group.PeekStrandSegments().size()))
            .c_str());
    ImGui::Text(
        (std::string("Subdivided strand count: ") + std::to_string(subdivided_strand_group.PeekStrands().size()))
            .c_str());
    ImGui::Text((std::string("Subdivided strand segment count: ") +
                 std::to_string(subdivided_strand_group.PeekStrandSegments().size()))
                    .c_str());
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Trunk cutting")) {
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
      Subdivide(segment_length, temp_strand_group);
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Particles")) {
    if (ImGui::Button("Build particles for original")) {
      InitializeStrandParticles(strand_group);
    }

    if (ImGui::Button("Build particles for subdivided")) {
      InitializeStrandParticles(subdivided_strand_group);
    }
    if (ImGui::Button("Clear particles")) {
      ClearStrandParticles();
    }
    ImGui::TreePop();
  }
  ImGui::Checkbox("Physics", &step_parameters.physics);
  ImGui::Checkbox("Render", &step_parameters.render);
  if (!step_parameters.physics) {
    if (ImGui::Button("Simulate 1 step")) {
      step_parameters.physics = true;
      const bool resume_render = step_parameters.render; 
      step_parameters.render = false;
      dynamic_strands.Step(step_parameters);
      step_parameters.physics = false;
      step_parameters.render = resume_render;
    }
  }
  if (!step_parameters.physics && !step_parameters.render) {
    if (ImGui::Button("Download strands")) {
      dynamic_strands.Download();
      EVOENGINE_LOG("Downloaded data from GPU")
    }
  }
  return false;
}
void BrokenBranch::LateUpdate() {
  if (enable_simulation && !dynamic_strands.strand_segments.empty()) {
    const auto owner = GetOwner();
    const auto scene = GetScene();
    const auto current_root_transform = scene->GetDataComponent<GlobalTransform>(owner);
    GlobalTransform original_inverse_root_transform;
    original_inverse_root_transform.value = glm::inverse(initialize_parameters.root_transform.value);
    const auto& position_update = std::dynamic_pointer_cast<DsPositionUpdate>(dynamic_strands.operators.back());
    Jobs::RunParallelFor(position_update->commands.size(), [&](const size_t i) {
      position_update->commands[i].strand_segment_index = dynamic_strands.strands[i].begin_segment_handle;
      position_update->commands[i]
          .new_position = current_root_transform.TransformPoint(original_inverse_root_transform.TransformPoint(
          dynamic_strands.strand_segments[position_update->commands[i].strand_segment_index].x0));
    });
    const auto editor_layer = Application::GetLayer<EditorLayer>();
    step_parameters.render_parameters.target_camera = editor_layer->GetSceneCamera();
    dynamic_strands.Step(step_parameters);
  }
}

void BrokenBranch::FixedUpdate() {
  
}

void BrokenBranch::OnCreate() {
  dynamic_strands.operators.clear();
  dynamic_strands.constraints.clear();

  dynamic_strands.operators.emplace_back(std::make_shared<DsExternalForce>());
  dynamic_strands.operators.emplace_back(std::make_shared<DsPositionUpdate>());

  dynamic_strands.pre_step = std::make_shared<DynamicStrandsPreStep>();
  dynamic_strands.constraints.emplace_back(std::make_shared<DsStiffRod>());

  dynamic_strands.post_step = std::make_shared<DynamicStrandsPostStep>();
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

void BrokenBranch::Subdivide(const float segment_length, const StrandModelStrandGroup& src) {
  src.UniformlySubdivide(subdivided_strand_group, segment_length, segment_length * .01f);
  subdivided_strand_group.RandomAssignColor();
  UpdateDynamicStrands();
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
