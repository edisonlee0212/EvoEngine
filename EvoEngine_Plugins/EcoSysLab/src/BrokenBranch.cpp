
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

  external_force->commands.resize(dynamic_strands.particles.size());
  Jobs::RunParallelFor(external_force->commands.size(), [&](const size_t i) {
    external_force->commands[i].particle_index = static_cast<uint32_t>(i);
    if (dynamic_strands.particles[i].inv_mass != 0.0)
      external_force->commands[i].force = glm::vec3(0, -1, 0) * (1.f / dynamic_strands.particles[i].inv_mass);
    else
      external_force->commands[i].force = glm::vec3(0.f);
  });

  position_update->commands.resize(dynamic_strands.strands.size() * 2);
  Jobs::RunParallelFor(dynamic_strands.strands.size(), [&](const size_t i) {
    const auto& segment_handle = dynamic_strands.strands[i].begin_segment_handle;
    const auto& segment = dynamic_strands.segments[segment_handle];
    position_update->commands[i * 2].particle_index = segment.particle0_handle;
    position_update->commands[i * 2].new_position = dynamic_strands.particles[segment.particle0_handle].x0;

    position_update->commands[i * 2 + 1].particle_index = segment.particle1_handle;
    position_update->commands[i * 2 + 1].new_position = dynamic_strands.particles[segment.particle1_handle].x0;
  });

  rotation_update->commands.resize(dynamic_strands.strands.size());
  Jobs::RunParallelFor(dynamic_strands.strands.size(), [&](const size_t i) {
    const auto& segment_handle = dynamic_strands.strands[i].begin_segment_handle;
    const auto& segment = dynamic_strands.segments[segment_handle];
    rotation_update->commands[i].segment_index = segment_handle;
    rotation_update->commands[i].new_rotation = segment.q0;
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
  if (!step_parameters.physics) {
    if (ImGui::Button("Simulate 1 step")) {
      step_parameters.physics = true;
      const bool resume_render = step_parameters.render;
      step_parameters.render = false;
      Step();
      step_parameters.physics = false;
      step_parameters.render = resume_render;
    }
  }
  if (ImGui::TreeNode("Physics settings")) {
    ImGui::DragFloat("Time step", &step_parameters.physics_parameters.time_step, 0.001f, 0.0f, 1.0f);
    if(ImGui::DragInt("Constraint iteration", &step_parameters.physics_parameters.constraint_iteration, 1, 1, 100)) {
      step_parameters.physics_parameters.constraint_iteration =
          glm::clamp(step_parameters.physics_parameters.constraint_iteration, 1, 100);
    }
    ImGui::TreePop();
  }
  ImGui::Checkbox("Render", &step_parameters.render);
  if (step_parameters.render) {
    if (ImGui::TreeNode("Render settings")) {
      ImGui::Combo("Render Mode", {"Default", "Bend/twist strain", "Stretch/shear strain", "Connectivity strain"},
                   step_parameters.render_parameters.render_mode);
      ImGui::ColorEdit4("Min Color", &step_parameters.render_parameters.min_color.x);
      ImGui::ColorEdit4("Max Color", &step_parameters.render_parameters.max_color.x);
      ImGui::DragFloat("Multiplier", &step_parameters.render_parameters.multiplier, 0.1f, 0.1f, 1000.f);
      ImGui::TreePop();
    }
  }
  
  if (!step_parameters.physics && !step_parameters.render) {
    if (ImGui::Button("Download strands")) {
      dynamic_strands.Download();
      EVOENGINE_LOG("Downloaded data from GPU")
    }
    ImGui::SameLine();
    if (ImGui::Button("Upload strands")) {
      dynamic_strands.Upload();
      EVOENGINE_LOG("Uploaded data from GPU")
    }
  }

  return false;
}
void BrokenBranch::LateUpdate() {
  Step();
}

void BrokenBranch::FixedUpdate() {
}

void BrokenBranch::OnCreate() {
  dynamic_strands.operators.clear();
  dynamic_strands.constraints.clear();
  external_force = std::make_shared<DsExternalForce>();
  position_update = std::make_shared<DsPositionUpdate>();
  rotation_update = std::make_shared<DsRotationUpdate>();
  dynamic_strands.pre_step = std::make_shared<DynamicStrandsPreStep>();
  dynamic_strands.constraints.emplace_back(std::make_shared<DsStiffRod>());

  dynamic_strands.operators.emplace_back(position_update);
  dynamic_strands.operators.emplace_back(rotation_update);
  dynamic_strands.constraints.emplace_back(std::make_shared<DsParticleNeighbor>());
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
  segment1.end_position = glm::vec3(0, 5.0f, 0.);
  strand1.start_color = segment1.end_color = glm::vec4(1, 0, 0, 0);
  strand1.start_thickness = segment1.end_thickness = 0.1f;

  strand_group.CalculateRotations();

  Subdivide(0.5, strand_group);
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

void BrokenBranch::Step() {
  if (!dynamic_strands.segments.empty()) {
    if (step_parameters.physics) {
      const auto owner = GetOwner();
      const auto scene = GetScene();
      const auto current_root_transform = scene->GetDataComponent<GlobalTransform>(owner);
      if (current_root_transform.value != previous_global_transform.value) {
        previous_global_transform = current_root_transform;
        GlobalTransform original_inverse_root_transform;
        original_inverse_root_transform.value = glm::inverse(initialize_parameters.root_transform.value);
        Jobs::RunParallelFor(position_update->commands.size(), [&](const size_t i) {
          auto& command = position_update->commands[i];
          command.new_position = current_root_transform.TransformPoint(
              original_inverse_root_transform.TransformPoint(dynamic_strands.particles[command.particle_index].x0));
        });
        
        position_update->enabled = true;
        const glm::quat rotation = current_root_transform.GetRotation() * original_inverse_root_transform.GetRotation();
        Jobs::RunParallelFor(rotation_update->commands.size(), [&](const size_t i) {
          auto& command = rotation_update->commands[i];
          command.new_rotation = rotation * dynamic_strands.segments[command.segment_index].q0;
        });
        rotation_update->enabled = true;
      }else {
        position_update->enabled = false;
        rotation_update->enabled = false;
      }
    }
    const auto editor_layer = Application::GetLayer<EditorLayer>();
    step_parameters.render_parameters.target_camera = editor_layer->GetSceneCamera();
    for (const auto& constraint : dynamic_strands.constraints)
      constraint->enabled = true;
    dynamic_strands.Step(step_parameters);
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
