
#include "DynamicTreeStrands.hpp"
#include "Delaunay.hpp"
#include "DynamicStrandsPhysics.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

void DynamicTreeStrands::UpdateDynamicStrands() {
  if (step_parameters.physics || step_parameters.visualization) {
    EVOENGINE_WARNING("Auto turned physics and visualization off.");
  }
  step_parameters.physics = false;
  step_parameters.visualization = false;
  const auto owner = GetOwner();
  const auto scene = GetScene();
  initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(owner);
  dynamic_strands->Initialize(initialize_parameters, strand_model_skeleton, subdivided_strand_group);

  base_position_update->commands.resize(dynamic_strands->strands.size() * 2);
  Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
    const auto& segment_handle = dynamic_strands->strands[i].begin_segment_handle;
    const auto& segment = dynamic_strands->segments[segment_handle];
    base_position_update->commands[i * 2].particle_index = segment.particle0_handle;
    base_position_update->commands[i * 2].new_position = dynamic_strands->particles[segment.particle0_handle].x0;

    base_position_update->commands[i * 2 + 1].particle_index = segment.particle1_handle;
    base_position_update->commands[i * 2 + 1].new_position = dynamic_strands->particles[segment.particle1_handle].x0;
  });

  base_rotation_update->commands.resize(dynamic_strands->strands.size());
  Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
    const auto& segment_handle = dynamic_strands->strands[i].begin_segment_handle;
    const auto& segment = dynamic_strands->segments[segment_handle];
    base_rotation_update->commands[i].segment_index = segment_handle;
    base_rotation_update->commands[i].new_rotation = segment.q0;
  });
}

void DynamicTreeStrands::Serialize(YAML::Emitter& out) const {
}

void DynamicTreeStrands::Deserialize(const YAML::Node& in) {
}

bool DynamicTreeStrands::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  static bool auto_subdivide = true;
  static float segment_length = 0.05f;
  if (ImGui::TreeNode("Initialization settings")) {
    ImGui::DragFloat("Subdivision length", &segment_length, .0001f, 0.0001f, 1.f, "%.5f");
    ImGui::Checkbox("Auto subdivide", &auto_subdivide);
    initialize_parameters.OnInspect(editor_layer);
    ImGui::TreePop();
  }
  if (EditorLayer::DragAndDropButton<Tree>(tree_ref, "Download Strands from Tree...")) {
    if (const auto tree = tree_ref.Get<Tree>()) {
      tree->BuildStrandModel();
      strand_model_skeleton = tree->strand_model.strand_model_skeleton;
      if (auto_subdivide) {
        Subdivide(segment_length, strand_model_skeleton.data.strand_group);
      }
      tree_ref.Clear();
    }
  }
  
  auto& strand_group = strand_model_skeleton.data.strand_group;
  if (ImGui::Button("Re-subdivide")) {
    Subdivide(segment_length, strand_group);
  }
  if (ImGui::Button("Single Rod Experiment")) {
    SingleRodExperimentSetup(1.0, segment_length);
  }
  if (ImGui::Button("Multiple Rod Experiment")) {
    MultipleRodExperimentSetup(1.0, segment_length, 0.002f, {20, 20});
  }
  EditorLayer::DragAndDropButton(operator_entity_ref, "Operator");
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
      const bool resume_render = step_parameters.visualization;
      step_parameters.visualization = false;
      Step();
      step_parameters.physics = false;
      step_parameters.visualization = resume_render;
    }
  }
  if (ImGui::TreeNode("Physics settings")) {
    if (ImGui::TreeNode("Physics parameters")) {
      step_parameters.physics_parameters.OnInspect(editor_layer);
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Operators")) {
      for (auto& i : dynamic_strands->operators)
        i->OnInspect(editor_layer);
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Constraint")) {
      for (auto& i : dynamic_strands->constraints)
        i->OnInspect(editor_layer);
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  ImGui::Checkbox("Visualization", &step_parameters.visualization);
  if (step_parameters.visualization) {
    if (ImGui::TreeNode("Visualization settings")) {
      step_parameters.visualization_parameters.OnInspect(editor_layer);
      ImGui::TreePop();
    }
  }

  if (!step_parameters.physics && !step_parameters.visualization) {
    if (ImGui::Button("Download strands")) {
      dynamic_strands->Download();
      EVOENGINE_LOG("Downloaded data from GPU")
    }
    ImGui::SameLine();
    if (ImGui::Button("Upload strands")) {
      dynamic_strands->Upload();
      EVOENGINE_LOG("Uploaded data from GPU")
    }
  }

  return false;
}
void DynamicTreeStrands::LateUpdate() {
  Step();
}

void DynamicTreeStrands::FixedUpdate() {
}

void DynamicTreeStrands::OnCreate() {
  dynamic_strands = std::make_shared<DynamicStrands>();

  base_position_update = std::make_shared<DsPositionUpdate>();
  base_rotation_update = std::make_shared<DsRotationUpdate>();

  operator_position_update = std::make_shared<DsPositionUpdate>();
  operator_rotation_update = std::make_shared<DsRotationUpdate>();

  external_force = std::make_shared<DsExternalForce>();
  gravity_force = std::make_shared<DsGravityForce>();

  dynamic_strands->operators.emplace_back(base_position_update);
  dynamic_strands->operators.emplace_back(base_rotation_update);

  dynamic_strands->operators.emplace_back(operator_position_update);
  dynamic_strands->operators.emplace_back(operator_rotation_update);

  dynamic_strands->operators.emplace_back(external_force);
  dynamic_strands->operators.emplace_back(gravity_force);
  dynamic_strands->constraints.emplace_back(std::make_shared<DsStiffRod>());
  dynamic_strands->constraints.emplace_back(std::make_shared<DsParticleNeighbor>());
}

void DynamicTreeStrands::OnDestroy() {
}

void DynamicTreeStrands::CollectAssetRef(std::vector<AssetRef>& list) {
}

void DynamicTreeStrands::SingleRodExperimentSetup(const float total_length, const float segment_length) {
  strand_model_skeleton = {1};
  auto& strand_group = strand_model_skeleton.data.strand_group;
  const auto strand1_handle = strand_group.AllocateStrand();
  const auto base_segment_handle = strand_group.Extend(strand1_handle);
  auto& segment1 = strand_group.RefStrandSegment(base_segment_handle);
  auto& strand1 = strand_group.RefStrand(strand1_handle);
  segment1.end_position = glm::vec3(0, total_length, 0.);
  strand1.start_color = segment1.end_color = glm::vec4(1, 0, 0, 0);
  strand1.start_thickness = segment1.end_thickness = 0.1f;
  strand_group.RefStrandSegmentData(base_segment_handle).node_handle = 0;
  strand_group.CalculateRotations();

  Subdivide(segment_length, strand_group);
}

void DynamicTreeStrands::MultipleRodExperimentSetup(const float total_length, const float segment_length,
                                                    const float radius,
                                                    const glm::vec2& intersection) {
  strand_model_skeleton = {1};
  auto& strand_group = strand_model_skeleton.data.strand_group;
  for (int x = 0; x < intersection.x; x++) {
    for (int y = 0; y < intersection.y; y++) {
      const auto strand_handle = strand_group.AllocateStrand();
      auto& strand = strand_group.RefStrand(strand_handle);
      const auto base_segment_handle = strand_group.Extend(strand_handle);
      auto& segment = strand_group.RefStrandSegment(base_segment_handle);
      segment.end_position =
          glm::vec3(total_length, radius * (y - intersection.y / 2.f) * 2.f, radius * (x - intersection.x / 2.f) * 2.f);
      strand.start_position =
          glm::vec3(0.0f, radius * (y - intersection.y / 2.f) * 2.f, radius * (x - intersection.x / 2.f) * 2.f);
      strand.start_color = segment.end_color = glm::vec4(1, 0, 0, 0);
      strand.start_thickness = segment.end_thickness = radius;
      strand_group.RefStrandSegmentData(base_segment_handle).node_handle = 0;
    }
  }

  strand_group.CalculateRotations();
  strand_group.UniformlySubdivide(subdivided_strand_group, segment_length, segment_length * .01f);
  subdivided_strand_group.RandomAssignColor();
  UpdateDynamicStrands();

  const auto scene = Application::GetActiveScene();
  const auto children = scene->GetChildren(GetOwner());
  Entity operator_entity{};
  for(const auto& child : children) {
    if (scene->GetEntityName(child) == "Operator") {
      operator_entity = child;
      break;
    }
  }
  if (!scene->IsEntityValid(operator_entity)) operator_entity = scene->CreateEntity("Operator");
  operator_entity_ref = operator_entity;

  operator_root_transform = GlobalTransform();
  operator_root_transform.SetPosition(glm::vec3(total_length, 0, 0));
  scene->SetDataComponent(operator_entity, operator_root_transform);
  scene->SetParent(operator_entity, GetOwner());
  operator_position_update->commands.resize(dynamic_strands->strands.size() * 2);
  Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
    const auto& segment_handle = dynamic_strands->strands[i].end_segment_handle;
    const auto& segment = dynamic_strands->segments[segment_handle];
    dynamic_strands->particles[segment.particle1_handle].inv_mass = 0;

    operator_position_update->commands[i * 2].particle_index = segment.particle0_handle;
    operator_position_update->commands[i * 2].new_position = dynamic_strands->particles[segment.particle0_handle].x0;

    operator_position_update->commands[i * 2 + 1].particle_index = segment.particle1_handle;
    operator_position_update->commands[i * 2 + 1].new_position = dynamic_strands->particles[segment.particle1_handle].x0;
  });

  operator_rotation_update->commands.resize(dynamic_strands->strands.size());
  Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
    const auto& segment_handle = dynamic_strands->strands[i].end_segment_handle;
    const auto& segment = dynamic_strands->segments[segment_handle];
    operator_rotation_update->commands[i].segment_index = segment_handle;
    operator_rotation_update->commands[i].new_rotation = segment.q0;
  });

  dynamic_strands->Upload();
}

void DynamicTreeStrands::Subdivide(const float segment_length, const StrandModelStrandGroup& src) {
  src.UniformlySubdivide(subdivided_strand_group, segment_length, segment_length * .01f);
  subdivided_strand_group.RandomAssignColor();
  UpdateDynamicStrands();
  dynamic_strands->Upload();
}

void DynamicTreeStrands::InitializeStrandParticles(const StrandModelStrandGroup& target_strand_group) const {
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

void DynamicTreeStrands::ClearStrandParticles() const {
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

void DynamicTreeStrands::Step() {
  if (!dynamic_strands->segments.empty()) {
    if (step_parameters.physics) {
      const auto owner = GetOwner();
      const auto scene = GetScene();
      const auto current_root_transform = scene->GetDataComponent<GlobalTransform>(owner);
      
      if (current_root_transform.value != previous_global_transform.value) {
        GlobalTransform original_inverse_root_transform;
        original_inverse_root_transform.value = glm::inverse(initialize_parameters.root_transform.value);
        previous_global_transform = current_root_transform;
        Jobs::RunParallelFor(base_position_update->commands.size(), [&](const size_t i) {
          auto& command = base_position_update->commands[i];
          command.new_position = current_root_transform.TransformPoint(
              original_inverse_root_transform.TransformPoint(dynamic_strands->particles[command.particle_index].x0));
        });

        base_position_update->enabled = true;
        const glm::quat rotation = current_root_transform.GetRotation() * original_inverse_root_transform.GetRotation();
        Jobs::RunParallelFor(base_rotation_update->commands.size(), [&](const size_t i) {
          auto& command = base_rotation_update->commands[i];
          command.new_rotation = rotation * dynamic_strands->segments[command.segment_index].q0;
        });
        base_rotation_update->enabled = true;
      } else {
        base_position_update->enabled = false;
        base_rotation_update->enabled = false;
      }

      if (const auto operator_entity = operator_entity_ref.Get(); scene->IsEntityValid(operator_entity)) {
        GlobalTransform original_inverse_operator_transform;
        original_inverse_operator_transform.value = glm::inverse(operator_root_transform.value);
        const auto current_operator_transform = scene->GetDataComponent<GlobalTransform>(operator_entity);
        Jobs::RunParallelFor(operator_position_update->commands.size(), [&](const size_t i) {
          auto& command = operator_position_update->commands[i];
          command.new_position = current_operator_transform.TransformPoint(original_inverse_operator_transform.TransformPoint(
                  dynamic_strands->particles[command.particle_index].x0));
        });

        const glm::quat rotation =
            current_operator_transform.GetRotation() * original_inverse_operator_transform.GetRotation();
        Jobs::RunParallelFor(operator_rotation_update->commands.size(), [&](const size_t i) {
          auto& command = operator_rotation_update->commands[i];
          command.new_rotation = rotation * dynamic_strands->segments[command.segment_index].q0;
        });
        operator_position_update->enabled = true;
        operator_rotation_update->enabled = true;
      }else {
        operator_position_update->enabled = false;
        operator_rotation_update->enabled = false;
      }
    }
    const auto editor_layer = Application::GetLayer<EditorLayer>();
    step_parameters.visualization_parameters.target_visualization_camera = editor_layer->GetSceneCamera();
    dynamic_strands->Step(step_parameters);
  }
}
