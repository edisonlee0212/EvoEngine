
#include "DynamicTreeStrands.hpp"
#include "Delaunay.hpp"
#include "DynamicStrandsOperators.hpp"
#include "DynamicStrandsPhysics.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

void DynamicTreeStrands::UpdateDynamicStrands() {
  attraction_operators.clear();
  transform_operators.clear();
  transform_operators.emplace_back();
  const auto owner = GetOwner();
  const auto scene = GetScene();
  initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(owner);
  dynamic_strands->Initialize(initialize_parameters, strand_model_skeleton, subdivided_strand_group);

  auto& transform_operator = transform_operators.back();
  std::vector<uint32_t> segment_handles(dynamic_strands->strands.size());
  Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
    const auto& segment_handle = dynamic_strands->strands[i].begin_segment_handle;
    segment_handles[i] = segment_handle;
  });
  transform_operator.target_entity = owner;
  transform_operator.ds_transform = std::make_shared<DsTransform>();
  transform_operator.ds_transform->Initialize(initialize_parameters.root_transform, dynamic_strands, segment_handles);
}

void DynamicTreeStrands::Serialize(YAML::Emitter& out) const {
}

void DynamicTreeStrands::Deserialize(const YAML::Node& in) {
}

bool DynamicTreeStrands::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  static bool auto_subdivide = true;
  static bool uniformly_subdivide = true;
  static float min_segment_length = 0.03f;
  static float max_segment_length = 0.06f;
  static int sub_segment_count = 1;

  if (ImGui::TreeNode("Initialization settings")) {
    ImGui::DragFloat("Min subdivision length", &min_segment_length, .0001f, 0.0001f, max_segment_length, "%.5f");
    ImGui::DragFloat("Max subdivision length", &max_segment_length, .0001f, min_segment_length, 1.f, "%.5f");
    ImGui::Checkbox("Auto subdivide", &auto_subdivide);
    ImGui::Checkbox("Uniformly subdivide", &uniformly_subdivide);
    if (uniformly_subdivide) {
      ImGui::DragInt("Sub segment count", &sub_segment_count, 1, 1, 100);
      sub_segment_count = glm::clamp(sub_segment_count, 1, 100);
    }
    initialize_parameters.OnInspect(editor_layer);
    ImGui::TreePop();
  }
  if (EditorLayer::DragAndDropButton<Tree>(tree_ref, "Download Strands from Tree...")) {
    if (const auto tree = tree_ref.Get<Tree>()) {
      tree->BuildStrandModel();
      strand_model_skeleton = tree->strand_model.strand_model_skeleton;
      if (auto_subdivide) {
        if (!uniformly_subdivide) {
          Subdivide(min_segment_length, max_segment_length, strand_model_skeleton.data.strand_group);
        }else {
          UniformSubdivide(static_cast<uint32_t>(sub_segment_count), strand_model_skeleton.data.strand_group);
        }
      }
      tree_ref.Clear();
    }
  }

  auto& strand_group = strand_model_skeleton.data.strand_group;
  if (ImGui::Button("Re-subdivide")) {
    if (!uniformly_subdivide) {
      Subdivide(min_segment_length, max_segment_length, strand_group);
    } else {
      UniformSubdivide(static_cast<uint32_t>(sub_segment_count), strand_group);
    }
  }
  if (ImGui::TreeNodeEx("Experiments", ImGuiTreeNodeFlags_DefaultOpen)) {
    static float rod_length = 1.0f;
    ImGui::DragFloat("Rod length", &rod_length, 0.01f, 0.01f, 10.0f);

    static glm::ivec2 rod_dimension = {10, 10};
    ImGui::DragInt2("Rod dimension", &rod_dimension.x, 1, 1, 1000);
    static bool add_operator = false;
    ImGui::Checkbox("Operator", &add_operator);
    if (ImGui::Button("Multiple Rod Experiment")) {
      MultipleRodExperimentSetup(rod_length, min_segment_length, max_segment_length, 0.002f, rod_dimension,
                                 add_operator);
    }
    ImGui::TreePop();
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
      if (!uniformly_subdivide) {
        Subdivide(min_segment_length, max_segment_length, temp_strand_group);
      } else {
        UniformSubdivide(static_cast<uint32_t>(sub_segment_count), temp_strand_group);
      }
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
  ImGui::Checkbox("Physics", &enable_physics);
  if (!enable_physics) {
    if (ImGui::Button("Simulate 1 step")) {
      enable_physics = true;
      PhysicsStep();
      enable_physics = false;
    }
  }
  if (ImGui::TreeNode("Physics settings")) {
    if (ImGui::TreeNode("Physics parameters")) {
      physics_parameters.OnInspect(editor_layer);
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Operators")) {
      if (ImGui::TreeNode("Transform operators")) {
        for (auto& i : transform_operators) {
          i.ds_transform->OnInspect(editor_layer);
        }
        ImGui::TreePop();
      }
      if (ImGui::TreeNode("Drag force operators")) {
        for (auto& i : attraction_operators) {
          i.ds_attraction->OnInspect(editor_layer);
        }
        ImGui::TreePop();
      }
      if (gravity) {
        if (ImGui::TreeNode("Gravity")) {
          gravity->OnInspect(editor_layer);
          ImGui::TreePop();
        }
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Constraint")) {
      for (auto& i : dynamic_strands->constraints)
        i->OnInspect(editor_layer);
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Visualization settings")) {
    visualization_parameters.OnInspect(editor_layer);
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Render settings")) {
    render_parameters.OnInspect(editor_layer);
    ImGui::TreePop();
  }
  if (ImGui::Button("Download strands")) {
    dynamic_strands->Download();
    EVOENGINE_LOG("Downloaded data from GPU")
  }
  ImGui::SameLine();
  if (ImGui::Button("Upload strands")) {
    dynamic_strands->Upload();
    EVOENGINE_LOG("Uploaded data from GPU")
  }

  return false;
}

void DynamicTreeStrands::OnCreate() {
  dynamic_strands = std::make_shared<DynamicStrands>();
  gravity = std::make_shared<DsGravity>();

  box_selection_operator = std::make_shared<DsBoxSelection>();
  drag_operator = std::make_shared<DsDrag>();

  dynamic_strands->constraints.emplace_back(std::make_shared<DsStiffRod>());
  dynamic_strands->constraints.emplace_back(std::make_shared<DsBundle>());
}

void DynamicTreeStrands::OnDestroy() {
}

void DynamicTreeStrands::CollectAssetRef(std::vector<AssetRef>& list) {
}

void DynamicTreeStrands::MultipleRodExperimentSetup(const float total_length, const float min_segment_length,
                                                    const float max_segment_length, const float radius,
                                                    const glm::ivec2& rod_dimension, const bool add_operator) {
  strand_model_skeleton = {1};
  auto& strand_group = strand_model_skeleton.data.strand_group;
  for (int x = 0; x < rod_dimension.x; x++) {
    for (int y = 0; y < rod_dimension.y; y++) {
      const auto strand_handle = strand_group.AllocateStrand();
      auto& strand = strand_group.RefStrand(strand_handle);
      strand.start_position =
          glm::vec3(0.0f, radius * (y - rod_dimension.y / 2.f) * 2.f, radius * (x - rod_dimension.x / 2.f) * 2.f);
      strand.start_color = glm::vec4(1, 1, 1, 1);
      strand.start_thickness = radius;

      const auto segment_handle = strand_group.Extend(strand_handle);
      auto& segment = strand_group.RefStrandSegment(segment_handle);
      segment.end_position =
          glm::vec3(total_length, radius * (static_cast<float>(y) - static_cast<float>(rod_dimension.y) / 2.f) * 2.f,
                    radius * (static_cast<float>(x) - static_cast<float>(rod_dimension.x) / 2.f) * 2.f);
      segment.end_color = glm::vec4(1, 1, 1, 1);
      segment.end_thickness = radius;
      strand_group.RefStrandSegmentData(segment_handle).node_handle = 0;
    }
  }
  strand_group.CalculateRotations();
  Subdivide(min_segment_length, max_segment_length, strand_group);

  if (add_operator) {
    const auto scene = Application::GetActiveScene();
    const auto children = scene->GetChildren(GetOwner());
    Entity operator_entity{};
    for (const auto& child : children) {
      if (scene->GetEntityName(child) == "Operator") {
        operator_entity = child;
        break;
      }
    }
    if (!scene->IsEntityValid(operator_entity))
      operator_entity = scene->CreateEntity("Operator");
    transform_operators.emplace_back();
    auto& transform_operator = transform_operators.back();

    auto operator_root_transform = GlobalTransform();
    operator_root_transform.SetPosition(
        initialize_parameters.root_transform.TransformPoint(glm::vec3(total_length, 0, 0)));
    scene->SetDataComponent(operator_entity, operator_root_transform);
    scene->SetParent(operator_entity, GetOwner());

    std::vector<uint32_t> segment_handles(dynamic_strands->strands.size());
    Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
      const auto& segment_handle = dynamic_strands->strands[i].end_segment_handle;
      segment_handles[i] = segment_handle;
    });
    transform_operator.target_entity = operator_entity;
    transform_operator.ds_transform = std::make_shared<DsTransform>();
    transform_operator.ds_transform->Initialize(operator_root_transform, dynamic_strands, segment_handles);
  }
}

void DynamicTreeStrands::UniformMultipleRodExperimentSetup(const float segment_length, const float radius,
                                                           const glm::ivec3& rod_dimension, const bool add_operator) {
  strand_model_skeleton = {1};
  auto& strand_group = strand_model_skeleton.data.strand_group;
  for (int x = 0; x < rod_dimension.x; x++) {
    for (int y = 0; y < rod_dimension.y; y++) {
      const auto strand_handle = strand_group.AllocateStrand();
      auto& strand = strand_group.RefStrand(strand_handle);
      strand.start_position =
          glm::vec3(0.0f, radius * (y - rod_dimension.y / 2.f) * 2.f, radius * (x - rod_dimension.x / 2.f) * 2.f);
      strand.start_color = glm::vec4(1, 1, 1, 1);
      strand.start_thickness = radius;

      for (int z = 0; z < rod_dimension.z; z++) {
        const auto segment_handle = strand_group.Extend(strand_handle);
        auto& segment = strand_group.RefStrandSegment(segment_handle);
        segment.end_position =
            glm::vec3(segment_length * (static_cast<float>(z) + 1.f),
                      radius * (static_cast<float>(y) - static_cast<float>(rod_dimension.y) / 2.f) * 2.f,
                      radius * (static_cast<float>(x) - static_cast<float>(rod_dimension.x) / 2.f) * 2.f);
        segment.end_color = glm::vec4(1, 1, 1, 1);
        segment.end_thickness = radius;
        strand_group.RefStrandSegmentData(segment_handle).node_handle = 0;
      }
    }
  }
  strand_group.CalculateRotations();
  subdivided_strand_group = strand_group;
  subdivided_strand_group.RandomAssignColor();
  UpdateDynamicStrands();

  if (add_operator) {
    const auto scene = Application::GetActiveScene();
    const auto children = scene->GetChildren(GetOwner());
    Entity operator_entity{};
    for (const auto& child : children) {
      if (scene->GetEntityName(child) == "Operator") {
        operator_entity = child;
        break;
      }
    }
    if (!scene->IsEntityValid(operator_entity))
      operator_entity = scene->CreateEntity("Operator");
    transform_operators.emplace_back();
    auto& transform_operator = transform_operators.back();

    auto operator_root_transform = GlobalTransform();
    operator_root_transform.SetPosition(initialize_parameters.root_transform.TransformPoint(
        glm::vec3(static_cast<float>(rod_dimension.z + 1) * segment_length, 0, 0)));
    scene->SetDataComponent(operator_entity, operator_root_transform);
    scene->SetParent(operator_entity, GetOwner());

    std::vector<uint32_t> segment_handles(dynamic_strands->strands.size());
    Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
      const auto& segment_handle = dynamic_strands->strands[i].end_segment_handle;
      segment_handles[i] = segment_handle;
    });
    transform_operator.target_entity = operator_entity;
    transform_operator.ds_transform = std::make_shared<DsTransform>();
    transform_operator.ds_transform->Initialize(operator_root_transform, dynamic_strands, segment_handles);
  }
}

void DynamicTreeStrands::Subdivide(const float min_segment_length, const float max_segment_length,
                                   const StrandModelStrandGroup& src) {
  src.Subdivide(
      subdivided_strand_group,
      [&]() {
        return glm::linearRand(min_segment_length, max_segment_length);
      },
      (min_segment_length + max_segment_length) * .5f * .01f);
  subdivided_strand_group.RandomAssignColor();
  UpdateDynamicStrands();
}

void DynamicTreeStrands::UniformSubdivide(const uint32_t subdivision, const StrandModelStrandGroup& src) {
  src.UniformlySubdivide(subdivided_strand_group, subdivision);
  subdivided_strand_group.RandomAssignColor();
  UpdateDynamicStrands();
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

void DynamicTreeStrands::PhysicsStep() const {
  if (!dynamic_strands->segments.empty()) {
    const auto scene = GetScene();
    const auto editor_layer = Application::GetLayer<EditorLayer>();
    if (!dynamic_strands->WaitForUpload()) {
      for (const auto& transform_operator : transform_operators) {
        if (scene->IsEntityValid(transform_operator.target_entity)) {
          const auto global_transform = scene->GetDataComponent<GlobalTransform>(transform_operator.target_entity);
          transform_operator.ds_transform->Update(global_transform, dynamic_strands);
        }
      }

      for (const auto& attraction_operator : attraction_operators) {
        if (scene->IsEntityValid(attraction_operator.target_entity)) {
          const auto global_position =
              scene->GetDataComponent<GlobalTransform>(attraction_operator.target_entity).GetPosition();
          attraction_operator.ds_attraction->Update(global_position);
        }
      }
      dynamic_strands->Physics(physics_parameters, [&]() {
        for (const auto& transform_operator : transform_operators) {
          if (transform_operator.ds_transform->enabled && scene->IsEntityValid(transform_operator.target_entity))
            transform_operator.ds_transform->Execute(physics_parameters, dynamic_strands);
        }
        if (gravity->enabled)
          gravity->Execute(physics_parameters, dynamic_strands);
        for (const auto& attraction_operator : attraction_operators) {
          if (attraction_operator.ds_attraction->enabled && scene->IsEntityValid(attraction_operator.target_entity))
            attraction_operator.ds_attraction->Execute(physics_parameters, dynamic_strands);
        }

        if (box_selection_operator->enabled) {
          box_selection_operator->Execute(dynamic_strands);
        }
        if (drag_operator->enabled) {
          drag_operator->Execute(physics_parameters, dynamic_strands);
        }
      });
    }
  }
}

void DynamicTreeStrands::Visualization(const std::shared_ptr<Camera>& target_camera) const {
  if (!dynamic_strands->segments.empty()) {
    if (!dynamic_strands->WaitForUpload()) {
      dynamic_strands->Visualize(target_camera, visualization_parameters);
    }
  }
}

void DynamicTreeStrands::Render(const std::shared_ptr<Camera>& target_camera) const {
  if (!dynamic_strands->segments.empty()) {
    if (!dynamic_strands->WaitForUpload()) {
      dynamic_strands->Render(target_camera, render_parameters);
    }
  }
}
