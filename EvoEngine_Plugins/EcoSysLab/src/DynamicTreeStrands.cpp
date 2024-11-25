
#include "DynamicTreeStrands.hpp"
#include "Delaunay.hpp"
#include "DynamicStrandsOperators.hpp"
#include "DynamicStrandsPhysics.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

void DynamicTreeStrands::UpdateDynamicStrands() {
  auto temp_strand_group = strand_model_skeleton.data.strand_group;
  if (limit_strand_length) {
    const auto size = temp_strand_group.PeekStrands().size();
    for (StrandHandle strand_handle = 0; strand_handle < temp_strand_group.PeekStrands().size(); strand_handle++) {
      StrandSegmentHandle segment_handle;
      float t;
      temp_strand_group.FindStrandT(strand_handle, segment_handle, t, max_strand_length);
      if (t <= 0.f)
        continue;
      const auto new_strand_handle = temp_strand_group.Cut(segment_handle, t);
      if (new_strand_handle == -1)
        continue;
      if (new_strand_handle >= size) {
        temp_strand_group.RemoveStrand(new_strand_handle);
      }
    }
  }

  dynamic_strands->constraints.clear();
  if (random_subdivision) {
    temp_strand_group.Subdivide<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData>(
        subdivided_strand_group,
        [&]() {
          return glm::linearRand(min_segment_length, max_segment_length);
        },
        [](StrandHandle src_handle, DtsStrandData& strand_data) {
        },
        [&](const StrandSegmentHandle src_handle, const float segment_t, DtsStrandSegmentData& segment_data) {
          const auto& src_segment_data = temp_strand_group.PeekStrandSegmentData(src_handle);
          segment_data.node_handle = src_segment_data.node_handle;
          segment_data.segment_t = segment_t;
          segment_data.original_segment_handle = src_handle;

          const auto& strand_segment = temp_strand_group.PeekStrandSegment(src_handle);
          const auto& strand = temp_strand_group.PeekStrand(strand_segment.GetStrandHandle());
          const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();
          float d0, d1, d3;
          float d2 = src_segment_data.initial_distance_to_boundary;
          if (src_handle == strand_segment_handles.front()) {
            d1 = d2;
            d0 = d1 * 2.0f - d2;
          } else if (strand_segment.GetPrevHandle() == strand_segment_handles.front()) {
            const auto& prev_segment_data = temp_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
            d0 = d2;
            d1 = prev_segment_data.initial_distance_to_boundary;
          } else {
            const auto& prev_segment = temp_strand_group.PeekStrandSegment(strand_segment.GetPrevHandle());
            const auto& prev_segment_data = temp_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
            const auto& prev_prev_segment_data = temp_strand_group.PeekStrandSegmentData(prev_segment.GetPrevHandle());
            d0 = prev_prev_segment_data.initial_distance_to_boundary;
            d1 = prev_segment_data.initial_distance_to_boundary;
          }
          if (src_handle == strand_segment_handles.back()) {
            d3 = d2 * 2.0f - d1;
          } else {
            const auto& next_segment_data = temp_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
            d3 = next_segment_data.initial_distance_to_boundary;
          }
          segment_data.initial_distance_to_boundary = Strands::CubicInterpolation(d0, d1, d2, d3, segment_t);
        },
        (min_segment_length + max_segment_length) * .5f * .01f);
    dynamic_strands->constraints.emplace_back(std::make_shared<DsRandomBundle>());
  } else {
    temp_strand_group.UniformlySubdivide<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData>(
        subdivided_strand_group, initialize_parameters.sub_segment,
        [&](const StrandHandle src_handle, DtsStrandData& strand_data) {

        },
        [&](const StrandSegmentHandle src_handle, DtsStrandSegmentData& segment_data) {
          const auto& src_segment_data = temp_strand_group.PeekStrandSegmentData(src_handle);
          segment_data.node_handle = src_segment_data.node_handle;
          segment_data.original_segment_handle = src_handle;

          
        });
    dynamic_strands->constraints.emplace_back(std::make_shared<DsUniformBundle>());
  }

  dynamic_strands->constraints.emplace_back(std::make_shared<DsStiffRod>());
  dynamic_strands->constraints.emplace_back(std::make_shared<DsGroundPlane>());
  //subdivided_strand_group.RandomAssignColor();

  attraction_operators.clear();
  transform_operators.clear();
  transform_operators.emplace_back();
  const auto owner = GetOwner();
  const auto scene = GetScene();
  initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(owner);

  Jobs::RunParallelFor(subdivided_strand_group.PeekStrands().size(), [&](const size_t strand_index) {
    const auto& strand = subdivided_strand_group.PeekStrands()[strand_index];
    for (int sub_segment_index = 0; sub_segment_index < strand.PeekStrandSegmentHandles().size(); sub_segment_index++) {
      auto& segment_data =
          subdivided_strand_group.RefStrandSegmentData(strand.PeekStrandSegmentHandles()[sub_segment_index]);
      segment_data.segment_index = sub_segment_index;
    }
  });

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
  ImGui::Checkbox("Random subdivision", &random_subdivision);
  if (ImGui::TreeNode("Initialization settings")) {
    initialize_parameters.OnInspect(editor_layer);
    ImGui::Checkbox("Auto subdivide", &auto_subdivide);
    if (random_subdivision) {
      ImGui::DragFloat("Min segment length", &min_segment_length, 0.001f, 0.001f, max_segment_length);
      ImGui::DragFloat("Max segment length", &max_segment_length, 0.001f, min_segment_length, 1.0f);
    }

    ImGui::Checkbox("Limit strand length", &limit_strand_length);
    if (limit_strand_length) {
      ImGui::DragFloat("Max strand length", &max_strand_length, 0.01f, 0.01f, 10.0f);
    }
    ImGui::TreePop();
  }
  if (EditorLayer::DragAndDropButton<Tree>(tree_ref, "Download Strands from Tree...")) {
    if (const auto tree = tree_ref.Get<Tree>()) {
      tree->BuildStrandModel();
      strand_model_skeleton = tree->strand_model.strand_model_skeleton;
      if (auto_subdivide) {
        UpdateDynamicStrands();
      }
      tree_ref.Clear();
    }
  }

  const auto& strand_group = strand_model_skeleton.data.strand_group;
  if (ImGui::Button("Re-subdivide")) {
    UpdateDynamicStrands();
  }
  if (ImGui::TreeNodeEx("Experiments", ImGuiTreeNodeFlags_DefaultOpen)) {
    static MultipleRodExperimentSetupSettings multiple_rod_experiment_setup_settings{};

    ImGui::DragFloat("Rod length", &multiple_rod_experiment_setup_settings.segment_length, 0.01f, 0.01f, 10.0f);
    ImGui::DragInt3("Rod dimension (3D)", &multiple_rod_experiment_setup_settings.rod_dimension.x, 1, 1, 1000);
    ImGui::Checkbox("Operator", &multiple_rod_experiment_setup_settings.add_operator);
    if (ImGui::Button("Multiple Rod Experiment")) {
      MultipleRodExperimentSetup(multiple_rod_experiment_setup_settings);
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
}

void DynamicTreeStrands::OnDestroy() {
}

void DynamicTreeStrands::CollectAssetRef(std::vector<AssetRef>& list) {
}

void DynamicTreeStrands::MultipleRodExperimentSetup(const MultipleRodExperimentSetupSettings& settings) {
  strand_model_skeleton = {1};
  auto& strand_group = strand_model_skeleton.data.strand_group;

  auto& root_node = strand_model_skeleton.RefNode(0);
  root_node.info.global_position = glm::vec3(0.0f);
  root_node.info.global_rotation = glm::quatLookAt(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0));
  for (int z = 1; z < settings.rod_dimension.z; z++) {
    const auto new_node_handle = strand_model_skeleton.Extend(z - 1, false);
    auto& new_node = strand_model_skeleton.RefNode(new_node_handle);
    new_node.info.global_position = glm::vec3(settings.segment_length * (static_cast<float>(z) + 1.f), 0.0f, 0.0f);
    new_node.info.global_rotation = glm::quatLookAt(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0));
  }
  strand_model_skeleton.CalculateRegulatedGlobalRotation();
  strand_model_skeleton.SortLists();
  for (int x = 0; x < settings.rod_dimension.x; x++) {
    for (int y = 0; y < settings.rod_dimension.y; y++) {
      const auto strand_handle = strand_group.AllocateStrand();
      auto& strand = strand_group.RefStrand(strand_handle);
      strand.start_position = glm::vec3(0.0f, settings.radius * (y - settings.rod_dimension.y / 2.f) * 2.f,
                                        settings.radius * (x - settings.rod_dimension.x / 2.f) * 2.f);
      strand.start_color = glm::vec4(1, 1, 1, 1);
      strand.start_thickness = settings.radius;

      for (int z = 0; z < settings.rod_dimension.z; z++) {
        const auto segment_handle = strand_group.Extend(strand_handle);
        auto& segment = strand_group.RefStrandSegment(segment_handle);
        segment.end_position = glm::vec3(
            settings.segment_length * (static_cast<float>(z) + 1.f),
            settings.radius * (static_cast<float>(y) - static_cast<float>(settings.rod_dimension.y) / 2.f) * 2.f,
            settings.radius * (static_cast<float>(x) - static_cast<float>(settings.rod_dimension.x) / 2.f) * 2.f);
        segment.end_color = glm::vec4(1, 1, 1, 1);
        segment.end_thickness = settings.radius;
        strand_group.RefStrandSegmentData(segment_handle).node_handle = z;
      }
    }
  }
  strand_group.CalculateRotations();
  const bool saved_strand_length_limit = limit_strand_length;
  limit_strand_length = false;
  UpdateDynamicStrands();
  limit_strand_length = saved_strand_length_limit;
  if (settings.add_operator) {
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
        glm::vec3(static_cast<float>(settings.rod_dimension.z + 1) * settings.segment_length, 0, 0)));
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

void DynamicTreeStrands::InitializeStrandParticles(const DtsStrandGroup& target_strand_group) const {
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
      dynamic_strands->Physics(
          physics_parameters,
          [&]() {
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
            
          },
          [&]() {
            
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
