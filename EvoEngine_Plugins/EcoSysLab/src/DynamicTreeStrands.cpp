
#include "DynamicTreeStrands.hpp"
#include "Delaunay.hpp"
#include "DsConstraints.hpp"
#include "DsOperators.hpp"
#include "DsPhysics.hpp"
#include "DynamicStrands.hpp"
#include "Tree.hpp"
using namespace eco_sys_lab_plugin;

void DynamicTreeStrands::UpdateDynamicStrands() {
  auto source_strand_group = strand_model_skeleton.data.strand_group;
  if (limit_strand_length) {
    const auto size = source_strand_group.PeekStrands().size();
    for (StrandHandle strand_handle = 0; strand_handle < source_strand_group.PeekStrands().size(); strand_handle++) {
      StrandSegmentHandle segment_handle;
      float t;
      source_strand_group.FindStrandT(strand_handle, segment_handle, t, max_strand_length);
      if (t <= 0.f)
        continue;
      const auto new_strand_handle = source_strand_group.Cut(segment_handle, t);
      if (new_strand_handle == -1)
        continue;
      if (new_strand_handle >= size) {
        source_strand_group.RemoveStrand(new_strand_handle);
      }
    }
  }

  dynamic_strands->constraints.clear();

  source_strand_group.Subdivide<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData>(
      subdivided_strand_group,
      [&]() {
        return glm::linearRand(initialize_parameters.min_segment_length, initialize_parameters.max_segment_length);
      },
      [](StrandHandle src_handle, DtsStrandData& strand_data) {
      },
      [&](const float start_root_distance, const float end_root_distance, const StrandSegmentHandle src_handle,
          const uint32_t original_segment_index, const float segment_t, DtsStrandSegmentData& segment_data,
          const uint32_t sub_segment_index) {
        const auto& src_segment_data = source_strand_group.PeekStrandSegmentData(src_handle);
        segment_data.node_handle = src_segment_data.node_handle;
        segment_data.original_segment_t = segment_t;
        segment_data.original_segment_handle = src_handle;
        segment_data.original_segment_index = original_segment_index;
        segment_data.segment_index = sub_segment_index;
        segment_data.start_root_distance = start_root_distance;
        segment_data.end_root_distance = end_root_distance;
        const auto& strand_segment = source_strand_group.PeekStrandSegment(src_handle);
        const auto& strand = source_strand_group.PeekStrand(strand_segment.GetStrandHandle());
        const auto& strand_segment_handles = strand.PeekStrandSegmentHandles();
        glm::vec2 p0, p1, p3;
        const glm::vec2 p2 = src_segment_data.profile_position;
        float d0, d1, d3;
        const float d2 = src_segment_data.initial_distance_to_boundary;
        if (src_handle == strand_segment_handles.front()) {
          d1 = d2;
          d0 = d1 * 2.0f - d2;

          p1 = p2;
          p0 = p1 * 2.0f - p2;
        } else if (strand_segment.GetPrevHandle() == strand_segment_handles.front()) {
          const auto& prev_segment_data = source_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          d0 = d2;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = p2;
          p1 = prev_segment_data.profile_position;
        } else {
          const auto& prev_segment = source_strand_group.PeekStrandSegment(strand_segment.GetPrevHandle());
          const auto& prev_segment_data = source_strand_group.PeekStrandSegmentData(strand_segment.GetPrevHandle());
          const auto& prev_prev_segment_data = source_strand_group.PeekStrandSegmentData(prev_segment.GetPrevHandle());
          d0 = prev_prev_segment_data.initial_distance_to_boundary;
          d1 = prev_segment_data.initial_distance_to_boundary;

          p0 = prev_prev_segment_data.profile_position;
          p1 = prev_segment_data.profile_position;
        }
        if (src_handle == strand_segment_handles.back()) {
          d3 = d2 * 2.0f - d1;

          p3 = p2 * 2.0f - p1;
        } else {
          const auto& next_segment_data = source_strand_group.PeekStrandSegmentData(strand_segment.GetNextHandle());
          d3 = next_segment_data.initial_distance_to_boundary;

          p3 = next_segment_data.profile_position;
        }
        segment_data.initial_distance_to_boundary = Strands::CubicInterpolation(d0, d1, d2, d3, segment_t);
        segment_data.profile_position = Strands::CubicInterpolation(p0, p1, p2, p3, segment_t);
      },
      (initialize_parameters.min_segment_length + initialize_parameters.max_segment_length) * .5f * .01f);

  dynamic_strands->constraints.emplace_back(std::make_shared<DsStiffRod>());
  dynamic_strands->constraints.emplace_back(std::make_shared<DsRandomBundle>());

  dynamic_strands->constraints.emplace_back(std::make_shared<DsGroundPlane>());
  subdivided_strand_group.RandomAssignColor();

  transform_pivots.clear();
  const auto owner = GetOwner();
  const auto scene = GetScene();
  initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(owner);

  dynamic_strands->Initialize(initialize_parameters, strand_model_skeleton, source_strand_group,
                              subdivided_strand_group);
}

void DynamicTreeStrands::CreateStaticRoot() {
  transform_pivots.emplace_back();
  const auto owner = GetOwner();
  /*
  Jobs::RunParallelFor(subdivided_strand_group.PeekStrands().size(), [&](const size_t strand_index) {
    const auto& strand = subdivided_strand_group.PeekStrands()[strand_index];
    for (int sub_segment_index = 0; sub_segment_index < strand.PeekStrandSegmentHandles().size(); sub_segment_index++) {
      auto& segment_data =
          subdivided_strand_group.RefStrandSegmentData(strand.PeekStrandSegmentHandles()[sub_segment_index]);
      segment_data.segment_index = sub_segment_index;
    }
  });
  */
  auto& transform_operator = transform_pivots.back();
  std::vector<std::pair<uint32_t, std::pair<bool, bool>>> segment_list(dynamic_strands->strands.size());
  Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
    const auto& segment_handle = dynamic_strands->strands[i].begin_segment_handle;
    segment_list[i].first = segment_handle;
    segment_list[i].second.first = true;
    segment_list[i].second.second = false;
  });
  transform_operator.target_entity = owner;
  transform_operator.ds_pivot_transform = std::make_shared<DsPivotTransform>();
  transform_operator.ds_pivot_transform->Initialize(initialize_parameters.root_transform, dynamic_strands,
                                                    segment_list);

  dynamic_strands->constraints.emplace_back(transform_operator.ds_pivot_transform);
}

void DynamicTreeStrands::Serialize(YAML::Emitter& out) const {
  material_ref.Save("material_ref", out);
  leaf_material_ref.Save("leaf_material_ref", out);
}

void DynamicTreeStrands::Deserialize(const YAML::Node& in) {
  material_ref.Load("material_ref", in);
  leaf_material_ref.Load("leaf_material_ref", in);
}

bool DynamicTreeStrands::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  editor_layer->DragAndDropButton<Material>(material_ref, "Bark Material");
  editor_layer->DragAndDropButton<Material>(leaf_material_ref, "Leaf Material");

  if (ImGui::TreeNode("Initialization settings")) {
    initialize_parameters.OnInspect(editor_layer);
    ImGui::Checkbox("Limit strand length", &limit_strand_length);
    if (limit_strand_length) {
      ImGui::DragFloat("Max strand length", &max_strand_length, 0.01f, 0.01f, 10.0f);
    }
    ImGui::TreePop();
  }
  static PrivateComponentRef dynamic_tree_strands_tree_ref{};
  if (EditorLayer::DragAndDropButton<Tree>(dynamic_tree_strands_tree_ref, "Download Strands from Tree...")) {
    if (const auto tree = dynamic_tree_strands_tree_ref.Get<Tree>()) {
      tree->BuildStrandModel();
      strand_model_skeleton = tree->strand_model.strand_model_skeleton;
      UpdateDynamicStrands();
      CreateStaticRoot();
      dynamic_tree_strands_tree_ref.Clear();
    }
  }

  const auto& strand_group = strand_model_skeleton.data.strand_group;
  if (ImGui::Button("Re-subdivide")) {
    UpdateDynamicStrands();
  }
  if (ImGui::TreeNodeEx("Experiments", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::TreeNode("Board Experiment")) {
      static BoardExperimentSetupSettings multiple_rod_experiment_setup_settings{};
      ImGui::DragFloat("Rod length", &multiple_rod_experiment_setup_settings.segment_length, 0.01f, 0.01f, 10.0f);
      ImGui::DragFloat("Rod radius", &multiple_rod_experiment_setup_settings.radius, 0.001f, 0.001f, 1.0f);
      ImGui::DragInt3("Rod dimension (3D)", &multiple_rod_experiment_setup_settings.rod_dimension.x, 1, 1, 1000);
      ImGui::Combo("Left Pivot Type", {"Empty", "Point", "Axis", "Transform"},
                   multiple_rod_experiment_setup_settings.left_pivot_type);
      ImGui::Combo("Right Pivot Type", {"Empty", "Point", "Axis", "Transform"},
                   multiple_rod_experiment_setup_settings.right_pivot_type);
      if (ImGui::Button("Initialize")) {
        BoardExperimentSetup(multiple_rod_experiment_setup_settings);
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Log Experiment")) {
      static LogExperimentSetupSettings log_experiment_setup_settings{};
      ImGui::DragFloat("Rod length", &log_experiment_setup_settings.segment_length, 0.01f, 0.01f, 10.0f);
      ImGui::DragFloat("Rod radius", &log_experiment_setup_settings.radius, 0.001f, 0.001f, 1.0f);
      ImGui::DragInt("Rod size", &log_experiment_setup_settings.rod_size, 1, 1, 1000);
      ImGui::DragInt("Rod segment size", &log_experiment_setup_settings.rod_segment_count, 1, 1, 1000);
      ImGui::Combo("Left Pivot Type", {"Empty", "Point", "Axis", "Transform"},
                   log_experiment_setup_settings.left_pivot_type);
      ImGui::Combo("Right Pivot Type", {"Empty", "Point", "Axis", "Transform"},
                   log_experiment_setup_settings.right_pivot_type);
      if (ImGui::Button("Initialize")) {
        LogExperimentSetup(log_experiment_setup_settings);
      }
      ImGui::TreePop();
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
  if (ImGui::TreeNode("Physics settings")) {
    if (ImGui::TreeNode("Operators")) {
      if (ImGui::TreeNode("Transform operators")) {
        for (auto& i : transform_pivots) {
          i.ds_pivot_transform->OnInspect(editor_layer);
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
    if (ImGui::TreeNode("Dynamic Hashed Grid")) {
      dynamic_strands->dynamic_hashed_grid->OnInspect(editor_layer);
      ImGui::TreePop();
    }
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
  line_cut_operator = std::make_shared<DsLineCut>();
  saw_operator = std::make_shared<DsSaw>();

  if (!material_ref.Get<Material>()) {
    material_ref = ProjectManager::CreateTemporaryAsset<Material>();
  }
  if (!leaf_material_ref.Get<Material>()) {
    leaf_material_ref = ProjectManager::CreateTemporaryAsset<Material>();
  }
  foliage_rendering_instance_handle = Handle();
}

void DynamicTreeStrands::OnDestroy() {
  dynamic_strands.reset();
  gravity.reset();
  box_selection_operator.reset();
  drag_operator.reset();
  saw_operator.reset();
}

void DynamicTreeStrands::CollectAssetRef(std::vector<AssetRef>& list) {
}

void DynamicTreeStrands::BoardExperimentSetup(const BoardExperimentSetupSettings& settings) {
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
  strand_model_skeleton.SortLists();
  strand_model_skeleton.CalculateRegulatedGlobalRotation();
  for (int x = 0; x < settings.rod_dimension.x; x++) {
    for (int y = 0; y < settings.rod_dimension.y; y++) {
      const auto strand_handle = strand_group.AllocateStrand();
      auto& strand = strand_group.RefStrand(strand_handle);
      strand.start_position = glm::vec3(0.0f, settings.radius * (y - settings.rod_dimension.y / 2.f) * 2.f,
                                        settings.radius * (x - settings.rod_dimension.x / 2.f) * 2.f);
      strand.start_color = glm::vec4(1, 1, 1, 1);
      strand.start_thickness = settings.radius * 2.f;
      const float distance_to_boundary =
          glm::min(glm::min(x, y), glm::min(settings.rod_dimension.x - x, settings.rod_dimension.y - y));
      for (int z = 0; z < settings.rod_dimension.z; z++) {
        const auto segment_handle = strand_group.Extend(strand_handle);
        auto& segment = strand_group.RefStrandSegment(segment_handle);
        segment.end_position = glm::vec3(
            settings.segment_length * (static_cast<float>(z) + 1.f),
            settings.radius * (static_cast<float>(y) - static_cast<float>(settings.rod_dimension.y) / 2.f) * 2.f,
            settings.radius * (static_cast<float>(x) - static_cast<float>(settings.rod_dimension.x) / 2.f) * 2.f);
        segment.end_color = glm::vec4(1, 1, 1, 1);
        segment.end_thickness = settings.radius * 2.f;
        auto& segment_data = strand_group.RefStrandSegmentData(segment_handle);
        segment_data.profile_position = glm::vec2(x, y);
        segment_data.node_handle = z;
        segment_data.initial_distance_to_boundary = distance_to_boundary;
      }
    }
  }
  strand_group.CalculateRotations();
  const bool saved_strand_length_limit = limit_strand_length;
  limit_strand_length = false;
  UpdateDynamicStrands();
  limit_strand_length = saved_strand_length_limit;

  switch (static_cast<PivotType>(settings.left_pivot_type)) {
    case PivotType::Point: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Left Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Left Pivot");
      point_pivots.emplace_back();
      auto& pivot_operator = point_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.GetPosition());
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, bool>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].begin_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second = false;
      });
      pivot_operator.target_entity = operator_entity;
      pivot_operator.ds_pivot_point = std::make_shared<DsPivotPoint>();
      pivot_operator.ds_pivot_point->Initialize(operator_root_transform, dynamic_strands, segment_list);
      dynamic_strands->constraints.emplace_back(pivot_operator.ds_pivot_point);
      break;
    }
    case PivotType::Axis: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Left Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Left Pivot");
      axis_pivots.emplace_back();
      auto& pivot_operator = axis_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.GetPosition());
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, bool>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].begin_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second = false;
      });
      pivot_operator.target_entity = operator_entity;
      pivot_operator.ds_pivot_axis = std::make_shared<DsPivotAxis>();
      pivot_operator.ds_pivot_axis->Initialize(operator_root_transform, dynamic_strands, segment_list);
      dynamic_strands->constraints.emplace_back(pivot_operator.ds_pivot_axis);
      break;
    }
    case PivotType::Transform: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Left Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Left Pivot");
      transform_pivots.emplace_back();
      auto& transform_operator = transform_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.GetPosition());
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, std::pair<bool, bool>>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].begin_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second.first = true;
        segment_list[i].second.second = false;
      });
      transform_operator.target_entity = operator_entity;
      transform_operator.ds_pivot_transform = std::make_shared<DsPivotTransform>();
      transform_operator.ds_pivot_transform->Initialize(initialize_parameters.root_transform, dynamic_strands,
                                                        segment_list);

      dynamic_strands->constraints.emplace_back(transform_operator.ds_pivot_transform);
      break;
    }
  }

  switch (static_cast<PivotType>(settings.right_pivot_type)) {
    case PivotType::Point: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Right Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Right Pivot");
      point_pivots.emplace_back();
      auto& pivot_operator = point_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.TransformPoint(
          glm::vec3(static_cast<float>(settings.rod_dimension.z + 2) * settings.segment_length, 0, 0)));
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, bool>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].end_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second = false;
      });
      pivot_operator.target_entity = operator_entity;
      pivot_operator.ds_pivot_point = std::make_shared<DsPivotPoint>();
      pivot_operator.ds_pivot_point->Initialize(operator_root_transform, dynamic_strands, segment_list);
      dynamic_strands->constraints.emplace_back(pivot_operator.ds_pivot_point);
      break;
    }
    case PivotType::Axis: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Right Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Right Pivot");
      axis_pivots.emplace_back();
      auto& pivot_operator = axis_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.TransformPoint(
          glm::vec3(static_cast<float>(settings.rod_dimension.z + 2) * settings.segment_length, 0, 0)));
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, bool>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].end_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second = false;
      });
      pivot_operator.target_entity = operator_entity;
      pivot_operator.ds_pivot_axis = std::make_shared<DsPivotAxis>();
      pivot_operator.ds_pivot_axis->Initialize(operator_root_transform, dynamic_strands, segment_list);
      dynamic_strands->constraints.emplace_back(pivot_operator.ds_pivot_axis);
      break;
    }
    case PivotType::Transform: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Right Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Right Pivot");
      transform_pivots.emplace_back();
      auto& transform_operator = transform_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.TransformPoint(
          glm::vec3(static_cast<float>(settings.rod_dimension.z + 2) * settings.segment_length, 0, 0)));
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, std::pair<bool, bool>>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].end_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second.first = false;
        segment_list[i].second.second = true;
      });
      transform_operator.target_entity = operator_entity;
      transform_operator.ds_pivot_transform = std::make_shared<DsPivotTransform>();
      transform_operator.ds_pivot_transform->Initialize(operator_root_transform, dynamic_strands, segment_list);
      dynamic_strands->constraints.emplace_back(transform_operator.ds_pivot_transform);
      break;
    }
  }
}

void DynamicTreeStrands::LogExperimentSetup(const LogExperimentSetupSettings& settings) {
  strand_model_skeleton = {1};
  auto& strand_group = strand_model_skeleton.data.strand_group;

  auto& root_node = strand_model_skeleton.RefNode(0);
  root_node.info.global_position = glm::vec3(0.0f);
  root_node.info.global_rotation = glm::quatLookAt(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0));
  for (int z = 1; z < settings.rod_segment_count; z++) {
    const auto new_node_handle = strand_model_skeleton.Extend(z - 1, false);
    auto& new_node = strand_model_skeleton.RefNode(new_node_handle);
    new_node.info.global_position = glm::vec3(settings.segment_length * (static_cast<float>(z) + 1.f), 0.0f, 0.0f);
    new_node.info.global_rotation = glm::quatLookAt(glm::vec3(1, 0, 0), glm::vec3(0, 1, 0));
  }
  strand_model_skeleton.SortLists();
  strand_model_skeleton.CalculateRegulatedGlobalRotation();

  StrandModelProfile<CellParticlePhysicsData> profile;
  for (int i = 0; i < settings.rod_size; i++) {
    const auto new_particle_handle = profile.AllocateParticle();
    auto& new_particle = profile.RefParticle(new_particle_handle);
    new_particle.strand_handle = i;
    new_particle.strand_segment_handle = 0;
    new_particle.base = false;
    const auto position =
        settings.rod_size == 1 ? glm::vec2(0.f) : glm::diskRand(glm::sqrt(static_cast<float>(settings.rod_size)));
    new_particle.SetPosition(position);
    new_particle.SetInitialPosition(position);
  }
  for (int i = 0; i < 1024; i++) {
    profile.Simulate(
        1,
        [&](auto& grid, const bool grid_resized) {
        },
        [&](auto& particle) {
          auto acceleration = glm::vec2(0.f);
          if (!profile.particle_grid_2d.PeekCells().empty()) {
            const auto& cell = profile.particle_grid_2d.RefCell(particle.GetPosition());
            if (glm::length(cell.target) > glm::epsilon<float>()) {
              acceleration += settings.center_attraction_strength * glm::normalize(cell.target);
            }
          }
          particle.SetAcceleration(acceleration);
        });
  }
  profile.CalculateBoundaries(true);
  for (int i = 0; i < settings.rod_size; i++) {
    const auto strand_handle = strand_group.AllocateStrand();
    auto& strand = strand_group.RefStrand(strand_handle);
    const auto& particle = profile.PeekParticle(i);
    const auto profile_position = particle.GetPosition();
    strand.start_position = glm::vec3(0.0f, settings.radius * profile_position.x, settings.radius * profile_position.y);
    strand.start_color = glm::vec4(1, 1, 1, 1);
    strand.start_thickness = settings.radius * 2.f;
    const float distance_to_boundary = particle.GetDistanceToBoundary();
    for (int z = 0; z < settings.rod_segment_count; z++) {
      const auto segment_handle = strand_group.Extend(strand_handle);
      auto& segment = strand_group.RefStrandSegment(segment_handle);
      segment.end_position = glm::vec3(settings.segment_length * (static_cast<float>(z) + 1.f),
                                       settings.radius * profile_position.x, settings.radius * profile_position.y);
      segment.end_color = glm::vec4(1, 1, 1, 1);
      segment.end_thickness = settings.radius * 2.f;
      auto& segment_data = strand_group.RefStrandSegmentData(segment_handle);
      segment_data.profile_position = profile_position;
      segment_data.node_handle = z;
      segment_data.initial_distance_to_boundary = distance_to_boundary;
    }
  }
  strand_group.CalculateRotations();
  const bool saved_strand_length_limit = limit_strand_length;
  limit_strand_length = false;
  UpdateDynamicStrands();
  limit_strand_length = saved_strand_length_limit;

  switch (static_cast<PivotType>(settings.left_pivot_type)) {
    case PivotType::Point: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Left Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Left Pivot");
      point_pivots.emplace_back();
      auto& pivot_operator = point_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.GetPosition());
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, bool>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].begin_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second = false;
      });
      pivot_operator.target_entity = operator_entity;
      pivot_operator.ds_pivot_point = std::make_shared<DsPivotPoint>();
      pivot_operator.ds_pivot_point->Initialize(operator_root_transform, dynamic_strands, segment_list);
      dynamic_strands->constraints.emplace_back(pivot_operator.ds_pivot_point);
      break;
    }
    case PivotType::Axis: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Left Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Left Pivot");
      axis_pivots.emplace_back();
      auto& pivot_operator = axis_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.GetPosition());
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, bool>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].begin_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second = false;
      });
      pivot_operator.target_entity = operator_entity;
      pivot_operator.ds_pivot_axis = std::make_shared<DsPivotAxis>();
      pivot_operator.ds_pivot_axis->Initialize(operator_root_transform, dynamic_strands, segment_list);
      dynamic_strands->constraints.emplace_back(pivot_operator.ds_pivot_axis);
      break;
    }
    case PivotType::Transform: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Left Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Left Pivot");
      transform_pivots.emplace_back();
      auto& transform_operator = transform_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.GetPosition());
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, std::pair<bool, bool>>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].begin_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second.first = true;
        segment_list[i].second.second = false;
      });
      transform_operator.target_entity = operator_entity;
      transform_operator.ds_pivot_transform = std::make_shared<DsPivotTransform>();
      transform_operator.ds_pivot_transform->Initialize(initialize_parameters.root_transform, dynamic_strands,
                                                        segment_list);

      dynamic_strands->constraints.emplace_back(transform_operator.ds_pivot_transform);
      break;
    }
  }

  switch (static_cast<PivotType>(settings.right_pivot_type)) {
    case PivotType::Point: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Right Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Right Pivot");
      point_pivots.emplace_back();
      auto& pivot_operator = point_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.TransformPoint(
          glm::vec3(static_cast<float>(settings.rod_segment_count + 1) * settings.segment_length, 0, 0)));
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, bool>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].end_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second = false;
      });
      pivot_operator.target_entity = operator_entity;
      pivot_operator.ds_pivot_point = std::make_shared<DsPivotPoint>();
      pivot_operator.ds_pivot_point->Initialize(operator_root_transform, dynamic_strands, segment_list);
      dynamic_strands->constraints.emplace_back(pivot_operator.ds_pivot_point);
      break;
      break;
    }
    case PivotType::Axis: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Right Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Right Pivot");
      axis_pivots.emplace_back();
      auto& pivot_operator = axis_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.TransformPoint(
          glm::vec3(static_cast<float>(settings.rod_segment_count + 1) * settings.segment_length, 0, 0)));
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, bool>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].end_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second = false;
      });
      pivot_operator.target_entity = operator_entity;
      pivot_operator.ds_pivot_axis = std::make_shared<DsPivotAxis>();
      pivot_operator.ds_pivot_axis->Initialize(operator_root_transform, dynamic_strands, segment_list);
      dynamic_strands->constraints.emplace_back(pivot_operator.ds_pivot_axis);
      break;
    }
    case PivotType::Transform: {
      const auto scene = Application::GetActiveScene();
      const auto children = scene->GetChildren(GetOwner());
      for (const auto& child : children) {
        if (scene->GetEntityName(child) == "Right Pivot") {
          scene->DeleteEntity(child);
        }
      }
      const Entity operator_entity = scene->CreateEntity("Right Pivot");
      transform_pivots.emplace_back();
      auto& transform_operator = transform_pivots.back();

      auto operator_root_transform = GlobalTransform();
      operator_root_transform.SetPosition(initialize_parameters.root_transform.TransformPoint(
          glm::vec3(static_cast<float>(settings.rod_segment_count + 1) * settings.segment_length, 0, 0)));
      scene->SetDataComponent(operator_entity, operator_root_transform);
      scene->SetParent(operator_entity, GetOwner());

      std::vector<std::pair<uint32_t, std::pair<bool, bool>>> segment_list(dynamic_strands->strands.size());
      Jobs::RunParallelFor(dynamic_strands->strands.size(), [&](const size_t i) {
        const auto& segment_handle = dynamic_strands->strands[i].end_segment_handle;
        segment_list[i].first = segment_handle;
        segment_list[i].second.first = false;
        segment_list[i].second.second = true;
      });
      transform_operator.target_entity = operator_entity;
      transform_operator.ds_pivot_transform = std::make_shared<DsPivotTransform>();
      transform_operator.ds_pivot_transform->Initialize(operator_root_transform, dynamic_strands, segment_list);
      dynamic_strands->constraints.emplace_back(transform_operator.ds_pivot_transform);
      break;
    }
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

void DynamicTreeStrands::InteractionStep() const {
  if (box_selection_operator->enabled) {
    box_selection_operator->Execute(dynamic_strands);
  }
}

void DynamicTreeStrands::PhysicsStep(const DynamicStrands::PhysicsParameters& physics_parameters) const {
  if (!dynamic_strands->segments.empty()) {
    if (!dynamic_strands->WaitForUpload()) {
      const auto scene = GetScene();
      for (const auto& pivot_operator : transform_pivots) {
        if (scene->IsEntityValid(pivot_operator.target_entity)) {
          const auto global_transform = scene->GetDataComponent<GlobalTransform>(pivot_operator.target_entity);
          pivot_operator.ds_pivot_transform->Update(global_transform, dynamic_strands);
        }
      }
      for (const auto& pivot_operator : axis_pivots) {
        if (scene->IsEntityValid(pivot_operator.target_entity)) {
          const auto global_transform = scene->GetDataComponent<GlobalTransform>(pivot_operator.target_entity);
          pivot_operator.ds_pivot_axis->Update(global_transform);
        }
      }
      for (const auto& pivot_operator : point_pivots) {
        if (scene->IsEntityValid(pivot_operator.target_entity)) {
          const auto global_transform = scene->GetDataComponent<GlobalTransform>(pivot_operator.target_entity);
          pivot_operator.ds_pivot_point->Update(global_transform);
        }
      }
      dynamic_strands->Physics(
          physics_parameters,
          [&]() {

          },
          [&]() {
            if (gravity->enabled)
              gravity->Execute(physics_parameters, dynamic_strands);

            if (drag_operator->enabled) {
              drag_operator->Execute(physics_parameters, dynamic_strands);
            }

            if (line_cut_operator->enabled) {
              line_cut_operator->Execute(dynamic_strands);
            }
            if (saw_operator->enabled) {
              saw_operator->Execute(dynamic_strands);
            }
          });
    }
  }
}
void DynamicTreeStrands::Visualization(const std::shared_ptr<Camera>& target_camera,
                                       const DynamicStrands::VisualizationParameters& visualization_parameters) const {
  if (!dynamic_strands->segments.empty()) {
    if (!dynamic_strands->WaitForUpload()) {
      dynamic_strands->Visualize(target_camera, visualization_parameters);
    }
  }
}

void DynamicTreeStrands::RegisterRenderInstance(const DynamicStrands::RenderParameters& render_parameters) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  if (const auto material = material_ref.Get<Material>()) {
    if (!dynamic_strands->segments.empty()) {
      if (!dynamic_strands->WaitForUpload()) {
        if (DynamicStrands::point_light_render_pipeline && DynamicStrands::point_light_render_pipeline->Initialized()) {
          const auto dynamic_strands_copy = dynamic_strands;
          render_layer->RenderToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
            return dynamic_strands_copy->RenderToPointLightShadowMap(render_parameters, vk_command_buffer, view);
          });
        }
        if (DynamicStrands::spot_light_render_pipeline && DynamicStrands::spot_light_render_pipeline->Initialized()) {
          const auto dynamic_strands_copy = dynamic_strands;
          render_layer->RenderToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
            return dynamic_strands_copy->RenderToSpotLightShadowMap(render_parameters, vk_command_buffer, view);
          });
        }
        if (DynamicStrands::directional_light_render_pipeline &&
            DynamicStrands::directional_light_render_pipeline->Initialized()) {
          const auto dynamic_strands_copy = dynamic_strands;
          render_layer->RenderToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
            return dynamic_strands_copy->RenderToDirectionalLightShadowMap(render_parameters, vk_command_buffer, view);
          });
        }
        if (DynamicStrands::render_pipeline && DynamicStrands::render_pipeline->Initialized()) {
          const auto dynamic_strands_copy = dynamic_strands;
          const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
          const auto renderer_handle = GetHandle();
          current_render_storage->RegisterRenderInstance(GetScene(), GetOwner(), renderer_handle, material);
          render_layer->DeferredRenderingAllCameras(
              [=](const VkCommandBuffer vk_command_buffer,
                  const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                  const RenderLayer::DeferredRenderingView& view) {
                return dynamic_strands_copy->RenderToCameraDeferred(
                    renderer_handle, render_parameters, vk_command_buffer, geometry_pass_color_attachment_infos, view);
              });
        }
      }
    }
  }
}

void DynamicTreeStrands::RegisterFoliageRenderInstance(
    const DynamicStrands::FoliageRenderParameters& render_parameters) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  if (const auto material = leaf_material_ref.Get<Material>()) {
    if (!dynamic_strands->foliage.empty()) {
      if (!dynamic_strands->WaitForUpload()) {
        if (DynamicStrands::foliage_point_light_render_pipeline &&
            DynamicStrands::foliage_point_light_render_pipeline->Initialized()) {
          const auto dynamic_strands_copy = dynamic_strands;
          render_layer->RenderToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
            return dynamic_strands_copy->RenderFoliageToPointLightShadowMap(render_parameters, vk_command_buffer, view);
          });
        }
        if (DynamicStrands::foliage_spot_light_render_pipeline &&
            DynamicStrands::foliage_spot_light_render_pipeline->Initialized()) {
          const auto dynamic_strands_copy = dynamic_strands;
          render_layer->RenderToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
            return dynamic_strands_copy->RenderFoliageToSpotLightShadowMap(render_parameters, vk_command_buffer, view);
          });
        }
        if (DynamicStrands::foliage_directional_light_render_pipeline &&
            DynamicStrands::foliage_directional_light_render_pipeline->Initialized()) {
          const auto dynamic_strands_copy = dynamic_strands;
          render_layer->RenderToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
            return dynamic_strands_copy->RenderFoliageToDirectionalLightShadowMap(render_parameters, vk_command_buffer,
                                                                                  view);
          });
        }
        if (DynamicStrands::foliage_render_pipeline && DynamicStrands::foliage_render_pipeline->Initialized()) {
          const auto dynamic_strands_copy = dynamic_strands;
          const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
          const auto renderer_handle = foliage_rendering_instance_handle;
          current_render_storage->RegisterRenderInstance(GetScene(), GetOwner(), renderer_handle, material);
          render_layer->DeferredRenderingAllCameras(
              [=](const VkCommandBuffer vk_command_buffer,
                  const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                  const RenderLayer::DeferredRenderingView& view) {
                return dynamic_strands_copy->RenderFoliageToCameraDeferred(
                    renderer_handle, render_parameters, vk_command_buffer, geometry_pass_color_attachment_infos, view);
              });
        }
      }
    }
  }
}
