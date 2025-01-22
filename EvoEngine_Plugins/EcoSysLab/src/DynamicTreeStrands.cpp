
#include "DynamicTreeStrands.hpp"

#include "BarkDescriptor.hpp"
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
  std::mt19937 random_engine(seed);

  source_strand_group.Subdivide<DtsStrandGroupData, DtsStrandData, DtsStrandSegmentData>(
      subdivided_strand_group,
      [&]() {
        return Random::Uniform(random_engine, initialize_parameters.min_segment_length,
                               initialize_parameters.max_segment_length);
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
        const auto calculate_polar_coordinates = [](const glm::vec2& profile_position) {
          const auto r = glm::length(profile_position);
          if (r <= glm::epsilon<float>()) {
            return glm::vec2(0.0f);
          }
          if (profile_position.y >= 0)
            return glm::vec2(r, glm::acos(profile_position.x / r));
          return glm::vec2(r, -glm::acos(profile_position.x / r));
        };

        segment_data.profile_polar_coordinate = calculate_polar_coordinates(segment_data.profile_position);
      },
      (initialize_parameters.min_segment_length + initialize_parameters.max_segment_length) * .5f * .01f);

  dynamic_strands->constraints.emplace_back(std::make_shared<DsStiffRod>());
  dynamic_strands->constraints.emplace_back(std::make_shared<DsBundle>());
  dynamic_strands->constraints.emplace_back(std::make_shared<DsLeafAttachment>());
  subdivided_strand_group.RandomAssignColor();

  transform_pivots.clear();
  const auto owner = GetOwner();
  const auto scene = GetScene();
  initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(owner);

  dynamic_strands->InitializeData(initialize_parameters, strand_model_skeleton, source_strand_group,
                                  subdivided_strand_group);
}

void DynamicTreeStrands::CreateStaticRoot() {
  transform_pivots.emplace_back();
  const auto owner = GetOwner();
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
  bark_material_ref.Save("bark_material_ref", out);
  inner_wood_material_ref.Save("inner_wood_material_ref", out);
  splinter_material_ref.Save("splinter_material_ref", out);
  leaf_material_ref.Save("leaf_material_ref", out);
  snow_material_ref.Save("snow_material_ref", out);
}

void DynamicTreeStrands::Deserialize(const YAML::Node& in) {
  bark_material_ref.Load("bark_material_ref", in);
  inner_wood_material_ref.Load("inner_wood_material_ref", in);
  splinter_material_ref.Load("splinter_material_ref", in);
  leaf_material_ref.Load("leaf_material_ref", in);
  snow_material_ref.Load("snow_material_ref", in);
}

bool DynamicTreeStrands::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::DragInt("Seed", &seed, 1, 0, INT_MAX);
  editor_layer->DragAndDropButton<Material>(bark_material_ref, "Bark Material");
  editor_layer->DragAndDropButton<Material>(inner_wood_material_ref, "Inner wood Material");
  editor_layer->DragAndDropButton<Material>(splinter_material_ref, "Splinter Material");
  editor_layer->DragAndDropButton<Material>(leaf_material_ref, "Leaf Material");
  editor_layer->DragAndDropButton<Material>(snow_material_ref, "Snow Material");
  if (ImGui::TreeNode("Initialization settings")) {
    initialize_parameters.OnInspect(editor_layer);
    if (ImGui::Button("Re-initialize mesh")) {
      dynamic_strands->InitializeMesh(initialize_parameters);
    }
    ImGui::Checkbox("Limit strand length", &limit_strand_length);
    if (limit_strand_length) {
      ImGui::DragFloat("Max strand length", &max_strand_length, 0.01f, 0.01f, 10.0f);
    }
    ImGui::TreePop();
  }
  static PrivateComponentRef dynamic_tree_strands_tree_ref{};
  if (EditorLayer::DragAndDropButton<Tree>(dynamic_tree_strands_tree_ref, "Download Strands from Tree...")) {
    if (const auto tree = dynamic_tree_strands_tree_ref.Get<Tree>()) {
      InitializeFromTree(tree);
      dynamic_tree_strands_tree_ref.Clear();
    }
  }

  const auto& strand_group = strand_model_skeleton.data.strand_group;
  if (ImGui::Button("Re-subdivide")) {
    UpdateDynamicStrands();
    dynamic_strands->Upload();
    dynamic_strands->InitializeMesh(initialize_parameters);
  }
  if (ImGui::TreeNodeEx("Experiments", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::TreeNode("Board Experiment")) {
      static BoardExperimentSetupSettings multiple_rod_experiment_setup_settings{};
      multiple_rod_experiment_setup_settings.OnInspect(editor_layer);
      if (ImGui::Button("Initialize")) {
        BoardExperimentSetup(multiple_rod_experiment_setup_settings);
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Log Experiment")) {
      static LogExperimentSetupSettings log_experiment_setup_settings{};
      log_experiment_setup_settings.OnInspect(editor_layer);
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
    if (ImGui::TreeNodeEx("Prediction", ImGuiTreeNodeFlags_DefaultOpen)) {
      dynamic_strands->prediction->OnInspect(editor_layer);
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Operators", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::TreeNode("Transform operators")) {
        for (auto& i : transform_pivots) {
          i.ds_pivot_transform->OnInspect(editor_layer);
        }
        ImGui::TreePop();
      }
      if (leaf_drop) {
        if (ImGui::TreeNodeEx("Leaf Drop", ImGuiTreeNodeFlags_DefaultOpen)) {
          leaf_drop->OnInspect(editor_layer);
          ImGui::TreePop();
        }
      }
      if (snow) {
        if (ImGui::TreeNodeEx("Snow", ImGuiTreeNodeFlags_DefaultOpen)) {
          snow->OnInspect(editor_layer);
          ImGui::TreePop();
        }
      }
      if (wind) {
        if (ImGui::TreeNodeEx("Wind", ImGuiTreeNodeFlags_DefaultOpen)) {
          wind->OnInspect(editor_layer);
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
  leaf_drop = std::make_shared<DsLeafDrop>();
  snow = std::make_shared<DsSnow>();
  wind = std::make_shared<DsWind>();
  box_selection_operator = std::make_shared<DsBoxSelection>();
  point_cut_operator = std::make_shared<DsPointCut>();
  drag_operator = std::make_shared<DsDrag>();
  line_cut_operator = std::make_shared<DsLineCut>();
  saw_operator = std::make_shared<DsSaw>();
  stop_all = std::make_shared<DsStopAll>();
  enable_physics = true;
  if (!bark_material_ref.Get<Material>()) {
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    bark_material_ref = material;
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.1f;
    material->material_properties.albedo_color = glm::vec3(0.4f, 0.3f, 0.2f);
  }
  if (!inner_wood_material_ref.Get<Material>()) {
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    inner_wood_material_ref = material;
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.0f;
    material->material_properties.albedo_color = glm::vec3(1.f, 0.6f, 0.3f);
  }
  if (!splinter_material_ref.Get<Material>()) {
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    splinter_material_ref = material;
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.0f;
    material->material_properties.albedo_color = glm::vec3(1.f, 0.6f, 0.3f);
  }
  if (!leaf_material_ref.Get<Material>()) {
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    leaf_material_ref = material;
    material->material_properties.roughness = 1.f;
    material->material_properties.metallic = 0.3f;
    material->material_properties.albedo_color = glm::vec3(0.2f, 0.5f, 0.05f);
  }
  if (!snow_material_ref.Get<Material>()) {
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    snow_material_ref = material;
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.0f;
    material->material_properties.albedo_color = glm::vec3(1.0f);
  }

  if (!segment_pair_material_ref.Get<Material>()) {
    const auto material = ProjectManager::CreateTemporaryAsset<Material>();
    segment_pair_material_ref = material;
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.0f;
    material->material_properties.albedo_color = glm::vec3(1.0f);
    material->material_properties.transmission = 0.5f;
  }
  foliage_rendering_instance_handle = Handle();
  small_segments_rendering_instance_handle = Handle();
}

void DynamicTreeStrands::OnDestroy() {
  dynamic_strands.reset();
  leaf_drop.reset();
  snow.reset();
  wind.reset();
  point_cut_operator.reset();
  box_selection_operator.reset();
  drag_operator.reset();
  saw_operator.reset();
}

void DynamicTreeStrands::CollectAssetRef(std::vector<AssetRef>& list) {
}

bool DynamicTreeStrands::BoardExperimentSetupSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::DragFloat("Rod length", &segment_length, 0.01f, 0.01f, 10.0f);
  ImGui::DragFloat("Rod radius", &radius, 0.001f, 0.001f, 1.0f);
  ImGui::DragInt3("Rod dimension (3D)", &rod_dimension.x, 1, 1, 1000);

  ImGui::DragFloat("Center damage", &center_damage, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Center damage offset", &center_distance_offset, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Center damage transition", &center_damage_transition, 0.001f, 0.001f, 1.0f);

  ImGui::Combo("Left Pivot Type", {"Empty", "Point", "Axis", "Transform"}, left_pivot_type);
  ImGui::Combo("Right Pivot Type", {"Empty", "Point", "Axis", "Transform"}, right_pivot_type);

  ImGui::DragFloat3("Initial velocity", &initial_velocity.x, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat3("Initial angular velocity", &initial_angular_velocity.x, 0.01f, 0.0f, 1.0f);
  return false;
}

bool DynamicTreeStrands::LogExperimentSetupSettings::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  ImGui::DragFloat("Rod length", &segment_length, 0.01f, 0.01f, 10.0f);
  ImGui::DragFloat("Rod radius", &radius, 0.001f, 0.001f, 1.0f);
  ImGui::DragInt("Rod size", &rod_size, 1, 1, 1000);
  ImGui::DragInt("Rod segment size", &rod_segment_count, 1, 1, 1000);

  ImGui::DragFloat("Center damage", &center_damage, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Center damage offset", &center_distance_offset, 0.01f, 0.01f, 1.0f);
  ImGui::DragFloat("Center damage transition", &center_damage_transition, 0.001f, 0.001f, 1.0f);

  ImGui::Combo("Left Pivot Type", {"Empty", "Point", "Axis", "Transform"}, left_pivot_type);
  ImGui::Combo("Right Pivot Type", {"Empty", "Point", "Axis", "Transform"}, right_pivot_type);

  ImGui::DragFloat3("Initial velocity", &initial_velocity.x, 0.01f, 0.0f, 1.0f);
  ImGui::DragFloat3("Initial angular velocity", &initial_angular_velocity.x, 0.01f, 0.0f, 1.0f);
  return false;
}

void DynamicTreeStrands::BoardExperimentSetup(const BoardExperimentSetupSettings& settings) {
  strand_model_skeleton = {1};
  auto& strand_group = strand_model_skeleton.data.strand_group;
  float board_length = static_cast<float>(settings.rod_dimension.z) * settings.segment_length;

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

  const bool trunk = initialize_parameters.trunk;
  initialize_parameters.trunk = false;
  UpdateDynamicStrands();

  const auto& target_strand_segment_data_list = subdivided_strand_group.PeekStrandSegmentDataList();

  Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
    auto& segment = dynamic_strands->segments[i];
    const auto& strand_segment_data = target_strand_segment_data_list[i];
    const float root_distance = (strand_segment_data.start_root_distance + strand_segment_data.end_root_distance) * .5f;
    const float distance_to_center = glm::abs(root_distance - board_length * .5f);
    segment.strength -=
        glm::clamp(ActivationFunction::Sigmoid(settings.center_damage, 0.f, settings.center_distance_offset,
                                               1.f / settings.center_damage_transition, distance_to_center),
                   0.f, 1.f);
    segment.particle0.v = settings.initial_velocity;
    segment.particle1.v = settings.initial_velocity;
    segment.angular_v = settings.initial_angular_velocity;
  });

  dynamic_strands->Upload();
  dynamic_strands->InitializeMesh(initialize_parameters);
  initialize_parameters.trunk = trunk;

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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());
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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());
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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());

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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());

      operator_root_transform.SetPosition(
          initialize_parameters.root_transform.TransformPoint(glm::vec3(board_length, 0, 0)));
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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());
      operator_root_transform.SetPosition(
          initialize_parameters.root_transform.TransformPoint(glm::vec3(board_length, 0, 0)));
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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());
      operator_root_transform.SetPosition(
          initialize_parameters.root_transform.TransformPoint(glm::vec3(board_length, 0, 0)));
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
  const float log_length = static_cast<float>(settings.rod_segment_count) * settings.segment_length;
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
  std::mt19937 random_engine(seed);

  StrandModelProfile<CellParticlePhysicsData> profile;
  for (int i = 0; i < settings.rod_size; i++) {
    const auto new_particle_handle = profile.AllocateParticle();
    auto& new_particle = profile.RefParticle(new_particle_handle);
    new_particle.strand_handle = i;
    new_particle.strand_segment_handle = 0;
    new_particle.base = false;
    const auto position = settings.rod_size == 1
                              ? glm::vec2(0.f)
                              : Random::Disk(random_engine, glm::sqrt(static_cast<float>(settings.rod_size)));
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

  const bool trunk = initialize_parameters.trunk;
  initialize_parameters.trunk = false;
  UpdateDynamicStrands();

  const auto& target_strand_segment_data_list = subdivided_strand_group.PeekStrandSegmentDataList();

  GlobalTransform inv_root_transform;
  inv_root_transform.value = glm::inverse(initialize_parameters.root_transform.value);

  Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
    auto& segment = dynamic_strands->segments[i];
    const auto& strand_segment_data = target_strand_segment_data_list[i];
    const float root_distance = (strand_segment_data.start_root_distance + strand_segment_data.end_root_distance) * .5f;
    const float distance_to_center = glm::abs(root_distance - log_length * .5f);
    segment.strength -=
        glm::clamp(ActivationFunction::Sigmoid(settings.center_damage, 0.f, settings.center_distance_offset,
                                               1.f / settings.center_damage_transition, distance_to_center),
                   0.f, 1.f);
    segment.particle0.v = settings.initial_velocity;
    segment.particle1.v = settings.initial_velocity;
    segment.angular_v = settings.initial_angular_velocity;
  });
  if (settings.lock_upper) {
    Jobs::RunParallelFor(dynamic_strands->segment_pairs.size(), [&](const auto i) {
      auto& segment_pair = dynamic_strands->segment_pairs[i];
      const auto& segment0 = dynamic_strands->segments[segment_pair.segment0_handle];
      const auto& segment1 = dynamic_strands->segments[segment_pair.segment1_handle];
      const auto segment0_x0 = (segment0.particle0.x0 + segment0.particle1.x0) * .5f;
      const auto segment1_x0 = (segment1.particle0.x0 + segment1.particle1.x0) * .5f;
      const auto segment_pair_x0 = inv_root_transform.TransformPoint((segment0_x0 + segment1_x0) * .5f);
      if (segment_pair_x0.y > 0.f) {
        segment_pair.compression_lock = 1;
        segment_pair.positional_lock = 1;
        segment_pair.positional_lock = 1;
        segment_pair.rotational_lock = 1;
      }
    });
  }
  if (settings.t_cut) {
    Jobs::RunParallelFor(dynamic_strands->segment_pairs.size(), [&](const auto i) {
      auto& segment_pair = dynamic_strands->segment_pairs[i];
      auto& segment0 = dynamic_strands->segments[segment_pair.segment0_handle];
      auto& segment1 = dynamic_strands->segments[segment_pair.segment1_handle];
      const auto segment0_x0 = inv_root_transform.TransformPoint((segment0.particle0.x0 + segment0.particle1.x0) * .5f);
      const auto segment1_x0 = inv_root_transform.TransformPoint((segment1.particle0.x0 + segment1.particle1.x0) * .5f);
      const auto segment_pair_x0 = (segment0_x0 + segment1_x0) * .5f;
      const auto half_log_length = log_length * .5f;
      if (glm::abs(segment0_x0.x - half_log_length) / half_log_length < settings.t_cut_width ||
          glm::abs(segment1_x0.x - half_log_length) / half_log_length < settings.t_cut_width) {
        if ((segment0_x0.y >= 0.f && segment1_x0.y <= 0.f) || (segment0_x0.y <= 0.f && segment1_x0.y >= 0.f)) {
          segment_pair.bend_twist_bundle_integrity = 0.f;
          segment_pair.connectivity_integrity = 0.f;
        }
      }
      if (glm::abs(segment0_x0.x - half_log_length) / half_log_length < 0.05f ||
          glm::abs(segment1_x0.x - half_log_length) / half_log_length < 0.05f) {
        if (glm::linearRand(0.f, 1.f) > 0.5f && segment0_x0.y <= 0.f && segment1_x0.y <= 0.f) {
          segment0.strength = 0.01f;
          segment1.strength = 0.01f;
        }
      }
    });
  }
  dynamic_strands->Upload();
  dynamic_strands->InitializeMesh(initialize_parameters);
  initialize_parameters.trunk = trunk;

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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());
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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());
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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());
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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());
      operator_root_transform.SetPosition(
          initialize_parameters.root_transform.TransformPoint(glm::vec3(log_length, 0, 0)));
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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());
      operator_root_transform.SetPosition(
          initialize_parameters.root_transform.TransformPoint(glm::vec3(log_length, 0, 0)));
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
      operator_root_transform.SetRotation(initialize_parameters.root_transform.GetRotation());
      operator_root_transform.SetPosition(
          initialize_parameters.root_transform.TransformPoint(glm::vec3(log_length, 0, 0)));
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
  renderer->mesh = Resources::TryGetResource<Mesh>("PRIMITIVE_CUBE");
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

void DynamicTreeStrands::InitializeFromTree(const std::shared_ptr<Tree>& tree) {
  tree->BuildStrandModel();
  if (const auto td = tree->tree_descriptor_ref.Get<TreeDescriptor>()) {
    initialize_parameters.foliage_descriptor = td->foliage_descriptor;
    if (const auto fd = td->foliage_descriptor.Get<FoliageDescriptor>()) {
      if (const auto mat = fd->leaf_material_ref.Get<Material>())
        leaf_material_ref = mat;
    }
    if (const auto bd = td->bark_descriptor.Get<BarkDescriptor>()) {
      if (const auto mat = bd->bark_material_ref.Get<Material>())
        bark_material_ref = mat;
    }
  }
  strand_model_skeleton = tree->strand_model.strand_model_skeleton;
  UpdateDynamicStrands();
  dynamic_strands->Upload();
  dynamic_strands->InitializeMesh(initialize_parameters);
  CreateStaticRoot();
}

void DynamicTreeStrands::PhysicsStep(const DynamicStrands::PhysicsParameters& physics_parameters) const {
  if (!dynamic_strands->segments.empty()) {
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
    dynamic_strands->Physics(physics_parameters, [&]() {
      if (leaf_drop->enabled)
        leaf_drop->Execute(physics_parameters, dynamic_strands);
      if (drag_operator->enabled) {
        drag_operator->Execute(physics_parameters, dynamic_strands);
      }
      if (snow->enabled) {
        snow->Execute(physics_parameters, dynamic_strands);
      }
      if (wind->enabled) {
        wind->Execute(physics_parameters, dynamic_strands);
      }
      if (stop_all->enabled) {
        stop_all->Execute(physics_parameters, dynamic_strands);
      }

      if (line_cut_operator->enabled) {
        line_cut_operator->Execute(dynamic_strands);
      }
      if (saw_operator->enabled) {
        saw_operator->Execute(dynamic_strands);
      }
      if (point_cut_operator->enabled) {
        point_cut_operator->Execute(dynamic_strands);
      }
    });
  }
}
void DynamicTreeStrands::Visualization(const std::shared_ptr<Camera>& target_camera,
                                       const DynamicStrands::VisualizationParameters& visualization_parameters) const {
  if (!dynamic_strands->segments.empty()) {
    dynamic_strands->Visualize(target_camera, initialize_parameters, visualization_parameters);
  }
}

void DynamicTreeStrands::RegisterBranchesRenderInstance(
    const DynamicStrands::BranchesRenderParameters& render_parameters) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto inner_wood_material = inner_wood_material_ref.Get<Material>();
  const auto snow_material = snow_material_ref.Get<Material>();
  if (const auto bark_material = bark_material_ref.Get<Material>();
      bark_material && inner_wood_material && snow_material) {
    if (!dynamic_strands->segments.empty()) {
      if (DynamicStrands::branches_point_light_render_pipeline &&
          DynamicStrands::branches_point_light_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        render_layer->RenderToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return dynamic_strands_copy->RenderBranchesToPointLightShadowMap(render_parameters, vk_command_buffer, view);
        });
      }
      if (DynamicStrands::branches_spot_light_render_pipeline &&
          DynamicStrands::branches_spot_light_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        render_layer->RenderToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return dynamic_strands_copy->RenderBranchesToSpotLightShadowMap(render_parameters, vk_command_buffer, view);
        });
      }
      if (DynamicStrands::branches_directional_light_render_pipeline &&
          DynamicStrands::branches_directional_light_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        render_layer->RenderToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return dynamic_strands_copy->RenderBranchesToDirectionalLightShadowMap(render_parameters, vk_command_buffer,
                                                                                 view);
        });
      }
      if (DynamicStrands::branches_render_pipeline && DynamicStrands::branches_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto renderer_handle = GetHandle();
        current_render_storage->RegisterRenderInstance(GetScene(), GetOwner(), renderer_handle, bark_material);
        const auto inner_material_index = current_render_storage->RegisterMaterial(inner_wood_material);
        const auto snow_material_index = current_render_storage->RegisterMaterial(snow_material);
        render_layer->DeferredRenderingAllCameras(
            [=](const VkCommandBuffer vk_command_buffer,
                const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                const RenderLayer::DeferredRenderingView& view) {
              return dynamic_strands_copy->RenderBranchesToCameraDeferred(
                  renderer_handle, inner_material_index, snow_material_index, render_parameters, vk_command_buffer,
                  geometry_pass_color_attachment_infos, view);
            });
      }
    }
  }
}

void DynamicTreeStrands::RegisterSmallSegmentsRenderInstance(
    const DynamicStrands::SmallSegmentsRenderParameters& render_parameters) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  const auto bark_material = bark_material_ref.Get<Material>();
  if (const auto splinter_material = splinter_material_ref.Get<Material>(); bark_material && splinter_material) {
    if (!dynamic_strands->segments.empty()) {
      if (DynamicStrands::small_segments_point_light_render_pipeline &&
          DynamicStrands::small_segments_point_light_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        render_layer->RenderToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return dynamic_strands_copy->RenderSmallSegmentsToPointLightShadowMap(render_parameters, vk_command_buffer,
                                                                                view);
        });
      }
      if (DynamicStrands::small_segments_spot_light_render_pipeline &&
          DynamicStrands::small_segments_spot_light_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        render_layer->RenderToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return dynamic_strands_copy->RenderSmallSegmentsToSpotLightShadowMap(render_parameters, vk_command_buffer,
                                                                               view);
        });
      }
      if (DynamicStrands::small_segments_directional_light_render_pipeline &&
          DynamicStrands::small_segments_directional_light_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        render_layer->RenderToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return dynamic_strands_copy->RenderSmallSegmentsToDirectionalLightShadowMap(render_parameters,
                                                                                      vk_command_buffer, view);
        });
      }

      if (DynamicStrands::small_segments_render_pipeline &&
          DynamicStrands::small_segments_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto renderer_handle = small_segments_rendering_instance_handle;
        current_render_storage->RegisterRenderInstance(GetScene(), GetOwner(), renderer_handle, bark_material);
        const auto splinter_material_index = current_render_storage->RegisterMaterial(splinter_material);
        render_layer->DeferredRenderingAllCameras(
            [=](const VkCommandBuffer vk_command_buffer,
                const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                const RenderLayer::DeferredRenderingView& view) {
              return dynamic_strands_copy->RenderSmallSegmentsToCameraDeferred(
                  renderer_handle, splinter_material_index, render_parameters, vk_command_buffer,
                  geometry_pass_color_attachment_infos, view);
            });
      }
    }
  }
}

void DynamicTreeStrands::RegisterSmallSegmentsVisualizationRenderInstance(
    const DynamicStrands::SmallSegmentsRenderParameters& render_parameters,
    const DynamicStrands::SmallSegmentsVisualizationRenderParameters& visualization_render_parameters) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  if (const auto material = splinter_material_ref.Get<Material>()) {
    if (!dynamic_strands->segments.empty()) {
      if (DynamicStrands::small_segments_point_light_render_pipeline &&
          DynamicStrands::small_segments_point_light_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        render_layer->RenderToPointLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return dynamic_strands_copy->RenderSmallSegmentsToPointLightShadowMap(render_parameters, vk_command_buffer,
                                                                                view);
        });
      }
      if (DynamicStrands::small_segments_spot_light_render_pipeline &&
          DynamicStrands::small_segments_spot_light_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        render_layer->RenderToSpotLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return dynamic_strands_copy->RenderSmallSegmentsToSpotLightShadowMap(render_parameters, vk_command_buffer,
                                                                               view);
        });
      }
      if (DynamicStrands::small_segments_directional_light_render_pipeline &&
          DynamicStrands::small_segments_directional_light_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        render_layer->RenderToDirectionalLightShadowMap([=](VkCommandBuffer vk_command_buffer, const auto& view) {
          return dynamic_strands_copy->RenderSmallSegmentsToDirectionalLightShadowMap(render_parameters,
                                                                                      vk_command_buffer, view);
        });
      }
      if (DynamicStrands::small_segments_visualization_render_pipeline &&
          DynamicStrands::small_segments_visualization_render_pipeline->Initialized()) {
        const auto dynamic_strands_copy = dynamic_strands;
        const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto renderer_handle = small_segments_rendering_instance_handle;
        current_render_storage->RegisterRenderInstance(GetScene(), GetOwner(), renderer_handle, material);
        render_layer->DeferredRenderingAllCameras(
            [=](const VkCommandBuffer vk_command_buffer,
                const std::vector<VkRenderingAttachmentInfo>& geometry_pass_color_attachment_infos,
                const RenderLayer::DeferredRenderingView& view) {
              return dynamic_strands_copy->RenderSmallSegmentsVisualizationToCameraDeferred(
                  renderer_handle, initialize_parameters, visualization_render_parameters, vk_command_buffer,
                  geometry_pass_color_attachment_infos, view);
            });
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

void DynamicTreeStrands::RegisterSegmentPairRenderInstance(
    const DynamicStrands::SegmentPairsRenderParameters& render_parameters) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  if (const auto material = segment_pair_material_ref.Get<Material>()) {
    if (!dynamic_strands->segment_pairs.empty()) {
      if (DynamicStrands::segment_pairs_visualization_render_pipeline &&
          DynamicStrands::segment_pairs_visualization_render_pipeline->Initialized()) {
        const auto current_render_storage = Application::GetLayer<RenderLayer>()->GetCurrentRenderInstanceStorage();
        const auto dynamic_strands_copy = dynamic_strands;
        const auto material_index = current_render_storage->RegisterMaterial(material);
        render_layer->ForwardRenderingAllCameras([=](const VkCommandBuffer vk_command_buffer,
                                                     const std::shared_ptr<Camera>& target_camera,
                                                     const RenderLayer::ForwardRenderingView& view) {
          return dynamic_strands_copy->RenderSegmentPairsToCameraForward(
              material_index, initialize_parameters, render_parameters, vk_command_buffer, target_camera, view);
        });
      }
    }
  }
}
