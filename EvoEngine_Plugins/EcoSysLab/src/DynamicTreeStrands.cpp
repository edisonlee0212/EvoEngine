#include "DynamicTreeStrands.hpp"
#include "BasicBarkDescriptor.hpp"
#include "DsAlphaShapeMeshing.hpp"
#include "DsConstraints.hpp"
#include "DsKineticVoronoiMeshing.hpp"
#include "DsMaterials.hpp"
#include "DsOperators.hpp"
#include "DsPhysics.hpp"
#include "DynamicStrands.hpp"
#include "Tree.hpp"
#include "VoronoiMeshGenerator.hpp"
using namespace eco_sys_lab_plugin;

void DynamicTreeStrands::UpdateDynamicStrands(DtsStrandGroup& randomly_subdivided_strand_group,
                                              DtsStrandGroup& uniformly_subdivided_strand_group) {
  auto strand_model_strand_group = strand_model.strand_model_skeleton.data.strand_group;
  if (limit_strand_length) {
    const auto size = strand_model_strand_group.PeekStrands().size();
    for (StrandHandle strand_handle = 0; strand_handle < strand_model_strand_group.PeekStrands().size();
         strand_handle++) {
      StrandSegmentHandle segment_handle;
      float t;
      strand_model_strand_group.FindStrandT(strand_handle, segment_handle, t, max_strand_length);
      if (t <= 0.f)
        continue;
      const auto new_strand_handle = strand_model_strand_group.Cut(segment_handle, t);
      if (new_strand_handle == -1)
        continue;
      if (new_strand_handle >= size) {
        strand_model_strand_group.RemoveStrand(new_strand_handle);
      }
    }
  }

  std::mt19937 random_engine(seed);

  transform_pivots.clear();
  const auto owner = GetOwner();
  const auto scene = GetScene();
  initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(owner);
  // initialize_parameters.min_segment_length = 0.005f;
  // initialize_parameters.max_segment_length = 0.01f;

  dynamic_strands->InitializeData(random_engine, initialize_parameters, strand_model.strand_model_skeleton,
                                  strand_model_strand_group, randomly_subdivided_strand_group,
                                  uniformly_subdivided_strand_group);
  if (initialized_from_tree) {
    CreateStaticRoot();
  }
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
  out << YAML::Key << "seed" << YAML::Value << seed;
  out << YAML::Key << "enable_physics" << YAML::Value << enable_physics;
  out << YAML::Key << "limit_strand_length" << YAML::Value << limit_strand_length;
  out << YAML::Key << "max_strand_length" << YAML::Value << max_strand_length;

  materials.bark_material_ref.Save("bark_material_ref", out);
  materials.inner_wood_material_ref.Save("inner_wood_material_ref", out);
  materials.splinter_material_ref.Save("splinter_material_ref", out);
  materials.leaf_material_ref.Save("leaf_material_ref", out);
  materials.snow_material_ref.Save("snow_material_ref", out);
  materials.segment_pair_material_ref.Save("segment_pair_material_ref", out);
  materials.wireframe_material_ref.Save("wireframe_material_ref", out);

  strand_model.Save("shoot_strand_model", out);
  initialize_parameters.Save("initialize_parameters", out);

  out << YAML::Key << "initialized_from_tree" << YAML::Value << initialized_from_tree;
}

void DynamicTreeStrands::Deserialize(const YAML::Node& in) {
  if (in["seed"])
    seed = in["seed"].as<int>();
  if (in["initialized_from_tree"])
    initialized_from_tree = in["initialized_from_tree"].as<bool>();

  if (in["enable_physics"])
    enable_physics = in["enable_physics"].as<bool>();
  if (in["limit_strand_length"])
    limit_strand_length = in["limit_strand_length"].as<bool>();
  if (in["max_strand_length"])
    max_strand_length = in["max_strand_length"].as<float>();

  materials.bark_material_ref.Load("bark_material_ref", in);
  materials.inner_wood_material_ref.Load("inner_wood_material_ref", in);
  materials.splinter_material_ref.Load("splinter_material_ref", in);
  materials.leaf_material_ref.Load("leaf_material_ref", in);
  materials.snow_material_ref.Load("snow_material_ref", in);
  materials.segment_pair_material_ref.Load("segment_pair_material_ref", in);
  materials.wireframe_material_ref.Load("wireframe_material_ref", in);

  strand_model.Load("shoot_strand_model", in);
  initialize_parameters.Load("initialize_parameters", in);
}

bool DynamicTreeStrands::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  if (ImGui::TreeNode("Preset settings")) {
    if (ImGui::Button("Oak Trunk")) {
      initialize_parameters.min_segment_length = 0.005f;
      initialize_parameters.max_segment_length = 0.01f;
    }
    if (ImGui::Button("Spruce")) {
      if (dynamic_strands) {
        for (auto& constraint : dynamic_strands->constraints) {
          if (auto bundle = std::dynamic_pointer_cast<DsBundle>(constraint)) {
            bundle->sub_iteration = 5;
            break;  // stop once we found the bundle constraint
          }
        }
      }
    }
    if (ImGui::Button("Oak")) {
      if (dynamic_strands) {
        for (auto& constraint : dynamic_strands->constraints) {
          if (auto bundle = std::dynamic_pointer_cast<DsBundle>(constraint)) {
            bundle->sub_iteration = 3;
            break;  // stop once we found the bundle constraint
          }
        }
      }
    }
    ImGui::TreePop();
  }
  ImGui::RadioButton("Kinetic Voronoi Meshing", reinterpret_cast<int*>(&initialize_parameters.meshing_type),
                     static_cast<int>(MeshingType::KineticVoronoi));
  ImGui::SameLine();
  ImGui::RadioButton("Alpha Shape Meshing", reinterpret_cast<int*>(&initialize_parameters.meshing_type),
                     static_cast<int>(MeshingType::AlphaShape));
  ImGui::DragInt("Seed", &seed, 1, 0, INT_MAX);
  editor_layer->DragAndDropButton<Material>(materials.bark_material_ref, "Bark Material");
  editor_layer->DragAndDropButton<Material>(materials.inner_wood_material_ref, "Inner wood Material");
  editor_layer->DragAndDropButton<Material>(materials.splinter_material_ref, "Splinter Material");
  editor_layer->DragAndDropButton<Material>(materials.leaf_material_ref, "Leaf Material");
  editor_layer->DragAndDropButton<Material>(materials.snow_material_ref, "Snow Material");
  editor_layer->DragAndDropButton<Material>(materials.wireframe_material_ref, "Wireframe Material");
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
  if (editor_layer->DragAndDropButton<Tree>(dynamic_tree_strands_tree_ref, "Download Strands from Tree...")) {
    if (const auto tree = dynamic_tree_strands_tree_ref.Get<Tree>()) {
      InitializeFromTree(tree);
      dynamic_tree_strands_tree_ref.Clear();
    }
  }

  const auto& strand_group = strand_model.strand_model_skeleton.data.strand_group;
  if (ImGui::Button("Re-subdivide")) {
    // initialize_parameters.min_segment_length = 0.005f;
    // initialize_parameters.max_segment_length = 0.01f;

    DtsStrandGroup randomly_subdivided_strand_group{}, uniformly_subdivided_strand_group{};
    UpdateDynamicStrands(randomly_subdivided_strand_group, uniformly_subdivided_strand_group);

    Region INIT{0.0f, 100.0f, -glm::pi<float>(), glm::pi<float>(), 0.0f, 1.0f};
    std::vector<Region> regions;
    Node_tilt* root = build_bsp_tilt(INIT, /*N=*/12800, 0.1f, 1.8f, 10,
                                     /*tilt_eps=*/0.0f, /*enable_tilt=*/false, regions, 1500.f,
                                     3000.f);  // ZY: N=800 for pull operator test

    // Node_tilt* root = build_bsp_tilt(INIT, /*N=*/12800, 0.1f, 1.8f, 10,
    //                                  /*tilt_eps=*/0.0f, /*enable_tilt=*/false, regions, 1500.f, 3000.f);

    // Node_tilt* root = build_bsp_tilt(INIT, /*N=*/12800, 0.1f, 1.8f, 10,
    //                                /*tilt_eps=*/0.0f, /*enable_tilt=*/false, regions, 1500.f,
    //                                3000.f);

    std::mt19937 rng(std::random_device{}());
    int K = (int)regions.size();
    std::uniform_real_distribution<float> dc(0.0f, 1.0f);
    std::vector<glm::vec4> region_colors(K);
    for (int i = 0; i < K; ++i) {
      region_colors[i] = glm::vec4(dc(rng), dc(rng), dc(rng), 1.0f);
    }
    Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
      auto& segment = dynamic_strands->segments[i];
      // std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 2.f, segment.profile_polar_coordinate[1],
      //                            segment.particle0.x[1] * 0.5 + 0.1f};

      std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 2.f, segment.profile_polar_coordinate[1],
                                 segment.particle0.root_distance * 0.5f + 0.1f};  // for general

      // std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 1.5f, segment.profile_polar_coordinate[1]
      // * 3.0f,
      //                            segment.particle0.root_distance * 0.15f + 0.1f};  // ZY: thin along longitude

      // std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 1.5f, segment.profile_polar_coordinate[1]
      // * 2.0f,
      //                            segment.particle0.root_distance * 0.4f + 0.1f};  // ZY: for irregular log

      // std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 2.f, segment.profile_polar_coordinate[1]
      // * 2.0f,
      //                            segment.particle0.root_distance * 0.3f + 0.1f}; //ZY: for oak trunk ONLY
      int id = classify_point_jitter_axis(pt, root, 0.0f, 0xA53A5F1Bu, false);  // ZY:false for pull operator test
      segment.color = region_colors[id];
    });

    /*Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
      auto& segment = dynamic_strands->segments[i];
      auto cur_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f);
      segment.color = cur_color;
    });*/

    dynamic_strands->Upload();
    dynamic_strands->InitializeMesh(initialize_parameters);
  }
  ImGui::SameLine();
  if (ImGui::Button("Test Static Mesh")) {
    // TODO: Pass the strands to the voronoi mesh generator to test creation of static meshes
    StrandModelMeshGeneratorSettings settings;
    std::vector<Vertex> vertices;
    std::vector<glm::vec2> tex_coords;
    std::vector<std::pair<unsigned int, unsigned int>> index_pairs;

    DtsStrandGroup randomly_subdivided_strand_group{}, uniformly_subdivided_strand_group{};
    UpdateDynamicStrands(randomly_subdivided_strand_group, uniformly_subdivided_strand_group);

    VoronoiMeshGenerator::Generate(uniformly_subdivided_strand_group, strand_model, vertices, tex_coords, index_pairs,
                                   settings);
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
    ImGui::Text((std::string("Subdivided strand count: ") + std::to_string(dynamic_strands->strands.size())).c_str());
    ImGui::Text((std::string("Segment count: ") + std::to_string(dynamic_strands->segments.size())).c_str());
    ImGui::Text((std::string("Segment pair count: ") + std::to_string(dynamic_strands->segment_pairs.size())).c_str());
    if (ImGui::TreeNode("Meshing")) {
      dynamic_strands->meshing->Stats(editor_layer);
      ImGui::TreePop();
    }
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

  dynamic_strands->meshing->OnInspect(editor_layer);

  return false;
}

void DynamicTreeStrands::OnCreate() {
  dynamic_strands = std::make_shared<DynamicStrands>(materials);

  // initialize_parameters.meshing_type = MeshingType::KineticVoronoi;
  // initialize_parameters.max_segment_length = 0.01f;
  // initialize_parameters.min_segment_length = 0.005f; //OAK TRUNK SETTINGS

  dynamic_strands->Init(initialize_parameters.meshing_type);
  leaf_drop = std::make_shared<DsLeafDrop>();
  snow = std::make_shared<DsSnow>();
  wind = std::make_shared<DsWind>();
  box_selection_operator = std::make_shared<DsBoxSelection>();
  point_cut_operator = std::make_shared<DsPointCut>();
  drag_operator = std::make_shared<DsDrag>();
  line_cut_operator = std::make_shared<DsLineCut>();
  saw_operator = std::make_shared<DsSaw>();
  stop_all = std::make_shared<DsStopAll>();
  fungus_injection_operator = std::make_shared<DsFungusInjection>();
  enable_physics = true;
  if (!materials.bark_material_ref.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    materials.bark_material_ref = material;
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.1f;
    material->material_properties.albedo_color = glm::vec3(0.4f, 0.3f, 0.2f);
  }
  if (!materials.inner_wood_material_ref.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    materials.inner_wood_material_ref = material;
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.0f;
    material->material_properties.albedo_color = glm::vec3(1.f, 0.6f, 0.3f);
  }
  if (!materials.splinter_material_ref.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    materials.splinter_material_ref = material;
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.0f;
    material->material_properties.albedo_color = glm::vec3(1.f, 0.6f, 0.3f);
  }
  if (!materials.leaf_material_ref.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    materials.leaf_material_ref = material;
    material->material_properties.roughness = 1.f;
    material->material_properties.metallic = 0.3f;
    material->material_properties.albedo_color = glm::vec3(0.2f, 0.5f, 0.05f);
  }
  if (!materials.snow_material_ref.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    materials.snow_material_ref = material;
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.0f;
    material->material_properties.albedo_color = glm::vec3(1.0f);
  }

  if (!materials.segment_pair_material_ref.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    materials.segment_pair_material_ref = material;
    material->material_properties.roughness = 0.5f;
    material->material_properties.metallic = 0.0f;
    material->material_properties.albedo_color = glm::vec3(1.0f);
    material->material_properties.transmission = 0.5f;
  }

  if (!materials.wireframe_material_ref.Get<Material>()) {
    const auto material = AssetManager::CreateTemporaryAsset<Material>();
    materials.wireframe_material_ref = material;
    material->material_properties.roughness = 0.0f;
    material->material_properties.metallic = 0.0f;
    material->material_properties.albedo_color = glm::vec3(0.0f);
    material->material_properties.transmission = 0.0f;
  }
  foliage_rendering_instance_handle = Handle();
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
  fungus_injection_operator.reset();
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
  auto& strand_model_skeleton = strand_model.strand_model_skeleton;
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
        // TODO: set end_t (?)
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

  const bool trunk = initialize_parameters.trunk_additional_strength;
  initialize_parameters.trunk_additional_strength = false;
  DtsStrandGroup randomly_subdivided_strand_group{}, uniformly_subdivided_strand_group{};
  initialized_from_tree = false;
  UpdateDynamicStrands(randomly_subdivided_strand_group, uniformly_subdivided_strand_group);

  const auto& target_strand_segment_data_list = randomly_subdivided_strand_group.PeekStrandSegmentDataList();

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

  if (settings.fungus_test) {
    {
      /*auto& segment = dynamic_strands->segments[0];
      segment.RB = 1.0f;
      segment.RB_pre = 1.0f;*/
      Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
        auto& segment = dynamic_strands->segments[i];
        // segment.C = 0.0f;
        // segment.C_pre = 0.0f;
        if (segment.particle0.x0[0] < 0.005f && segment.particle0.x0[1] > 1.015f && segment.particle0.x0[2] > 0.0f &&
            segment.particle0.x0[2] < 0.01f) {
          segment.RB = 1.0f;
          segment.RB_pre = 1.0f;
        }
      });
      Region INIT{0.0f, 100.0f, -glm::pi<float>(), glm::pi<float>(), 0.0f, 1.0f};
      std::vector<Region> regions;
      Node_tilt* root = build_bsp_tilt(INIT, /*N=*/3200, 0.1f, 1.8f, 10,
                                       /*tilt_eps=*/0.0f, /*enable_tilt=*/false, regions);

      std::mt19937 rng(std::random_device{}());
      int K = (int)regions.size();
      std::uniform_real_distribution<float> dc(0.0f, 1.0f);
      std::vector<glm::vec4> region_colors(K);
      for (int i = 0; i < K; ++i) {
        region_colors[i] = glm::vec4(dc(rng), dc(rng), dc(rng), 1.0f);
      }
      Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
        auto& segment = dynamic_strands->segments[i];
        std::array<float, 3> pt = {segment.profile_position[0] + 1.0f, segment.profile_position[1],
                                   segment.particle0.x[0] + 0.05f};
        int id = classify_point_jitter_axis(pt, root, 0.000f, 0xA53A5F1Bu, true);
        segment.color = region_colors[id];
      });

      // Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
      //   auto& segment = dynamic_strands->segments[i];
      //   float midX = (segment.particle0.x0[0] + segment.particle1.x0[0]) / 2.0f;
      //   float midZ = (segment.particle0.x0[2] + segment.particle1.x0[2]) / 2.0f;
      //   if (!isInsideP(midX, midZ)) {
      //     segment.HC = 0.0f;
      //     segment.HC_pre = 0.0f;
      //     segment.HL = 0.0f;
      //     segment.HL_pre = 0.0f;
      //   }
      // });
    }
  }
  dynamic_strands->Upload();
  dynamic_strands->InitializeMesh(initialize_parameters);
  initialize_parameters.trunk_additional_strength = trunk;

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
  // TODO: move dynamic strands initialization here

  auto& strand_model_skeleton = strand_model.strand_model_skeleton;
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
    float offset = 0.0f;
    strand.start_position =
        glm::vec3(0.0f, settings.radius * profile_position.x + offset, settings.radius * profile_position.y);
    strand.start_color = glm::vec4(1, 1, 1, 1);
    strand.start_thickness = settings.radius * 2.f;
    const float distance_to_boundary = particle.GetDistanceToBoundary();
    for (int z = 0; z < settings.rod_segment_count; z++) {
      const auto segment_handle = strand_group.Extend(strand_handle);
      auto& segment = strand_group.RefStrandSegment(segment_handle);
      // TODO: set end_t (?)
      segment.end_position =
          glm::vec3(settings.segment_length * (static_cast<float>(z) + 1.f),
                    settings.radius * profile_position.x + offset, settings.radius * profile_position.y);
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

  const bool trunk = initialize_parameters.trunk_additional_strength;
  initialize_parameters.trunk_additional_strength = false;
  DtsStrandGroup randomly_subdivided_strand_group{}, uniformly_subdivided_strand_group{};
  initialized_from_tree = false;
  // if (settings.fungus_test) {
  //   initialize_parameters.min_segment_length = 0.015f;
  //   initialize_parameters.max_segment_length = 0.03f;
  // }
  UpdateDynamicStrands(randomly_subdivided_strand_group, uniformly_subdivided_strand_group);

  const auto& target_strand_segment_data_list = randomly_subdivided_strand_group.PeekStrandSegmentDataList();

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
  if (settings.competition_setting) {
    Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
      auto& segment = dynamic_strands->segments[i];
      segment.C = 0.0f;
      segment.C_pre = 0.0f;
      if (segment.boundary_distance < 0.01f && segment.profile_polar_coordinate.y > 0.7f &&
          segment.profile_polar_coordinate.y < 0.75f) {
        if (segment.particle0.x0[0] < 0.005f) {
          segment.RB = 1.0f;
          segment.RB_pre = 1.0f;
        }
        if (segment.particle1.x0[0] > 0.5f - 0.004f) {
          segment.RW = 1.0f;
          segment.RW_pre = 1.0f;
        }
      }
    });
  }
  if (settings.fungus_test) {
    if (settings.competition_setting == false) {
      auto& segment = dynamic_strands->segments[0];
      // segment.RW = 1.0f;
      segment.RB = 1.0f;
      // segment.RW_pre = 1.0f;
      segment.RB_pre = 1.0f;
      /*Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
        auto& segment = dynamic_strands->segments[i];
        segment.C = 0.0f;
        segment.C_pre = 0.0f;
        if (segment.boundary_distance < 0.02f && segment.profile_polar_coordinate.y > -0.75f &&
            segment.profile_polar_coordinate.y < -0.7f) {
          if (segment.particle0.x0[0] < 0.005f) {
            segment.RB = 1.0f;
            segment.RB_pre = 1.0f;
          }
        }
      });*/
    }
    if (false) {
      // Uniformly subdivided strand groups
      Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
        auto& segment = dynamic_strands->segments[i];
        const float angleStep = 0.2f;
        const float radiusStep = 5.0f;
        const float xStep = 0.04f;

        const float maxRadius = 100.f;
        const float xMin = -1.0f;
        const float xMax = 1.0f;

        int ai = int((segment.profile_polar_coordinate[1] + glm::pi<float>()) / angleStep);
        int ri = int(segment.profile_polar_coordinate[0] / radiusStep);
        int xi = int((segment.particle0.x[0] - xMin) / xStep);

        ai = std::max(ai, 0);
        ri = std::max(ri, 0);
        xi = std::max(xi, 0);

        int maxAi = int(std::ceil((2.0f * glm::pi<float>()) / angleStep));
        int maxRi = int(std::ceil(maxRadius / radiusStep));
        int maxXi = int(std::ceil((xMax - xMin) / xStep));
        ai = std::min(ai, maxAi - 1);
        ri = std::min(ri, maxRi - 1);
        xi = std::min(xi, maxXi - 1);

        std::size_t gid = (std::size_t(ai) * 73856093u) ^ (std::size_t(ri) * 19349663u) ^ (std::size_t(xi) * 83492791u);

        std::mt19937_64 rng(gid);
        std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        segment.color = glm::vec4(dist(rng), dist(rng), dist(rng), 1.0f);
      });
    } else {
      Region INIT{0.0f, 100.0f, -glm::pi<float>(), glm::pi<float>(), 0.0f, 1.0f};
      std::vector<Region> regions;
      // Node* root = build_bsp(INIT,
      //                        /*N=*/3200,
      //                        /*p_half=*/0.1f,
      //                        /*max_ratio=*/2.0f,
      //                        /*max_tries=*/10, regions);

      Node_tilt* root = build_bsp_tilt(INIT, /*N=*/12800, 0.1f, 1.8f, 10,
                                       /*tilt_eps=*/0.2f, /*enable_tilt=*/true, regions);

      std::mt19937 rng(std::random_device{}());
      int K = (int)regions.size();
      std::uniform_real_distribution<float> dc(0.0f, 1.0f);
      std::vector<glm::vec4> region_colors(K);
      for (int i = 0; i < K; ++i) {
        region_colors[i] = glm::vec4(dc(rng), dc(rng), dc(rng), 1.0f);
      }
      Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
        auto& segment = dynamic_strands->segments[i];
        std::array<float, 3> pt = {segment.profile_polar_coordinate[0], segment.profile_polar_coordinate[1],
                                   segment.particle0.x[0]};
        // int id = classify_point(pt, root);
        // int id = classify_point_tilt(pt, root);
        int id = classify_point_jitter_axis(pt, root, 0.0001f, 0xA53A5F1Bu, false);
        segment.color = region_colors[id];
      });
    }
    if (settings.internal_pattern) {
      Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
        auto& segment = dynamic_strands->segments[i];
        segment.internal_pattern = 1;
      });
    }
    if (settings.cube_pattern) {
      Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
        auto& segment = dynamic_strands->segments[i];
        segment.cube_pattern = 1;
      });
    }
  }
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
  initialize_parameters.trunk_additional_strength = trunk;

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
    case PivotType::Partial_Transform: {
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
        const auto& segment = dynamic_strands->segments[segment_handle];
        if (segment.boundary_distance >= 0.0f) {
          // Only lock segments that are not near a boundary
          segment_list[i].first = segment_handle;
          segment_list[i].second.first = true;
          segment_list[i].second.second = false;
        } else {
          segment_list[i].first = UINT32_MAX;
          segment_list[i].second.first = false;
          segment_list[i].second.second = false;
        }
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
    case PivotType::Partial_Transform: {
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
        const auto& segment = dynamic_strands->segments[segment_handle];
        if (segment.boundary_distance >= 0.0f) {
          // Only lock segments that are not near a boundary
          segment_list[i].first = segment_handle;
          segment_list[i].second.first = false;
          segment_list[i].second.second = true;
        } else {
          segment_list[i].first = UINT32_MAX;
          segment_list[i].second.first = false;
          segment_list[i].second.second = false;
        }
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

  const auto particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  std::vector<ParticleInfo> particle_infos;
  target_strand_group.BuildParticles(particle_infos);
  particle_info_list->SetParticleInfos(particle_infos);

  renderer->particle_info_list = particle_info_list;
  renderer->mesh = Resources::Primitives::cube;
  const auto material = AssetManager::CreateTemporaryAsset<Material>();

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
    if (const auto fd = td->foliage_descriptor.Get<BasicFoliageDescriptor>()) {
      if (const auto mat = fd->leaf_material_ref.Get<Material>())
        materials.leaf_material_ref = mat;
    }
    if (const auto bd = td->bark_descriptor.Get<BasicBarkDescriptor>()) {
      if (const auto mat = bd->bark_material_ref.Get<Material>())
        materials.bark_material_ref = mat;
    }
  }
  strand_model = tree->shoot_strand_model;
  DtsStrandGroup randomly_subdivided_strand_group{}, uniformly_subdivided_strand_group{};
  initialized_from_tree = true;
  UpdateDynamicStrands(randomly_subdivided_strand_group, uniformly_subdivided_strand_group);

  Region INIT{0.0f, 100.0f, -glm::pi<float>(), glm::pi<float>(), 0.0f, 1.0f};
  std::vector<Region> regions;
  Node_tilt* root = build_bsp_tilt(INIT, /*N=*/12800, 0.1f, 1.8f, 10,
                                   /*tilt_eps=*/0.0f, /*enable_tilt=*/false, regions, 1500.f,
                                   3000.f);  // ZY: 800 for pull operator test

  std::mt19937 rng(std::random_device{}());
  int K = (int)regions.size();
  std::uniform_real_distribution<float> dc(0.0f, 1.0f);
  std::vector<glm::vec4> region_colors(K);
  for (int i = 0; i < K; ++i) {
    region_colors[i] = glm::vec4(dc(rng), dc(rng), dc(rng), 1.0f);
  }
  Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
    auto& segment = dynamic_strands->segments[i];
    // std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 2.f, segment.profile_polar_coordinate[1],
    //                            segment.particle0.x[1] * 0.5 + 0.1f};
    std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 2.f, segment.profile_polar_coordinate[1],
                               segment.particle0.root_distance * 0.5f + 0.1f};  // for general
    // std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 2.f, segment.profile_polar_coordinate[1]
    // * 2.0f,
    //                            segment.particle0.root_distance * 0.3f + 0.1f}; //ZY: for oak trunk ONLY
    int id = classify_point_jitter_axis(pt, root, 0.0f, 0xA53A5F1Bu, false);  // ZY:false for pull operator test
    segment.color = region_colors[id];
  });

  dynamic_strands->Upload();
  dynamic_strands->InitializeMesh(initialize_parameters);
}

void DynamicTreeStrands::Cubic_pattern() {
  Region INIT{0.0f, 100.0f, -glm::pi<float>(), glm::pi<float>(), 0.0f, 1.0f};
  std::vector<Region> regions;
  Node_tilt* root = build_bsp_tilt(INIT, /*N=*/12800, 0.1f, 1.8f, 10,
                                   /*tilt_eps=*/0.0f, /*enable_tilt=*/false, regions, 1500.f,
                                   3000.f);  // ZY: 800 for pull operator test

  std::mt19937 rng(std::random_device{}());
  int K = (int)regions.size();
  std::uniform_real_distribution<float> dc(0.0f, 1.0f);
  std::vector<glm::vec4> region_colors(K);
  for (int i = 0; i < K; ++i) {
    region_colors[i] = glm::vec4(dc(rng), dc(rng), dc(rng), 1.0f);
  }
  Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
    auto& segment = dynamic_strands->segments[i];
    // std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 2.f, segment.profile_polar_coordinate[1],
    //                            segment.particle0.x[1] * 0.5 + 0.1f};
    // std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 2.f, segment.profile_polar_coordinate[1],
    //                           segment.particle0.root_distance * 0.5f + 0.1f};  // for general
    std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 1.5f, segment.profile_polar_coordinate[1] * 2.3f,
                               segment.particle0.root_distance * 0.2f + 0.1f};  // ZY: for oak trunk ONLY
    int id = classify_point_jitter_axis(pt, root, 0.0f, 0xA53A5F1Bu, false);    // ZY:false for pull operator test
    segment.color = region_colors[id];
  });
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
      if (fungus_injection_operator->enabled) {
        fungus_injection_operator->Execute(dynamic_strands);
      }
    });
  }
}

void DynamicTreeStrands::Visualization(const std::shared_ptr<Camera>& target_camera,
                                       const DynamicStrandsVisualizationParameters& visualization_parameters) const {
  if (!dynamic_strands->segments.empty()) {
    dynamic_strands->Visualize(target_camera, initialize_parameters, visualization_parameters);
  }
}

void DynamicTreeStrands::RegisterFoliageRenderInstance(const FoliageRenderParameters& render_parameters) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  if (const auto material = materials.leaf_material_ref.Get<Material>()) {
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

void DynamicTreeStrands::RegisterSegmentPairRenderInstance(const SegmentPairsRenderParameters& render_parameters) {
  const auto render_layer = Application::GetLayer<RenderLayer>();
  if (!render_layer) {
    EVOENGINE_LOG("Failed to render! RenderLayer not present!")
    return;
  }
  if (const auto material = materials.segment_pair_material_ref.Get<Material>()) {
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

void DynamicTreeStrands::split_one(const Region& c, float p_min, float p_max, float max_ratio, int max_tries,
                                   Region& c1, Region& c2, Axis3& out_axis, float& out_coord, float r_scale_coef,
                                   float p_scale_coef) {
  float r_scale = r_scale_coef;
  float p_scale = p_scale_coef / c.r_max;
  float dr = (c.r_max - c.r_min) / r_scale;
  float dp = (c.phi_max - c.phi_min) / p_scale;
  float dx = c.x_max - c.x_min;
  float rr = dr / std::max(dp, dx);
  float rp = dp / std::max(dr, dx);
  float rx = dx / std::max(dr, dp);
  if (rr > max_ratio)
    out_axis = AX_R;
  else if (rp > max_ratio)
    out_axis = AX_PHI;
  else if (rx > max_ratio)
    out_axis = AX_X;
  else {
    if (dr >= dp && dr >= dx)
      out_axis = AX_R;
    else if (dp >= dr && dp >= dx)
      out_axis = AX_PHI;
    else
      out_axis = AX_X;
  }

  std::mt19937 rng(std::random_device{}());
  // std::mt19937 rng(42u);
  std::uniform_real_distribution<float> dist(p_min, p_max);

  for (int i = 0; i < max_tries; ++i) {
    float p = dist(rng), mid;
    if (out_axis == AX_R) {
      mid = c.r_min + dr * p * r_scale;
      c1 = {c.r_min, mid, c.phi_min, c.phi_max, c.x_min, c.x_max};
      c2 = {mid, c.r_max, c.phi_min, c.phi_max, c.x_min, c.x_max};
    } else if (out_axis == AX_PHI) {
      mid = c.phi_min + dp * p * p_scale;
      c1 = {c.r_min, c.r_max, c.phi_min, mid, c.x_min, c.x_max};
      c2 = {c.r_min, c.r_max, mid, c.phi_max, c.x_min, c.x_max};
    } else {
      // AX_X
      mid = c.x_min + dx * p;
      c1 = {c.r_min, c.r_max, c.phi_min, c.phi_max, c.x_min, mid};
      c2 = {c.r_min, c.r_max, c.phi_min, c.phi_max, mid, c.x_max};
    }
    out_coord = mid;

    auto good = [&](const Region& R) {
      float e[3] = {(R.r_max - R.r_min) / r_scale, (R.phi_max - R.phi_min) / p_scale, R.x_max - R.x_min};
      std::sort(e, e + 3);
      return (e[2] / e[1] <= max_ratio) && (e[1] / e[0] <= max_ratio);
    };
    if (good(c1) && good(c2))
      return;
  }

  float mid;
  if (out_axis == AX_R) {
    mid = 0.5f * (c.r_min + c.r_max);
    out_coord = mid;
    c1 = {c.r_min, mid, c.phi_min, c.phi_max, c.x_min, c.x_max};
    c2 = {mid, c.r_max, c.phi_min, c.phi_max, c.x_min, c.x_max};
  } else if (out_axis == AX_PHI) {
    mid = 0.5f * (c.phi_min + c.phi_max);
    out_coord = mid;
    c1 = {c.r_min, c.r_max, c.phi_min, mid, c.x_min, c.x_max};
    c2 = {c.r_min, c.r_max, mid, c.phi_max, c.x_min, c.x_max};
  } else {
    mid = 0.5f * (c.x_min + c.x_max);
    out_coord = mid;
    c1 = {c.r_min, c.r_max, c.phi_min, c.phi_max, c.x_min, mid};
    c2 = {c.r_min, c.r_max, c.phi_min, c.phi_max, mid, c.x_max};
  }
}

DynamicTreeStrands::Node* DynamicTreeStrands::build_bsp(const Region& init, int N, float p_half, float max_ratio,
                                                        int max_tries, std::vector<Region>& out_regions,
                                                        float r_scale_coef, float p_scale_coef) {
  float p_min = 0.5f - p_half, p_max = 0.5f + p_half;
  Node* root = new Node(0);
  std::map<int, Region> leaves;
  leaves[0] = init;
  int next_id = 1;

  while ((int)leaves.size() < N) {
    int pick = -1;
    float best = -1.0f;
    for (auto& kv : leaves) {
      const Region& R = kv.second;
      float vol = (R.r_max - R.r_min) * (R.phi_max - R.phi_min) * (R.x_max - R.x_min);
      if (vol > best) {
        best = vol;
        pick = kv.first;
      }
    }
    Region cur = leaves[pick];
    leaves.erase(pick);

    Region c1, c2;
    Axis3 axis;
    float coord;
    split_one(cur, p_min, p_max, max_ratio, max_tries, c1, c2, axis, coord, r_scale_coef, p_scale_coef);

    std::function<bool(Node*)> ins = [&](Node* n) -> bool {
      if (n->leaf_id == pick) {
        n->axis = axis;
        n->coord = coord;
        n->leaf_id = -1;
        n->left = new Node(pick);
        n->right = new Node(next_id);
        return true;
      }
      return (n->left && ins(n->left)) || (n->right && ins(n->right));
    };
    ins(root);

    leaves[pick] = c1;
    leaves[next_id] = c2;
    ++next_id;
  }

  out_regions.resize(leaves.size());
  for (auto& kv : leaves) {
    out_regions[kv.first] = kv.second;
  }
  return root;
}

int DynamicTreeStrands::classify_point(const std::array<float, 3>& pt, Node* node) {
  if (node->leaf_id >= 0)
    return node->leaf_id;
  float v = (node->axis == AX_R ? pt[0] : node->axis == AX_PHI ? pt[1] : pt[2]);
  return classify_point(pt, v < node->coord ? node->left : node->right);
}

glm::vec3 normalize_param(float r, float phi, float x) {
  float rn = (r) / (100.f);
  float phin = (phi + glm::pi<float>()) / (2 * glm::pi<float>());
  float xn = (x) / (1.f);
  return {rn, phin, xn};
}

float DynamicTreeStrands::normalize_coord(Axis3 axis, float coord) {
  if (axis == AX_R)
    return (coord) / (100.f);
  if (axis == AX_PHI)
    return (coord + glm::pi<float>()) / (2 * glm::pi<float>());
  /* AX_X */
  return (coord) / (1.f);
}

inline float wrap01(float v) {
  return v - std::floor(v);
}

inline uint32_t wang_hash(uint32_t x) {
  x = (x ^ 61u) ^ (x >> 16);
  x += (x << 3);
  x ^= (x >> 4);
  x *= 0x27d4eb2du;
  x ^= (x >> 15);
  return x;
}

inline float hash_to_symmetric01(uint32_t& state) {
  state = wang_hash(state);
  return (float(state) / 4294967295.0f) * 2.0f - 1.0f;
}

inline uint32_t seed_from_point(const std::array<float, 3>& pt, uint32_t base = 0x9E3779B9u) {
  auto pack = [](float f) -> uint32_t {
    return (uint32_t)std::floor(f * 65536.0f);
  };
  uint32_t x = pack(pt[0]);
  uint32_t y = pack(pt[1]);
  uint32_t z = pack(pt[2]);
  uint32_t s = base;
  s ^= x * 73856093u;
  s ^= y * 19349663u;
  s ^= z * 83492791u;
  return wang_hash(s);
}

DynamicTreeStrands::Node_tilt* DynamicTreeStrands::build_bsp_tilt(const Region& init, int N, float p_half,
                                                                  float max_ratio, int max_tries, float tilt_eps,
                                                                  bool enable_tilt, std::vector<Region>& out_regions,
                                                                  float r_scale_coef, float p_scale_coef) {
  float p_min = 0.5f - p_half, p_max = 0.5f + p_half;

  Node_tilt* root = new Node_tilt(0);
  std::map<int, Region> leaves;
  leaves[0] = init;
  int next_id = 1;

  std::function<bool(Node_tilt*, int, Axis3, float, const Region&)> attach =
      [&](Node_tilt* n, int target, Axis3 axis, float coord, const Region& cur) -> bool {
    if (n->leaf_id == target) {
      n->axis = axis;
      n->coord = coord;
      n->leaf_id = -1;
      n->left = new Node_tilt(target);
      n->right = new Node_tilt(next_id);

      if (enable_tilt) {
        float rc = 0.5f * (cur.r_min + cur.r_max);
        float phic = 0.5f * (cur.phi_min + cur.phi_max);
        float xc = 0.5f * (cur.x_min + cur.x_max);
        if (axis == AX_R)
          rc = coord;
        else if (axis == AX_PHI)
          phic = coord;
        else
          xc = coord;
        n->p0_n = normalize_param(rc, phic, xc);

        glm::vec3 base = (axis == AX_R)     ? glm::vec3(1, 0, 0)
                         : (axis == AX_PHI) ? glm::vec3(0, 1, 0)
                                            : glm::vec3(0, 0, 1);
        std::uniform_real_distribution<float> d(-tilt_eps, tilt_eps);
        glm::vec3 nn = base;
        std::mt19937 g_rng(std::random_device{}());
        // std::mt19937 g_rng(42u);
        if (axis == AX_R) {
          // nn += glm::vec3(0, d(g_rng), d(g_rng));
        } else if (axis == AX_PHI)
          nn += glm::vec3(d(g_rng), 0, d(g_rng));
        else
          nn += glm::vec3(d(g_rng), d(g_rng), 0);
        n->n_tilt = glm::normalize(nn);
        n->has_tilt = true;
      }
      return true;
    }
    return (n->left && attach(n->left, target, axis, coord, cur)) ||
           (n->right && attach(n->right, target, axis, coord, cur));
  };

  while ((int)leaves.size() < N) {
    int pick = -1;
    float bestV = -1.0f;
    for (auto& kv : leaves) {
      const Region& R = kv.second;
      float v = (R.r_max - R.r_min) * (R.phi_max - R.phi_min) * (R.x_max - R.x_min);
      if (v > bestV) {
        bestV = v;
        pick = kv.first;
      }
    }
    Region cur = leaves[pick];
    leaves.erase(pick);

    Region c1, c2;
    Axis3 axis;
    float coord;
    split_one(cur, p_min, p_max, max_ratio, max_tries, c1, c2, axis, coord, r_scale_coef, p_scale_coef);

    attach(root, pick, axis, coord, cur);
    leaves[pick] = c1;
    leaves[next_id] = c2;
    ++next_id;
  }

  out_regions.resize(leaves.size());
  for (auto& kv : leaves)
    out_regions[kv.first] = kv.second;
  return root;
}

int DynamicTreeStrands::classify_point_tilt(const std::array<float, 3>& pt, const Node_tilt* node) {
  if (node->leaf_id >= 0)
    return node->leaf_id;

  if (!node->has_tilt) {
    float v = (node->axis == AX_R ? pt[0] : node->axis == AX_PHI ? pt[1] : pt[2]);
    return classify_point_tilt(pt, (v < node->coord) ? node->left : node->right);
  }
  glm::vec3 p_n = normalize_param(pt[0], pt[1], pt[2]);
  float side = glm::dot(node->n_tilt, p_n - node->p0_n);
  return classify_point_tilt(pt, (side < 0.0f) ? node->left : node->right);
}

void DynamicTreeStrands::delete_tree(Node_tilt* n) {
  if (!n)
    return;
  delete_tree(n->left);
  delete_tree(n->right);
  delete n;
}

int DynamicTreeStrands::classify_point_jitter_axis(const std::array<float, 3>& pt, const Node_tilt* node,
                                                   float eps_norm, uint32_t base_seed, bool wrap) {
  glm::vec3 pn = normalize_param(pt[0], pt[1], pt[2]);

  const Node_tilt* n = node;
  while (n->leaf_id < 0) {
    float v_n = (n->axis == AX_R ? pn.x : n->axis == AX_PHI ? pn.y : pn.z);
    float coord_n = normalize_coord(n->axis, n->coord);

    uint32_t salt = (uint32_t)((uintptr_t)n & 0xFFFFFFFFu);
    uint32_t state = seed_from_point(pt, base_seed ^ salt);

    float j = hash_to_symmetric01(state) * eps_norm;

    float vj = v_n + j;
    if (n->axis == AX_PHI)
      vj = wrap01(vj);
    if (n->axis == AX_R) {
      if (wrap) {
        vj = wrap01(vj);
      } else {
        vj = glm::clamp(vj, 0.0f, 1.0f);
      }
    }
    if (n->axis == AX_X)
      if (wrap) {
        vj = wrap01(vj);
      } else {
        vj = glm::clamp(vj, 0.0f, 1.0f);
      }
    n = (vj < coord_n) ? n->left : n->right;
  }
  return n->leaf_id;
}

bool DynamicTreeStrands::isInsideS(float x, float z) {
  float xMin = 0.0f, xMax = 0.8f;
  float zMin = -0.25f, zMax = 0.25f;
  float thickness = 0.1f;

  float xMid = (xMin + xMax) / 2.0f;

  if (x >= xMin && x <= xMin + thickness && z >= zMin && z <= zMax)
    return true;
  // Middle Bar
  if (x >= xMid - thickness / 2.0f && x <= xMid + thickness / 2.0f && z >= zMin && z <= zMax)
    return true;
  // Right Bar
  if (x >= xMax - thickness && x <= xMax && z >= zMin && z <= zMax)
    return true;

  if (z >= zMax - thickness && z <= zMax && x >= xMin && x <= xMid)
    return true;

  if (z >= zMin && z <= zMin + thickness && x >= xMid && x <= xMax)
    return true;

  return false;
}

bool DynamicTreeStrands::isInsideG(float x, float z) {
  float xMin = 0.0f, xMax = 0.8f;
  float zMin = -0.25f, zMax = 0.25f;
  float thickness = 0.1f;

  float xMid = (xMin + xMax) / 2.0f;

  if (z >= zMin && z <= zMin + thickness && x >= xMin && x <= xMax)
    return true;

  if (x >= xMax - thickness && x <= xMax && z >= zMin && z <= zMax)
    return true;

  if (x >= xMin && x <= xMin + thickness && z >= zMin && z <= zMax)
    return true;

  if (z >= zMax - thickness && z <= zMax && x >= xMin && x <= xMid)
    return true;

  if (x >= xMid - thickness / 2.0f && x <= xMid + thickness / 2.0f && z >= 0.0f && z <= zMax)
    return true;

  return false;
}

bool DynamicTreeStrands::isInsideP(float x, float z) {
  float xMin = 0.0f, xMax = 0.8f;
  float zMin = -0.25f, zMax = 0.25f;
  float thickness = 0.1f;

  float xMid = (xMin + xMax) / 2.0f;

  if (z >= zMin && z <= zMin + thickness && x >= xMin && x <= xMax)
    return true;

  if (x >= xMax - thickness && x <= xMax && z >= zMin && z <= zMax)
    return true;

  if (x >= xMid - thickness / 2.0f && x <= xMid + thickness / 2.0f && z >= zMin && z <= zMax)
    return true;

  if (z >= zMax - thickness && z <= zMax && x >= xMid && x <= xMax)
    return true;

  return false;
}