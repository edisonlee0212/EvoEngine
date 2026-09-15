#include "Application.hpp"
#include "BasicBarkDescriptor.hpp"
#include "DsAlphaShapeMeshing.hpp"
#include "DsConstraints.hpp"
#include "DsKineticVoronoiMeshing.hpp"
#include "DsMaterials.hpp"
#include "DsOperators.hpp"
#include "DsPhysics.hpp"
#include "DynamicStrands.hpp"
#include "DynamicStrandsComponentInspectors.hpp"
#include "DynamicStrandsMeshingInspector.hpp"
#include "DynamicTreeStrands.hpp"
#include "DynamicsSettingsEditor.hpp"
#include "EcoSysLabSerializationAdapters.hpp"
#include "EditorLayer.hpp"
#include "EditorWidgets.hpp"
#include "Tree.hpp"
#include "VoronoiMeshGenerator.hpp"

using namespace evo_engine;
using namespace eco_sys_lab_package;
bool DynamicTreeStrandsInspector::Inspect(InspectorContext& context, DynamicTreeStrands& target) {
  const auto& editor_layer = context.editor_layer;
  auto& inspector_context = context;
  if (ImGui::TreeNode("Preset settings")) {
    const auto apply_bundle_preset = [&](const BundleSolverMode mode, const int iterations) {
      target.initialize_parameters.bundle_solver.mode = mode;
      target.initialize_parameters.bundle_solver.legacy_iterations = iterations;
      target.initialize_parameters.bundle_solver.pair_iterations = iterations;
      target.initialize_parameters.bundle_solver.coarse_iterations = glm::max(1, iterations / 2);
      if (mode == BundleSolverMode::Legacy && target.dynamic_strands) {
        for (auto& constraint : target.dynamic_strands->constraints) {
          if (const auto bundle = std::dynamic_pointer_cast<DsBundle>(constraint)) {
            bundle->solver_settings = target.initialize_parameters.bundle_solver;
            bundle->sub_iteration = iterations;
            break;
          }
        }
      }
    };
    if (ImGui::Button("Oak Trunk")) {
      target.initialize_parameters.min_segment_length = 0.005f;
      target.initialize_parameters.max_segment_length = 0.01f;
    }
    if (ImGui::Button("Spruce (Legacy)"))
      apply_bundle_preset(BundleSolverMode::Legacy, 5);
    ImGui::SameLine();
    if (ImGui::Button("Spruce (Coupled XPBD)"))
      apply_bundle_preset(BundleSolverMode::CoupledXpbd, 5);
    ImGui::SameLine();
    if (ImGui::Button("Spruce (Hybrid)"))
      apply_bundle_preset(BundleSolverMode::Hybrid, 5);
    if (ImGui::Button("Oak (Legacy)"))
      apply_bundle_preset(BundleSolverMode::Legacy, 3);
    ImGui::SameLine();
    if (ImGui::Button("Oak (Coupled XPBD)"))
      apply_bundle_preset(BundleSolverMode::CoupledXpbd, 3);
    ImGui::SameLine();
    if (ImGui::Button("Oak (Hybrid)"))
      apply_bundle_preset(BundleSolverMode::Hybrid, 3);
    ImGui::TextDisabled("Coupled XPBD and Hybrid preset changes apply after re-subdivision.");
    ImGui::TreePop();
  }
  ImGui::RadioButton("Kinetic Voronoi Meshing", reinterpret_cast<int*>(&target.initialize_parameters.meshing_type),
                     static_cast<int>(MeshingType::KineticVoronoi));
  ImGui::SameLine();
  ImGui::RadioButton("Alpha Shape Meshing", reinterpret_cast<int*>(&target.initialize_parameters.meshing_type),
                     static_cast<int>(MeshingType::AlphaShape));
  ImGui::DragInt("Seed", &target.seed, 1, 0, INT_MAX);
  editor_layer->DragAndDropButton<Material>(target.materials.bark_material_ref, "Bark Material");
  editor_layer->DragAndDropButton<Material>(target.materials.inner_wood_material_ref, "Inner wood Material");
  editor_layer->DragAndDropButton<Material>(target.materials.splinter_material_ref, "Splinter Material");
  editor_layer->DragAndDropButton<Material>(target.materials.leaf_material_ref, "Leaf Material");
  editor_layer->DragAndDropButton<Material>(target.materials.snow_material_ref, "Snow Material");
  editor_layer->DragAndDropButton<Material>(target.materials.wireframe_material_ref, "Wireframe Material");
  if (ImGui::TreeNode("Initialization settings")) {
    InspectorRegistry::GetInstance().InspectValue(inspector_context, target.initialize_parameters);
    if (ImGui::Button("Re-initialize mesh")) {
      target.dynamic_strands->InitializeMesh(target.initialize_parameters);
    }
    ImGui::Checkbox("Limit strand length", &target.limit_strand_length);
    if (target.limit_strand_length) {
      ImGui::DragFloat("Max strand length", &target.max_strand_length, 0.01f, 0.01f, 10.0f);
    }
    ImGui::TreePop();
  }

  if (editor_layer->DragAndDropButton<Tree>(dynamic_tree_strands_tree_ref, "Download Strands from Tree...")) {
    if (const auto tree = dynamic_tree_strands_tree_ref.Get<Tree>()) {
      target.InitializeFromTree(tree);
      dynamic_tree_strands_tree_ref.Clear();
    }
  }

  const auto& strand_group = target.strand_model.strand_model_skeleton.data.strand_group;
  if (ImGui::Button("Re-subdivide")) {
    // initialize_parameters.min_segment_length = 0.005f;
    // initialize_parameters.max_segment_length = 0.01f;

    DtsStrandGroup randomly_subdivided_strand_group{}, uniformly_subdivided_strand_group{};
    target.UpdateDynamicStrands(randomly_subdivided_strand_group, uniformly_subdivided_strand_group);

    DynamicTreeStrands::Region INIT{0.0f, 100.0f, -glm::pi<float>(), glm::pi<float>(), 0.0f, 1.0f};
    std::vector<DynamicTreeStrands::Region> regions;
    DynamicTreeStrands::Node_tilt* root =
        target.build_bsp_tilt(INIT, /*N=*/12800, 0.1f, 1.8f, 10,
                              /*tilt_eps=*/0.0f, /*enable_tilt=*/false, regions, 1500.f,
                              3000.f);  // ZY: 800 for pull operator test

    std::mt19937 rng(std::random_device{}());
    int K = (int)regions.size();
    std::uniform_real_distribution<float> dc(0.0f, 1.0f);
    std::vector<glm::vec4> region_colors(K);
    for (int i = 0; i < K; ++i) {
      region_colors[i] = glm::vec4(dc(rng), dc(rng), dc(rng), 1.0f);
    }
    Jobs::RunParallelFor(target.dynamic_strands->segments.size(), [&](const auto i) {
      auto& segment = target.dynamic_strands->segments[i];
      // std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 2.f, segment.profile_polar_coordinate[1],
      //                            segment.particle0.x[1] * 0.5 + 0.1f};
      std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 2.f, segment.profile_polar_coordinate[1],
                                 segment.particle0.root_distance * 0.5f + 0.1f};  // for general
      // std::array<float, 3> pt = {segment.profile_polar_coordinate[0] * 2.f, segment.profile_polar_coordinate[1]
      // * 2.0f,
      //                            segment.particle0.root_distance * 0.3f + 0.1f}; //ZY: for oak trunk ONLY
      int id =
          target.classify_point_jitter_axis(pt, root, 0.0f, 0xA53A5F1Bu, false);  // ZY:false for pull operator test
      segment.color = region_colors[id];
    });

    /*Jobs::RunParallelFor(dynamic_strands->segments.size(), [&](const auto i) {
      auto& segment = dynamic_strands->segments[i];
      auto cur_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f);
      segment.color = cur_color;
    });*/

    target.dynamic_strands->Upload();
    target.dynamic_strands->InitializeMesh(target.initialize_parameters);
  }
  ImGui::SameLine();
  if (ImGui::Button("Test Static Mesh")) {
    // TODO: Pass the strands to the voronoi mesh generator to test creation of static meshes
    StrandModelMeshGeneratorSettings settings;
    std::vector<Vertex> vertices;
    std::vector<glm::vec2> tex_coords;
    std::vector<std::pair<unsigned int, unsigned int>> index_pairs;

    DtsStrandGroup randomly_subdivided_strand_group{}, uniformly_subdivided_strand_group{};
    target.UpdateDynamicStrands(randomly_subdivided_strand_group, uniformly_subdivided_strand_group);

    VoronoiMeshGenerator::Generate(uniformly_subdivided_strand_group, target.strand_model, vertices, tex_coords,
                                   index_pairs, settings);
  }
  if (ImGui::TreeNodeEx("Experiments", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::TreeNode("Board Experiment")) {
      InspectSettings(multiple_rod_experiment_setup_settings, editor_layer);
      if (ImGui::Button("Initialize")) {
        target.BoardExperimentSetup(multiple_rod_experiment_setup_settings);
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Log Experiment")) {
      InspectSettings(log_experiment_setup_settings, editor_layer);
      if (ImGui::Button("Initialize")) {
        target.LogExperimentSetup(log_experiment_setup_settings);
      }
      ImGui::TreePop();
    }
    if (!target.bundle_experiment_name.empty() && ImGui::Button("Capture bundle diagnostics")) {
      target.dynamic_strands->Download();
      for (const auto& constraint : target.dynamic_strands->constraints) {
        if (const auto bundle = std::dynamic_pointer_cast<DsBundle>(constraint)) {
          constexpr const char* mode_names[] = {"legacy", "coupled-xpbd", "hybrid"};
          const std::string mode_suffix =
              bundle->solver_settings.mode == BundleSolverMode::Legacy
                  ? ""
                  : std::string("-") + mode_names[static_cast<int>(bundle->solver_settings.mode)];
          const auto path = ProjectManager::GetProjectPath().parent_path() / "Diagnostics" /
                            (target.bundle_experiment_name + mode_suffix + "-bundle-diagnostics.yaml");
          CaptureBundleExperimentDiagnostics(target.bundle_experiment_name, *target.dynamic_strands, *bundle,
                                             target.bundle_experiment_reference)
              .Save(path);
          EVOENGINE_LOG("Saved dynamic-strands bundle diagnostics: " + path.string());
          break;
        }
      }
    }
    ImGui::TreePop();
  }
  if (ImGui::TreeNode("Stats")) {
    ImGui::Text((std::string("Original strand count: ") + std::to_string(strand_group.PeekStrands().size())).c_str());
    ImGui::Text(
        (std::string("Original strand segment count: ") + std::to_string(strand_group.PeekStrandSegments().size()))
            .c_str());
    ImGui::Text(
        (std::string("Subdivided strand count: ") + std::to_string(target.dynamic_strands->strands.size())).c_str());
    ImGui::Text((std::string("Segment count: ") + std::to_string(target.dynamic_strands->segments.size())).c_str());
    ImGui::Text(
        (std::string("Segment pair count: ") + std::to_string(target.dynamic_strands->segment_pairs.size())).c_str());
    if (ImGui::TreeNode("Meshing")) {
      DynamicStrandsMeshingInspector::DrawStats(*target.dynamic_strands->meshing);
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }

  ImGui::Checkbox("Physics", &target.enable_physics);
  if (ImGui::TreeNode("Physics settings")) {
    if (ImGui::TreeNodeEx("Prediction", ImGuiTreeNodeFlags_DefaultOpen)) {
      InspectorRegistry::GetInstance().InspectValue(inspector_context, *target.dynamic_strands->prediction);
      ImGui::TreePop();
    }
    if (ImGui::TreeNodeEx("Operators", ImGuiTreeNodeFlags_DefaultOpen)) {
      if (ImGui::TreeNode("Transform operators")) {
        for (auto& i : target.transform_pivots) {
          InspectorRegistry::GetInstance().InspectValue(inspector_context, *i.ds_pivot_transform);
        }
        ImGui::TreePop();
      }
      if (target.leaf_drop) {
        if (ImGui::TreeNodeEx("Leaf Drop", ImGuiTreeNodeFlags_DefaultOpen)) {
          InspectorRegistry::GetInstance().InspectValue(inspector_context, *target.leaf_drop);
          ImGui::TreePop();
        }
      }
      if (target.snow) {
        if (ImGui::TreeNodeEx("Snow", ImGuiTreeNodeFlags_DefaultOpen)) {
          InspectorRegistry::GetInstance().InspectValue(inspector_context, *target.snow);
          ImGui::TreePop();
        }
      }
      if (target.wind) {
        if (ImGui::TreeNodeEx("Wind", ImGuiTreeNodeFlags_DefaultOpen)) {
          InspectorRegistry::GetInstance().InspectValue(inspector_context, *target.wind);
          ImGui::TreePop();
        }
      }
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Constraint")) {
      for (auto& i : target.dynamic_strands->constraints)
        InspectorRegistry::GetInstance().InspectValue(inspector_context, *i);
      ImGui::TreePop();
    }
    if (ImGui::TreeNode("Dynamic Hashed Grid")) {
      InspectorRegistry::GetInstance().InspectValue(inspector_context, *target.dynamic_strands->dynamic_hashed_grid);
      ImGui::TreePop();
    }
    ImGui::TreePop();
  }

  if (ImGui::Button("Download strands")) {
    target.dynamic_strands->Download();
    EVOENGINE_LOG("Downloaded data from GPU")
  }
  ImGui::SameLine();
  if (ImGui::Button("Upload strands")) {
    target.dynamic_strands->Upload();
    EVOENGINE_LOG("Uploaded data from GPU")
  }

  InspectorRegistry::GetInstance().InspectValue(inspector_context, *target.dynamic_strands->meshing);

  return false;
}
