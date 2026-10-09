#include "DynamicStrandsDemo.hpp"
#include "imgui.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
#include <unordered_map>

#ifdef _WIN32
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <Windows.h>
#endif

#include "AlphaHullFractalExperiment.hpp"
#include "BufferExporter.hpp"
#include "DsAlphaShapeMeshing.hpp"
#include "DsAlphaShapeVolumeUtils.hpp"
#include "DsColliders.hpp"
#include "DsIntersectionBoundaryMesh.hpp"
#include "DsIntersectionBoundaryMeshGroup.hpp"
#include "DsKineticVoronoiMeshing.hpp"
#include "DsKineticVoronoiVolumeUtils.hpp"
#include "DynamicTreeStrands.hpp"
#include "EcoSysLabLayer.hpp"
#include "EcoSysLabPaths.hpp"
#include "KineticMeshCachePipeline.hpp"
#include "ProjectManager.hpp"
#include "SceneCameraBridge.hpp"
#include "Tree.hpp"
#include "VolumetricTreeExperiment.hpp"
#include "kinDS/kinDS/ObjExporter.hpp"
#include "kinDS/kinDS/Statistics.hpp"
#include "kinDS/kinDS/StrandTree.hpp"

using namespace eco_sys_lab_package;

namespace {

constexpr float kLogExperimentPivotDuration = 60.f;
constexpr float kLogExperimentMaxBreakAngle = glm::pi<float>() * 0.5f;

DynamicStrandsDemo::DemoType DemoTypeForExperiment(const VolumetricTreeExperimentId id) {
  switch (id) {
    case VolumetricTreeExperimentId::SmallTrunk:
    case VolumetricTreeExperimentId::SmallTrunkAlpha1e9:
      return DynamicStrandsDemo::DemoType::SmallTrunk;
    case VolumetricTreeExperimentId::NormalTrunk:
      return DynamicStrandsDemo::DemoType::NormalTrunk;
    case VolumetricTreeExperimentId::StockyTrunk:
      return DynamicStrandsDemo::DemoType::StockyTrunk;
    case VolumetricTreeExperimentId::OakThickStump100:
    case VolumetricTreeExperimentId::OakThickStump100Alpha10:
    case VolumetricTreeExperimentId::OakThickStump100Alpha20:
      return DynamicStrandsDemo::DemoType::OakThickStump;
    case VolumetricTreeExperimentId::OakThickStump200:
    case VolumetricTreeExperimentId::OakThickStump200Alpha10:
    case VolumetricTreeExperimentId::OakThickStump200Alpha20:
      return DynamicStrandsDemo::DemoType::OakThickStump200;
    case VolumetricTreeExperimentId::OakThickStump300:
    case VolumetricTreeExperimentId::OakThickStump300Alpha10:
    case VolumetricTreeExperimentId::OakThickStump300Alpha20:
      return DynamicStrandsDemo::DemoType::OakThickStump300;
    case VolumetricTreeExperimentId::OakThickStump400:
    case VolumetricTreeExperimentId::OakThickStump400Alpha10:
    case VolumetricTreeExperimentId::OakThickStump400Alpha20:
      return DynamicStrandsDemo::DemoType::OakThickStump400;
    case VolumetricTreeExperimentId::OakTwoYearSparse:
      return DynamicStrandsDemo::DemoType::OakTwoYearSparse;
    case VolumetricTreeExperimentId::OakTwoYear20:
      return DynamicStrandsDemo::DemoType::OakTwoYear20;
    case VolumetricTreeExperimentId::OakTwoYear50:
      return DynamicStrandsDemo::DemoType::OakTwoYear50;
    case VolumetricTreeExperimentId::OakTwoYear100:
      return DynamicStrandsDemo::DemoType::OakTwoYear100;
    case VolumetricTreeExperimentId::OakThreeYear4:
      return DynamicStrandsDemo::DemoType::OakThreeYear4;
    case VolumetricTreeExperimentId::OakThreeYear10:
      return DynamicStrandsDemo::DemoType::OakThreeYear10;
    case VolumetricTreeExperimentId::OakThreeYear20:
      return DynamicStrandsDemo::DemoType::OakThreeYear20;
    case VolumetricTreeExperimentId::OakThreeYear50:
      return DynamicStrandsDemo::DemoType::OakThreeYear50;
    case VolumetricTreeExperimentId::OakFourYearSparse:
      return DynamicStrandsDemo::DemoType::OakFourYearSparse;
    case VolumetricTreeExperimentId::OakSixYearSparse:
      return DynamicStrandsDemo::DemoType::OakSixYearSparse;
    case VolumetricTreeExperimentId::OakEightYearSparse:
      return DynamicStrandsDemo::DemoType::OakEightYearSparse;
    default:
      return DynamicStrandsDemo::DemoType::Empty;
  }
}

std::string MakeAutomatedExportTimestamp() {
  const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm local_tm{};
#if defined(_WIN32)
  localtime_s(&local_tm, &now);
#else
  localtime_r(&now, &local_tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&local_tm, "%Y-%m-%d_%H-%M-%S");
  return oss.str();
}

void ApplyLogExperimentPivotTransforms(const GlobalTransform& owner_gt, const GlobalTransform& root_transform,
                                       const float board_distance, const float simulated_time,
                                       GlobalTransform& left_transform, GlobalTransform& right_transform) {
  const float rotation_t = glm::clamp(simulated_time / kLogExperimentPivotDuration, 0.f, 1.f);
  const float angle = rotation_t * kLogExperimentMaxBreakAngle;

  left_transform.SetPosition(root_transform.TransformPoint(glm::vec3(0.f, 0.f, 0.f)));
  right_transform.SetPosition(root_transform.TransformPoint(glm::vec3(board_distance, 0.f, 0.f)));
  left_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0.f, 0.f, -angle)));
  right_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0.f, 0.f, angle)));
}

void ApplyLogCutUprightPivotTransforms(const GlobalTransform& owner_gt, const GlobalTransform& root_transform,
                                       const float board_distance, const float simulated_time,
                                       GlobalTransform& lower_transform, GlobalTransform& upper_transform) {
  const float rotation_t = glm::clamp(simulated_time / kLogExperimentPivotDuration, 0.f, 1.f);
  // Negative sign: with the "upright" owner rotation (+90° around world Z), this swings towards +X.
  const float angle = -rotation_t * kLogExperimentMaxBreakAngle;

  const glm::vec3 lower_pos = root_transform.TransformPoint(glm::vec3(0.f, 0.f, 0.f));
  lower_transform.SetPosition(lower_pos);
  lower_transform.SetRotation(owner_gt.GetRotation());

  const glm::quat upper_rotation = owner_gt.GetRotation() * glm::quat(glm::vec3(0.f, 0.f, angle));
  upper_transform.SetRotation(upper_rotation);
  const glm::vec3 upper_offset_world = upper_rotation * glm::vec3(board_distance, 0.f, 0.f);
  upper_transform.SetPosition(lower_pos + upper_offset_world);
}

void SetupLogCutUprightBunnyIntersectionBoundary(const std::shared_ptr<Scene>& scene, const Entity& owner,
                                                 const GlobalTransform& owner_gt, const float log_length) {
  if (!scene) {
    return;
  }

  const glm::vec3 lower_pivot_world = owner_gt.GetPosition();

  const auto bunny_path = ProjectManager::GetAssetsFolderPath() / "Models/bunny_simple.obj";
  kinDS::VoronoiMesh bunny_mesh = kinDS::ObjExporter::readMesh(bunny_path);
  const auto& verts = bunny_mesh.getVertices();
  if (verts.empty()) {
    EVOENGINE_ERROR("Upright log experiment: bunny OBJ has no vertices: " << bunny_path.string());
    return;
  }

  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double min_z = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  double max_z = std::numeric_limits<double>::lowest();
  for (const auto& v : verts) {
    const double x = static_cast<double>(v.x);
    const double y = static_cast<double>(v.y);
    const double z = static_cast<double>(v.z);
    if (x < min_x)
      min_x = x;
    if (y < min_y)
      min_y = y;
    if (z < min_z)
      min_z = z;
    if (x > max_x)
      max_x = x;
    if (y > max_y)
      max_y = y;
    if (z > max_z)
      max_z = z;
  }

  const double bunny_height = max_y - min_y;
  if (bunny_height <= std::numeric_limits<double>::epsilon()) {
    EVOENGINE_ERROR("Upright log experiment: bunny OBJ has degenerate Y span: " << bunny_path.string());
    return;
  }
  const double bunny_scale = static_cast<double>(log_length) / bunny_height;

  const double bunny_center_x = (min_x + max_x) * 0.5;
  const double bunny_center_z = (min_z + max_z) * 0.5;

  glm::mat4 bunny_transform(1.0f);
  bunny_transform[0][0] = static_cast<float>(bunny_scale);
  bunny_transform[1][1] = static_cast<float>(bunny_scale);
  bunny_transform[2][2] = static_cast<float>(bunny_scale);
  bunny_transform[3][0] = static_cast<float>(lower_pivot_world.x - bunny_scale * bunny_center_x);
  bunny_transform[3][1] = static_cast<float>(lower_pivot_world.y - bunny_scale * min_y);
  bunny_transform[3][2] = static_cast<float>(lower_pivot_world.z - bunny_scale * bunny_center_z);

  const Entity group = scene->CreateEntity("Intersection Meshes (Log cut upright + bunny)");
  scene->SetParent(group, owner);
  GlobalTransform group_gt{};
  group_gt.value = glm::mat4(1.0f);
  scene->SetDataComponent(group, group_gt);
  scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMeshGroup>(group);

  const auto child = scene->CreateEntity("Intersection Mesh (bunny_simple)");
  scene->SetParent(child, group);
  GlobalTransform child_gt{};
  child_gt.value = bunny_transform;
  scene->SetDataComponent(child, child_gt);

  auto ibm = scene->GetOrSetPrivateComponent<DsIntersectionBoundaryMesh>(child).lock();
  if (!ibm) {
    EVOENGINE_ERROR("Upright log experiment: failed to create DsIntersectionBoundaryMesh for bunny.");
    return;
  }
  ibm->LoadMesh(std::move(bunny_mesh), bunny_path);
}

void ApplyLogBreakPivotTransforms(const GlobalTransform& owner_gt, const GlobalTransform& root_transform,
                                  const float board_distance, const float progress, const float target_factor0,
                                  const float target_factor1, const float separation_scale,
                                  GlobalTransform& left_transform, GlobalTransform& right_transform) {
  const float left_x = board_distance * 0.5f * progress * target_factor0 * separation_scale;
  const float right_x = board_distance * (1.f - 0.5f * progress * target_factor0 * separation_scale);
  const float angle = glm::acos(glm::clamp(1.f - progress * target_factor1, -1.f, 1.f));

  left_transform.SetPosition(root_transform.TransformPoint(glm::vec3(left_x, 0.f, 0.f)));
  right_transform.SetPosition(root_transform.TransformPoint(glm::vec3(right_x, 0.f, 0.f)));
  left_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0.f, 0.f, -angle)));
  right_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0.f, 0.f, angle)));
}

/// Fewer strands + weaker center attraction → less circular packing; thicker base strand radii.
// Strand / physics presets for volumetric demos live in VolumetricTreeExperiment.cpp.

constexpr float kDefaultPhysicsDemoHeight = 0.0f;
constexpr float kVolumetricLogExperimentHeight = 0.25f;
constexpr float kVolumetricLogExperimentCameraHeight = 0.55f;

void SetupVolumetricLogExperimentHeight(const std::shared_ptr<Scene>& scene, const Entity& owner,
                                        const std::shared_ptr<DynamicTreeStrands>& dts) {
  GlobalTransform owner_gt = scene->GetDataComponent<GlobalTransform>(owner);
  const glm::vec3 pos = owner_gt.GetPosition();
  owner_gt.SetPosition(glm::vec3(pos.x, kVolumetricLogExperimentHeight, pos.z));
  scene->SetDataComponent(owner, owner_gt);
  dts->initialize_parameters.root_transform = owner_gt;
}

void ResetPhysicsDemoHeight(const std::shared_ptr<Scene>& scene, const Entity& owner,
                            const std::shared_ptr<DynamicTreeStrands>& dts) {
  GlobalTransform owner_gt = scene->GetDataComponent<GlobalTransform>(owner);
  const glm::vec3 pos = owner_gt.GetPosition();
  owner_gt.SetPosition(glm::vec3(pos.x, kDefaultPhysicsDemoHeight, pos.z));
  scene->SetDataComponent(owner, owner_gt);
  dts->initialize_parameters.root_transform = owner_gt;
}

}  // namespace

void DynamicStrandsDemo::BeginTreeAutoGrow() {
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  if (!eco_sys_lab_layer) {
    EVOENGINE_ERROR("Tree growth demo: EcoSysLabLayer is missing.");
    demo_type = DemoType::Empty;
    demo_status = DemoStatus::Idle;
    tree_auto_grow_started_ = false;
    return;
  }
  tree_auto_grow_started_ = true;
  // Continue meshing after EcoSysLab finishes async auto-grow (layer Update always runs;
  // private-component Update only runs while the application is Playing).
  eco_sys_lab_layer->SetOnAutoGrowFinished([this]() {
    TryFinishTreeGrowthAndStartMeshing();
  });
  if (target_growth_iterations > 0) {
    eco_sys_lab_layer->StartAutoGrowIterations(target_growth_iterations);
    EVOENGINE_LOG("Demo: waiting for EcoSysLab tree auto-grow (" << target_growth_iterations << " iterations)...");
  } else {
    eco_sys_lab_layer->StartAutoGrow(target_growth_time);
    EVOENGINE_LOG("Demo: waiting for EcoSysLab tree auto-grow (" << target_growth_time << " years)...");
  }
}

void DynamicStrandsDemo::StartVolumetricTreeExperiment(const VolumetricTreeExperiment& experiment) {
  pending_precomputed_cli_id_.clear();
  pending_precomputed_strand_tree_.clear();
  pending_precomputed_mesh_bin_.clear();
  pending_precomputed_hash_.clear();
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(owner).lock();
  dts->prescribed_subdivisions_by_strand.clear();
  ResetEnvironment();
  demo_type = DemoTypeForExperiment(experiment.id);
  demo_status = DemoStatus::TreeGrowth;
  pending_volumetric_experiment_id_ = experiment.id;
  pending_meshing_buffer_description = experiment.meshing_buffer_description;
  DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance = 0.f;
  const auto tree_entity = scene->CreateEntity("Tree");
  tree_entity_ref = tree_entity;
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
  scene->SetDataComponent(owner, tree_initial_pose);
  scene->SetDataComponent(tree_entity, tree_initial_pose);
  target_growth_time = experiment.growth_years;
  target_growth_iterations = experiment.growth_iterations;
  tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset(experiment.tree_descriptor_path);
  tree->shoot_model.seed = experiment.seed;
  tree->shoot_strand_model.seed = experiment.seed;
  dts->seed = experiment.seed;
  dts->fixed_subdivision_seed = true;
  ApplyVolumetricTreeStrandPreset(experiment, tree);
  ApplyVolumetricTreePhysicsPreset(physics_parameters);
  dts->enable_physics = false;
  dts->initialize_parameters.min_segment_length = experiment.min_segment_length;
  dts->initialize_parameters.max_segment_length = experiment.max_segment_length;
  if (experiment.growth_iterations > 0) {
    EVOENGINE_LOG(experiment.display_name << ": growing for " << experiment.growth_iterations << " iterations...");
  } else {
    EVOENGINE_LOG(experiment.display_name << ": growing for " << experiment.growth_years << " years...");
  }
  ApplySceneCamera(camera_pose.GetPosition(), camera_pose.GetRotation(), false, 0.f);
  BeginTreeAutoGrow();
}

void DynamicStrandsDemo::StartPrecomputedExperiment(const PrecomputedExperimentAssets& assets) {
  const VolumetricTreeExperiment* experiment = FindVolumetricTreeExperiment(assets.cli_id.c_str());
  if (!experiment) {
    EVOENGINE_ERROR("Precomputed experiment: unknown cli_id " << assets.cli_id);
    return;
  }
  if (!std::filesystem::exists(assets.strand_tree_path) || !std::filesystem::exists(assets.mesh_buffer_bin_path)) {
    EVOENGINE_ERROR("Precomputed experiment assets missing for " << assets.cli_id);
    return;
  }
  pending_precomputed_cli_id_ = assets.cli_id;
  pending_precomputed_strand_tree_ = assets.strand_tree_path;
  pending_precomputed_mesh_bin_ = assets.mesh_buffer_bin_path;
  pending_precomputed_hash_ = assets.input_hash;
  EVOENGINE_LOG("Precomputed Experiments: loading " << assets.display_name << " (hash " << assets.input_hash
                                                    << ") from " << assets.assets_dir.string());
  // StartVolumetricTreeExperiment clears pending_* — restore after.
  StartVolumetricTreeExperiment(*experiment);
  pending_precomputed_cli_id_ = assets.cli_id;
  pending_precomputed_strand_tree_ = assets.strand_tree_path;
  pending_precomputed_mesh_bin_ = assets.mesh_buffer_bin_path;
  pending_precomputed_hash_ = assets.input_hash;

  // Kinetic Voronoi from MeshBuffers cache; Alpha Shape / tets generated fresh.
  const auto dts = GetScene()->GetOrSetPrivateComponent<DynamicTreeStrands>(GetOwner()).lock();
  dts->initialize_parameters.meshing_type = MeshingType::Both;
}

void DynamicStrandsDemo::DrawPrecomputedExperimentsUi() {
  if (!ImGui::TreeNodeEx("Precomputed Experiments", ImGuiTreeNodeFlags_DefaultOpen)) {
    return;
  }
  const auto assets = SyncAndListPrecomputedExperimentAssets();
  if (assets.empty()) {
    ImGui::TextWrapped(
        "No succeeded precomputes found. Need Metadata/precompute_jobs/<id>/*.strandtree and a matching "
        "MeshBuffers/<hash>.bin (copied into Assets/PrecomputedExperiments on open).");
    ImGui::TreePop();
    return;
  }
  ImGui::TextWrapped(
      "Grow the experiment tree, restore StrandTree subdivisions onto physics rods, force-load Kinetic "
      "Voronoi MeshBuffers (hash still compared), and generate the Alpha Shape tetrahedral mesh (meshing = Both).");
  for (const auto& entry : assets) {
    const std::string label = entry.display_name + "##precomputed_" + entry.cli_id;
    if (ImGui::Button(label.c_str())) {
      StartPrecomputedExperiment(entry);
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("hash %s\n%s", entry.input_hash.c_str(), entry.assets_dir.string().c_str());
    }
  }
  ImGui::TreePop();
}

void DynamicStrandsDemo::TryFinishTreeGrowthAndStartMeshing() {
  if (demo_status != DemoStatus::TreeGrowth) {
    return;
  }
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  if (!eco_sys_lab_layer) {
    EVOENGINE_ERROR("Tree growth demo: EcoSysLabLayer is missing.");
    demo_type = DemoType::Empty;
    demo_status = DemoStatus::Idle;
    tree_auto_grow_started_ = false;
    return;
  }

  // Still growing asynchronously in EcoSysLabLayer::Update — do not block the frame.
  if (eco_sys_lab_layer->IsAutoGrowing()) {
    return;
  }
  if (!tree_auto_grow_started_) {
    BeginTreeAutoGrow();
    return;
  }

  if (target_growth_iterations > 0) {
    // Iteration-based demos finish when StartAutoGrowIterations completes; year check does not apply.
  } else {
    const float target_days = target_growth_time * 365.f;
    if (eco_sys_lab_layer->GetSimulatedTime() + 1e-3f < target_days) {
      EVOENGINE_ERROR("Tree auto-grow ended before reaching the target age ("
                      << (eco_sys_lab_layer->GetSimulatedTime() / 365.f) << " / " << target_growth_time << " years).");
      demo_type = DemoType::Empty;
      demo_status = DemoStatus::Idle;
      tree_auto_grow_started_ = false;
      return;
    }
  }

  const auto scene = GetScene();
  const auto tree_entity = tree_entity_ref.Get();
  if (!scene || !scene->IsEntityValid(tree_entity)) {
    EVOENGINE_ERROR("Tree growth demo: tree entity is missing after auto-grow.");
    demo_type = DemoType::Empty;
    demo_status = DemoStatus::Idle;
    tree_auto_grow_started_ = false;
    return;
  }

  const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
  // Tree-descriptor demos mesh into PhysicsDemo's DynamicTreeStrands (not a DTS on the Tree entity).
  const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(GetOwner()).lock();
  const VolumetricTreeExperiment* experiment = FindVolumetricTreeExperiment(pending_volumetric_experiment_id_);
  if (!experiment) {
    for (const auto& candidate : GetVolumetricTreeExperiments()) {
      if (DemoTypeForExperiment(candidate.id) == demo_type) {
        experiment = &candidate;
        break;
      }
    }
  }
  pending_volumetric_experiment_id_ = VolumetricTreeExperimentId::Count;
  if (experiment) {
    ApplyVolumetricTreeStrandPreset(*experiment, tree);
    ApplyVolumetricTreePhysicsPreset(physics_parameters);
    dts->initialize_parameters.min_segment_length = experiment->min_segment_length;
    dts->initialize_parameters.max_segment_length = experiment->max_segment_length;
    dts->seed = experiment->seed;
    dts->fixed_subdivision_seed = true;
    if (experiment->growth_iterations > 0) {
      EVOENGINE_LOG(experiment->display_name << ": tree growth finished (" << experiment->growth_iterations
                                             << " iterations, " << experiment->end_node_strands
                                             << " end strands). Building strands and meshing...");
    } else {
      EVOENGINE_LOG(experiment->display_name << ": tree growth finished (" << experiment->growth_years
                                             << " years). Building strands and meshing...");
    }
  }
  ApplySegmentSubdivisionOverride(dts->initialize_parameters);
  if (precompute_growing_job_index_ != static_cast<size_t>(-1)) {
    FinishPrecomputeGrowthAndSpawnWorker();
    return;
  }
  if (!pending_precomputed_cli_id_.empty()) {
    // Kinetic Voronoi from cached MeshBuffers; Alpha Shape tetrahedral mesh generated fresh.
    dts->initialize_parameters.meshing_type = MeshingType::Both;
    try {
      const kinDS::StrandTree strand_tree = kinDS::StrandTree::loadFromFile(pending_precomputed_strand_tree_);
      dts->prescribed_subdivisions_by_strand = strand_tree.getSubdivisionsByStrand();
      EVOENGINE_LOG("Precomputed " << pending_precomputed_cli_id_ << ": applied "
                                   << dts->prescribed_subdivisions_by_strand.size() << " strand subdivision lists from "
                                   << pending_precomputed_strand_tree_.string());
    } catch (const std::exception& ex) {
      EVOENGINE_ERROR("Precomputed " << pending_precomputed_cli_id_ << ": failed to load StrandTree (" << ex.what()
                                     << "); falling back to seeded random subdivision.");
      dts->prescribed_subdivisions_by_strand.clear();
    }
    DsKineticVoronoiMeshing::meshing_settings.force_load_meshing_buffer_bin = pending_precomputed_mesh_bin_;
    DsKineticVoronoiMeshing::meshing_settings.force_load_expected_hash = pending_precomputed_hash_;
    DsKineticVoronoiMeshing::meshing_settings.override_meshing_buffer = false;
    EVOENGINE_LOG("Precomputed " << pending_precomputed_cli_id_
                                 << ": meshing=Both (force-load Kinetic Voronoi buffer, generate Alpha Shape tets).");
    pending_precomputed_cli_id_.clear();
    pending_precomputed_strand_tree_.clear();
    pending_precomputed_mesh_bin_.clear();
    pending_precomputed_hash_.clear();
  } else {
    dts->prescribed_subdivisions_by_strand.clear();
  }
  if (!alpha_sweep_.empty()) {
    if (alpha_hull_fractal_pending_) {
      RunAlphaHullFractalExperiment(tree, dts);
    } else {
      RunAlphaSweepMeshing(tree, dts);
    }
  } else {
    dts->InitializeFromTree(tree, pending_meshing_buffer_description);
    pending_meshing_buffer_description.clear();
  }
  dts->prescribed_subdivisions_by_strand.clear();
  tree_auto_grow_started_ = false;
  demo_status = DemoStatus::Simulation;
  simulated_time = 0.f;
  ResetAutomatedExportSchedule();
}

void DynamicStrandsDemo::RunAlphaSweepMeshing(const std::shared_ptr<Tree>& tree,
                                              const std::shared_ptr<DynamicTreeStrands>& dts) {
  if (!tree || !dts || alpha_sweep_.empty()) {
    alpha_sweep_.clear();
    alpha_sweep_experiment_name_.clear();
    return;
  }

  std::string experiment_tag = alpha_sweep_experiment_name_.empty() ? "Alpha_sweep" : alpha_sweep_experiment_name_;
  std::replace(experiment_tag.begin(), experiment_tag.end(), ' ', '_');
  for (char& c : experiment_tag) {
    if (c == '+' || c == '/' || c == '\\' || c == ':') {
      c = '_';
    }
  }
  std::string experiment_folder_name = experiment_tag;
  std::replace(experiment_folder_name.begin(), experiment_folder_name.end(), '_', ' ');
  experiment_folder_name += " alpha sweep";
  const std::string log_prefix = experiment_folder_name;

  const bool previous_override = DsKineticVoronoiMeshing::meshing_settings.override_meshing_buffer;
  const double previous_alpha_cutoff = DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff;
  const double previous_branch_alpha_cutoff = DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff;
  // Force remesh for each alpha so deferred statistics CSVs are written (cache hits skip collection).
  DsKineticVoronoiMeshing::meshing_settings.override_meshing_buffer = true;

  const std::string base_description =
      pending_meshing_buffer_description.empty()
          ? ("created from DynamicStrandsDemo scripted experiment: " + experiment_folder_name)
          : pending_meshing_buffer_description;
  pending_meshing_buffer_description.clear();

  const auto project_path = ProjectManager::GetProjectPath();
  std::filesystem::path export_folder;
  if (!project_path.empty()) {
    export_folder =
        project_path.parent_path() / "PhysicsDemoExports" / experiment_folder_name / MakeAutomatedExportTimestamp();
    std::error_code ec;
    std::filesystem::create_directories(export_folder, ec);
    if (ec) {
      EVOENGINE_ERROR(log_prefix << ": failed to create export folder " << export_folder.string() << " ("
                                 << ec.message() << ").");
      export_folder.clear();
    } else {
      EVOENGINE_LOG(log_prefix << ": export folder " << export_folder.string());
    }
  } else {
    EVOENGINE_ERROR(log_prefix << ": project path is empty; mesh OBJ export skipped.");
  }

  struct SweepTotalsRow {
    double alpha = 0.0;
    double cutoff = 0.0;
    bool succeeded = false;
    size_t section_count = 0;
    double runtime_s = 0.0;
    std::optional<size_t> strand_count{};
    std::optional<size_t> branch_count{};
    std::array<size_t, kinDS::kineticEventTypeCount> event_counts{};
    std::optional<double> alpha_recorded{};
    std::optional<size_t> triangle_count{};
    std::optional<size_t> vertex_count{};
    std::string failure;
  };
  std::vector<SweepTotalsRow> summary_rows;
  summary_rows.reserve(alpha_sweep_.size());

  const std::filesystem::path partial_summary_path =
      EcoSysLabMetadataPath("meshing_statistics_" + experiment_tag + "_alpha_sweep_summary_partial.csv");
  const std::filesystem::path export_summary_path =
      export_folder.empty() ? std::filesystem::path{} : (export_folder / "meshing_statistics_summary.csv");

  auto write_summary_header = [](std::ostream& out) {
    out << "alpha,cutoff,succeeded,section_count,runtime_s,strand_count,branch_count,segment_count";
    for (size_t e = 0; e < kinDS::kineticEventTypeCount; ++e) {
      out << ',' << kinDS::kineticEventTypeName(static_cast<kinDS::KineticEventType>(e));
    }
    out << ",alpha_recorded,triangle_count,vertex_count,failure\n";
  };

  auto write_summary_row = [](std::ostream& out, const SweepTotalsRow& row) {
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    auto write_optional_size = [&](const std::optional<size_t>& value) {
      if (value.has_value()) {
        out << value.value();
      }
    };
    auto write_optional_double = [&](const std::optional<double>& value) {
      if (value.has_value()) {
        out << value.value();
      }
    };
    auto write_csv_string = [&](const std::string& value) {
      out << '"';
      for (const char c : value) {
        if (c == '"') {
          out << "\"\"";
        } else {
          out << c;
        }
      }
      out << '"';
    };
    out << row.alpha << ',' << row.cutoff << ',' << (row.succeeded ? 1 : 0) << ',' << row.section_count << ','
        << row.runtime_s << ',';
    write_optional_size(row.strand_count);
    out << ',';
    write_optional_size(row.branch_count);
    out << ',';
    if (row.strand_count.has_value()) {
      const size_t subdivision = row.event_counts[static_cast<size_t>(kinDS::KineticEventType::Subdivision)];
      out << (row.strand_count.value() + subdivision);
    }
    for (size_t e = 0; e < kinDS::kineticEventTypeCount; ++e) {
      out << ',' << row.event_counts[e];
    }
    out << ',';
    write_optional_double(row.alpha_recorded);
    out << ',';
    write_optional_size(row.triangle_count);
    out << ',';
    write_optional_size(row.vertex_count);
    out << ',';
    if (!row.failure.empty()) {
      write_csv_string(row.failure);
    }
    out << '\n';
  };

  auto write_full_summary_csv = [&](const std::filesystem::path& path) {
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out) {
      EVOENGINE_ERROR(log_prefix << ": failed to write summary CSV " << path.string());
      return;
    }
    write_summary_header(out);
    for (const SweepTotalsRow& row : summary_rows) {
      write_summary_row(out, row);
    }
    out.flush();
    EVOENGINE_LOG(log_prefix << ": wrote summary CSV " << path.string() << " (" << summary_rows.size()
                             << " alpha row(s)).");
  };

  // Truncate and write header up front; append + flush after each alpha so a crash keeps completed rows.
  std::ofstream partial_summary_out(partial_summary_path, std::ios::out | std::ios::trunc);
  std::ofstream export_summary_out;
  if (!partial_summary_out) {
    EVOENGINE_ERROR(log_prefix << ": failed to open incremental summary CSV " << partial_summary_path.string());
  } else {
    write_summary_header(partial_summary_out);
    partial_summary_out.flush();
    EVOENGINE_LOG(log_prefix << ": incremental summary CSV " << partial_summary_path.string());
  }
  if (!export_summary_path.empty()) {
    export_summary_out.open(export_summary_path, std::ios::out | std::ios::trunc);
    if (!export_summary_out) {
      EVOENGINE_ERROR(log_prefix << ": failed to open incremental summary CSV " << export_summary_path.string());
    } else {
      write_summary_header(export_summary_out);
      export_summary_out.flush();
    }
  }

  auto flush_summary_row = [&](const SweepTotalsRow& row) {
    if (partial_summary_out) {
      write_summary_row(partial_summary_out, row);
      partial_summary_out.flush();
    }
    if (export_summary_out) {
      write_summary_row(export_summary_out, row);
      export_summary_out.flush();
    }
  };

  auto append_failure_summary = [&](const double alpha, const double cutoff, const std::string& failure) {
    SweepTotalsRow row;
    row.alpha = alpha;
    row.cutoff = cutoff;
    row.succeeded = false;
    row.alpha_recorded = alpha;
    row.failure = failure;
    flush_summary_row(row);
    summary_rows.push_back(std::move(row));
  };

  auto append_success_summary = [&](const double alpha, const double cutoff, DsKineticVoronoiMeshing& dskvm) {
    SweepTotalsRow row;
    row.alpha = alpha;
    row.cutoff = cutoff;
    row.succeeded = true;
    row.alpha_recorded = alpha;
    if (dskvm.tree_mesher_) {
      if (const kinDS::Statistics* stats = dskvm.tree_mesher_->getMeshingStatistics()) {
        const auto& totals = stats->totals();
        row.section_count = stats->sections().size();
        row.runtime_s = totals.runtime_seconds;
        row.strand_count = totals.strand_count;
        row.branch_count = totals.branch_count;
        row.event_counts = totals.event_counts;
        row.alpha_recorded = stats->totalsAlpha().value_or(alpha);
        row.triangle_count = stats->totalsTriangleCount();
        row.vertex_count = stats->totalsVertexCount();
        if (stats->totalsFailure().has_value()) {
          row.failure = stats->totalsFailure().value();
          row.succeeded = false;
        }
      }
    }
    if (!row.triangle_count.has_value()) {
      row.triangle_count = dskvm.segment_meshlet_triangles.size();
      row.vertex_count = dskvm.segment_meshlet_vertices.size();
    }
    flush_summary_row(row);
    summary_rows.push_back(std::move(row));
  };

  for (size_t i = 0; i < alpha_sweep_.size(); ++i) {
    const double alpha = alpha_sweep_[i];
    const double cutoff = std::sqrt(alpha);
    const int alpha_i = static_cast<int>(std::lround(alpha));
    DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff = cutoff;
    DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff = cutoff;
    DsKineticVoronoiMeshing::meshing_settings.meshing_statistics_experiment_name =
        experiment_tag + "_alpha_" + std::to_string(alpha_i);

    const std::string description =
        base_description + " (alpha=" + std::to_string(alpha_i) + ", cutoff=" + std::to_string(cutoff) + ")";
    EVOENGINE_LOG(log_prefix << ": meshing " << (i + 1) << "/" << alpha_sweep_.size() << " (alpha=" << alpha
                             << ", cutoff=" << cutoff << ")...");

    try {
      dts->InitializeFromTree(tree, description);
    } catch (const std::exception& ex) {
      EVOENGINE_ERROR(log_prefix << ": InitializeFromTree threw for alpha=" << alpha << ": " << ex.what()
                                 << "; continuing.");
      if (auto* dskvm = dts->dynamic_strands ? dts->dynamic_strands->GetKineticVoronoiMeshing() : nullptr) {
        if (!DsKineticVoronoiMeshing::meshing_settings.meshing_statistics_experiment_name.empty()) {
          dskvm->WriteMeshingFailureStatistics(ex.what());
        }
      }
      append_failure_summary(alpha, cutoff, ex.what());
      continue;
    }

    auto* dskvm = dts->dynamic_strands ? dts->dynamic_strands->GetKineticVoronoiMeshing() : nullptr;
    if (!dskvm || !dskvm->last_meshing_succeeded_) {
      EVOENGINE_ERROR(log_prefix << ": meshing failed for alpha=" << alpha << "; continuing with next alpha.");
      if (dskvm && !DsKineticVoronoiMeshing::meshing_settings.meshing_statistics_experiment_name.empty()) {
        dskvm->WriteMeshingFailureStatistics("meshing failed");
      }
      append_failure_summary(alpha, cutoff, "meshing failed");
      continue;
    }

    append_success_summary(alpha, cutoff, *dskvm);

    if (dskvm->segment_meshlet_vertices.empty() || dskvm->segment_meshlet_triangles.empty()) {
      EVOENGINE_ERROR(log_prefix << ": meshlet buffers empty after success for alpha=" << alpha
                                 << "; skipping OBJ export.");
      continue;
    }
    if (export_folder.empty()) {
      continue;
    }

    dts->dynamic_strands->Download();
    const auto path = export_folder / ("meshlets_alpha_" + std::to_string(alpha_i) + ".obj");
    try {
      MeshletObjExport::ExportObj(
          path, dskvm->segment_meshlet_vertices, dskvm->segment_meshlet_triangles, dts->dynamic_strands->segments,
          DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.uv_height_factor,
          DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.uv_circum_factor,
          DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance,
          dts->dynamic_strands->segment_pairs, dts->dynamic_strands->segment_data_list,
          dskvm->segment_meshlet_vertex_metadata, dskvm->segment_meshlet_face_metadata);
      EVOENGINE_LOG(log_prefix << ": wrote " << path.string());
    } catch (const std::exception& ex) {
      EVOENGINE_ERROR(log_prefix << ": OBJ export failed for alpha=" << alpha << ": " << ex.what());
    }
  }

  if (partial_summary_out) {
    partial_summary_out.flush();
    partial_summary_out.close();
  }
  if (export_summary_out) {
    export_summary_out.flush();
    export_summary_out.close();
  }

  if (!summary_rows.empty()) {
    const std::string stamp = MakeAutomatedExportTimestamp();
    write_full_summary_csv(
        EcoSysLabMetadataPath("meshing_statistics_" + experiment_tag + "_alpha_sweep_summary_" + stamp + ".csv"));
    if (!export_summary_path.empty()) {
      // Re-write export folder copy once more so it matches the final stamped CSV.
      write_full_summary_csv(export_summary_path);
    }
  }

  DsKineticVoronoiMeshing::meshing_settings.override_meshing_buffer = previous_override;
  DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff = previous_alpha_cutoff;
  DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff = previous_branch_alpha_cutoff;
  DsKineticVoronoiMeshing::meshing_settings.meshing_statistics_experiment_name.clear();
  alpha_sweep_.clear();
  alpha_sweep_experiment_name_.clear();
  EVOENGINE_LOG(log_prefix << ": finished all alpha values.");
}

void DynamicStrandsDemo::RunAlphaHullFractalExperiment(const std::shared_ptr<Tree>& tree,
                                                       const std::shared_ptr<DynamicTreeStrands>& dts) {
  const bool pending = alpha_hull_fractal_pending_;
  alpha_hull_fractal_pending_ = false;
  if (!tree || !dts || !pending || alpha_sweep_.empty()) {
    alpha_sweep_.clear();
    alpha_sweep_experiment_name_.clear();
    return;
  }

  std::string experiment_tag =
      alpha_sweep_experiment_name_.empty() ? "Alpha_hull_fractal" : alpha_sweep_experiment_name_;
  std::replace(experiment_tag.begin(), experiment_tag.end(), ' ', '_');
  for (char& c : experiment_tag) {
    if (c == '+' || c == '/' || c == '\\' || c == ':') {
      c = '_';
    }
  }
  std::string experiment_folder_name = experiment_tag;
  std::replace(experiment_folder_name.begin(), experiment_folder_name.end(), '_', ' ');
  experiment_folder_name += " alpha hull fractal";
  const std::string log_prefix = experiment_folder_name;

  const auto alphas = alpha_sweep_;
  alpha_sweep_.clear();
  alpha_sweep_experiment_name_.clear();

  const bool previous_dry_run = DsKineticVoronoiMeshing::meshing_settings.dry_run_strand_tree_only;
  DsKineticVoronoiMeshing::meshing_settings.dry_run_strand_tree_only = true;
  DsKineticVoronoiMeshing::meshing_settings.override_meshing_buffer = false;
  DsKineticVoronoiMeshing::meshing_settings.meshing_statistics_experiment_name = experiment_tag + "_alpha_hull_fractal";

  const std::string description =
      pending_meshing_buffer_description.empty()
          ? ("created from DynamicStrandsDemo scripted experiment: " + experiment_folder_name)
          : pending_meshing_buffer_description;
  pending_meshing_buffer_description.clear();

  EVOENGINE_LOG(log_prefix << ": building StrandTree (dry-run, no meshing)...");
  dts->InitializeFromTree(tree, description);
  DsKineticVoronoiMeshing::meshing_settings.dry_run_strand_tree_only = previous_dry_run;
  DsKineticVoronoiMeshing::meshing_settings.meshing_statistics_experiment_name.clear();

  auto* kvm = dts->dynamic_strands ? dts->dynamic_strands->GetKineticVoronoiMeshing() : nullptr;
  if (!kvm || !kvm->strand_tree) {
    EVOENGINE_ERROR(log_prefix << ": StrandTree missing after dry-run.");
    return;
  }
  const kinDS::StrandTree& strand_tree = *kvm->strand_tree;
  // getHeight() is the max inclusive section index (pts.size() - 1), so 0..25 means height == 25.
  const size_t tree_height_max = strand_tree.getHeight();
  if (strand_tree.getPoints().empty()) {
    EVOENGINE_ERROR(log_prefix << ": StrandTree has no sections.");
    return;
  }

  const size_t height_min = alpha_hull_height_min_;
  const size_t height_max_requested = alpha_hull_height_max_;
  if (height_min > height_max_requested) {
    EVOENGINE_ERROR(log_prefix << ": invalid height range [" << height_min << ", " << height_max_requested << "].");
    return;
  }
  const size_t height_max = std::min(height_max_requested, tree_height_max);
  if (height_max < height_min) {
    EVOENGINE_ERROR(log_prefix << ": StrandTree max height " << tree_height_max
                               << " has no sections in requested range [" << height_min << ", " << height_max_requested
                               << "].");
    return;
  }
  if (height_max < height_max_requested) {
    EVOENGINE_WARNING(log_prefix << ": StrandTree max height is " << tree_height_max << "; analyzing sections "
                                 << height_min << ".." << height_max << " (requested up to " << height_max_requested
                                 << ").");
  }

  const auto project_path = ProjectManager::GetProjectPath();
  std::filesystem::path export_folder;
  if (!project_path.empty()) {
    export_folder =
        project_path.parent_path() / "PhysicsDemoExports" / experiment_folder_name / MakeAutomatedExportTimestamp();
    std::error_code ec;
    std::filesystem::create_directories(export_folder, ec);
  }

  size_t analyzed = 0;
  size_t skipped_heights = 0;
  for (size_t height = height_min; height <= height_max; ++height) {
    const size_t branch_count = strand_tree.getBranchCount(height);
    size_t analyzed_at_height = 0;
    for (size_t branch_id = 0; branch_id < branch_count; ++branch_id) {
      const auto& strand_ids = strand_tree.getStrandsByBranch(height, branch_id);
      std::vector<glm::dvec2> points;
      points.reserve(strand_ids.size());
      for (const size_t sid : strand_ids) {
        points.push_back(strand_tree.getPointTransformed(sid, height, branch_id));
      }
      if (points.size() < 3) {
        EVOENGINE_LOG(log_prefix << ": branch " << branch_id << " at height " << height << " has only " << points.size()
                                 << " points; skipping.");
        continue;
      }

      AlphaHullFractalReport report;
      const std::filesystem::path section_dir =
          export_folder.empty()
              ? std::filesystem::path{}
              : (export_folder / ("h" + std::to_string(height)) / ("branch_" + std::to_string(branch_id)));
      if (!AnalyzeAlphaHullFractalDimensions(points, alphas, height, branch_id, report, section_dir)) {
        EVOENGINE_ERROR(log_prefix << ": analysis failed for height " << height << " branch " << branch_id);
        continue;
      }
      ++analyzed;
      ++analyzed_at_height;

      std::filesystem::path csv_path;
      if (!export_folder.empty()) {
        csv_path = export_folder /
                   ("alpha_hull_fractal_h" + std::to_string(height) + "_b" + std::to_string(branch_id) + ".csv");
      } else {
        csv_path = EcoSysLabMetadataPath("alpha_hull_fractal_" + experiment_tag + "_h" + std::to_string(height) + "_b" +
                                         std::to_string(branch_id) + ".csv");
      }
      if (WriteAlphaHullFractalCsv(report, csv_path)) {
        EVOENGINE_LOG(log_prefix << ": wrote " << csv_path.string() << " (points=" << report.point_count
                                 << ", triangles=" << report.delaunay_triangle_count
                                 << ", hull_circ=" << report.convex_hull.circumference
                                 << ", hull_corners=" << report.convex_hull.corner_count
                                 << ", hull_D=" << report.convex_hull.fractal_dimension << ").");
      } else {
        EVOENGINE_ERROR(log_prefix << ": failed to write CSV " << csv_path.string());
      }

      for (const auto& row : report.alpha_rows) {
        EVOENGINE_LOG(log_prefix << ": h=" << height << " branch=" << branch_id << " alpha=" << row.alpha
                                 << " kept_tris=" << row.kept_triangle_count << " circ=" << row.circumference
                                 << " circ/hull=" << row.circumference_to_hull_ratio << " corners=" << row.corner_count
                                 << " corners/hull=" << row.corner_count_to_hull_ratio << " D=" << row.fractal_dimension
                                 << " R2=" << row.fractal_fit_r2 << (row.ok ? "" : (" [" + row.note + "]")));
      }
    }
    if (analyzed_at_height == 0) {
      ++skipped_heights;
    }
  }

  if (analyzed == 0) {
    EVOENGINE_ERROR(log_prefix << ": no branches analyzed for sections " << height_min << ".." << height_max << ".");
  } else {
    EVOENGINE_LOG(log_prefix << ": finished (" << analyzed << " section/branch analyses over heights " << height_min
                             << ".." << height_max << ", " << skipped_heights << " empty height(s)).");
  }
}

void DynamicStrandsDemo::ResetEnvironment() {
  const auto owner = GetOwner();
  const auto scene = GetScene();
  const auto children = scene->GetChildren(owner);

  // Preserve meshing mode across DTS recreate (e.g. Both) so PhysicsDemo experiments do not reset it.
  MeshingType preserved_meshing_type = MeshingType::Both;
  if (scene->HasPrivateComponent<DynamicTreeStrands>(owner)) {
    if (const auto existing_dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(owner).lock()) {
      preserved_meshing_type = existing_dts->initialize_parameters.meshing_type;
    }
    scene->RemovePrivateComponent<DynamicTreeStrands>(owner);
  }
  const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(owner).lock();
  dts->initialize_parameters.meshing_type = preserved_meshing_type;
  ResetPhysicsDemoHeight(scene, owner, dts);

  target_simulation_time = 100.f;
  simulated_time = 0.f;
  // target_factor0 = 1.f;
  target_factor1 = 1.f;
  target_growth_time = 0.f;
  target_growth_iterations = 0;
  automated_export_upper = 25.f;
  automated_export_stepsize = 5.f;
  ResetAutomatedExportSchedule();
  // Restore process-global meshing defaults (experiments may override cutoffs / alpha).
  DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff = 10.0;
  DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff = 10.0;
  dts->initialize_parameters.alpha = 0.00005f;
  dts->initialize_parameters.bifurcation_alpha = 0.00005f;
  DsAlphaShapeMeshing::render_settings.branches_render_parameters.alpha = 0.00005f;
  DsAlphaShapeMeshing::render_settings.branches_render_parameters.bifurcation_alpha = 0.00005f;
  DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance = 0.0004f;
  DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.uv_height_factor = 0.025f;
  physics_parameters = {};
  physics_parameters.time_step = 0.005f;
  physics_parameters.enable_segment_collision = false;

  board_experiment_setup_settings.center_damage = 0.f;
  board_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  board_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
  log_experiment_setup_settings.lock_upper = false;
  log_experiment_setup_settings.t_cut = false;
  log_experiment_setup_settings.t_cut_width = 0.7f;
  board_experiment_setup_settings.rod_dimension = {20, 40, 20};

  dts->initialize_parameters.strength_graph.SetShearStretchStrength({500.f, 250.f});
  dts->initialize_parameters.strength_graph.SetBendingStrength({500.f, 250.f});
  dts->initialize_parameters.strength_graph.SetTwistingStrength({500.f, 250.f});
  dts->initialize_parameters.strength_graph.SetBundleStrength({500.f, 250.f});
  dts->initialize_parameters.strength_graph.SetConnectivityStrength({250.f, 125.f});

  dts->initialize_parameters.max_segment_length = 0.06f;
  dts->initialize_parameters.min_segment_length = 0.03f;
  dts->initialize_parameters.damage_scale_factor = glm::vec3(0.01f);
  dts->initialize_parameters.damage_graph.Reset();
  dts->enable_physics = false;
  object_initial_pose = {};
  tree_initial_pose = {};
  tree_initial_pose.SetPosition(glm::vec3(0, 0, 0));
  camera_pose = {};
  camera_pose.SetPosition(glm::vec3(0, 1, 4.5));
  camera_pose.SetEulerRotation(glm::radians(glm::vec3(0, 0, 0)));
  ApplySceneCamera(camera_pose.GetPosition(), camera_pose.GetRotation(), true, 120.f);
  for (const auto& child : children) {
    scene->DeleteEntity(child);
  }
  const auto temp_entity = temp_entity1_ref.Get();
  if (scene->IsEntityValid(temp_entity)) {
    scene->DeleteEntity(temp_entity);
  }
  const auto tree_entity = tree_entity_ref.Get();
  if (scene->IsEntityValid(tree_entity)) {
    scene->DeleteEntity(tree_entity);
  }
  const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>();
  const std::vector<Entity>* tree_entities = scene->UnsafeGetPrivateComponentOwnersList<Tree>();
  eco_sys_lab_layer->ResetAllTrees(tree_entities);
  tree_auto_grow_started_ = false;
  pending_meshing_buffer_description.clear();
  pending_volumetric_experiment_id_ = VolumetricTreeExperimentId::Count;
  pending_precomputed_cli_id_.clear();
  pending_precomputed_strand_tree_.clear();
  pending_precomputed_mesh_bin_.clear();
  pending_precomputed_hash_.clear();
  DsKineticVoronoiMeshing::meshing_settings.force_load_meshing_buffer_bin.clear();
  DsKineticVoronoiMeshing::meshing_settings.force_load_expected_hash.clear();
  alpha_sweep_.clear();
  alpha_sweep_experiment_name_.clear();
  alpha_hull_fractal_pending_ = false;
  physics_parameters.enable_structural_damage = true;
  physics_parameters.enable_segment_compression_disconnection = true;
  physics_parameters.segment_velocity_damping = 1.f;
  physics_parameters.segment_angular_velocity_damping = 1.f;
}

void DynamicStrandsDemo::ApplySegmentSubdivisionOverride(
    DynamicStrandsInitializeParameters& initialize_parameters) const {
  if (!override_experiment_segment_subdivision) {
    return;
  }
  initialize_parameters.min_segment_length = override_min_segment_length;
  initialize_parameters.max_segment_length = override_max_segment_length;
  if (initialize_parameters.max_segment_length < initialize_parameters.min_segment_length) {
    initialize_parameters.max_segment_length = initialize_parameters.min_segment_length;
  }
}

void DynamicStrandsDemo::RunLogExperimentSetup(const std::shared_ptr<DynamicTreeStrands>& dts) {
  ApplySegmentSubdivisionOverride(dts->initialize_parameters);
  dts->LogExperimentSetup(log_experiment_setup_settings);
}

void DynamicStrandsDemo::RunBoardExperimentSetup(const std::shared_ptr<DynamicTreeStrands>& dts) {
  ApplySegmentSubdivisionOverride(dts->initialize_parameters);
  dts->BoardExperimentSetup(board_experiment_setup_settings);
}

bool DynamicStrandsDemo::DrawVolumetricMeshingUi() {
  if (demo_type != DemoType::Empty) {
    const char* experiment_name = DemoTypeExportFolderName(demo_type);
    if (const VolumetricTreeExperiment* experiment = FindVolumetricTreeExperiment(pending_volumetric_experiment_id_)) {
      experiment_name = experiment->display_name;
    } else if (!alpha_sweep_experiment_name_.empty()) {
      experiment_name = alpha_sweep_experiment_name_.c_str();
    }
    ImGui::Text(
        "Experiment: %s%s", experiment_name,
        alpha_hull_fractal_pending_ ? " (alpha-hull fractal)" : (!alpha_sweep_.empty() ? " (alpha sweep)" : ""));
    if (demo_status == DemoStatus::TreeGrowth) {
      if (target_growth_iterations > 0) {
        int grown_iterations = 0;
        if (const auto scene = GetScene()) {
          const auto tree_entity = tree_entity_ref.Get();
          if (scene->IsEntityValid(tree_entity) && scene->HasPrivateComponent<Tree>(tree_entity)) {
            grown_iterations =
                scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock()->shoot_model.CurrentIteration();
          }
        }
        ImGui::Text("Tree growth: %d / %d iterations", grown_iterations, target_growth_iterations);
      } else {
        float grown_years = 0.f;
        if (const auto eco_sys_lab_layer = ApplicationContext::Get().GetLayer<EcoSysLabLayer>()) {
          grown_years = eco_sys_lab_layer->GetSimulatedTime() / 365.f;
        }
        ImGui::Text("Tree growth: %.2f / %.2f years", grown_years, target_growth_time);
      }
      TryFinishTreeGrowthAndStartMeshing();
    }
    return false;
  }
  bool changed = false;
  const auto owner = GetOwner();
  const auto scene = GetScene();
  const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(owner).lock();

  if (ImGui::TreeNodeEx("Volumetric Meshing", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (ImGui::Button("Log cut")) {
      ResetEnvironment();
      camera_pose.SetPosition(glm::vec3(-0.3, kVolumetricLogExperimentCameraHeight, 0.2));
      camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
      log_experiment_setup_settings.rod_segment_count = 20;
      log_experiment_setup_settings.rod_size = 3200;
      log_experiment_setup_settings.segment_length = 0.025f;
      log_experiment_setup_settings.fungus_test = false;
      log_experiment_setup_settings.cube_pattern = false;
      log_experiment_setup_settings.internal_pattern = false;
      log_experiment_setup_settings.competition_setting = false;
      // Transform pivots + break motion (same as Log/Board break).
      log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
      log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
      target_factor0 = 1.f;
      target_factor1 = 1.f;
      target_simulation_time = kLogExperimentPivotDuration;
      automated_export_upper = 15.f;
      automated_export_stepsize = 1.f;
      ResetAutomatedExportSchedule();
      // Disable crack opening offset (x - fracture_distance * shift) for volumetric demos.
      DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance = 0.f;
      physics_parameters.enable_fungus = false;
      physics_parameters.enable_segment_collision = false;
      // Coarser random subdivision than Fungus [Cubical] (0.005–0.01) for longer physics segments.
      dts->initialize_parameters.min_segment_length = 0.02f;
      dts->initialize_parameters.max_segment_length = 0.04f;
      SetupVolumetricLogExperimentHeight(scene, owner, dts);

      log_experiment_setup_settings.meshing_buffer_description =
          "created from DynamicStrandsDemo scripted experiment: Log cut";
      RunLogExperimentSetup(dts);
      const auto yaml_path = ProjectManager::GetAssetsFolderPath() / "IntersectionSetups" / "log_cut.yml";
      DsKineticVoronoiMeshing::LoadIntersectionSetup(scene, owner, yaml_path);

      ApplySceneCamera(camera_pose.GetPosition(), camera_pose.GetRotation(), false, 0.f);
      demo_type = DemoType::LogCut;
      demo_status = DemoStatus::Simulation;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Build the Fungus [Cubical] log without fungus, mesh it (uses MeshBuffers cache when available), "
          "load Assets/IntersectionSetups/log_cut.yml (intersect manually), then pull pivots apart on Play.");
    }

    if (ImGui::Button("Log break comparison")) {
      ResetEnvironment();
      camera_pose.SetPosition(glm::vec3(-0.3, kVolumetricLogExperimentCameraHeight, 0.2));
      camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
      log_experiment_setup_settings.rod_segment_count = 20;
      log_experiment_setup_settings.rod_size = 1600;
      log_experiment_setup_settings.segment_length = 0.025f;
      log_experiment_setup_settings.fungus_test = false;
      log_experiment_setup_settings.cube_pattern = false;
      log_experiment_setup_settings.internal_pattern = false;
      log_experiment_setup_settings.competition_setting = false;
      log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
      log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
      target_factor0 = 1.f;
      target_factor1 = 1.f;
      target_simulation_time = kLogExperimentPivotDuration;
      automated_export_upper = 25.f;
      automated_export_stepsize = 5.f;
      ResetAutomatedExportSchedule();
      DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance = 0.f;
      physics_parameters.enable_fungus = false;
      physics_parameters.enable_segment_collision = false;
      dts->initialize_parameters.min_segment_length = 0.02f;
      dts->initialize_parameters.max_segment_length = 0.04f;
      dts->initialize_parameters.meshing_type = MeshingType::Both;
      SetupVolumetricLogExperimentHeight(scene, owner, dts);

      log_experiment_setup_settings.meshing_buffer_description =
          "created from DynamicStrandsDemo scripted experiment: Log break comparison";
      RunLogExperimentSetup(dts);

      ApplySceneCamera(camera_pose.GetPosition(), camera_pose.GetRotation(), false, 0.f);
      demo_type = DemoType::LogBreakComparison;
      demo_status = DemoStatus::Simulation;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Volumetric log setup like Log cut (no intersection meshes), with 1600 strands for comparison "
          "against Stressful Trees Log break (800). Pull pivots apart on Play.");
    }

    if (ImGui::Button("Log break comparison (1/2 cutoff, alpha 1.3e-4)")) {
      ResetEnvironment();
      camera_pose.SetPosition(glm::vec3(-0.3, kVolumetricLogExperimentCameraHeight, 0.2));
      camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
      log_experiment_setup_settings.rod_segment_count = 20;
      log_experiment_setup_settings.rod_size = 1600;
      log_experiment_setup_settings.segment_length = 0.025f;
      log_experiment_setup_settings.fungus_test = false;
      log_experiment_setup_settings.cube_pattern = false;
      log_experiment_setup_settings.internal_pattern = false;
      log_experiment_setup_settings.competition_setting = false;
      log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
      log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
      target_factor0 = 1.f;
      target_factor1 = 1.f;
      target_simulation_time = kLogExperimentPivotDuration;
      automated_export_upper = 25.f;
      automated_export_stepsize = 5.f;
      ResetAutomatedExportSchedule();
      DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance = 0.f;
      physics_parameters.enable_fungus = false;
      physics_parameters.enable_segment_collision = false;
      dts->initialize_parameters.min_segment_length = 0.02f;
      dts->initialize_parameters.max_segment_length = 0.04f;
      dts->initialize_parameters.meshing_type = MeshingType::Both;
      // Half Kinetic Voronoi cutoff (default 10 → 5); Alpha Shape alpha 1.3e-4 (default 5e-5).
      DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff = 5.0;
      DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff = 5.0;
      dts->initialize_parameters.alpha = 0.00013f;
      dts->initialize_parameters.bifurcation_alpha = 0.00013f;
      DsAlphaShapeMeshing::render_settings.branches_render_parameters.alpha = 0.00013f;
      DsAlphaShapeMeshing::render_settings.branches_render_parameters.bifurcation_alpha = 0.00013f;
      SetupVolumetricLogExperimentHeight(scene, owner, dts);

      log_experiment_setup_settings.meshing_buffer_description =
          "created from DynamicStrandsDemo scripted experiment: Log break comparison (1/2 cutoff, alpha 1.3e-4)";
      RunLogExperimentSetup(dts);

      ApplySceneCamera(camera_pose.GetPosition(), camera_pose.GetRotation(), false, 0.f);
      demo_type = DemoType::LogBreakComparisonHalfCutoffDoubleAlpha;
      demo_status = DemoStatus::Simulation;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Same as Log break comparison, but Kinetic Voronoi alpha_cutoff/branch_alpha_cutoff are halved (10→5) "
          "and Alpha Shape alpha/bifurcation_alpha are set to 1.3e-4 (default 5e-5).");
    }

    if (ImGui::Button("Log Spoon")) {
      ResetEnvironment();
      camera_pose.SetPosition(glm::vec3(-0.3, kVolumetricLogExperimentCameraHeight, 0.2));
      camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));
      log_experiment_setup_settings.rod_segment_count = 20;
      log_experiment_setup_settings.rod_size = 3200;
      log_experiment_setup_settings.segment_length = 0.025f;
      log_experiment_setup_settings.fungus_test = false;
      log_experiment_setup_settings.cube_pattern = false;
      log_experiment_setup_settings.internal_pattern = false;
      log_experiment_setup_settings.competition_setting = false;
      log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
      log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
      target_factor0 = 1.f;
      target_factor1 = 1.f;
      target_simulation_time = kLogExperimentPivotDuration;
      DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance = 0.f;
      physics_parameters.enable_fungus = false;
      physics_parameters.enable_segment_collision = false;
      // Coarser random subdivision than Fungus [Cubical] (0.005–0.01) for longer physics segments.
      dts->initialize_parameters.min_segment_length = 0.02f;
      dts->initialize_parameters.max_segment_length = 0.04f;
      SetupVolumetricLogExperimentHeight(scene, owner, dts);

      log_experiment_setup_settings.meshing_buffer_description =
          "created from DynamicStrandsDemo scripted experiment: Log Spoon";
      RunLogExperimentSetup(dts);
      const auto yaml_path = ProjectManager::GetAssetsFolderPath() / "IntersectionSetups" / "log_spoon.yml";
      DsKineticVoronoiMeshing::LoadIntersectionSetup(scene, owner, yaml_path);

      ApplySceneCamera(camera_pose.GetPosition(), camera_pose.GetRotation(), false, 0.f);
      demo_type = DemoType::LogSpoon;
      demo_status = DemoStatus::Simulation;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Same as Log cut with Assets/IntersectionSetups/log_spoon.yml (intersect manually), "
          "then pull pivots apart on Play.");
    }

    if (ImGui::Button("Log cut upright + bunny")) {
      ResetEnvironment();
      camera_pose.SetPosition(glm::vec3(-0.3, kVolumetricLogExperimentCameraHeight, 0.2));
      camera_pose.SetEulerRotation(glm::radians(glm::vec3(-30, -60, 0)));

      // Half-length log so the bunny fits between bottom/top.
      log_experiment_setup_settings.rod_segment_count = 20;
      log_experiment_setup_settings.rod_size = 3200;
      log_experiment_setup_settings.segment_length = 0.0125f;
      // 20% wider than the default radius (0.002) to better match the bunny width.
      log_experiment_setup_settings.radius = 0.0024f;
      log_experiment_setup_settings.fungus_test = false;
      log_experiment_setup_settings.cube_pattern = false;
      log_experiment_setup_settings.internal_pattern = false;
      log_experiment_setup_settings.competition_setting = false;
      // Transform pivots + break motion (same pivot type as Log/Board break).
      log_experiment_setup_settings.left_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);
      log_experiment_setup_settings.right_pivot_type = static_cast<unsigned>(DynamicTreeStrands::PivotType::Transform);

      target_factor0 = 1.f;
      target_factor1 = 1.f;
      target_simulation_time = kLogExperimentPivotDuration;
      automated_export_upper = 15.f;
      automated_export_stepsize = 1.f;
      ResetAutomatedExportSchedule();

      // Disable crack opening offset (x - fracture_distance * shift) for volumetric demos.
      DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance = 0.f;

      physics_parameters.enable_fungus = false;
      physics_parameters.enable_segment_collision = false;
      // Coarser random subdivision than Fungus [Cubical] (0.005–0.01) for longer physics segments.
      dts->initialize_parameters.min_segment_length = 0.02f;
      dts->initialize_parameters.max_segment_length = 0.04f;

      // Rotate owner so the log's local +X rod axis becomes world +Y (upright).
      GlobalTransform owner_gt = scene->GetDataComponent<GlobalTransform>(owner);
      owner_gt.SetEulerRotation(glm::radians(glm::vec3(0.f, 0.f, 90.f)));
      scene->SetDataComponent(owner, owner_gt);

      SetupVolumetricLogExperimentHeight(scene, owner, dts);

      log_experiment_setup_settings.meshing_buffer_description =
          "created from DynamicStrandsDemo scripted experiment: Log cut upright + bunny";

      RunLogExperimentSetup(dts);

      const float log_length = static_cast<float>(log_experiment_setup_settings.rod_segment_count) *
                               log_experiment_setup_settings.segment_length;
      const auto upright_owner_gt = scene->GetDataComponent<GlobalTransform>(owner);
      SetupLogCutUprightBunnyIntersectionBoundary(scene, owner, upright_owner_gt, log_length);

      ApplySceneCamera(camera_pose.GetPosition(), camera_pose.GetRotation(), false, 0.f);
      demo_type = DemoType::LogCutUprightBunny;
      demo_status = DemoStatus::Simulation;
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Half-length log cut with a bunny boundary mesh. The lower pivot stays fixed; only the upper pivot "
          "rotates downwards around it.");
    }

    const auto& volumetric_experiments = GetVolumetricTreeExperiments();
    for (const auto& experiment : volumetric_experiments) {
      if (ImGui::Button(experiment.display_name)) {
        StartVolumetricTreeExperiment(experiment);
      }
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", experiment.tooltip);
      }
    }

    if (ImGui::Button("Small Trunk alpha sweep")) {
      const VolumetricTreeExperiment* base = FindVolumetricTreeExperiment(VolumetricTreeExperimentId::SmallTrunk);
      ResetEnvironment();
      demo_type = DemoType::SmallTrunk;
      demo_status = DemoStatus::TreeGrowth;
      alpha_sweep_ = {2500.0, 900.0, 400.0, 100.0, 25.0, 9.0, 4.0, 1.0};
      alpha_sweep_experiment_name_ = "Small_Trunk";
      pending_meshing_buffer_description =
          "created from DynamicStrandsDemo scripted experiment: Small Trunk alpha sweep";
      DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance = 0.f;
      const auto tree_entity = scene->CreateEntity("Tree");
      tree_entity_ref = tree_entity;
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      scene->SetDataComponent(owner, tree_initial_pose);
      scene->SetDataComponent(tree_entity, tree_initial_pose);
      target_growth_time = base->growth_years;
      target_growth_iterations = 0;
      tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset(base->tree_descriptor_path);
      ApplyVolumetricTreeStrandPreset(*base, tree);
      ApplyVolumetricTreePhysicsPreset(physics_parameters);
      dts->enable_physics = false;
      dts->initialize_parameters.min_segment_length = base->min_segment_length;
      dts->initialize_parameters.max_segment_length = base->max_segment_length;
      EVOENGINE_LOG("Small Trunk alpha sweep: growing for "
                    << target_growth_time << " years, then meshing alphas 2500,900,400,100,25,9,4,1...");
      ApplySceneCamera(camera_pose.GetPosition(), camera_pose.GetRotation(), false, 0.f);
      BeginTreeAutoGrow();
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Same growth as Small Trunk, then remesh for alpha = cutoff^2 in "
          "{2500,900,400,100,25,9,4,1}. Forces remesh each time, writes meshing statistics "
          "as Small_Trunk_alpha_<N>, exports meshlets_alpha_<N>.obj under PhysicsDemoExports, "
          "and continues to the next alpha if meshing fails (failure logged in the stats CSV).");
    }

    if (ImGui::Button("Oak thick stump alpha sweep")) {
      const VolumetricTreeExperiment* base = FindVolumetricTreeExperiment(VolumetricTreeExperimentId::OakThickStump100);
      ResetEnvironment();
      demo_type = DemoType::OakThickStump;
      demo_status = DemoStatus::TreeGrowth;
      alpha_sweep_ = {2500.0, 900.0, 400.0, 100.0, 25.0, 9.0, 4.0, 1.0};
      alpha_sweep_experiment_name_ = "Oak_thick_stump";
      pending_meshing_buffer_description =
          "created from DynamicStrandsDemo scripted experiment: Oak thick stump alpha sweep";
      DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance = 0.f;
      const auto tree_entity = scene->CreateEntity("Tree");
      tree_entity_ref = tree_entity;
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      scene->SetDataComponent(owner, tree_initial_pose);
      scene->SetDataComponent(tree_entity, tree_initial_pose);
      target_growth_time = 0.f;
      target_growth_iterations = base->growth_iterations;
      tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset(base->tree_descriptor_path);
      ApplyVolumetricTreeStrandPreset(*base, tree);
      ApplyVolumetricTreePhysicsPreset(physics_parameters);
      dts->enable_physics = false;
      dts->initialize_parameters.min_segment_length = base->min_segment_length;
      dts->initialize_parameters.max_segment_length = base->max_segment_length;
      EVOENGINE_LOG("Oak thick stump alpha sweep: growing for "
                    << target_growth_iterations << " iterations, then meshing alphas 2500,900,400,100,25,9,4,1...");
      ApplySceneCamera(camera_pose.GetPosition(), camera_pose.GetRotation(), false, 0.f);
      BeginTreeAutoGrow();
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Same growth as Oak thick stump, then remesh for alpha = cutoff^2 in "
          "{2500,900,400,100,25,9,4,1}. Forces remesh each time, writes meshing statistics "
          "as Oak_thick_stump_alpha_<N>, exports meshlets_alpha_<N>.obj under PhysicsDemoExports, "
          "and continues to the next alpha if meshing fails (failure logged in the stats CSV).");
    }

    if (ImGui::Button("Small Trunk alpha-hull fractal")) {
      const VolumetricTreeExperiment* base = FindVolumetricTreeExperiment(VolumetricTreeExperimentId::SmallTrunk);
      ResetEnvironment();
      demo_type = DemoType::SmallTrunk;
      demo_status = DemoStatus::TreeGrowth;
      alpha_sweep_ = {2500.0, 900.0, 400.0, 100.0, 25.0, 9.0, 4.0, 1.0};
      alpha_sweep_experiment_name_ = "Small_Trunk";
      alpha_hull_fractal_pending_ = true;
      alpha_hull_height_min_ = 0;
      alpha_hull_height_max_ = 25;
      pending_meshing_buffer_description =
          "created from DynamicStrandsDemo scripted experiment: Small Trunk alpha-hull fractal";
      DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance = 0.f;
      const auto tree_entity = scene->CreateEntity("Tree");
      tree_entity_ref = tree_entity;
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      scene->SetDataComponent(owner, tree_initial_pose);
      scene->SetDataComponent(tree_entity, tree_initial_pose);
      target_growth_time = base->growth_years;
      target_growth_iterations = 0;
      tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset(base->tree_descriptor_path);
      ApplyVolumetricTreeStrandPreset(*base, tree);
      ApplyVolumetricTreePhysicsPreset(physics_parameters);
      dts->enable_physics = false;
      dts->initialize_parameters.min_segment_length = base->min_segment_length;
      dts->initialize_parameters.max_segment_length = base->max_segment_length;
      EVOENGINE_LOG("Small Trunk alpha-hull fractal: growing for " << target_growth_time
                                                                   << " years, then analyzing sections 0..25...");
      ApplySceneCamera(camera_pose.GetPosition(), camera_pose.GetRotation(), false, 0.f);
      BeginTreeAutoGrow();
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Same growth as Small Trunk, then dry-run StrandTree (no meshing). For each profile section 0..25, "
          "Delaunay-triangulate strand points, keep triangles with R^2 < alpha for "
          "{2500,900,400,100,25,9,4,1}, extract alpha-hull polygons + convex hull, and measure "
          "box-counting fractal dimension. Writes CSV/SVG under PhysicsDemoExports.");
    }

    if (ImGui::Button("Oak thick stump alpha-hull fractal")) {
      const VolumetricTreeExperiment* base = FindVolumetricTreeExperiment(VolumetricTreeExperimentId::OakThickStump100);
      ResetEnvironment();
      demo_type = DemoType::OakThickStump;
      demo_status = DemoStatus::TreeGrowth;
      alpha_sweep_ = {2500.0, 900.0, 400.0, 100.0, 25.0, 9.0, 4.0, 1.0};
      alpha_sweep_experiment_name_ = "Oak_thick_stump";
      alpha_hull_fractal_pending_ = true;
      alpha_hull_height_min_ = 0;
      alpha_hull_height_max_ = 25;
      pending_meshing_buffer_description =
          "created from DynamicStrandsDemo scripted experiment: Oak thick stump alpha-hull fractal";
      DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance = 0.f;
      const auto tree_entity = scene->CreateEntity("Tree");
      tree_entity_ref = tree_entity;
      const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
      scene->SetDataComponent(owner, tree_initial_pose);
      scene->SetDataComponent(tree_entity, tree_initial_pose);
      target_growth_time = 0.f;
      target_growth_iterations = base->growth_iterations;
      tree->tree_descriptor_ref = ProjectManager::GetOrCreateAsset(base->tree_descriptor_path);
      ApplyVolumetricTreeStrandPreset(*base, tree);
      ApplyVolumetricTreePhysicsPreset(physics_parameters);
      dts->enable_physics = false;
      dts->initialize_parameters.min_segment_length = base->min_segment_length;
      dts->initialize_parameters.max_segment_length = base->max_segment_length;
      EVOENGINE_LOG("Oak thick stump alpha-hull fractal: growing for "
                    << target_growth_iterations << " iterations, then analyzing sections 0..25...");
      ApplySceneCamera(camera_pose.GetPosition(), camera_pose.GetRotation(), false, 0.f);
      BeginTreeAutoGrow();
    }
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip(
          "Same growth as Oak thick stump, then dry-run StrandTree (no meshing). For each profile section 0..25, "
          "Delaunay + alpha hulls for alphas {2500,900,400,100,25,9,4,1} (R^2 < alpha), convex hull, "
          "and box-counting fractal dimensions. Writes CSV/SVG under PhysicsDemoExports.");
    }

    ImGui::TreePop();
  }

  DrawPrecomputedExperimentsUi();
  DrawVolumetricPrecomputeUi();

  return changed;
}

void DynamicStrandsDemo::EnsurePrecomputeSelectionSize() {
  const size_t count = GetVolumetricTreeExperiments().size();
  if (precompute_selected_.size() != count) {
    precompute_selected_.assign(count, 0);
  }
}

const char* DynamicStrandsDemo::PrecomputeJobStatusLabel(const PrecomputeJobStatus status) {
  switch (status) {
    case PrecomputeJobStatus::Queued:
      return "Queued";
    case PrecomputeJobStatus::Growing:
      return "Growing";
    case PrecomputeJobStatus::Exporting:
      return "Exporting";
    case PrecomputeJobStatus::ReadyToMesh:
      return "ReadyToMesh";
    case PrecomputeJobStatus::Running:
      return "Running";
    case PrecomputeJobStatus::CacheHit:
      return "CacheHit";
    case PrecomputeJobStatus::Done:
      return "Done";
    case PrecomputeJobStatus::Failed:
      return "Failed";
    case PrecomputeJobStatus::Crashed:
      return "Crashed";
  }
  return "?";
}

void DynamicStrandsDemo::DrawVolumetricPrecomputeUi() {
  EnsurePrecomputeSelectionSize();
  PollPrecomputeJobs();
  PumpPrecomputeQueue();

  if (!ImGui::TreeNodeEx("Parallel Kinetic Mesh Precompute", ImGuiTreeNodeFlags_DefaultOpen)) {
    return;
  }

  const auto& experiments = GetVolumetricTreeExperiments();
  if (ImGui::Button("Select all")) {
    std::fill(precompute_selected_.begin(), precompute_selected_.end(), 1);
  }
  ImGui::SameLine();
  if (ImGui::Button("Select none")) {
    std::fill(precompute_selected_.begin(), precompute_selected_.end(), 0);
  }
  ImGui::DragInt("Max concurrent jobs", &precompute_max_concurrent_, 1, 1, 48);

  for (size_t i = 0; i < experiments.size(); ++i) {
    bool selected = precompute_selected_[i] != 0;
    if (ImGui::Checkbox(experiments[i].display_name, &selected)) {
      precompute_selected_[i] = selected ? 1 : 0;
    }
  }

  const bool busy = precompute_active_;
  if (!busy) {
    if (ImGui::Button("Start precompute")) {
      StartPrecomputeQueue();
    }
  } else {
    if (ImGui::Button("Cancel (no new spawns)")) {
      precompute_cancel_requested_ = true;
    }
  }

  if (!precompute_jobs_.empty()) {
    ImGui::Separator();
    ImGui::Text("Jobs:");
    for (const auto& job : precompute_jobs_) {
      ImGui::BulletText("%s: %s", job.display_name.c_str(), PrecomputeJobStatusLabel(job.status));
      if (!job.log_path.empty() && ImGui::IsItemHovered()) {
        ImGui::SetTooltip("%s", job.log_path.string().c_str());
      }
    }
  }
  ImGui::TreePop();
}

void DynamicStrandsDemo::StartPrecomputeQueue() {
  EnsurePrecomputeSelectionSize();
  PollPrecomputeJobs();
  if (precompute_active_) {
    return;
  }
  precompute_jobs_.clear();
  precompute_cancel_requested_ = false;
  precompute_growing_job_index_ = static_cast<size_t>(-1);
  {
    std::lock_guard<std::mutex> lock(precompute_export_sync_->mutex);
    precompute_export_sync_->results.clear();
  }
  precompute_export_sync_->exporting_count = 0;
  const auto& experiments = GetVolumetricTreeExperiments();
  for (size_t i = 0; i < experiments.size(); ++i) {
    if (!precompute_selected_[i]) {
      continue;
    }
    PrecomputeJob job;
    job.experiment_cli_id = experiments[i].cli_id;
    job.display_name = experiments[i].display_name;
    job.status = PrecomputeJobStatus::Queued;
    precompute_jobs_.push_back(std::move(job));
  }
  precompute_active_ = !precompute_jobs_.empty();
  PumpPrecomputeQueue();
}

void DynamicStrandsDemo::PollPrecomputeJobs() {
  std::vector<PrecomputeExportResult> export_results;
  {
    std::lock_guard<std::mutex> lock(precompute_export_sync_->mutex);
    export_results.swap(precompute_export_sync_->results);
  }
  for (const auto& result : export_results) {
    if (result.job_index >= precompute_jobs_.size()) {
      continue;
    }
    auto& job = precompute_jobs_[result.job_index];
    if (job.status != PrecomputeJobStatus::Exporting) {
      continue;
    }
    if (precompute_cancel_requested_ || !result.ok) {
      job.status = PrecomputeJobStatus::Failed;
      EVOENGINE_ERROR("Precompute export failed for " << job.display_name << " [" << job.experiment_cli_id << "]");
    } else {
      job.status = PrecomputeJobStatus::ReadyToMesh;
      EVOENGINE_LOG("Precompute export ready for kinDS worker: " << job.display_name);
    }
  }
  if (!export_results.empty()) {
    PumpPrecomputeQueue();
  }

#ifdef _WIN32
  for (auto& job : precompute_jobs_) {
    if (job.status != PrecomputeJobStatus::Running || !job.process_handle) {
      continue;
    }
    DWORD exit_code = STILL_ACTIVE;
    if (!GetExitCodeProcess(static_cast<HANDLE>(job.process_handle), &exit_code)) {
      const DWORD err = GetLastError();
      CloseHandle(static_cast<HANDLE>(job.process_handle));
      job.process_handle = nullptr;
      job.status = PrecomputeJobStatus::Crashed;
      EVOENGINE_ERROR("Precompute worker died (GetExitCodeProcess failed) for "
                      << job.display_name << " [" << job.experiment_cli_id << "] GetLastError=" << err);
      continue;
    }
    if (exit_code == STILL_ACTIVE) {
      continue;
    }
    CloseHandle(static_cast<HANDLE>(job.process_handle));
    job.process_handle = nullptr;
    if (exit_code == 0) {
      bool cache_hit = false;
      if (!job.log_path.empty() && std::filesystem::exists(job.log_path)) {
        std::ifstream in(job.log_path);
        std::string line;
        while (std::getline(in, line)) {
          if (line.find("cache hit") != std::string::npos || line.find("CacheHit") != std::string::npos ||
              line.find("precompute skip load") != std::string::npos ||
              line.find("already present, skipping") != std::string::npos) {
            cache_hit = true;
          }
        }
      }
      job.status = cache_hit ? PrecomputeJobStatus::CacheHit : PrecomputeJobStatus::Done;
    } else if (exit_code > 0 && exit_code < 1000) {
      job.status = PrecomputeJobStatus::Failed;
    } else {
      job.status = PrecomputeJobStatus::Crashed;
    }
    EVOENGINE_LOG("Precompute worker exited: " << job.display_name << " [" << job.experiment_cli_id
                                               << "] status=" << PrecomputeJobStatusLabel(job.status)
                                               << " exit_code=" << exit_code << " log=" << job.log_path.string());
  }
#endif

  bool any_running_or_queued = false;
  for (const auto& job : precompute_jobs_) {
    if (job.status == PrecomputeJobStatus::Queued || job.status == PrecomputeJobStatus::Growing ||
        job.status == PrecomputeJobStatus::Exporting || job.status == PrecomputeJobStatus::ReadyToMesh ||
        job.status == PrecomputeJobStatus::Running) {
      any_running_or_queued = true;
      break;
    }
  }
  if (!any_running_or_queued && precompute_export_sync_->exporting_count == 0) {
    precompute_active_ = false;
  }
}

void DynamicStrandsDemo::StartPrecomputeGrowth(const size_t job_index) {
  if (job_index >= precompute_jobs_.size()) {
    return;
  }
  const VolumetricTreeExperiment* experiment =
      FindVolumetricTreeExperiment(precompute_jobs_[job_index].experiment_cli_id.c_str());
  if (!experiment) {
    EVOENGINE_ERROR("Precompute: unknown experiment " << precompute_jobs_[job_index].experiment_cli_id);
    precompute_jobs_[job_index].status = PrecomputeJobStatus::Failed;
    return;
  }
  precompute_growing_job_index_ = job_index;
  precompute_jobs_[job_index].status = PrecomputeJobStatus::Growing;
  EVOENGINE_LOG("Precompute growing " << experiment->display_name << " in parent before kinDS worker...");
  StartVolumetricTreeExperiment(*experiment);
}

void DynamicStrandsDemo::FinishPrecomputeGrowthAndSpawnWorker() {
  const size_t job_index = precompute_growing_job_index_;
  precompute_growing_job_index_ = static_cast<size_t>(-1);
  tree_auto_grow_started_ = false;
  demo_status = DemoStatus::Idle;
  demo_type = DemoType::Empty;

  if (job_index >= precompute_jobs_.size()) {
    return;
  }
  auto& job = precompute_jobs_[job_index];
  if (precompute_cancel_requested_) {
    job.status = PrecomputeJobStatus::Failed;
    return;
  }

  const VolumetricTreeExperiment* experiment = FindVolumetricTreeExperiment(job.experiment_cli_id.c_str());
  const auto scene = GetScene();
  const auto tree_entity = tree_entity_ref.Get();
  if (!experiment || !scene || !scene->IsEntityValid(tree_entity)) {
    EVOENGINE_ERROR("Precompute export: tree/experiment missing for " << job.experiment_cli_id);
    job.status = PrecomputeJobStatus::Failed;
    return;
  }
  const auto tree = scene->GetOrSetPrivateComponent<Tree>(tree_entity).lock();
  if (!tree) {
    job.status = PrecomputeJobStatus::Failed;
    return;
  }
  // Scene-owned work stays on the main thread; StrandTree/job export runs in the background.
  tree->BuildStrandModel();
  StrandModel strand_model = tree->shoot_strand_model;

  const std::filesystem::path job_dir = EcoSysLabMetadataDirectory() / "precompute_jobs" / job.experiment_cli_id;
  const std::filesystem::path log_dir = EcoSysLabMetadataDirectory() / "precompute_logs";
  std::error_code ec;
  std::filesystem::create_directories(job_dir, ec);
  std::filesystem::create_directories(log_dir, ec);
  const std::string stamp = MakeAutomatedExportTimestamp();
  job.strand_tree_path = job_dir / (job.experiment_cli_id + ".strandtree");
  job.job_path = job_dir / (job.experiment_cli_id + ".job.txt");
  job.log_path = log_dir / (job.experiment_cli_id + "_" + stamp + ".txt");
  const std::filesystem::path stats_csv = EcoSysLabMetadataPath("meshing_statistics.csv");

  DynamicStrandsInitializeParameters initialize_parameters{};
  initialize_parameters.min_segment_length = experiment->min_segment_length;
  initialize_parameters.max_segment_length = experiment->max_segment_length;
  initialize_parameters.root_transform = scene->GetDataComponent<GlobalTransform>(tree_entity);
  initialize_parameters.meshing_type = MeshingType::KineticVoronoi;
  ApplySegmentSubdivisionOverride(initialize_parameters);

  // Precompute must always use a fixed subdivision seed so MeshBuffers hashes are reproducible.
  const int seed = experiment->seed >= 0 ? experiment->seed : 0;
  constexpr bool fixed_seed = true;
  const std::string description = experiment->meshing_buffer_description;
  const std::string experiment_tag = experiment->cli_id;
  const std::filesystem::path strand_tree_path = job.strand_tree_path;
  const std::filesystem::path job_path = job.job_path;
  const std::filesystem::path log_path = job.log_path;

  job.status = PrecomputeJobStatus::Exporting;
  ++precompute_export_sync_->exporting_count;
  EVOENGINE_LOG("Precompute exporting StrandTree/job off-thread for " << job.display_name << "...");

  std::thread([this, job_index, strand_model = std::move(strand_model), initialize_parameters, seed, fixed_seed,
               description, experiment_tag, strand_tree_path, job_path, stats_csv, log_path]() mutable {
    const bool ok = ExportPrecomputeStrandTreeJob(strand_model, initialize_parameters, seed, fixed_seed, description,
                                                  experiment_tag, strand_tree_path, job_path, stats_csv, log_path, 4);
    {
      std::lock_guard<std::mutex> lock(precompute_export_sync_->mutex);
      precompute_export_sync_->results.push_back(PrecomputeExportResult{job_index, ok});
    }
    --precompute_export_sync_->exporting_count;
  }).detach();

  // Continue growing the next queued experiment while export/meshing proceed asynchronously.
  PumpPrecomputeQueue();
}

bool DynamicStrandsDemo::SpawnPrecomputeWorker(PrecomputeJob& job) {
#ifdef _WIN32
  char module_path[MAX_PATH]{};
  if (GetModuleFileNameA(nullptr, module_path, MAX_PATH) == 0) {
    EVOENGINE_ERROR("Precompute: GetModuleFileNameA failed");
    return false;
  }
  const std::filesystem::path exe_dir = std::filesystem::path(module_path).parent_path();
  const std::filesystem::path worker_exe = exe_dir / "KineticMeshPrecomputeApp.exe";
  if (!std::filesystem::exists(worker_exe)) {
    EVOENGINE_ERROR("Precompute worker missing: " << worker_exe.string());
    return false;
  }

  std::ostringstream cmd;
  cmd << '"' << worker_exe.string() << '"' << " --job \"" << job.job_path.string() << '"';
  std::string cmdline = cmd.str();

  SECURITY_ATTRIBUTES sa{};
  sa.nLength = sizeof(sa);
  sa.bInheritHandle = TRUE;
  HANDLE log_file = CreateFileA(job.log_path.string().c_str(), GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, &sa,
                                CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
  STARTUPINFOA si{};
  si.cb = sizeof(si);
  if (log_file != INVALID_HANDLE_VALUE) {
    si.dwFlags |= STARTF_USESTDHANDLES;
    si.hStdOutput = log_file;
    si.hStdError = log_file;
    si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
  }
  PROCESS_INFORMATION pi{};
  std::vector<char> mutable_cmd(cmdline.begin(), cmdline.end());
  mutable_cmd.push_back('\0');
  const auto project_path = ProjectManager::GetProjectPath();
  const BOOL ok = CreateProcessA(nullptr, mutable_cmd.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW, nullptr,
                                 project_path.parent_path().string().c_str(), &si, &pi);
  if (log_file != INVALID_HANDLE_VALUE) {
    CloseHandle(log_file);
  }
  if (!ok) {
    EVOENGINE_ERROR("CreateProcess failed for " << job.experiment_cli_id << " (err=" << GetLastError() << ")");
    return false;
  }
  CloseHandle(pi.hThread);
  job.process_handle = pi.hProcess;
  job.process_id = pi.dwProcessId;
  job.status = PrecomputeJobStatus::Running;
  EVOENGINE_LOG("Precompute spawned kinDS worker " << job.display_name << " -> " << job.log_path.string());
  return true;
#else
  EVOENGINE_ERROR("Parallel Kinetic Mesh Precompute currently requires Windows CreateProcess.");
  return false;
#endif
}

void DynamicStrandsDemo::PumpPrecomputeQueue() {
  if (!precompute_active_ || precompute_cancel_requested_) {
    return;
  }

  int running = 0;
  for (const auto& job : precompute_jobs_) {
    if (job.status == PrecomputeJobStatus::Running) {
      ++running;
    }
  }
  const int max_concurrent = std::max(1, precompute_max_concurrent_);
  for (auto& job : precompute_jobs_) {
    if (running >= max_concurrent) {
      break;
    }
    if (job.status != PrecomputeJobStatus::ReadyToMesh) {
      continue;
    }
    if (!SpawnPrecomputeWorker(job)) {
      job.status = PrecomputeJobStatus::Failed;
      continue;
    }
    ++running;
  }

  // Parent-side growth is serial. Also wait for off-thread export to finish before the next
  // growth: ExportPrecomputeStrandTreeJob mutates process-global meshing_settings.
  if (precompute_growing_job_index_ != static_cast<size_t>(-1) || precompute_export_sync_->exporting_count > 0) {
    return;
  }
  for (size_t i = 0; i < precompute_jobs_.size(); ++i) {
    if (precompute_jobs_[i].status == PrecomputeJobStatus::Queued) {
      StartPrecomputeGrowth(i);
      return;
    }
  }
}

bool DynamicStrandsDemo::ControlsStrands(Entity entity) {
  if (demo_status != DemoStatus::Simulation)
    return false;
  if (entity == GetOwner())
    return true;
  return (demo_type == DemoType::TrunkStrength || demo_type == DemoType::Wind ||
          demo_type == DemoType::TreeCollision) &&
         entity == tree_entity_ref.Get();
}

void DynamicStrandsDemo::Update() {
  PollPrecomputeJobs();
  PumpPrecomputeQueue();
  if (demo_status == DemoStatus::Idle)
    return;
  if (demo_status == DemoStatus::Simulation && simulated_time >= target_simulation_time) {
    demo_type = DemoType::Empty;
    demo_status = DemoStatus::Idle;
    return;
  }
  const auto owner = GetOwner();
  const auto scene = GetScene();
  const auto dts = scene->GetOrSetPrivateComponent<DynamicTreeStrands>(owner).lock();
  dts->dynamic_strands->UpdateBindings();
  if (demo_status == DemoStatus::TreeGrowth) {
    // Growth advances in EcoSysLabLayer::Update; meshing is started by SetOnAutoGrowFinished.
    // Keep a poll here for Playing mode / missed callbacks.
    TryFinishTreeGrowthAndStartMeshing();
    return;
  }

  const auto children = scene->GetChildren(owner);
  const auto owner_gt = scene->GetDataComponent<GlobalTransform>(owner);
  Entity left_pivot, right_pivot;
  for (const auto& child : children) {
    if (scene->GetEntityName(child) == "Left Pivot") {
      left_pivot = child;
    } else if (scene->GetEntityName(child) == "Right Pivot") {
      right_pivot = child;
    }
  }

  const float progress = simulated_time / target_simulation_time;

  // Snapshot scheduled exports at the current clock before integrating this frame, so t=0
  // (and every later slot) is taken before PhysicsStep advances the strands.
  TryAutomatedExportsUpTo(simulated_time);

  switch (demo_type) {
    case DemoType::LogBreak: {
      const float board_distance = static_cast<float>(log_experiment_setup_settings.rod_segment_count) *
                                   log_experiment_setup_settings.segment_length;
      auto left_operator_root_transform = GlobalTransform();
      auto right_operator_root_transform = GlobalTransform();
      ApplyLogBreakPivotTransforms(owner_gt, dts->initialize_parameters.root_transform, board_distance, progress,
                                   target_factor0, target_factor1, 1.f, left_operator_root_transform,
                                   right_operator_root_transform);
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
      dts->PhysicsStep(physics_parameters);
    } break;
    case DemoType::LogCutUprightBunny: {
      const float board_distance = static_cast<float>(log_experiment_setup_settings.rod_segment_count) *
                                   log_experiment_setup_settings.segment_length;
      GlobalTransform lower_operator_root_transform = GlobalTransform();
      GlobalTransform upper_operator_root_transform = GlobalTransform();
      ApplyLogCutUprightPivotTransforms(owner_gt, dts->initialize_parameters.root_transform, board_distance,
                                        simulated_time, lower_operator_root_transform, upper_operator_root_transform);
      scene->SetDataComponent(left_pivot, lower_operator_root_transform);
      scene->SetDataComponent(right_pivot, upper_operator_root_transform);
      dts->PhysicsStep(physics_parameters);
    } break;
    case DemoType::LogCut:
    case DemoType::LogBreakComparison:
    case DemoType::LogBreakComparisonHalfCutoffDoubleAlpha:
    case DemoType::LogSpoon: {
      const float board_distance = static_cast<float>(log_experiment_setup_settings.rod_segment_count) *
                                   log_experiment_setup_settings.segment_length;
      auto left_operator_root_transform = GlobalTransform();
      auto right_operator_root_transform = GlobalTransform();
      ApplyLogExperimentPivotTransforms(owner_gt, dts->initialize_parameters.root_transform, board_distance,
                                        simulated_time, left_operator_root_transform, right_operator_root_transform);
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
      dts->PhysicsStep(physics_parameters);
    } break;
    case DemoType::BoardBreak: {
      const float board_distance = static_cast<float>(board_experiment_setup_settings.rod_dimension.z) *
                                   board_experiment_setup_settings.segment_length;
      const float left_distance = board_distance * 0.5f * progress * target_factor0;
      const float right_distance = board_distance * (1.f - 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(left_distance, 0, 0)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(right_distance, 0, 0)));
      const float angle = glm::acos(1.f - progress * target_factor1);
      left_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, -angle)));
      right_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, angle)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
      dts->PhysicsStep(physics_parameters);
    } break;
    case DemoType::TwistingBreak: {
      const float board_distance = static_cast<float>(board_experiment_setup_settings.rod_dimension.z) *
                                   board_experiment_setup_settings.segment_length;
      const float left_distance = board_distance * 0.5f * progress * target_factor0;
      const float right_distance = board_distance * (1.f - 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(left_distance, 0, 0)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(right_distance, 0, 0)));
      const float angle = glm::acos(1.f - progress * target_factor1);
      left_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(-angle, 0, 0)));
      right_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(angle, 0, 0)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
      dts->PhysicsStep(physics_parameters);
    } break;
    case DemoType::BendingBreak: {
      const float board_distance = static_cast<float>(board_experiment_setup_settings.rod_dimension.z) *
                                   board_experiment_setup_settings.segment_length;
      const float left_distance = board_distance * 0.5f * progress * target_factor0;
      const float right_distance = board_distance * (1.f - 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(left_distance, 0, 0)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(right_distance, 0, 0)));
      const float angle = glm::acos(1.f - progress * target_factor1);
      left_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, -angle)));
      right_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(0, 0, angle)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
      dts->PhysicsStep(physics_parameters);
    } break;
    case DemoType::ShearingBreak: {
      const float board_distance = static_cast<float>(board_experiment_setup_settings.rod_dimension.z) *
                                   board_experiment_setup_settings.segment_length;
      const float left_distance = board_distance * 0.5f * progress * target_factor0;
      const float right_distance = board_distance * (1.f - 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(dts->initialize_parameters.root_transform.TransformPoint(
          glm::vec3(left_distance, -board_distance * 0.5f * progress * target_factor1, 0)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(dts->initialize_parameters.root_transform.TransformPoint(
          glm::vec3(right_distance, board_distance * 0.5f * progress * target_factor1, 0)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
      dts->PhysicsStep(physics_parameters);
    } break;
    case DemoType::StretchingBreak: {
      const float board_distance = static_cast<float>(board_experiment_setup_settings.rod_dimension.z) *
                                   board_experiment_setup_settings.segment_length;
      const float left_distance = board_distance * 0.5f * progress * target_factor0;
      const float right_distance = board_distance * (1.f - 0.5f * progress * target_factor0);

      auto left_operator_root_transform = GlobalTransform();
      left_operator_root_transform.SetPosition(dts->initialize_parameters.root_transform.TransformPoint(
          glm::vec3(left_distance - board_distance * 0.5f * progress * target_factor1, 0.f, 0.f)));
      auto right_operator_root_transform = GlobalTransform();
      right_operator_root_transform.SetPosition(dts->initialize_parameters.root_transform.TransformPoint(
          glm::vec3(right_distance + board_distance * 0.5f * progress * target_factor1, 0.f, 0.f)));
      scene->SetDataComponent(left_pivot, left_operator_root_transform);
      scene->SetDataComponent(right_pivot, right_operator_root_transform);
      dts->PhysicsStep(physics_parameters);
    } break;
    case DemoType::SapHeart: {
      const float log_distance = static_cast<float>(log_experiment_setup_settings.rod_segment_count) *
                                 log_experiment_setup_settings.segment_length;
      const float left_distance = -log_distance * 0.5f * progress * target_factor0;
      auto leaf_operator_root_transform = GlobalTransform();
      leaf_operator_root_transform.SetPosition(
          dts->initialize_parameters.root_transform.TransformPoint(glm::vec3(left_distance, 0, 0)));
      const float angle = glm::acos(1.f - progress * target_factor1);
      leaf_operator_root_transform.SetRotation(owner_gt.GetRotation() * glm::quat(glm::vec3(angle, 0, 0)));
      scene->SetDataComponent(left_pivot, leaf_operator_root_transform);
      dts->PhysicsStep(physics_parameters);
      break;
    }
    case DemoType::BoardCollision: {
      GlobalTransform gt = object_initial_pose;
      const auto temp_entity = temp_entity1_ref.Get();
      gt.SetPosition(object_initial_pose.GetPosition() + glm::vec3(0, -50, 0) * progress);
      scene->SetDataComponent(temp_entity, gt);
      dts->PhysicsStep(physics_parameters);
      break;
    }
    case DemoType::TrunkStrength: {
      GlobalTransform gt = tree_initial_pose;
      const float real_progress = glm::clamp(simulated_time / (target_simulation_time * .005f), 0.f, 1.f);
      gt.SetEulerRotation(glm::radians(glm::vec3(0, glm::pow(real_progress, 2.f) * 180.f, 0)));
      // Root pivot lives on PhysicsDemo's DTS owner.
      scene->SetDataComponent(owner, gt);
      const auto tree_entity = tree_entity_ref.Get();
      if (scene->IsEntityValid(tree_entity)) {
        scene->SetDataComponent(tree_entity, gt);
      }
      dts->PhysicsStep(physics_parameters);
      break;
    }
    case DemoType::Wind: {
      const float real_progress = glm::clamp(simulated_time / (target_simulation_time * 0.05f), 0.f, 1.f);
      dts->wind->enabled = true;
      dts->wind->main_force = glm::vec3((simulated_time > 1.f ? 0.f : -target_factor0 * real_progress), 0, 0);
      dts->PhysicsStep(physics_parameters);
      break;
    }
    case DemoType::TreeCollision: {
      GlobalTransform gt = object_initial_pose;
      const auto temp_entity = temp_entity1_ref.Get();
      const float real_progress = glm::clamp((simulated_time - .5f) / (target_simulation_time * 0.02f), 0.f, 1.f);
      gt.SetPosition(object_initial_pose.GetPosition() + glm::vec3(2, 0, 0) * real_progress);
      scene->SetDataComponent(temp_entity, gt);
      dts->PhysicsStep(physics_parameters);
      break;
    }
    case DemoType::Fungus: {
      dts->PhysicsStep(physics_parameters);
      break;
    }
    case DemoType::SmallTrunk:
    case DemoType::NormalTrunk:
    case DemoType::StockyTrunk:
    case DemoType::OakThickStump:
    case DemoType::OakThickStump200:
    case DemoType::OakThickStump300:
    case DemoType::OakThickStump400:
    case DemoType::OakTwoYearSparse:
    case DemoType::OakTwoYear20:
    case DemoType::OakTwoYear50:
    case DemoType::OakTwoYear100:
    case DemoType::OakThreeYear4:
    case DemoType::OakThreeYear10:
    case DemoType::OakThreeYear20:
    case DemoType::OakThreeYear50:
    case DemoType::OakFourYearSparse:
    case DemoType::OakSixYearSparse:
    case DemoType::OakEightYearSparse: {
      dts->PhysicsStep(physics_parameters);
      break;
    }
    default:
      break;
  }

  simulated_time += physics_parameters.time_step;
}

void DynamicStrandsDemo::SnapshotAutomatedExportSettings() {
  automated_export_snapshot_ = automated_export;
  automated_export_meshes_snapshot_ = automated_export_meshes;
  automated_export_heatmaps_snapshot_ = automated_export_heatmaps;
  automated_export_csvs_snapshot_ = automated_export_csvs;
  automated_export_lower_snapshot_ = automated_export_lower;
  automated_export_upper_snapshot_ = automated_export_upper;
  automated_export_stepsize_snapshot_ = automated_export_stepsize;
}

bool DynamicStrandsDemo::AutomatedExportSettingsChanged() const {
  return automated_export != automated_export_snapshot_ ||
         automated_export_meshes != automated_export_meshes_snapshot_ ||
         automated_export_heatmaps != automated_export_heatmaps_snapshot_ ||
         automated_export_csvs != automated_export_csvs_snapshot_ ||
         automated_export_lower != automated_export_lower_snapshot_ ||
         automated_export_upper != automated_export_upper_snapshot_ ||
         automated_export_stepsize != automated_export_stepsize_snapshot_;
}

void DynamicStrandsDemo::AdvanceAutomatedExportScheduleFrom(const float current_time) {
  constexpr float kEps = 1.0e-4f;
  if (automated_export_stepsize <= 0.f) {
    return;
  }
  if (current_time + kEps < automated_export_lower) {
    next_automated_export_time_ = automated_export_lower;
    return;
  }

  const float relative = current_time - automated_export_lower;
  const float completed_steps = std::floor(relative / automated_export_stepsize + kEps);
  float next = automated_export_lower + (completed_steps + 1.f) * automated_export_stepsize;

  // Rescheduling at simulation start must still include the lower-bound export (t=0).
  if (next_automated_export_time_ <= automated_export_lower + kEps && current_time <= automated_export_lower + kEps) {
    next = automated_export_lower;
  }

  if (next > automated_export_upper + kEps) {
    next = automated_export_upper + 1.0e6f;
  }
  next_automated_export_time_ = next;
}

void DynamicStrandsDemo::ResetAutomatedExportSchedule() {
  next_automated_export_time_ = automated_export_lower;
  automated_export_folder_.clear();
  automated_kinetic_meshlet_columns_.clear();
  automated_alpha_tet_columns_.clear();
  SnapshotAutomatedExportSettings();
}

const char* DynamicStrandsDemo::DemoTypeExportFolderName(const DemoType type) {
  switch (type) {
    case DemoType::LogBreak:
      return "Log break";
    case DemoType::BoardBreak:
      return "Board break";
    case DemoType::TwistingBreak:
      return "Twisting break";
    case DemoType::BendingBreak:
      return "Bending break";
    case DemoType::ShearingBreak:
      return "Shearing break";
    case DemoType::StretchingBreak:
      return "Stretching break";
    case DemoType::SapHeart:
      return "SapHeart";
    case DemoType::BoardCollision:
      return "Board Collision";
    case DemoType::TrunkStrength:
      return "Trunk Strength";
    case DemoType::Wind:
      return "Wind";
    case DemoType::TreeCollision:
      return "Tree Collision";
    case DemoType::TreeBreak:
      return "Tree Break";
    case DemoType::Fungus:
      return "Fungus";
    case DemoType::SmallTrunk:
      return "Small Trunk";
    case DemoType::NormalTrunk:
      return "Normal Trunk";
    case DemoType::StockyTrunk:
      return "Stocky Trunk";
    case DemoType::OakThickStump:
      return "Oak thick stump";
    case DemoType::OakThickStump200:
      return "Oak thick stump 200";
    case DemoType::OakThickStump300:
      return "Oak thick stump 300";
    case DemoType::OakThickStump400:
      return "Oak thick stump 400";
    case DemoType::OakTwoYearSparse:
      return "Oak 2 years 10 strands";
    case DemoType::OakTwoYear20:
      return "Oak 2 years 20 strands";
    case DemoType::OakTwoYear50:
      return "Oak 2 years 50 strands";
    case DemoType::OakTwoYear100:
      return "Oak 2 years 100 strands";
    case DemoType::OakThreeYear4:
      return "Oak 3 years 4 strands";
    case DemoType::OakThreeYear10:
      return "Oak 3 years 10 strands";
    case DemoType::OakThreeYear20:
      return "Oak 3 years 20 strands";
    case DemoType::OakThreeYear50:
      return "Oak 3 years 50 strands";
    case DemoType::OakFourYearSparse:
      return "Oak 4 years 4 strands";
    case DemoType::OakSixYearSparse:
      return "Oak 6 years 4 strands";
    case DemoType::OakEightYearSparse:
      return "Oak 8 years 4 strands";
    case DemoType::LogCut:
      return "Log cut";
    case DemoType::LogBreakComparison:
      return "Log break comparison";
    case DemoType::LogBreakComparisonHalfCutoffDoubleAlpha:
      return "Log break comparison half cutoff double alpha";
    case DemoType::LogSpoon:
      return "Log Spoon";
    case DemoType::LogCutUprightBunny:
      return "Log cut upright + bunny";
    case DemoType::Empty:
    default:
      return "Unknown";
  }
}

std::shared_ptr<DynamicTreeStrands> DynamicStrandsDemo::GetActiveDynamicTreeStrands() {
  // Tree-descriptor demos reuse PhysicsDemo's DynamicTreeStrands rather than attaching one to the Tree.
  return GetScene()->GetOrSetPrivateComponent<DynamicTreeStrands>(GetOwner()).lock();
}

namespace {
constexpr int kVolumeCsvDigits = std::numeric_limits<double>::max_digits10;

void WriteVolumeCsvDouble(std::ostream& stream, const double value) {
  stream << std::defaultfloat << std::setprecision(kVolumeCsvDigits) << value;
}

/// Alpha per-tet cells: dead/invalid → exact "-1"; otherwise full-precision volume.
void WriteAlphaTetVolumeCsvCell(std::ostream& stream, const double value) {
  if (value < 0.0) {
    stream << "-1";
    return;
  }
  WriteVolumeCsvDouble(stream, value);
}

void AppendAutomatedVolumeCsv(const std::filesystem::path& path, const std::string& label, const double simulation_time,
                              const double cumulative_volume, const double initial_cumulative_volume) {
  const bool write_header = !std::filesystem::exists(path) || std::filesystem::file_size(path) == 0;
  std::ofstream stream(path, std::ios::out | std::ios::app);
  if (!stream) {
    EVOENGINE_ERROR("Automated export: failed to open volume CSV " << path.string());
    return;
  }
  if (write_header) {
    stream << "time,label,cumulative_volume,initial_cumulative_volume,percent_of_initial\n";
  }
  double percent = 0.0;
  if (initial_cumulative_volume > 1e-18) {
    percent = 100.0 * cumulative_volume / initial_cumulative_volume;
  }
  WriteVolumeCsvDouble(stream, simulation_time);
  stream << ',' << label << ',';
  WriteVolumeCsvDouble(stream, cumulative_volume);
  stream << ',';
  WriteVolumeCsvDouble(stream, initial_cumulative_volume);
  stream << ',';
  WriteVolumeCsvDouble(stream, percent);
  stream << '\n';
}

void EnsureKineticPerMeshletVolumeCsv(const std::filesystem::path& path, std::vector<unsigned int>& columns,
                                      const std::unordered_map<unsigned int, double>& initial_volumes_by_segment) {
  if (!columns.empty()) {
    return;
  }
  columns.reserve(initial_volumes_by_segment.size());
  for (const auto& [segment_index, _] : initial_volumes_by_segment) {
    columns.push_back(segment_index);
  }
  std::sort(columns.begin(), columns.end());
  if (columns.empty()) {
    return;
  }

  std::ofstream stream(path, std::ios::out | std::ios::trunc);
  if (!stream) {
    EVOENGINE_ERROR("Automated export: failed to create per-meshlet volume CSV " << path.string());
    columns.clear();
    return;
  }
  stream << "time";
  for (const unsigned int id : columns) {
    stream << ',' << id;
  }
  stream << '\n';
  WriteVolumeCsvDouble(stream, 0.0);
  for (const unsigned int id : columns) {
    const auto it = initial_volumes_by_segment.find(id);
    stream << ',';
    WriteVolumeCsvDouble(stream, it != initial_volumes_by_segment.end() ? it->second : 0.0);
  }
  stream << '\n';
  EVOENGINE_LOG("Automated export: wrote initial row for " << path.string() << " (" << columns.size() << " meshlets).");
}

void AppendKineticPerMeshletVolumeCsv(const std::filesystem::path& path, const std::vector<unsigned int>& columns,
                                      const double simulation_time,
                                      const DsKineticVoronoiVolumeUtils::VolumeResult& volumes) {
  if (columns.empty()) {
    return;
  }
  std::unordered_map<unsigned int, double> by_segment;
  by_segment.reserve(volumes.meshlets.size());
  for (const auto& entry : volumes.meshlets) {
    by_segment[entry.segment_index] = entry.volume;
  }
  std::ofstream stream(path, std::ios::out | std::ios::app);
  if (!stream) {
    EVOENGINE_ERROR("Automated export: failed to append per-meshlet volume CSV " << path.string());
    return;
  }
  WriteVolumeCsvDouble(stream, simulation_time);
  for (const unsigned int id : columns) {
    const auto it = by_segment.find(id);
    stream << ',';
    WriteVolumeCsvDouble(stream, it != by_segment.end() ? it->second : 0.0);
  }
  stream << '\n';
}

void EnsureAlphaPerTetVolumeCsv(const std::filesystem::path& path, std::vector<size_t>& columns,
                                const std::vector<double>& initial_tet_volumes) {
  if (!columns.empty()) {
    return;
  }
  columns.resize(initial_tet_volumes.size());
  for (size_t i = 0; i < columns.size(); ++i) {
    columns[i] = i;
  }
  if (columns.empty()) {
    return;
  }

  std::ofstream stream(path, std::ios::out | std::ios::trunc);
  if (!stream) {
    EVOENGINE_ERROR("Automated export: failed to create per-tet volume CSV " << path.string());
    columns.clear();
    return;
  }
  stream << "time";
  for (const size_t id : columns) {
    stream << ',' << id;
  }
  stream << '\n';
  // Dead tets are stored as -1; near-degenerate-at-init keep measured absolute volumes.
  WriteVolumeCsvDouble(stream, 0.0);
  for (const size_t id : columns) {
    stream << ',';
    WriteAlphaTetVolumeCsvCell(stream, id < initial_tet_volumes.size()
                                           ? initial_tet_volumes[id]
                                           : DsAlphaShapeVolumeUtils::kDeadTetrahedronVolumeSentinel);
  }
  stream << '\n';
  EVOENGINE_LOG("Automated export: wrote initial row for " << path.string() << " (" << columns.size() << " tets).");
}

void AppendAlphaPerTetVolumeCsv(const std::filesystem::path& path, const std::vector<size_t>& columns,
                                const double simulation_time, const std::vector<double>& per_tet_volume) {
  if (columns.empty()) {
    return;
  }
  std::ofstream stream(path, std::ios::out | std::ios::app);
  if (!stream) {
    EVOENGINE_ERROR("Automated export: failed to append per-tet volume CSV " << path.string());
    return;
  }
  WriteVolumeCsvDouble(stream, simulation_time);
  for (const size_t id : columns) {
    // Dead / invalid tets are reported as -1; callers treat that as volume 0 when evaluating.
    const double value =
        id < per_tet_volume.size() ? per_tet_volume[id] : DsAlphaShapeVolumeUtils::kDeadTetrahedronVolumeSentinel;
    stream << ',';
    WriteAlphaTetVolumeCsvCell(stream, value);
  }
  stream << '\n';
}
}  // namespace

void DynamicStrandsDemo::TryAutomatedExportsUpTo(const float time) {
  if (AutomatedExportSettingsChanged()) {
    if (automated_export && demo_status == DemoStatus::Simulation && demo_type != DemoType::Empty) {
      AdvanceAutomatedExportScheduleFrom(time);
    }
    SnapshotAutomatedExportSettings();
  }
  if (!automated_export || demo_status != DemoStatus::Simulation || demo_type == DemoType::Empty) {
    return;
  }
  if (automated_export_stepsize <= 0.f) {
    return;
  }
  if (!automated_export_meshes && !automated_export_heatmaps && !automated_export_csvs) {
    return;
  }

  constexpr float kEps = 1.0e-4f;
  while (next_automated_export_time_ <= automated_export_upper + kEps && time + kEps >= next_automated_export_time_) {
    const float export_time = next_automated_export_time_;
    next_automated_export_time_ += automated_export_stepsize;

    const auto dts = GetActiveDynamicTreeStrands();
    if (!dts || !dts->dynamic_strands) {
      EVOENGINE_ERROR("Automated export: no DynamicTreeStrands available.");
      continue;
    }
    auto* dskvm = dts->dynamic_strands->GetKineticVoronoiMeshing();
    auto* dsasm = dts->dynamic_strands->GetAlphaShapeMeshing();
    if (!dskvm && !dsasm) {
      EVOENGINE_ERROR("Automated export: neither Kinetic nor Alpha meshing is available.");
      continue;
    }

    dts->dynamic_strands->Download();

    const auto project_path = ProjectManager::GetProjectPath();
    if (project_path.empty()) {
      EVOENGINE_ERROR("Automated export: project path is empty.");
      continue;
    }
    if (automated_export_folder_.empty()) {
      automated_export_folder_ = project_path.parent_path() / "PhysicsDemoExports" /
                                 DemoTypeExportFolderName(demo_type) / MakeAutomatedExportTimestamp();
      EVOENGINE_LOG("Automated export folder: " << automated_export_folder_.string());
    }
    std::error_code ec;
    std::filesystem::create_directories(automated_export_folder_, ec);
    if (ec) {
      EVOENGINE_ERROR("Automated export: failed to create folder " << automated_export_folder_.string() << " ("
                                                                   << ec.message() << ").");
      continue;
    }

    std::ostringstream time_suffix;
    time_suffix << std::fixed << std::setprecision(3) << export_time;

    const auto heatmap_scale =
        VolumeChangeHeatmapExport::ColorScale::Symmetric(VolumeChangeHeatmapExport::max_abs_percent);

    if (dskvm) {
      if (dskvm->segment_meshlet_vertices.empty() || dskvm->segment_meshlet_triangles.empty()) {
        EVOENGINE_ERROR("Automated export: Kinetic meshlet buffers are empty at t=" << export_time << ".");
      } else {
        // Match ExportObj: optional export smoothing first, then mesh/volume/heatmap from the same verts.
        std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex> export_vertices = dskvm->segment_meshlet_vertices;
        std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle> export_triangles =
            dskvm->segment_meshlet_triangles;
        if (MeshletObjExport::enable_smoothing &&
            (automated_export_meshes || automated_export_heatmaps || automated_export_csvs)) {
          MeshletObjExport::ApplySmoothing(export_vertices, export_triangles, dts->dynamic_strands->segments,
                                           dts->dynamic_strands->segment_pairs,
                                           dts->dynamic_strands->segment_data_list);
        }

        if (automated_export_meshes) {
          const auto mesh_path = automated_export_folder_ / ("kinetic_meshlets_" + time_suffix.str() + ".obj");
          try {
            const bool previous_smoothing = MeshletObjExport::enable_smoothing;
            MeshletObjExport::enable_smoothing = false;  // already applied above
            MeshletObjExport::ExportObj(
                mesh_path, export_vertices, export_triangles, dts->dynamic_strands->segments,
                DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.uv_height_factor,
                DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.uv_circum_factor,
                DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.fracture_distance,
                dts->dynamic_strands->segment_pairs, dts->dynamic_strands->segment_data_list,
                dskvm->segment_meshlet_vertex_metadata, dskvm->segment_meshlet_face_metadata);
            MeshletObjExport::enable_smoothing = previous_smoothing;
            EVOENGINE_LOG("Automated export: wrote " << mesh_path.string());
          } catch (const std::exception& ex) {
            EVOENGINE_ERROR("Automated export failed for " << mesh_path.string() << ": " << ex.what());
          }
        }

        if (automated_export_csvs || automated_export_heatmaps) {
          const auto volumes =
              DsKineticVoronoiVolumeUtils::ComputeAllMeshletVolumes(export_vertices, export_triangles, true);
          if (automated_export_csvs) {
            AppendAutomatedVolumeCsv(automated_export_folder_ / "kinetic_volume_measure.csv", "kinetic", export_time,
                                     volumes.cumulative_volume, dskvm->initial_meshlet_cumulative_volume_);
          }
          if (dskvm->has_initial_meshlet_volumes_) {
            if (automated_export_csvs) {
              const auto per_meshlet_csv = automated_export_folder_ / "kinetic_meshlet_volumes.csv";
              EnsureKineticPerMeshletVolumeCsv(per_meshlet_csv, automated_kinetic_meshlet_columns_,
                                               dskvm->initial_meshlet_volumes_by_segment_);
              AppendKineticPerMeshletVolumeCsv(per_meshlet_csv, automated_kinetic_meshlet_columns_, export_time,
                                               volumes);
            }
            if (automated_export_heatmaps) {
              const auto heatmap_path =
                  automated_export_folder_ / ("kinetic_volume_change_heatmap_" + time_suffix.str() + ".obj");
              try {
                VolumeChangeHeatmapExport::ExportKineticMeshlets(
                    heatmap_path, export_vertices, export_triangles, dskvm->initial_meshlet_volumes_by_segment_,
                    dskvm->initial_meshlet_cumulative_volume_, nullptr, &heatmap_scale);
                EVOENGINE_LOG("Automated export: wrote " << heatmap_path.string());
              } catch (const std::exception& ex) {
                EVOENGINE_ERROR("Automated export heatmap failed for " << heatmap_path.string() << ": " << ex.what());
              }
            }
          } else if (automated_export_csvs || automated_export_heatmaps) {
            EVOENGINE_WARNING(
                "Automated export: no Kinetic initial meshlet volumes; skipping heatmap/per-meshlet CSV at t="
                << export_time << ".");
          }
        }
      }
    }

    if (dsasm) {
      if (dsasm->uniform_particles.empty() || dsasm->delaunay_tetrahedrons.empty()) {
        EVOENGINE_ERROR("Automated export: Alpha tetrahedra buffers are empty at t=" << export_time << ".");
      } else {
        if (automated_export_meshes) {
          const auto mesh_path = automated_export_folder_ / ("alpha_tetrahedra_" + time_suffix.str() + ".obj");
          try {
            AlphaShapeTetObjExport::ExportObj(mesh_path, dsasm->uniform_particles, dsasm->delaunay_tetrahedrons,
                                              &dsasm->profile_bundle_boundary_polygons_);
            EVOENGINE_LOG("Automated export: wrote " << mesh_path.string());
          } catch (const std::exception& ex) {
            EVOENGINE_ERROR("Automated export failed for " << mesh_path.string() << ": " << ex.what());
          }
        }

        if (automated_export_csvs || automated_export_heatmaps) {
          auto volumes = DsAlphaShapeVolumeUtils::ComputeTetrahedronVolumes(
              dsasm->uniform_particles, dsasm->delaunay_tetrahedrons, AlphaShapeTetObjExport::use_current_position);
          if (automated_export_csvs) {
            double cumulative = 0.0;
            for (size_t tet_id = 0; tet_id < volumes.per_tet_volume.size(); ++tet_id) {
              if (tet_id < dsasm->initial_near_degenerate_tets_.size() &&
                  dsasm->initial_near_degenerate_tets_[tet_id]) {
                continue;
              }
              cumulative += DsAlphaShapeVolumeUtils::EffectiveVolume(volumes.per_tet_volume[tet_id]);
            }
            AppendAutomatedVolumeCsv(automated_export_folder_ / "alpha_volume_measure.csv", "alpha", export_time,
                                     cumulative, dsasm->initial_tet_cumulative_volume_);
          }
          if (dsasm->has_initial_tet_volumes_) {
            if (automated_export_csvs) {
              const auto per_tet_csv = automated_export_folder_ / "alpha_tetrahedron_volumes.csv";
              EnsureAlphaPerTetVolumeCsv(per_tet_csv, automated_alpha_tet_columns_, dsasm->initial_tet_volumes_);
              AppendAlphaPerTetVolumeCsv(per_tet_csv, automated_alpha_tet_columns_, export_time,
                                         volumes.per_tet_volume);
            }
            if (automated_export_heatmaps) {
              const auto heatmap_path =
                  automated_export_folder_ / ("alpha_volume_change_heatmap_" + time_suffix.str() + ".obj");
              try {
                VolumeChangeHeatmapExport::ExportAlphaTetrahedra(
                    heatmap_path, dsasm->uniform_particles, dsasm->delaunay_tetrahedrons, dsasm->initial_tet_volumes_,
                    dsasm->initial_near_degenerate_tets_, dsasm->initial_tet_cumulative_volume_,
                    AlphaShapeTetObjExport::use_current_position, nullptr, &heatmap_scale);
                EVOENGINE_LOG("Automated export: wrote " << heatmap_path.string());
              } catch (const std::exception& ex) {
                EVOENGINE_ERROR("Automated export heatmap failed for " << heatmap_path.string() << ": " << ex.what());
              }
            }
          } else if (automated_export_csvs || automated_export_heatmaps) {
            EVOENGINE_WARNING("Automated export: no Alpha initial tet volumes; skipping heatmap/per-tet CSV at t="
                              << export_time << ".");
          }
        }
      }
    }
  }
}