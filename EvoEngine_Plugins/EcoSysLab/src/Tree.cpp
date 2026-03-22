//
// Created by lllll on 10/24/2022.
//
#include "Tree.hpp"
#include <Material.hpp>
#include <Mesh.hpp>
#include <TransformGraph.hpp>
#include "BasicShootDescriptor.hpp"
#include "SkeletonSerializer.hpp"
#include "StrandGroupSerializer.hpp"

#include "Application.hpp"
#include "Climate.hpp"
#include "EcoSysLabLayer.hpp"
#include "EditorLayer.hpp"
#include "Octree.hpp"
#include "Soil.hpp"
#include "StrandModelProfileSerializer.hpp"
#include "assimp/contrib/zip/src/miniz.h"

using namespace eco_sys_lab_plugin;

static void CalculateSourceSinkStrengthCombined(ShootModel* shoot_model, RootModel* root_model,
                                                const ShootGrowthController* shoot_growth_controller,
                                                const FoliageController* foliage_controller,
                                                const ShootReproductionController* reproduction_controller,
                                                const RootGrowthController* root_growth_controller,
                                                const ClimateModel* climate_model, float delta_time) {
  if (shoot_model && shoot_growth_controller) {
    shoot_model->CalculateSourceSinkStrength(
        *shoot_growth_controller, foliage_controller ? *foliage_controller : FoliageController{},
        reproduction_controller ? *reproduction_controller : ShootReproductionController{}, *climate_model, delta_time);
  }
  if (root_model && root_growth_controller && climate_model) {
    root_model->CalculateSourceSinkStrength(*root_growth_controller, *climate_model, delta_time);
  }
}

// --------------------------------------------------------------------------------
// [NEW] Unified Coupled Solver Structures
// --------------------------------------------------------------------------------
struct SolverNodeData {
  float pressure = 0.0f;       // Current pressure (v)
  float next_pressure = 0.0f;  // v at t+1
  float capacity = 1.0f;       // C
  float conductance = 0.0f;    // K (to parent)

  // Folding coefficients: v_child = A * v_parent + B
  float A = 0.0f;
  float B = 0.0f;

  float G_sum = 0.0f;  // Diagonal
  float RHS = 0.0f;    // Right Hand Side
};

// --------------------------------------------------------------------------------
// Helper: Folding Logic (Generic)
// --------------------------------------------------------------------------------
// Prepares G_sum and RHS for a single node, including its children's folded data.
// Returns {A, B} for this node to pass to its parent.
static std::pair<float, float> FoldNode(SolverNodeData& node_i, float carbohydrate_source, float carbohydrate_sink,
                                        float delta_time, float theta, float eps, float K_parent, float v_parent_old) {
  // 1. Base Equation Setup
  const float dt_term = node_i.capacity / delta_time;
  node_i.G_sum += dt_term;  // Add to existing G_sum (which might have children's A terms)
  node_i.RHS += dt_term * node_i.pressure;

  // Add Net Source/Sink
  float net_mass_change = carbohydrate_source - carbohydrate_sink;
  node_i.RHS += net_mass_change / delta_time;

  // 2. Parent Influence
  // Implicit Part (Theta)
  node_i.G_sum += theta * K_parent;
  // Explicit Part (1-Theta)
  node_i.RHS += (1.0f - theta) * K_parent * (v_parent_old - node_i.pressure);

  // 3. Compute A/B Coefficients
  float A = 0.0f;
  float B = 0.0f;
  if (node_i.G_sum > eps) {
    A = (theta * K_parent) / node_i.G_sum;
    B = node_i.RHS / node_i.G_sum;
  } else {
    B = node_i.pressure;
  }
  return {A, B};
}

// Add module-level season parameters (default: active season = Mar 1 (60) to Nov 30 (334))
static int season_start_day = 60;  // inclusive
static int season_end_day   = 334; // inclusive

static void SolveCoupledSystemCrankNicolson(ShootModel* shoot_model, RootModel* root_model, const ClimateModel& climate_model, float delta_time) {

  // Calendar-based transport gating via day-of-year
  const float days = climate_model.time * 365.0f;
  const int day_in_year = static_cast<int>(glm::floor(glm::mod(days, 365.0f)));
  const bool in_active_season = (season_start_day <= season_end_day)
                                ? (day_in_year >= season_start_day && day_in_year <= season_end_day)
                                : (day_in_year >= season_start_day || day_in_year <= season_end_day); // supports wrap-around

  if (!in_active_season) {
    if (shoot_model) {
      for (auto& n : shoot_model->RefShootSkeleton().RefRawNodes()) {
        n.data.conductance = 0.0f;
      }
    }
    if (root_model) {
      for (auto& n : root_model->RefRootSkeleton().RefRawNodes()) {
        n.data.conductance = 0.0f;
      }
    }
  }

  // --- STABILITY FIX ---
  const float theta = 1.0f;
  const float eps = 1e-9f;

  // --- 1. Data Preparation ---
  static std::vector<SolverNodeData> shoot_solver_data;
  static std::vector<SolverNodeData> root_solver_data;

  auto prepare_data = [&](auto& skeleton, std::vector<SolverNodeData>& buffer) {
    auto& raw = skeleton.RefRawNodes();
    if (buffer.size() < raw.size())
      buffer.resize(raw.size());
    for (size_t i = 0; i < raw.size(); ++i) {
      auto& b = raw[i];
      auto& s = buffer[i];
      s.capacity = glm::max(b.data.max_carbohydrate_mass, 1e-6f);
      s.conductance = std::isfinite(b.data.conductance) && b.data.conductance > 0.0f ? b.data.conductance : 0.0f;
      s.pressure = s.capacity > 0.0f ? (b.data.carbohydrate_mass / s.capacity) : 0.0f;
      if (!std::isfinite(s.pressure))
        s.pressure = 0.0f;
      s.G_sum = 0.0f;
      s.RHS = 0.0f;
      s.A = 0.0f;
      s.B = 0.0f;
      s.next_pressure = s.pressure;
    }
  };

  // Find ALL base nodes (nodes with no parent)
  auto find_base_indices = [&](auto& skeleton) -> std::vector<int> {
    std::vector<int> base_indices;
    auto& raw = skeleton.RefRawNodes();
    const auto& sorted = skeleton.PeekSortedNodeList();
    for (auto h : sorted) {
      if (raw[h].GetParentHandle() == -1)
        base_indices.push_back(static_cast<int>(h));
    }
    return base_indices;
  };

  if (shoot_model)
    prepare_data(shoot_model->RefShootSkeleton(), shoot_solver_data);
  if (root_model)
    prepare_data(root_model->RefRootSkeleton(), root_solver_data);

  // Get all base indices for both skeletons
  std::vector<int> shoot_base_indices;
  std::vector<int> root_base_indices;
  
  if (shoot_model && !shoot_model->RefShootSkeleton().RefRawNodes().empty()) {
    shoot_base_indices = find_base_indices(shoot_model->RefShootSkeleton());
  }
  if (root_model && !root_model->RefRootSkeleton().RefRawNodes().empty()) {
    root_base_indices = find_base_indices(root_model->RefRootSkeleton());
  }

  // For backward compatibility, keep single index references (use first base if available)
  const int shoot_base_idx = shoot_base_indices.empty() ? -1 : shoot_base_indices[0];
  const int root_base_idx = root_base_indices.empty() ? -1 : root_base_indices[0];

  // --- 2. Phase A: Fold Shoot (Leaves -> Trunk Base) ---
  // Store folded coefficients for each shoot base node
  std::vector<float> shoot_base_A_list;
  std::vector<float> shoot_base_B_list;
  std::vector<float> K_interface_list;

  if (shoot_model && !shoot_model->RefShootSkeleton().RefRawNodes().empty()) {
    auto& skeleton = shoot_model->RefShootSkeleton();
    const auto& sorted = skeleton.PeekSortedNodeList();
    auto& raw = skeleton.RefRawNodes();

    for (auto it = sorted.rbegin(); it != sorted.rend(); ++it) {
      const SkeletonNodeHandle h = *it;
      auto& s_node = shoot_solver_data[h];
      auto& b_node = raw[h];

      float K_p = 0.0f;
      float v_p_old = 0.0f;

      if (b_node.GetParentHandle() != -1) {
        K_p = s_node.conductance;
        v_p_old = shoot_solver_data[b_node.GetParentHandle()].pressure;
      } else if (root_model && !root_base_indices.empty()) {
        // This is a shoot base node - connect to root base(s)
        // For now, connect to first root base (could be extended to weighted average)
        K_p = s_node.conductance;
        v_p_old = root_solver_data[root_base_indices[0]].pressure;
      }

      // Fold
      {
        const float dt_term = s_node.capacity / delta_time;
        s_node.G_sum += dt_term;
        s_node.RHS += dt_term * s_node.pressure;

        const float net_mass_change = b_node.data.carbohydrate_source - b_node.data.carbohydrate_sink;
        s_node.RHS += net_mass_change / delta_time;

        s_node.G_sum += theta * K_p;
        s_node.RHS += (1.0f - theta) * K_p * (v_p_old - s_node.pressure);

        if (s_node.G_sum > eps) {
          s_node.A = (theta * K_p) / s_node.G_sum;
          s_node.B = s_node.RHS / s_node.G_sum;
        } else {
          s_node.A = 0.0f;
          s_node.B = s_node.pressure;
        }
      }

      // Apply to parent or collect base coefficients
      if (b_node.GetParentHandle() != -1) {
        auto& parent_s = shoot_solver_data[b_node.GetParentHandle()];
        const float K_child = s_node.conductance;
        parent_s.G_sum += theta * K_child;
        parent_s.G_sum -= theta * K_child * s_node.A;
        parent_s.RHS += theta * K_child * s_node.B;
        parent_s.RHS -= (1.0f - theta) * K_child * (v_p_old - s_node.pressure);
      } else {
        // This is a base node - store its folded coefficients
        shoot_base_A_list.push_back(s_node.A);
        shoot_base_B_list.push_back(s_node.B);
        K_interface_list.push_back(s_node.conductance);
      }
    }
  }

  // --- 3. Phase B: Fold Root (Tips -> Root Base) ---
  if (root_model && !root_model->RefRootSkeleton().RefRawNodes().empty()) {
    auto& skeleton = root_model->RefRootSkeleton();
    const auto& sorted = skeleton.PeekSortedNodeList();
    auto& raw = skeleton.RefRawNodes();

    // Inject ALL shoot base nodes' folded data into root base nodes
    // Distribute shoot influence across root bases proportionally
    if (shoot_model && !shoot_base_indices.empty() && !root_base_indices.empty()) {
      // Calculate total interface conductance for normalization
      float total_K_interface = 0.0f;
      for (const auto& K : K_interface_list) {
        total_K_interface += K;
      }
      
      // For each root base, inject the combined shoot influence
      for (size_t rb_idx = 0; rb_idx < root_base_indices.size(); ++rb_idx) {
        const int root_base_handle = root_base_indices[rb_idx];
        auto& root_base_s = root_solver_data[root_base_handle];
        const float v_root_old = root_base_s.pressure;
        
        // Weight for this root base (equal distribution among root bases)
        const float root_weight = 1.0f / static_cast<float>(root_base_indices.size());
        
        // Inject influence from all shoot bases
        for (size_t sb_idx = 0; sb_idx < shoot_base_indices.size(); ++sb_idx) {
          const int shoot_base_handle = shoot_base_indices[sb_idx];
          const float v_shoot_old = shoot_solver_data[shoot_base_handle].pressure;
          const float K_interface = K_interface_list[sb_idx] * root_weight;
          const float shoot_A = shoot_base_A_list[sb_idx];
          const float shoot_B = shoot_base_B_list[sb_idx];

          root_base_s.G_sum += theta * K_interface;
          root_base_s.G_sum -= theta * K_interface * shoot_A;
          root_base_s.RHS += theta * K_interface * shoot_B;
          root_base_s.RHS -= (1.0f - theta) * K_interface * (v_root_old - v_shoot_old);
        }
      }
    }

    for (auto it = sorted.rbegin(); it != sorted.rend(); ++it) {
      const SkeletonNodeHandle h = *it;
      auto& s_node = root_solver_data[h];
      auto& b_node = raw[h];

      float K_p = 0.0f;
      float v_p_old = 0.0f;
      if (b_node.GetParentHandle() != -1) {
        K_p = s_node.conductance;
        v_p_old = root_solver_data[b_node.GetParentHandle()].pressure;
      }

      // Fold
      {
        const float dt_term = s_node.capacity / delta_time;
        s_node.G_sum += dt_term;
        s_node.RHS += dt_term * s_node.pressure;

        const float net_mass_change = b_node.data.carbohydrate_source - b_node.data.carbohydrate_sink;
        s_node.RHS += net_mass_change / delta_time;

        s_node.G_sum += theta * K_p;
        s_node.RHS += (1.0f - theta) * K_p * (v_p_old - s_node.pressure);

        if (s_node.G_sum > eps) {
          s_node.A = (theta * K_p) / s_node.G_sum;
          s_node.B = s_node.RHS / s_node.G_sum;
        } else {
          s_node.A = 0.0f;
          s_node.B = s_node.pressure;
        }
      }

      // Apply to parent
      if (b_node.GetParentHandle() != -1) {
        auto& parent_s = root_solver_data[b_node.GetParentHandle()];
        const float K_child = s_node.conductance;
        parent_s.G_sum += theta * K_child;
        parent_s.G_sum -= theta * K_child * s_node.A;
        parent_s.RHS += theta * K_child * s_node.B;
        parent_s.RHS -= (1.0f - theta) * K_child * (v_p_old - s_node.pressure);
      }
    }
  }

  // --- 4. Phase C: Unfold Root (Root Base -> Tips) ---
  // Store the new pressures for all root base nodes
  std::vector<float> v_root_base_new_list;
  
  if (root_model && !root_model->RefRootSkeleton().RefRawNodes().empty()) {
    auto& skeleton = root_model->RefRootSkeleton();
    const auto& sorted = skeleton.PeekSortedNodeList();
    auto& raw = skeleton.RefRawNodes();

    for (const auto& h : sorted) {
      auto& s_node = root_solver_data[h];
      auto& b_node = raw[h];

      if (b_node.GetParentHandle() == -1) {
        // This is a root base node
        s_node.next_pressure = s_node.B;
        v_root_base_new_list.push_back(s_node.next_pressure);
      } else {
        const float v_p = root_solver_data[b_node.GetParentHandle()].next_pressure;
        s_node.next_pressure = s_node.A * v_p + s_node.B;
      }

      // Clamp write-back
      float unclamped_mass = s_node.next_pressure * s_node.capacity;
      if (!std::isfinite(unclamped_mass))
        unclamped_mass = 0.0f;
      const float max_mass = glm::max(b_node.data.max_carbohydrate_mass, 1e-6f);
      const float new_mass = glm::clamp(unclamped_mass, 0.0f, max_mass);

      const float bio_delta = b_node.data.carbohydrate_source - b_node.data.carbohydrate_sink;
      b_node.data.net_flow_balance = (new_mass - b_node.data.carbohydrate_mass) - bio_delta;
      b_node.data.carbohydrate_mass = new_mass;
      b_node.data.next_concentration = (s_node.capacity > 0.0f) ? (new_mass / s_node.capacity) : 0.0f;
    }
  }

  // Calculate average root base pressure for shoot unfolding
  float v_root_base_avg = 0.0f;
  if (!v_root_base_new_list.empty()) {
    for (const auto& v : v_root_base_new_list) {
      v_root_base_avg += v;
    }
    v_root_base_avg /= static_cast<float>(v_root_base_new_list.size());
  }

  // --- 5. Phase D: Unfold Shoot (Trunk Base -> Leaves) ---
  if (shoot_model && !shoot_model->RefShootSkeleton().RefRawNodes().empty()) {
    auto& skeleton = shoot_model->RefShootSkeleton();
    const auto& sorted = skeleton.PeekSortedNodeList();
    auto& raw = skeleton.RefRawNodes();

    for (const auto& h : sorted) {
      auto& s_node = shoot_solver_data[h];
      auto& b_node = raw[h];

      if (b_node.GetParentHandle() == -1) {
        // This is a shoot base node - use average root base pressure
        s_node.next_pressure = s_node.A * v_root_base_avg + s_node.B;
      } else {
        const float v_p = shoot_solver_data[b_node.GetParentHandle()].next_pressure;
        s_node.next_pressure = s_node.A * v_p + s_node.B;
      }

      // Clamp write-back
      float unclamped_mass = s_node.next_pressure * s_node.capacity;
      if (!std::isfinite(unclamped_mass))
        unclamped_mass = 0.0f;
      const float max_mass = glm::max(b_node.data.max_carbohydrate_mass, 1e-6f);
      const float new_mass = glm::clamp(unclamped_mass, 0.0f, max_mass);

      const float bio_delta = b_node.data.carbohydrate_source - b_node.data.carbohydrate_sink;
      b_node.data.net_flow_balance = (new_mass - b_node.data.carbohydrate_mass) - bio_delta;
      b_node.data.carbohydrate_mass = new_mass;
      b_node.data.next_concentration = (s_node.capacity > 0.0f) ? (new_mass / s_node.capacity) : 0.0f;
    }
  }

  // Logging
  static float log_timer = 0.0f;
  log_timer += delta_time;
  if (log_timer > 1.0f) {
    float total_mass = 0.0f;
    float total_sink = 0.0f;
    float total_source = 0.0f;

    auto sum_stats = [&](auto& skeleton) {
      for (const auto& n : skeleton.RefRawNodes()) {
        total_mass += n.data.carbohydrate_mass;
        total_sink += n.data.carbohydrate_sink;
        total_source += n.data.carbohydrate_source;
      }
    };
    if (shoot_model)
      sum_stats(shoot_model->RefShootSkeleton());
    if (root_model)
      sum_stats(root_model->RefRootSkeleton());

    EVOENGINE_LOG("[System Stats] Total Mass: " + std::to_string(total_mass) +
                  " | Sink: " + std::to_string(total_sink) + " | Source: " + std::to_string(total_source) +
                  " | Root bases: " + std::to_string(root_base_indices.size()) +
                  " | Shoot bases: " + std::to_string(shoot_base_indices.size()));
    log_timer = 0.0f;
  }
}

static void DistributeCarbohydratesCombined(ShootModel* shoot_model, RootModel* root_model, const ClimateModel& climate_model, float delta_time) {
  SolveCoupledSystemCrankNicolson(shoot_model, root_model, climate_model, delta_time);
}

TreeStatistics Tree::GetTreeStatistics() const {
  TreeStatistics ret_val{};
  const auto& skeleton = shoot_model.PeekShootSkeleton();
  ret_val.Calculate(skeleton);
  return ret_val;
}

void Tree::Reset() {
  const bool keep_proc_enabled = developmental_strand_model.enabled;
  const bool keep_gpu_profile_packing = developmental_strand_model.gpu_profile_packing;
  const bool keep_gpu_resident_rendering = developmental_strand_model.gpu_resident_rendering;
  const int keep_gpu_packing_iterations = developmental_strand_model.gpu_packing_iterations;
  const int keep_proc_seed = developmental_strand_model.seed;

  ClearSkeletalGraph();
  ClearGeometryEntities();
  ClearStrandModelMeshRenderer();
  ClearStrandRenderer();
  ClearDevelopmentalStrandRenderer();
  ClearAnimatedGeometryEntities();
  shoot_model.Clear();
  root_model.Clear();
  shoot_strand_model = {};
  developmental_strand_model.Reset();
  developmental_strand_model.enabled = keep_proc_enabled;
  developmental_strand_model.gpu_profile_packing = keep_gpu_profile_packing;
  developmental_strand_model.gpu_resident_rendering = keep_gpu_resident_rendering;
  developmental_strand_model.gpu_packing_iterations = keep_gpu_packing_iterations;
  developmental_strand_model.seed = keep_proc_seed;
  developmental_strand_renderer_dirty = false;
  shoot_model.shoot_skeleton_.data.entity_index = root_model.root_skeleton_.data.entity_index = GetOwner().GetIndex();
  shoot_visualizer.Reset(shoot_model);
  root_visualizer.Reset(root_model);
}

bool Tree::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  if (ImGui::TreeNode("Preset settings")) {
    if (ImGui::Button("Oak Trunk Crack Process")) {
      strand_model_parameters.end_node_strands = 3200;
      strand_model_parameters.strand_radius_distribution.mean.max_value = 0.003f;
      strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(1.0f, 0.8f, {0, 0}, {1, 1});
      changed = true;
    }
    if (ImGui::Button("Oak Trunk Full Process")) {
      strand_model_parameters.end_node_strands = 3200;
      strand_model_parameters.strand_radius_distribution.mean.max_value = 0.004f;
      strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(1.0f, 0.6f, {0, 0}, {1, 1});
      auto& values = strand_model_parameters.strand_radius_distribution.mean.curve.UnsafeGetValues();
      // First logical point (index 0)
      // values[0] = glm::vec2(-0.05f, 0.0f);  // left tangent offset
      values[2] = glm::vec2(0.0f, -0.4f);  // right tangent offset

      // Second logical point (index 1)
      values[3] = glm::vec2(-0.1f, 0.0f);  // left tangent offset
      // values[5] = glm::vec2(0.04f, 0.0f);     // right tangent offset
      changed = true;
    }
    if (ImGui::Button("Elm")) {
      strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(0.65f, 0.5f, {0, 0}, {1, 1});
      changed = true;
    }
    if (ImGui::Button("Spruce")) {
      strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(0.5f, 0.7f, {0, 0}, {1, 1});
      auto& values = strand_model_parameters.strand_radius_distribution.mean.curve.UnsafeGetValues();
      // First logical point (index 0)
      // values[0] = glm::vec2(-0.05f, 0.0f);  // left tangent offset
      values[2] = glm::vec2(0.1f, -0.03f);  // right tangent offset

      // Second logical point (index 1)
      values[3] = glm::vec2(-0.06f, -0.12f);  // left tangent offset
      // values[5] = glm::vec2(0.04f, 0.0f);     // right tangent offset
      changed = true;
    }
    if (ImGui::Button("Oak")) {
      strand_model_parameters.strand_radius_distribution.mean.max_value = 0.003f;
      strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(0.9f, 0.5f, {0, 0}, {1, 1});
      changed = true;
    }
    ImGui::TreePop();
  }
#ifdef BILLBOARD_CLOUDS_PLUGIN
  static BillboardCloud::GenerateSettings foliage_billboard_cloud_generate_settings{};

  foliage_billboard_cloud_generate_settings.OnInspect("Foliage billboard cloud settings");

  if (ImGui::Button("Generate billboard")) {
    GenerateBillboardClouds(foliage_billboard_cloud_generate_settings);
  }
#endif
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
  const auto scene = GetScene();
  editor_layer->DragAndDropButton<TreeDescriptor>(tree_descriptor_ref, "TreeDescriptor", true);
  static bool show_space_colonization_grid = true;

  static std::shared_ptr<ParticleInfoList> space_colonization_grid_particle_info_list;
  if (!space_colonization_grid_particle_info_list) {
    space_colonization_grid_particle_info_list = AssetManager::CreateTemporaryAsset<ParticleInfoList>();
  }

  if (const auto td = tree_descriptor_ref.Get<TreeDescriptor>()) {
    const auto sd = td->shoot_descriptor.Get<BasicShootDescriptor>();
    if (sd) {
      ImGui::DragInt("TreeModel Seed", &shoot_model.seed, 1, 0);
      ImGui::DragInt("StrandModel Seed", &shoot_strand_model.seed, 1, 0);
      if (ImGui::TreeNode("Tree settings")) {
        if (ImGui::DragFloat("Start time", &start_time, 0.01f, 0.0f, 100.f))
          changed = true;

        ImGui::Separator();
        ImGui::Text("Transport Active Season (Day-of-Year)");
        ImGui::DragInt("Season start day", &season_start_day, 1.0f, 0, 364);
        ImGui::DragInt("Season end day", &season_end_day, 1.0f, 0, 364);
        season_start_day = glm::clamp(season_start_day, 0, 364);
        season_end_day = glm::clamp(season_end_day, 0, 364);

        ImGui::Checkbox("Enable History", &enable_history);
        if (enable_history) {
          ImGui::DragInt("History per iteration", &history_iteration, 1, 1, 1000);
        }
        if (ImGui::TreeNode("Sagging")) {
          bool bending_changed = false;
          bending_changed =
              ImGui::DragFloat("Bending strength", &sd->gravity_bending_strength, 0.01f, 0.0f, 1.0f, "%.3f") ||
              bending_changed;
          bending_changed = ImGui::DragFloat("Bending thickness factor", &sd->gravity_bending_thickness_factor, 0.1f,
                                             0.0f, 10.f, "%.3f") ||
                            bending_changed;
          bending_changed =
              ImGui::DragFloat("Bending angle factor", &sd->gravity_bending_max, 0.01f, 0.0f, 1.0f, "%.3f") ||
              bending_changed;
          if (bending_changed) {
            shoot_growth_controller_.sagging = [=](std::mt19937& random_engine,
                                                   const ShootGrowthData& shoot_growth_data,
                                                   const SkeletonNode<InternodeGrowthData>& internode) {
              float strength =
                  internode.data.sagging_force * sd->gravity_bending_strength /
                  glm::pow(internode.info.thickness / sd->end_node_thickness, sd->gravity_bending_thickness_factor);
              strength = sd->gravity_bending_max * (1.f - glm::exp(-glm::abs(strength)));
              return strength;
            };
            shoot_model.CalculateTransform(shoot_growth_controller_, true);
            shoot_visualizer.need_update = true;
          }
          ImGui::TreePop();
        }
        if (shoot_model.tree_growth_settings.OnInspect(editor_layer))
          changed = true;

        if (shoot_model.tree_growth_settings.use_space_colonization &&
            !shoot_model.tree_growth_settings.space_colonization_auto_resize) {
          static float radius = 1.5f;
          static int markers_per_voxel = 5;
          ImGui::DragFloat("Import radius", &radius, 0.01f, 0.01f, 10.0f);
          ImGui::DragInt("Markers per voxel", &markers_per_voxel);
          FileUtils::OpenFile(
              "Load Voxel Data", "Binvox", {".binvox"},
              [&](const std::filesystem::path& path) {
                auto& occupancy_grid = shoot_model.tree_occupancy_grid;
                if (VoxelGrid<TreeOccupancyGridBasicData> input_grid{}; ParseBinvox(path, input_grid, 1.f)) {
                  occupancy_grid.Initialize(
                      input_grid, glm::vec3(-radius, 0, -radius), glm::vec3(radius, 2.0f * radius, radius),
                      sd->internode_length, shoot_model.tree_growth_settings.space_colonization_removal_distance_factor,
                      shoot_model.tree_growth_settings.space_colonization_theta,
                      shoot_model.tree_growth_settings.space_colonization_detection_distance_factor, markers_per_voxel);
                }
              },
              false);

          static PrivateComponentRef private_component_ref{};

          if (editor_layer->DragAndDropButton<MeshRenderer>(private_component_ref, "Add Obstacle")) {
            if (const auto mmr = private_component_ref.Get<MeshRenderer>()) {
              const auto cube_volume = AssetManager::CreateTemporaryAsset<CubeVolume>();
              cube_volume->ApplyMeshBounds(mmr->mesh.Get<Mesh>());
              const auto global_transform = scene->GetDataComponent<GlobalTransform>(mmr->GetOwner());
              shoot_model.tree_occupancy_grid.InsertObstacle(global_transform, cube_volume);
              private_component_ref.Clear();
            }
          }
        }

        ImGui::TreePop();
      }
      static int mesh_generate_iterations = 0;
      if (ImGui::TreeNode("Cylindrical Mesh generation settings")) {
        ImGui::DragInt("Iterations", &mesh_generate_iterations, 1, 0, shoot_model.CurrentIteration());
        mesh_generate_iterations = glm::clamp(mesh_generate_iterations, 0, shoot_model.CurrentIteration());
        tree_mesh_generator_settings.OnInspect(editor_layer);

        ImGui::TreePop();
      }
      if (ImGui::Button("Generate Cylindrical Mesh")) {
        GenerateGeometryEntities(tree_mesh_generator_settings, mesh_generate_iterations);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Cylindrical Mesh")) {
        ClearGeometryEntities();
      }

      if (ImGui::Button("Generate Animated Cylindrical Mesh")) {
        GenerateAnimatedGeometryEntities(tree_mesh_generator_settings, mesh_generate_iterations);
      }
      ImGui::SameLine();
      if (ImGui::Button("Clear Animated Cylindrical Mesh")) {
        ClearAnimatedGeometryEntities();
      }
    }

    if (shoot_model.tree_growth_settings.use_space_colonization) {
      bool need_grid_update = false;
      if (shoot_visualizer.need_update) {
        need_grid_update = true;
      }
      if (ImGui::Button("Update grids"))
        need_grid_update = true;
      ImGui::Checkbox("Show Space Colonization Grid", &show_space_colonization_grid);
      if (show_space_colonization_grid) {
        if (need_grid_update) {
          auto& occupancy_grid = shoot_model.tree_occupancy_grid;
          auto& voxel_grid = occupancy_grid.RefGrid();
          const auto num_voxels = voxel_grid.GetVoxelCount();
          std::vector<ParticleInfo> scalar_matrices{};

          if (scalar_matrices.size() != num_voxels) {
            scalar_matrices.resize(num_voxels);
          }

          if (scalar_matrices.size() != num_voxels) {
            scalar_matrices.reserve(occupancy_grid.GetMarkersPerVoxel() * num_voxels);
          }
          int i = 0;
          for (const auto& voxel : voxel_grid.RefData()) {
            for (const auto& marker : voxel.markers) {
              scalar_matrices.resize(i + 1);
              scalar_matrices[i].instance_matrix.value = glm::translate(marker.position) *
                                                         glm::mat4_cast(glm::quat(glm::vec3(0.0f))) *
                                                         glm::scale(glm::vec3(voxel_grid.GetVoxelSize() * 0.2f));
              if (marker.node_handle == -1)
                scalar_matrices[i].instance_color = glm::vec4(1.0f, 1.0f, 1.0f, 0.75f);
              else {
                scalar_matrices[i].instance_color =
                    glm::vec4(eco_sys_lab_layer->RandomColors()[marker.node_handle], 1.0f);
              }
              i++;
            }
          }
          space_colonization_grid_particle_info_list->SetParticleInfos(scalar_matrices);
        }
        GizmoSettings gizmo_settings{};
        gizmo_settings.draw_settings.blending = true;
        editor_layer->DrawGizmoMeshInstancedColored(Resources::Primitives::cube,
                                                    space_colonization_grid_particle_info_list, glm::mat4(1.0f), 1.0f,
                                                    gizmo_settings);
      }
    }

    if (enable_history) {
      if (ImGui::Button("Temporal Progression")) {
        temporal_progression = true;
        temporal_progression_iteration = 0;
      }
    }
  }

  /*
  ImGui::Checkbox("Split root test", &splitRootTest);
  ImGui::Checkbox("Biomass history", &record_biomass_history);

  if (splitRootTest) ImGui::Text(("Left/Right side biomass: [" + std::to_string(m_leftSideBiomass) + ", " +
  std::to_string(right_side_biomass) + "]").c_str());
  */

  if (ImGui::TreeNode("Strand Model")) {
    if (strand_model_parameters.OnInspect(editor_layer))
      changed = true;

    ImGui::Text(("Strand count: " +
                 std::to_string(shoot_strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size()))
                    .c_str());
    ImGui::Text(
        ("Total particle count: " + std::to_string(shoot_strand_model.strand_model_skeleton.data.num_of_particles))
            .c_str());

    if (ImGui::Button("Rebuild Strand Model")) {
      BuildStrandModel();
    }

    ImGui::SameLine();
    if (ImGui::Button("Clear Strand Model")) {
      shoot_strand_model = {};
    }

    if (ImGui::TreeNodeEx("Strand Model Mesh Generator Settings", ImGuiTreeNodeFlags_DefaultOpen)) {
      strand_model_mesh_generator_settings.OnInspect(editor_layer);
      ImGui::TreePop();
    }

    ImGui::TreePop();
  }

  if (ImGui::Button("Build StrandRenderer")) {
    InitializeStrandRenderer();
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear StrandRenderer")) {
    ClearStrandRenderer();
  }

  if (ImGui::TreeNodeEx("Procedural Strand Model", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Checkbox("Enable", &developmental_strand_model.enabled);
    ImGui::Checkbox("GPU Profile Packing", &developmental_strand_model.gpu_profile_packing);
    ImGui::Checkbox("GPU Resident Rendering", &developmental_strand_model.gpu_resident_rendering);
    ImGui::SameLine();
    ImGui::DragInt("Iterations##ProcStrand", &developmental_strand_model.gpu_packing_iterations, 1, 1, 500);
    ImGui::Text(("Strands: " + std::to_string(developmental_strand_model.skeleton.data.HasStrandData()
                                                  ? developmental_strand_model.skeleton.data.strand_data->strand_group
                                                        .PeekStrands().size()
                                                  : 0))
                    .c_str());
    if (ImGui::Button("Enable from Current Tree")) {
      developmental_strand_model.Enable(shoot_model.PeekShootSkeleton(), strand_model_parameters);
    }
    ImGui::SameLine();
    if (ImGui::Button("Disable##ProcStrand")) {
      developmental_strand_model.Disable();
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset##ProcStrand")) {
      developmental_strand_model.Reset();
      ClearDevelopmentalStrandRenderer();
    }
    if (ImGui::Button("Build Procedural Strands")) {
      InitializeDevelopmentalStrandRenderer();
    }
    ImGui::SameLine();
    if (ImGui::Button("Clear Procedural Strands")) {
      ClearDevelopmentalStrandRenderer();
    }
    if (ImGui::Checkbox("Enable Procedural Foliage", &developmental_strand_foliage_enabled)) {
      developmental_strand_renderer_dirty = true;
    }
    ImGui::TreePop();
  }
  if (ImGui::Button("Build Strand Particles")) {
    InitializeStrandParticles();
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Strand Particles")) {
    ClearStrandParticles();
  }
  if (ImGui::Button("Build Strand Mesh")) {
    InitializeStrandModelMeshRenderer(strand_model_mesh_generator_settings);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear Strand Mesh")) {
    ClearStrandModelMeshRenderer();
  }

  shoot_visualizer.Visualize(shoot_strand_model);
  if (ImGui::TreeNode("Skeletal graph settings")) {
    if (skeletal_graph_settings.OnInspect(editor_layer))
      changed = true;

    ImGui::TreePop();
  }
  if (ImGui::Button("Build skeletal graph")) {
    GenerateSkeletalGraph(skeletal_graph_settings, -1, Resources::Primitives::sphere, Resources::Primitives::cube);
  }
  ImGui::SameLine();
  if (ImGui::Button("Clear skeletal graph")) {
    ClearSkeletalGraph();
  }

  FileUtils::SaveFile(
      "Export Cylindrical Mesh", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        ExportObj(path, tree_mesh_generator_settings);
      },
      false);
  ImGui::SameLine();
  FileUtils::SaveFile(
      "Export Strand Mesh", "OBJ", {".obj"},
      [&](const std::filesystem::path& path) {
        ExportStrandModelObj(path, strand_model_mesh_generator_settings);
      },
      false);

  return changed;
}

void Tree::Update() {
  if (temporal_progression) {
    if (temporal_progression_iteration <= shoot_model.CurrentIteration()) {
      GenerateGeometryEntities(tree_mesh_generator_settings, temporal_progression_iteration);
      temporal_progression_iteration++;
    } else {
      temporal_progression_iteration = 0;
      temporal_progression = false;
    }
  }
  const auto editor_layer = Application::GetLayer<EditorLayer>();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();
}

void Tree::OnCreate() {
  shoot_visualizer.Initialize();
  shoot_visualizer.need_update = true;
  root_visualizer.Initialize();
  root_visualizer.need_update = true;

  strand_model_parameters.branch_twist_distribution.mean = {-60.0f, 60.0f};
  strand_model_parameters.branch_twist_distribution.deviation = {0.0f, 1.0f, {0, 0}};

  strand_model_parameters.junction_twist_distribution.mean = {-60.0f, 60.0f};
  strand_model_parameters.junction_twist_distribution.deviation = {0.0f, 1.0f, {0, 0}};

  strand_model_parameters.strand_radius_distribution.mean = {0.0f, 0.002f};
  strand_model_parameters.strand_radius_distribution.deviation = {0.0f, 1.0f, {0, 0}};

  strand_model_parameters.cladoptosis_distribution.mean = {0.0f, 0.02f};
  strand_model_parameters.cladoptosis_distribution.deviation = {0.0f, 1.0f, {0, 0}};
}

void Tree::OnDestroy() {
  shoot_model = {};
  root_model = {};
  shoot_strand_model = {};

  tree_descriptor_ref.Clear();
  soil.Clear();
  climate.Clear();
  enable_history = false;

  shoot_visualizer.Clear();
  root_visualizer.Clear();

  left_side_biomass = right_side_biomass = 0.0f;
  root_biomass_history.clear();
  shoot_biomass_history.clear();

  generate_mesh = true;
  start_time = 0.f;
}

void Tree::CalculateProfiles() {
  const float time = Times::Now();
  shoot_strand_model.strand_model_skeleton.Clone(shoot_model.RefShootSkeleton());
  shoot_strand_model.ResetAllProfiles(strand_model_parameters);
  shoot_strand_model.InitializeProfiles(strand_model_parameters);
  const auto worker_handle = shoot_strand_model.CalculateProfiles(strand_model_parameters);
  Jobs::Wait(worker_handle);
  const float profile_calculation_time = Times::Now() - time;
  std::string output;
  output += "\nProfile count: [" + std::to_string(shoot_strand_model.strand_model_skeleton.PeekSortedNodeList().size());
  output += "], Strand count: [" +
            std::to_string(shoot_strand_model.strand_model_skeleton.data.strand_group.PeekStrands().size());
  output += "], Particle count: [" + std::to_string(shoot_strand_model.strand_model_skeleton.data.num_of_particles);
  output += "]\nCalculate Profile Used time: " + std::to_string(profile_calculation_time) + "\n";
  EVOENGINE_LOG(output);
}

void Tree::BuildStrandModel() {
  std::string output;

  CalculateProfiles();
  const float time = Times::Now();
  for (const auto& node_handle : shoot_model.PeekShootSkeleton().PeekSortedNodeList()) {
    shoot_strand_model.strand_model_skeleton.RefNode(node_handle).info =
        shoot_model.PeekShootSkeleton().PeekNode(node_handle).info;
  }
  shoot_strand_model.CalculateStrandProfileAdjustedTransforms(strand_model_parameters);
  shoot_strand_model.ApplyProfiles(strand_model_parameters);
  const float strand_modeling_time = Times::Now() - time;
  output += "\nBuild Strand Model Used time: " + std::to_string(strand_modeling_time) + "\n";
  EVOENGINE_LOG(output);
}

bool Tree::TryGrow(const SimulationSettings& simulation_settings, const SkeletonNodeHandle base_internode_handle,
                   const bool pruning) {
  const auto scene = GetScene();
  const auto eco_sys_lab_layer = Application::GetLayer<EcoSysLabLayer>();

  const auto climate_candidate = EcoSysLabLayer::FindClimate();
  if (!climate_candidate.expired())
    climate = climate_candidate.lock();
  if (const auto soil_candidate = EcoSysLabLayer::FindSoil(); !soil_candidate.expired())
    soil = soil_candidate.lock();

  const auto s = soil.Get<Soil>();
  const auto c = climate.Get<Climate>();

  if (!s) {
    EVOENGINE_ERROR("No soil model!")
    return false;
  }
  if (!c) {
    EVOENGINE_ERROR("No climate model!")
    return false;
  }
  bool shoot_grown = false;
  bool root_grown = false;
  try {
    PrepareController(simulation_settings);
    if (shoot_growth_controller_.Initialized() && !shoot_model.initialized_) {
      shoot_model.Initialize(shoot_growth_controller_, foliage_controller_, shoot_reproduction_controller_);
      shoot_grown = true;
    }
    if (root_growth_controller_.Initialized() && !root_model.initialized_) {
      root_model.Initialize(root_growth_controller_);
      root_grown = true;
    }
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what())
    return false;
  }
  const auto owner = GetOwner();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(owner).value;
  Vigor shoot_vigor;
  Vigor root_vigor;
  if (shoot_growth_controller_.Initialized()) {
    shoot_vigor = shoot_model.SampleShootFlux(global_transform, c->climate_model, shoot_growth_controller_);
  } else {
    shoot_vigor.value = FLT_MAX;
  }
  if (root_growth_controller_.Initialized()) {
    root_vigor = root_model.SampleRootFlux(global_transform, s->soil_model, root_growth_controller_);
  } else {
    root_vigor.value = FLT_MAX;
  }
  Vigor total_vigor;
  total_vigor.value = glm::min(shoot_vigor.value, root_vigor.value);

  if (shoot_growth_controller_.Initialized()) {
    shoot_model.DistributeVigor(shoot_growth_controller_, total_vigor);
    if (base_internode_handle != -1) {
      shoot_grown = shoot_model.Grow(simulation_settings.delta_time, base_internode_handle, global_transform,
                                     c->climate_model, s->soil_model, shoot_growth_controller_, foliage_controller_,
                                     shoot_reproduction_controller_, shoot_pruning_controller_, pruning) ||
                    shoot_grown;
    } else {
      shoot_grown = shoot_model.Grow(simulation_settings.delta_time, global_transform, c->climate_model, s->soil_model,
                                     shoot_growth_controller_, foliage_controller_, shoot_reproduction_controller_,
                                     shoot_pruning_controller_, pruning) ||
                    shoot_grown;
    }
    if (shoot_grown) {
      if (pruning)
        shoot_visualizer.ClearSelections();
      shoot_visualizer.need_update = true;
      if (!shoot_model.PeekShootSkeleton().PeekSortedNodeList().empty())
        root_model.shoot_skeleton_base_thickness = shoot_model.PeekShootSkeleton().PeekNode(0).info.thickness;
    }

    // Procedural strand update: run every growth step so elongation/thickness changes
    // are reflected even when topology is unchanged.
    if (developmental_strand_model.enabled) {
      // If procedural mode was preserved across reset/load but has not been initialised yet,
      // rebuild from the current shoot skeleton before applying incremental events.
      if (!developmental_strand_model.skeleton.data.HasStrandData()) {
        if (!shoot_model.PeekShootSkeleton().PeekRawNodes().empty()) {
          developmental_strand_model.Enable(shoot_model.PeekShootSkeleton(), strand_model_parameters);
        }
      } else {
        developmental_strand_model.OnGrowthStep(
          shoot_model.PeekShootSkeleton(), shoot_model.PeekGrowthEvents(),
          shoot_model.PeekPruningEventBatches(), shoot_model.PruningOccurred(), strand_model_parameters);
      }
      // Defer renderer updates to the main thread (TryGrow can run inside worker jobs).
      developmental_strand_renderer_dirty = true;
    }
  }

  if (root_growth_controller_.Initialized()) {
    root_model.DistributeVigor(root_growth_controller_, total_vigor);
    if (base_internode_handle == -1) {
      root_grown = root_model.Grow(simulation_settings.delta_time, global_transform, c->climate_model, s->soil_model,
                                   root_growth_controller_, fine_root_controller_, root_reproduction_controller_,
                                   root_pruning_controller_, pruning) ||
                   root_grown;
    }
    if (root_grown) {
      if (pruning)
        root_visualizer.ClearSelections();
      root_visualizer.need_update = true;
    }
  }

  // Centralized source/sink update and distribution for both systems.
  CalculateSourceSinkStrengthCombined(shoot_growth_controller_.Initialized() ? &shoot_model : nullptr,
                                      root_growth_controller_.Initialized() ? &root_model : nullptr,
                                      shoot_growth_controller_.Initialized() ? &shoot_growth_controller_ : nullptr,
                                      &foliage_controller_, &shoot_reproduction_controller_,
                                      root_growth_controller_.Initialized() ? &root_growth_controller_ : nullptr,
                                      &c->climate_model, simulation_settings.delta_time);

  DistributeCarbohydratesCombined(shoot_growth_controller_.Initialized() ? &shoot_model : nullptr,
                                  root_growth_controller_.Initialized() ? &root_model : nullptr, c->climate_model,
                                  simulation_settings.delta_time);

  if (enable_history && shoot_model.iteration_ % history_iteration == 0) {
    shoot_model.Step();
    root_model.Step();
  }
  if (record_biomass_history) {
    const auto& base_shoot_node = shoot_model.RefShootSkeleton().RefNode(0);
    shoot_biomass_history.emplace_back(base_shoot_node.data.biomass_factor +
                                       base_shoot_node.data.descendant_total_biomass_factor);
  }
  return shoot_grown || root_grown;
}

void Tree::Serialize(YAML::Emitter& out) const {
  tree_descriptor_ref.Save("tree_descriptor_ref", out);

  strand_model_parameters.Save("strand_model_parameters", out);
  developmental_strand_model.Save("developmental_strand_model", out);
  tree_mesh_generator_settings.Save("tree_mesh_generator_settings", out);
  shoot_strand_model.Save("shoot_strand_model", out);
  shoot_model.Save("shoot_model", out);

  out << YAML::Key << "shoot_visualization_mode" << YAML::Value << shoot_visualizer.tree_visualizer_color_settings.visualization_mode;
  out << YAML::Key << "shoot_color_multiplier" << YAML::Value << shoot_visualizer.tree_visualizer_color_settings.color_multiplier;
  out << YAML::Key << "shoot_visualization" << YAML::Value << shoot_visualizer.visualization;
  out << YAML::Key << "shoot_leaf_visualization" << YAML::Value << shoot_visualizer.leaf_visualization_;
  out << YAML::Key << "shoot_flower_visualization" << YAML::Value << shoot_visualizer.flower_visualization_;
  out << YAML::Key << "shoot_fruit_visualization" << YAML::Value << shoot_visualizer.fruit_visualization_;
  out << YAML::Key << "shoot_profile_gui" << YAML::Value << shoot_visualizer.profile_gui;
  out << YAML::Key << "shoot_tree_hierarchy_gui" << YAML::Value << shoot_visualizer.tree_hierarchy_gui;

  out << YAML::Key << "root_visualization_mode" << YAML::Value << root_visualizer.root_visualizer_color_settings.visualization_mode;
  out << YAML::Key << "root_color_multiplier" << YAML::Value << root_visualizer.root_visualizer_color_settings.color_multiplier;
  out << YAML::Key << "root_visualization" << YAML::Value << root_visualizer.visualization;
  out << YAML::Key << "root_profile_gui" << YAML::Value << root_visualizer.profile_gui;
  out << YAML::Key << "root_tree_hierarchy_gui" << YAML::Value << root_visualizer.tree_hierarchy_gui;

  out << YAML::Key << "shoot_model_history_limit" << YAML::Value << shoot_model.history_limit;
  out << YAML::Key << "root_model_history_limit" << YAML::Value << root_model.history_limit;

  out << YAML::Key << "developmental_strand_foliage_enabled" << YAML::Value << developmental_strand_foliage_enabled;
}

void Tree::Deserialize(const YAML::Node& in) {
  tree_descriptor_ref.Load("tree_descriptor_ref", in);

  strand_model_parameters.Load("strand_model_parameters", in);
  developmental_strand_model.Load("developmental_strand_model", in);
  tree_mesh_generator_settings.Load("tree_mesh_generator_settings", in);

  shoot_strand_model.Load("shoot_strand_model", in);
  shoot_model.Load("shoot_model", in);

  if (in["shoot_visualization_mode"]) shoot_visualizer.tree_visualizer_color_settings.visualization_mode = in["shoot_visualization_mode"].as<int>();
  if (in["shoot_color_multiplier"]) shoot_visualizer.tree_visualizer_color_settings.color_multiplier = in["shoot_color_multiplier"].as<float>();
  if (in["shoot_visualization"]) shoot_visualizer.visualization = in["shoot_visualization"].as<bool>();
  if (in["shoot_leaf_visualization"]) shoot_visualizer.leaf_visualization_ = in["shoot_leaf_visualization"].as<bool>();
  if (in["shoot_flower_visualization"]) shoot_visualizer.flower_visualization_ = in["shoot_flower_visualization"].as<bool>();
  if (in["shoot_fruit_visualization"]) shoot_visualizer.fruit_visualization_ = in["shoot_fruit_visualization"].as<bool>();
  if (in["shoot_profile_gui"]) shoot_visualizer.profile_gui = in["shoot_profile_gui"].as<bool>();
  if (in["shoot_tree_hierarchy_gui"]) shoot_visualizer.tree_hierarchy_gui = in["shoot_tree_hierarchy_gui"].as<bool>();

  if (in["root_visualization_mode"]) root_visualizer.root_visualizer_color_settings.visualization_mode = in["root_visualization_mode"].as<int>();
  if (in["root_color_multiplier"]) root_visualizer.root_visualizer_color_settings.color_multiplier = in["root_color_multiplier"].as<float>();
  if (in["root_visualization"]) root_visualizer.visualization = in["root_visualization"].as<bool>();
  if (in["root_profile_gui"]) root_visualizer.profile_gui = in["root_profile_gui"].as<bool>();
  if (in["root_tree_hierarchy_gui"]) root_visualizer.tree_hierarchy_gui = in["root_tree_hierarchy_gui"].as<bool>();

  if (in["shoot_model_history_limit"]) shoot_model.history_limit = in["shoot_model_history_limit"].as<int>();
  if (in["root_model_history_limit"]) root_model.history_limit = in["root_model_history_limit"].as<int>();

  if (in["developmental_strand_foliage_enabled"]) developmental_strand_foliage_enabled = in["developmental_strand_foliage_enabled"].as<bool>();
}

void Tree::RegisterVoxel() {
  const auto scene = GetScene();
  const auto owner = GetOwner();
  const auto global_transform = scene->GetDataComponent<GlobalTransform>(owner).value;
  shoot_model.shoot_skeleton_.data.entity_index = owner.GetIndex();
  const auto c = climate.Get<Climate>();
  shoot_model.RegisterVoxel(global_transform, c->climate_model);
}

void Tree::ExportRadialBoundingVolume(const std::shared_ptr<RadialBoundingVolume>& rbv) const {
  const auto& sorted_internode_list = shoot_model.shoot_skeleton_.PeekSortedNodeList();
  const auto& skeleton = shoot_model.shoot_skeleton_;
  std::vector<glm::vec3> points;
  for (const auto& node_handle : sorted_internode_list) {
    const auto& node = skeleton.PeekNode(node_handle);
    points.emplace_back(node.info.global_position);
    points.emplace_back(node.info.GetGlobalEndPosition());
  }
  rbv->CalculateVolume(points);
}

void Tree::CollectAssetRef(std::vector<AssetRef>& list) {
  if (tree_descriptor_ref.Get<TreeDescriptor>()) {
    list.emplace_back(tree_descriptor_ref);
  }
}

void Tree::PrepareController(const SimulationSettings& simulation_settings) {
  const auto td = tree_descriptor_ref.Get<TreeDescriptor>();
  if (!td) {
    throw std::runtime_error("Growing tree without tree descriptor!");
  }
  const auto shoot_descriptor = td->shoot_descriptor.Get<IShootDescriptor>();
  if (!shoot_descriptor) {
    shoot_growth_controller_ = {};
    shoot_growth_controller_.initialized_ = false;
  } else {
    shoot_growth_controller_.initialized_ = true;
    shoot_descriptor->PrepareController(shoot_growth_controller_);
  }
  const auto root_descriptor = td->root_descriptor.Get<IRootDescriptor>();
  if (!root_descriptor) {
    root_growth_controller_ = {};
    root_growth_controller_.initialized_ = false;
  } else {
    root_growth_controller_.initialized_ = true;
    root_descriptor->PrepareController(root_growth_controller_);
  }
  const auto pruning_descriptor = td->pruning_descriptor.Get<IPruningDescriptor>();
  if (!pruning_descriptor) {
    shoot_pruning_controller_ = {};
    shoot_pruning_controller_.initialized_ = false;
  } else {
    shoot_pruning_controller_.initialized_ = true;
    pruning_descriptor->PrepareController(simulation_settings, shoot_pruning_controller_);
  }
  const auto foliage_descriptor = td->foliage_descriptor.Get<IFoliageDescriptor>();
  if (!foliage_descriptor) {
    foliage_controller_ = {};
    foliage_controller_.initialized_ = false;
  } else {
    foliage_descriptor->PrepareController(foliage_controller_);
    foliage_controller_.initialized_ = true;
  }
  const auto reproduction_module_descriptor = td->reproduction_module_descriptor.Get<IReproductionModuleDescriptor>();
  if (!reproduction_module_descriptor) {
    shoot_reproduction_controller_ = {};
    shoot_reproduction_controller_.initialized_ = false;
  } else {
    shoot_reproduction_controller_.initialized_ = true;
    reproduction_module_descriptor->PrepareController(shoot_reproduction_controller_);
  }
}