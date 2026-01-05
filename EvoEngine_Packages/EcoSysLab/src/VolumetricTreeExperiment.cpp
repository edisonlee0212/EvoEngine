#include "VolumetricTreeExperiment.hpp"

#include <cstring>

#include "DsKineticVoronoiMeshing.hpp"
#include "DynamicStrands.hpp"
#include "Tree.hpp"

namespace eco_sys_lab_package {
namespace {

void ApplyOakTrunkFullProcessTreePreset(const std::shared_ptr<Tree>& tree) {
  tree->strand_model_parameters.end_node_strands = 3200;
  tree->strand_model_parameters.strand_radius_distribution.mean.max_value = 0.004f;
  tree->strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(1.0f, 0.6f, {0, 0}, {1, 1});
  auto& values = tree->strand_model_parameters.strand_radius_distribution.mean.curve.UnsafeGetValues();
  values[2] = glm::vec2(0.0f, -0.4f);
  values[3] = glm::vec2(-0.1f, 0.0f);
}

void ApplyStockyTrunkTreePreset(const std::shared_ptr<Tree>& tree) {
  tree->strand_model_parameters.end_node_strands = 1400;
  tree->strand_model_parameters.center_attraction_strength = 6000.f;
  tree->strand_model_parameters.strand_radius_distribution.mean.max_value = 0.0055f;
  tree->strand_model_parameters.strand_radius_distribution.mean.curve = Curve2D(1.0f, 0.45f, {0, 0}, {1, 1});
  auto& values = tree->strand_model_parameters.strand_radius_distribution.mean.curve.UnsafeGetValues();
  values[2] = glm::vec2(0.15f, -0.55f);
  values[3] = glm::vec2(-0.05f, 0.0f);
}

void ApplyOakThickStumpPreset(const std::shared_ptr<Tree>& tree, const int end_node_strands,
                              const double alpha_cutoff) {
  tree->strand_model_parameters.end_node_strands = end_node_strands;
  DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff = alpha_cutoff;
  DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff = alpha_cutoff;
  DsKineticVoronoiMeshing::meshing_settings.spline_tension = 1.0f;
}

}  // namespace

const std::vector<VolumetricTreeExperiment>& GetVolumetricTreeExperiments() {
  static const std::vector<VolumetricTreeExperiment> kExperiments = {
      {VolumetricTreeExperimentId::SmallTrunk, "SmallTrunk", "Small Trunk", "./TreeDescriptors/Basic/Oak_trunk.tree",
       4.f, 0, VolumetricTreeExperiment::StrandPreset::OakTrunkFullProcess, 3200, 10.0, 0, 0.005f, 0.01f,
       "created from DynamicStrandsDemo scripted experiment: Small Trunk",
       "Grow Oak_trunk for 4 years, apply Oak Trunk Full Process presets, then mesh "
       "(uses MeshBuffers cache when available)."},
      {VolumetricTreeExperimentId::SmallTrunkAlpha1e9, "SmallTrunkAlpha1e9", "Small Trunk alpha=1e9",
       "./TreeDescriptors/Basic/Oak_trunk.tree", 4.f, 0, VolumetricTreeExperiment::StrandPreset::OakTrunkFullProcess,
       3200,
       // Meshing cutoff = sqrt(alpha); alpha = 1e9 matches the sweep/fractal alpha (R^2) convention.
       31622.776601683792, 0, 0.005f, 0.01f,
       "created from DynamicStrandsDemo scripted experiment: Small Trunk alpha=1e9",
       "Same growth as Small Trunk, then mesh with alpha = 1e9 (alpha_cutoff = sqrt(1e9))."},
      {VolumetricTreeExperimentId::NormalTrunk, "NormalTrunk", "Normal Trunk", "./TreeDescriptors/Basic/Oak_trunk.tree",
       8.f, 0, VolumetricTreeExperiment::StrandPreset::OakTrunkFullProcess, 3200, 10.0, 0, 0.005f, 0.01f,
       "created from DynamicStrandsDemo scripted experiment: Normal Trunk",
       "Same as Small Trunk but grows Oak_trunk for 8 years before meshing "
       "(uses MeshBuffers cache when available)."},
      {VolumetricTreeExperimentId::StockyTrunk, "StockyTrunk", "Stocky Trunk",
       "./TreeDescriptors/Basic/Oak_trunk_stocky.tree", 6.f, 0, VolumetricTreeExperiment::StrandPreset::StockyTrunk,
       1400, 10.0, 42, 0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Stocky Trunk",
       "Grow Oak_trunk_stocky for 6 years with seed 42: thicker root, milder skeleton bends, "
       "fewer strands / weaker packing for a less round cross-section, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakThickStump100, "OakThickStump100", "Oak thick stump",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 100, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump",
       "Grow Oak for 15 iterations (seed 0): 100 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakThickStump200, "OakThickStump200", "Oak thick stump 200",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 200, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump 200",
       "Grow Oak for 15 iterations (seed 0): 200 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakThickStump300, "OakThickStump300", "Oak thick stump 300",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 300, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump 300",
       "Grow Oak for 15 iterations (seed 0): 300 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakThickStump400, "OakThickStump400", "Oak thick stump 400",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 400, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump 400",
       "Grow Oak for 15 iterations (seed 0): 400 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakThickStump100Alpha10, "OakThickStump100_a10", "Oak thick stump a=10",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 100, 10.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump a=10",
       "Same as Oak thick stump (100 strands) with alpha cutoffs 10."},
      {VolumetricTreeExperimentId::OakThickStump200Alpha10, "OakThickStump200_a10", "Oak thick stump 200 a=10",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 200, 10.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump 200 a=10",
       "Same as Oak thick stump 200 with alpha cutoffs 10."},
      {VolumetricTreeExperimentId::OakThickStump300Alpha10, "OakThickStump300_a10", "Oak thick stump 300 a=10",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 300, 10.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump 300 a=10",
       "Same as Oak thick stump 300 with alpha cutoffs 10."},
      {VolumetricTreeExperimentId::OakThickStump400Alpha10, "OakThickStump400_a10", "Oak thick stump 400 a=10",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 400, 10.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump 400 a=10",
       "Same as Oak thick stump 400 with alpha cutoffs 10."},
      {VolumetricTreeExperimentId::OakThickStump100Alpha20, "OakThickStump100_a20", "Oak thick stump a=20",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 100, 20.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump a=20",
       "Same as Oak thick stump (100 strands) with alpha cutoffs 20."},
      {VolumetricTreeExperimentId::OakThickStump200Alpha20, "OakThickStump200_a20", "Oak thick stump 200 a=20",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 200, 20.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump 200 a=20",
       "Same as Oak thick stump 200 with alpha cutoffs 20."},
      {VolumetricTreeExperimentId::OakThickStump300Alpha20, "OakThickStump300_a20", "Oak thick stump 300 a=20",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 300, 20.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump 300 a=20",
       "Same as Oak thick stump 300 with alpha cutoffs 20."},
      {VolumetricTreeExperimentId::OakThickStump400Alpha20, "OakThickStump400_a20", "Oak thick stump 400 a=20",
       "./TreeDescriptors/Basic/Oak.tree", 0.f, 15, VolumetricTreeExperiment::StrandPreset::OakThickStump, 400, 20.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak thick stump 400 a=20",
       "Same as Oak thick stump 400 with alpha cutoffs 20."},
      {VolumetricTreeExperimentId::OakTwoYearSparse, "OakTwoYearSparse", "Oak 2 years 10 strands",
       "./TreeDescriptors/Basic/Oak.tree", 2.f, 0, VolumetricTreeExperiment::StrandPreset::OakThickStump, 10, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak 2 years 10 strands",
       "Grow Oak for 2 years (seed 0): 10 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakTwoYear20, "OakTwoYear20", "Oak 2 years 20 strands",
       "./TreeDescriptors/Basic/Oak.tree", 2.f, 0, VolumetricTreeExperiment::StrandPreset::OakThickStump, 20, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak 2 years 20 strands",
       "Grow Oak for 2 years (seed 0): 20 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakTwoYear50, "OakTwoYear50", "Oak 2 years 50 strands",
       "./TreeDescriptors/Basic/Oak.tree", 2.f, 0, VolumetricTreeExperiment::StrandPreset::OakThickStump, 50, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak 2 years 50 strands",
       "Grow Oak for 2 years (seed 0): 50 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakTwoYear100, "OakTwoYear100", "Oak 2 years 100 strands",
       "./TreeDescriptors/Basic/Oak.tree", 2.f, 0, VolumetricTreeExperiment::StrandPreset::OakThickStump, 100, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak 2 years 100 strands",
       "Grow Oak for 2 years (seed 0): 100 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakThreeYear4, "OakThreeYear4", "Oak 3 years 4 strands",
       "./TreeDescriptors/Basic/Oak.tree", 3.f, 0, VolumetricTreeExperiment::StrandPreset::OakThickStump, 4, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak 3 years 4 strands",
       "Grow Oak for 3 years (seed 0): 4 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakThreeYear10, "OakThreeYear10", "Oak 3 years 10 strands",
       "./TreeDescriptors/Basic/Oak.tree", 3.f, 0, VolumetricTreeExperiment::StrandPreset::OakThickStump, 10, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak 3 years 10 strands",
       "Grow Oak for 3 years (seed 0): 10 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakThreeYear20, "OakThreeYear20", "Oak 3 years 20 strands",
       "./TreeDescriptors/Basic/Oak.tree", 3.f, 0, VolumetricTreeExperiment::StrandPreset::OakThickStump, 20, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak 3 years 20 strands",
       "Grow Oak for 3 years (seed 0): 20 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakThreeYear50, "OakThreeYear50", "Oak 3 years 50 strands",
       "./TreeDescriptors/Basic/Oak.tree", 3.f, 0, VolumetricTreeExperiment::StrandPreset::OakThickStump, 50, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak 3 years 50 strands",
       "Grow Oak for 3 years (seed 0): 50 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakFourYearSparse, "OakFourYearSparse", "Oak 4 years 4 strands",
       "./TreeDescriptors/Basic/Oak.tree", 4.f, 0, VolumetricTreeExperiment::StrandPreset::OakThickStump, 4, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak 4 years 4 strands",
       "Grow Oak for 4 years (seed 0): 4 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakSixYearSparse, "OakSixYearSparse", "Oak 6 years 4 strands",
       "./TreeDescriptors/Basic/Oak.tree", 6.f, 0, VolumetricTreeExperiment::StrandPreset::OakThickStump, 4, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak 6 years 4 strands",
       "Grow Oak for 6 years (seed 0): 4 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
      {VolumetricTreeExperimentId::OakEightYearSparse, "OakEightYearSparse", "Oak 8 years 4 strands",
       "./TreeDescriptors/Basic/Oak.tree", 8.f, 0, VolumetricTreeExperiment::StrandPreset::OakThickStump, 4, 5.0, 0,
       0.005f, 0.01f, "created from DynamicStrandsDemo scripted experiment: Oak 8 years 4 strands",
       "Grow Oak for 8 years (seed 0): 4 end strands/branch, alpha cutoffs 5, "
       "strand tension 1, then volumetric mesh."},
  };
  return kExperiments;
}

const VolumetricTreeExperiment* FindVolumetricTreeExperiment(const char* cli_id) {
  if (!cli_id) {
    return nullptr;
  }
  for (const auto& experiment : GetVolumetricTreeExperiments()) {
    if (std::strcmp(experiment.cli_id, cli_id) == 0) {
      return &experiment;
    }
  }
  return nullptr;
}

const VolumetricTreeExperiment* FindVolumetricTreeExperiment(VolumetricTreeExperimentId id) {
  for (const auto& experiment : GetVolumetricTreeExperiments()) {
    if (experiment.id == id) {
      return &experiment;
    }
  }
  return nullptr;
}

void ApplyVolumetricTreeStrandPreset(const VolumetricTreeExperiment& experiment, const std::shared_ptr<Tree>& tree) {
  switch (experiment.strand_preset) {
    case VolumetricTreeExperiment::StrandPreset::OakTrunkFullProcess:
      ApplyOakTrunkFullProcessTreePreset(tree);
      break;
    case VolumetricTreeExperiment::StrandPreset::StockyTrunk:
      ApplyStockyTrunkTreePreset(tree);
      break;
    case VolumetricTreeExperiment::StrandPreset::OakThickStump:
      ApplyOakThickStumpPreset(tree, experiment.end_node_strands, experiment.alpha_cutoff);
      break;
  }
  // Apply for all presets (OakThickStump also sets this inside its helper).
  DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff = experiment.alpha_cutoff;
  DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff = experiment.alpha_cutoff;
}

void ApplyVolumetricTreePhysicsPreset(DynamicStrands::PhysicsParameters& physics_parameters) {
  physics_parameters.bundle_strength_factor = 1.0f;
  physics_parameters.crack_bd_shrinkage_offset = 0.0f;
  physics_parameters.crack_R_scale = 0.0f;
  physics_parameters.crack_T_scale = 1.0f;
  physics_parameters.boundary_strength_decay_factor = 6.0f;
  physics_parameters.internal_pattern = 1;
  physics_parameters.bd_offset = 0.06f;
  physics_parameters.HL_threshold = 0.1f;
  physics_parameters.matrixAb = glm::mat3(0.5f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f, 2.0f);
  physics_parameters.bb = 0.5f;
  physics_parameters.be = 0.5f;
}

}  // namespace eco_sys_lab_package
