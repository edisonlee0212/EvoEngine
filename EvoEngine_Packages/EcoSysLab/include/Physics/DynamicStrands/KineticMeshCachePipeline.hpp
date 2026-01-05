#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "DtsStrandGroup.hpp"
#include "DynamicStrandsInitializationParameters.hpp"
#include "ShootGrowthData.hpp"
#include "StrandModel.hpp"
#include "StrandModelData.hpp"
#include "Transform.hpp"
#include "glm/glm.hpp"

namespace eco_sys_lab_package {

class DsKineticVoronoiMeshing;
class DynamicStrands;

/// CPU inputs shared by Kinetic Voronoi meshing hash / TreeMesher / MeshBuffers cache.
struct KineticMeshingInputs {
  std::vector<std::vector<glm::dvec2>> support_points;
  std::vector<std::vector<double>> subdivisions_by_strand;
  std::vector<std::vector<int>> physics_strand_to_segment_indices;
  std::vector<std::vector<glm::dmat4>> transforms_by_height_and_branch;
  GlobalTransform root_transform{};
  std::vector<std::vector<size_t>> branch_indices;
  std::vector<std::vector<std::vector<size_t>>> strands_by_branch_id;
  float min_segment_length = 0.f;
  float max_segment_length = 0.f;
};

enum class KineticMeshCacheResult {
  CacheHit = 0,
  Saved = 1,
  Failed = 2,
};

/// Uniform + random subdiv guide points / branch maps / plane-spline support points (no Vulkan).
[[nodiscard]] bool BuildKineticMeshingInputs(const DynamicStrandsInitializeParameters& initialize_parameters,
                                             const StrandModelSkeleton& strand_model_skeleton,
                                             const StrandModelStrandGroup& strand_model_strand_group,
                                             DtsStrandGroup& randomly_subdivided_strand_group,
                                             DtsStrandGroup& uniformly_subdivided_strand_group,
                                             KineticMeshingInputs& out_inputs);

/// Post-StrandTree cache path: hash → cache hit / remesh → prepare → populate CPU meshlet vectors → SaveMeshingBuffer.
/// When @ref DsKineticVoronoiMeshing::MeshingSettings::cache_hit_skip_load is set, a hit returns without loading.
[[nodiscard]] KineticMeshCacheResult RunKineticMeshingCache(DsKineticVoronoiMeshing& meshing,
                                                            KineticMeshingInputs& inputs);

/// Grow/export hand-off for the kinDS-only precompute worker: build StrandTree (dry-run, no remesh)
/// and write a job.txt the worker can consume.
[[nodiscard]] bool ExportPrecomputeStrandTreeJob(
    StrandModel& strand_model, DynamicStrandsInitializeParameters initialize_parameters, int seed,
    bool fixed_subdivision_seed, const std::string& meshing_buffer_description,
    const std::string& statistics_experiment_tag, const std::filesystem::path& strand_tree_path,
    const std::filesystem::path& job_path, const std::filesystem::path& stats_csv_path,
    const std::filesystem::path& log_path, int workers = 4);

/// Catalog entry for a succeeded precompute (StrandTree + MeshBuffers.bin available).
struct PrecomputedExperimentAssets {
  std::string cli_id;
  std::string display_name;
  std::string input_hash;
  std::filesystem::path strand_tree_path;
  std::filesystem::path mesh_buffer_bin_path;
  std::filesystem::path assets_dir;
};

/// Copy succeeded precompute StrandTrees + MeshBuffers into Assets/PrecomputedExperiments/<cli_id>/.
/// Returns entries that have both a StrandTree and a MeshBuffers .bin on disk after sync.
[[nodiscard]] std::vector<PrecomputedExperimentAssets> SyncAndListPrecomputedExperimentAssets();

}  // namespace eco_sys_lab_package
