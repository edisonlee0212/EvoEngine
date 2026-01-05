#include "KineticMeshCachePipeline.hpp"

#include <algorithm>
#include <fstream>
#include <random>

#include "DsKineticVoronoiMeshing.hpp"
#include "DsMaterials.hpp"
#include "DynamicStrands.hpp"
#include "EcoSysLabPaths.hpp"
#include "ProjectManager.hpp"
#include "VolumetricTreeExperiment.hpp"
#include "kinDS/kinDS/StrandTree.hpp"

using namespace eco_sys_lab_package;
using namespace evo_engine;

namespace {

std::string SanitizeExperimentTag(std::string tag) {
  std::replace(tag.begin(), tag.end(), ' ', '_');
  for (char& c : tag) {
    if (c == '+' || c == '/' || c == '\\' || c == ':') {
      c = '_';
    }
  }
  return tag;
}

}  // namespace

bool eco_sys_lab_package::ExportPrecomputeStrandTreeJob(
    StrandModel& strand_model, DynamicStrandsInitializeParameters initialize_parameters, const int seed,
    const bool fixed_subdivision_seed, const std::string& meshing_buffer_description,
    const std::string& statistics_experiment_tag, const std::filesystem::path& strand_tree_path,
    const std::filesystem::path& job_path, const std::filesystem::path& stats_csv_path,
    const std::filesystem::path& log_path, const int workers) {
  const std::string experiment_tag =
      SanitizeExperimentTag(statistics_experiment_tag.empty() ? "precompute" : statistics_experiment_tag);

  DsKineticVoronoiMeshing::meshing_settings.meshing_buffer_description = meshing_buffer_description;
  DsKineticVoronoiMeshing::meshing_settings.override_meshing_buffer = false;
  DsKineticVoronoiMeshing::meshing_settings.cache_hit_skip_load = false;
  DsKineticVoronoiMeshing::meshing_settings.dry_run_strand_tree_only = true;
  DsKineticVoronoiMeshing::meshing_settings.debug_export_meshes = false;
  DsKineticVoronoiMeshing::meshing_settings.collect_meshing_statistics = false;
  DsKineticVoronoiMeshing::meshing_settings.meshing_statistics_experiment_name = experiment_tag;
  initialize_parameters.meshing_type = MeshingType::KineticVoronoi;

  DsMaterials materials;
  auto dynamic_strands = std::make_shared<DynamicStrands>(materials);
  dynamic_strands->InitMeshingAlgorithm(MeshingType::KineticVoronoi);

  auto strand_model_strand_group = strand_model.strand_model_skeleton.data.strand_group;
  std::mt19937 random_engine;
  if (fixed_subdivision_seed) {
    random_engine = std::mt19937(static_cast<uint32_t>(seed));
  } else {
    // Non-deterministic path — avoid for MeshBuffers / precompute cache keys.
    random_engine = std::mt19937(std::random_device{}());
    EVOENGINE_WARNING(
        "ExportPrecomputeStrandTreeJob: fixed_subdivision_seed=false; MeshBuffers hash will not be "
        "reproducible across runs");
  }

  DtsStrandGroup randomly_subdivided_strand_group{};
  DtsStrandGroup uniformly_subdivided_strand_group{};
  dynamic_strands->InitializeData(random_engine, initialize_parameters, strand_model.strand_model_skeleton,
                                  strand_model_strand_group, randomly_subdivided_strand_group,
                                  uniformly_subdivided_strand_group);

  DsKineticVoronoiMeshing::meshing_settings.dry_run_strand_tree_only = false;

  if (!dynamic_strands->kinetic_voronoi_meshing || !dynamic_strands->kinetic_voronoi_meshing->strand_tree) {
    EVOENGINE_ERROR("Precompute export: StrandTree was not built");
    return false;
  }

  std::error_code ec;
  std::filesystem::create_directories(strand_tree_path.parent_path(), ec);
  std::filesystem::create_directories(job_path.parent_path(), ec);
  dynamic_strands->kinetic_voronoi_meshing->strand_tree->saveToFile(strand_tree_path);

  std::ofstream job(job_path);
  if (!job) {
    EVOENGINE_ERROR("Precompute export: failed to write job file " << job_path.string());
    return false;
  }
  job << "strand_tree=" << strand_tree_path.string() << "\n";
  job << "stats_csv=" << stats_csv_path.string() << "\n";
  job << "log=" << log_path.string() << "\n";
  job << "experiment_tag=" << experiment_tag << "\n";
  job << "description=" << meshing_buffer_description << "\n";
  job << "alpha_cutoff=" << DsKineticVoronoiMeshing::meshing_settings.alpha_cutoff << "\n";
  job << "branch_alpha_cutoff=" << DsKineticVoronoiMeshing::meshing_settings.branch_alpha_cutoff << "\n";
  job << "look_ahead=" << DsKineticVoronoiMeshing::meshing_settings.look_ahead << "\n";
  job << "workers=" << std::max(1, workers) << "\n";

  const std::string& input_hash = dynamic_strands->kinetic_voronoi_meshing->last_meshing_input_hash_;
  if (input_hash.empty()) {
    EVOENGINE_ERROR("Precompute export: missing meshing input hash after dry-run");
    return false;
  }
  const std::filesystem::path mesh_buffer_bin =
      (ProjectManager::GetProjectPath().empty() ? std::filesystem::path("MeshBuffers")
                                                : ProjectManager::GetProjectPath().parent_path() / "MeshBuffers") /
      (input_hash + ".bin");
  job << "input_hash=" << input_hash << "\n";
  job << "mesh_buffer_bin=" << mesh_buffer_bin.string() << "\n";

  const glm::mat4& root = dynamic_strands->kinetic_voronoi_meshing->last_meshing_root_transform_;
  job << "root_transform=";
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      if (col != 0 || row != 0) {
        job << ',';
      }
      job << root[col][row];
    }
  }
  job << "\n";

  EVOENGINE_LOG("Precompute export wrote StrandTree " << strand_tree_path.string() << " and job " << job_path.string()
                                                      << " (hash=" << input_hash << ")");
  return true;
}

namespace {

std::string ReadJobKey(const std::filesystem::path& job_path, const std::string& key) {
  std::ifstream in(job_path);
  if (!in) {
    return {};
  }
  const std::string prefix = key + "=";
  std::string line;
  while (std::getline(in, line)) {
    if (line.rfind(prefix, 0) == 0) {
      return line.substr(prefix.size());
    }
  }
  return {};
}

bool CopyFileIfNeeded(const std::filesystem::path& src, const std::filesystem::path& dst) {
  if (!std::filesystem::exists(src)) {
    return false;
  }
  std::error_code ec;
  if (std::filesystem::exists(dst, ec)) {
    const auto src_size = std::filesystem::file_size(src, ec);
    const auto dst_size = std::filesystem::file_size(dst, ec);
    if (!ec && src_size == dst_size) {
      return true;
    }
  }
  std::filesystem::create_directories(dst.parent_path(), ec);
  std::filesystem::copy_file(src, dst, std::filesystem::copy_options::overwrite_existing, ec);
  return !ec && std::filesystem::exists(dst);
}

}  // namespace

std::vector<PrecomputedExperimentAssets> eco_sys_lab_package::SyncAndListPrecomputedExperimentAssets() {
  std::vector<PrecomputedExperimentAssets> result;
  const std::filesystem::path project_root = ProjectManager::GetProjectPath().empty()
                                                 ? std::filesystem::path(".")
                                                 : ProjectManager::GetProjectPath().parent_path();
  const std::filesystem::path jobs_root = EcoSysLabMetadataDirectory() / "precompute_jobs";
  const std::filesystem::path mesh_buffers_dir = project_root / "MeshBuffers";
  const std::filesystem::path assets_root = ProjectManager::GetAssetsFolderPath() / "PrecomputedExperiments";

  if (!std::filesystem::exists(jobs_root)) {
    return result;
  }

  for (const auto& experiment : GetVolumetricTreeExperiments()) {
    const std::filesystem::path job_dir = jobs_root / experiment.cli_id;
    const std::filesystem::path job_path = job_dir / (std::string(experiment.cli_id) + ".job.txt");
    const std::filesystem::path strand_tree_src = job_dir / (std::string(experiment.cli_id) + ".strandtree");
    if (!std::filesystem::exists(job_path) || !std::filesystem::exists(strand_tree_src)) {
      continue;
    }

    std::string input_hash = ReadJobKey(job_path, "input_hash");
    std::string mesh_buffer_bin = ReadJobKey(job_path, "mesh_buffer_bin");
    std::filesystem::path bin_src =
        mesh_buffer_bin.empty() ? std::filesystem::path{} : std::filesystem::path(mesh_buffer_bin);
    if ((bin_src.empty() || !std::filesystem::exists(bin_src)) && !input_hash.empty()) {
      bin_src = mesh_buffers_dir / (input_hash + ".bin");
    }
    if (input_hash.empty() && !bin_src.empty()) {
      input_hash = bin_src.stem().string();
    }
    if (bin_src.empty() || !std::filesystem::exists(bin_src) || input_hash.empty()) {
      continue;
    }

    const std::filesystem::path assets_dir = assets_root / experiment.cli_id;
    const std::filesystem::path strand_tree_dst = assets_dir / (std::string(experiment.cli_id) + ".strandtree");
    const std::filesystem::path bin_dst = assets_dir / (input_hash + ".bin");
    const std::filesystem::path catalog_dst = assets_dir / "catalog.txt";

    if (!CopyFileIfNeeded(strand_tree_src, strand_tree_dst)) {
      EVOENGINE_WARNING("PrecomputedExperiments: failed to copy StrandTree for " << experiment.cli_id);
      continue;
    }
    if (!CopyFileIfNeeded(bin_src, bin_dst)) {
      EVOENGINE_WARNING("PrecomputedExperiments: failed to copy MeshBuffers for " << experiment.cli_id);
      continue;
    }

    {
      std::ofstream catalog(catalog_dst);
      if (catalog) {
        catalog << "cli_id=" << experiment.cli_id << "\n";
        catalog << "display_name=" << experiment.display_name << "\n";
        catalog << "input_hash=" << input_hash << "\n";
        catalog << "strand_tree=" << strand_tree_dst.string() << "\n";
        catalog << "mesh_buffer_bin=" << bin_dst.string() << "\n";
      }
    }

    PrecomputedExperimentAssets entry;
    entry.cli_id = experiment.cli_id;
    entry.display_name = experiment.display_name;
    entry.input_hash = input_hash;
    entry.strand_tree_path = strand_tree_dst;
    entry.mesh_buffer_bin_path = bin_dst;
    entry.assets_dir = assets_dir;
    result.push_back(std::move(entry));
  }

  return result;
}
