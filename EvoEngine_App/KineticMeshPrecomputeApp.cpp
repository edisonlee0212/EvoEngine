#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "kinDS/kinDS/Logger.hpp"
#include "kinDS/kinDS/MeshingBuffer.hpp"
#include "kinDS/kinDS/SegmentBuilder.hpp"
#include "kinDS/kinDS/StrandTree.hpp"
#include "kinDS/kinDS/TreeMesher.hpp"

namespace {

struct JobConfig {
  std::filesystem::path strand_tree_path;
  std::filesystem::path stats_csv_path = "meshing_statistics.csv";
  std::filesystem::path log_path;
  std::filesystem::path mesh_buffer_bin;
  std::string experiment_tag = "precompute";
  std::string description;
  std::string input_hash;
  double alpha_cutoff = 10.0;
  double branch_alpha_cutoff = 10.0;
  size_t look_ahead = 0;
  int workers = 4;
  glm::mat4 root_transform{1.0f};
  bool has_root_transform = false;
};

void PrintUsage() {
  std::cerr
      << "Usage: KineticMeshPrecomputeApp --job <job.txt>\n"
      << "   or: KineticMeshPrecomputeApp --strand-tree <path> [options]\n"
      << "Minimal kinDS meshing worker (no EvoEngine).\n"
      << "Job file keys (one per line, key=value):\n"
      << "  strand_tree, stats_csv, log, experiment_tag, description,\n"
      << "  alpha_cutoff, branch_alpha_cutoff, look_ahead, workers,\n"
      << "  input_hash, mesh_buffer_bin, root_transform (16 comma-separated floats, column-major)\n";
}

bool ParseRootTransform(const std::string& value, glm::mat4& out) {
  std::stringstream ss(value);
  std::string token;
  float values[16]{};
  int count = 0;
  while (std::getline(ss, token, ',')) {
    if (count >= 16) {
      return false;
    }
    values[count++] = std::stof(token);
  }
  if (count != 16) {
    return false;
  }
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      out[col][row] = values[col * 4 + row];
    }
  }
  return true;
}

bool ParseKeyValueFile(const std::filesystem::path& path, JobConfig& out) {
  std::ifstream in(path);
  if (!in) {
    std::cerr << "Failed to open job file: " << path << "\n";
    return false;
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const auto eq = line.find('=');
    if (eq == std::string::npos) {
      continue;
    }
    const std::string key = line.substr(0, eq);
    const std::string value = line.substr(eq + 1);
    if (key == "strand_tree") {
      out.strand_tree_path = value;
    } else if (key == "stats_csv") {
      out.stats_csv_path = value;
    } else if (key == "log") {
      out.log_path = value;
    } else if (key == "experiment_tag") {
      out.experiment_tag = value;
    } else if (key == "description") {
      out.description = value;
    } else if (key == "alpha_cutoff") {
      out.alpha_cutoff = std::stod(value);
    } else if (key == "branch_alpha_cutoff") {
      out.branch_alpha_cutoff = std::stod(value);
    } else if (key == "look_ahead") {
      out.look_ahead = static_cast<size_t>(std::stoull(value));
    } else if (key == "workers") {
      out.workers = std::stoi(value);
    } else if (key == "input_hash") {
      out.input_hash = value;
    } else if (key == "mesh_buffer_bin") {
      out.mesh_buffer_bin = value;
    } else if (key == "root_transform") {
      if (!ParseRootTransform(value, out.root_transform)) {
        std::cerr << "Invalid root_transform (need 16 comma-separated floats)\n";
        return false;
      }
      out.has_root_transform = true;
    }
  }
  return !out.strand_tree_path.empty();
}

bool ParseArgs(int argc, char** argv, JobConfig& out) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::cerr << "Missing value for " << name << "\n";
        return nullptr;
      }
      return argv[++i];
    };
    if (arg == "--job") {
      const char* value = need("--job");
      if (!value) {
        return false;
      }
      return ParseKeyValueFile(value, out);
    }
    if (arg == "--strand-tree") {
      const char* value = need("--strand-tree");
      if (!value) {
        return false;
      }
      out.strand_tree_path = value;
    } else if (arg == "--stats-csv") {
      const char* value = need("--stats-csv");
      if (!value) {
        return false;
      }
      out.stats_csv_path = value;
    } else if (arg == "--log") {
      const char* value = need("--log");
      if (!value) {
        return false;
      }
      out.log_path = value;
    } else if (arg == "--experiment-tag") {
      const char* value = need("--experiment-tag");
      if (!value) {
        return false;
      }
      out.experiment_tag = value;
    } else if (arg == "--alpha-cutoff") {
      const char* value = need("--alpha-cutoff");
      if (!value) {
        return false;
      }
      out.alpha_cutoff = std::stod(value);
      out.branch_alpha_cutoff = out.alpha_cutoff;
    } else if (arg == "--branch-alpha-cutoff") {
      const char* value = need("--branch-alpha-cutoff");
      if (!value) {
        return false;
      }
      out.branch_alpha_cutoff = std::stod(value);
    } else if (arg == "--workers") {
      const char* value = need("--workers");
      if (!value) {
        return false;
      }
      out.workers = std::stoi(value);
    } else if (arg == "--mesh-buffer-bin") {
      const char* value = need("--mesh-buffer-bin");
      if (!value) {
        return false;
      }
      out.mesh_buffer_bin = value;
    } else if (arg == "--input-hash") {
      const char* value = need("--input-hash");
      if (!value) {
        return false;
      }
      out.input_hash = value;
    } else if (arg == "--help" || arg == "-h") {
      PrintUsage();
      return false;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      PrintUsage();
      return false;
    }
  }
  if (out.strand_tree_path.empty()) {
    PrintUsage();
    return false;
  }
  return true;
}

void SetupLogging(const JobConfig& config) {
  if (config.log_path.empty()) {
    return;
  }
  std::error_code ec;
  std::filesystem::create_directories(config.log_path.parent_path(), ec);
  if (kinDS::logger.setLogFile(config.log_path.string())) {
    kinDS::logger.setConsoleEnabled(false);
    KINDS_INFO("KineticMeshPrecomputeApp file logging: " << config.log_path.string());
  }
}

std::function<void(size_t, std::function<void(size_t)>)> MakeParallelFor(const int workers) {
  const int worker_count = std::max(1, workers);
  if (worker_count == 1) {
    return [](size_t count, std::function<void(size_t)> func) {
      for (size_t i = 0; i < count; ++i) {
        func(i);
      }
    };
  }
  return [worker_count](size_t count, std::function<void(size_t)> func) {
    if (count == 0) {
      return;
    }
    const size_t threads = std::min(static_cast<size_t>(worker_count), count);
    std::vector<std::thread> pool;
    pool.reserve(threads);
    for (size_t t = 0; t < threads; ++t) {
      pool.emplace_back([&, t]() {
        for (size_t i = t; i < count; i += threads) {
          func(i);
        }
      });
    }
    for (auto& thread : pool) {
      thread.join();
    }
  };
}

int RunJob(const JobConfig& config) {
  if (!std::filesystem::exists(config.strand_tree_path)) {
    KINDS_ERROR("StrandTree file missing: " << config.strand_tree_path.string());
    return 1;
  }

  std::filesystem::path bin_path = config.mesh_buffer_bin;
  if (bin_path.empty()) {
    if (config.input_hash.empty()) {
      KINDS_ERROR("mesh_buffer_bin or input_hash required to write MeshBuffers");
      return 1;
    }
    bin_path = std::filesystem::path("MeshBuffers") / (config.input_hash + ".bin");
  }

  if (std::filesystem::exists(bin_path)) {
    KINDS_INFO("MeshBuffers cache already present, skipping meshing: " << bin_path.string());
    return 0;
  }

  KINDS_INFO("Loading StrandTree: " << config.strand_tree_path.string());
  kinDS::StrandTree strand_tree = kinDS::StrandTree::loadFromFile(config.strand_tree_path);

  kinDS::TreeMesher mesher(strand_tree, MakeParallelFor(config.workers));
  auto& settings = mesher.getSettings();
  settings.transform_mesh_at_construction = true;
  settings.mesh_cap_at_start = true;
  settings.alpha_cutoff = config.alpha_cutoff;
  settings.branch_alpha_cutoff = config.branch_alpha_cutoff;
  settings.look_ahead = config.look_ahead;
  settings.collect_meshing_statistics = true;
  settings.defer_meshing_statistics_write = false;
  settings.flush_meshing_statistics_each_section = true;
  settings.meshing_statistics_csv_path = config.stats_csv_path;
  settings.meshing_statistics_experiment_tag = config.experiment_tag;
  settings.debug_export_meshes = false;

  if (!config.stats_csv_path.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(config.stats_csv_path.parent_path(), ec);
  }

  KINDS_INFO("Meshing experiment=" << config.experiment_tag << " alpha=" << config.alpha_cutoff
                                   << " branch_alpha=" << config.branch_alpha_cutoff
                                   << " workers=" << config.workers
                                   << (config.description.empty() ? "" : (" desc=" + config.description)));
  mesher.runMeshingAlgorithm(false);
  auto& meshlets = mesher.getSegmentMeshlets();
  auto& neighbors = mesher.getMeshingNeighborIndices();
  kinDS::closeCrossMeshletTJunctions(meshlets, neighbors);
  kinDS::closeIntraMeshletTJunctions(meshlets, neighbors);
  KINDS_INFO("Meshing finished: " << meshlets.size() << " meshlets for " << config.experiment_tag);

  kinDS::MeshingBufferPayload payload;
  payload.root_transform = config.has_root_transform ? config.root_transform : glm::mat4(1.0f);
  payload.meshlets = std::move(meshlets);
  payload.neighbors = std::move(neighbors);
  payload.meshing_to_physics = mesher.getMeshingToPhysicsSegmentIndices();
  payload.strand_to_segment = mesher.getMeshingStrandToSegmentIndices();
  // GPU blobs are EcoSysLab-only; parent rebuilds them on cache hit.
  payload.gpu_vertex_stride = 0;
  payload.gpu_triangle_stride = 0;

  kinDS::MeshingBufferMetadata metadata;
  metadata.input_hash = config.input_hash.empty() ? bin_path.stem().string() : config.input_hash;
  metadata.description = config.description;
  metadata.alpha_cutoff = config.alpha_cutoff;
  metadata.branch_alpha_cutoff = config.branch_alpha_cutoff;
  metadata.look_ahead = config.look_ahead;

  if (!kinDS::saveMeshingBuffer(bin_path, payload, metadata)) {
    KINDS_ERROR("Failed to save MeshBuffers to " << bin_path.string());
    return 1;
  }
  KINDS_INFO("Wrote MeshBuffers " << bin_path.string());
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  JobConfig config;
  if (!ParseArgs(argc, argv, config)) {
    return 1;
  }
  SetupLogging(config);
  try {
    return RunJob(config);
  } catch (const std::exception& ex) {
    KINDS_ERROR("KineticMeshPrecomputeApp exception: " << ex.what());
    std::cerr << "KineticMeshPrecomputeApp exception: " << ex.what() << "\n";
    return 1;
  } catch (...) {
    KINDS_ERROR("KineticMeshPrecomputeApp unknown exception");
    std::cerr << "KineticMeshPrecomputeApp unknown exception\n";
    return 1;
  }
}
