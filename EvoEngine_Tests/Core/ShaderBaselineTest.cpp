#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "Platform.hpp"
#include "Shader.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

using namespace evo_engine;

namespace {
std::filesystem::path RepoPath(const std::filesystem::path& relative_path) {
  return std::filesystem::path(EVOENGINE_TEST_SOURCE_DIR) / relative_path;
}

std::string ReadTextFile(const std::filesystem::path& path) {
  std::ifstream file(path);
  EXPECT_TRUE(file.good()) << path.string();
  return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

void SetEnvironment(const char* name, const std::string& value) {
#ifdef _WIN32
  _putenv_s(name, value.c_str());
#else
  setenv(name, value.c_str(), 1);
#endif
}

void ClearEnvironment(const char* name) {
#ifdef _WIN32
  _putenv_s(name, "");
#else
  unsetenv(name);
#endif
}

class ShaderBaselineScope {
 public:
  ShaderBaselineScope() {
    if (const auto* previous = std::getenv("EVOENGINE_SHADER_CACHE_DIR")) {
      previous_cache_directory_ = previous;
    }
    if (const auto* requested = std::getenv("EVOENGINE_SHADER_BASELINE_CACHE_DIR")) {
      root_ = requested;
    } else {
      const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
      root_ = std::filesystem::temp_directory_path() / ("EvoEngineShaderBaseline_" + std::to_string(suffix));
      remove_root_ = true;
    }
    std::filesystem::create_directories(root_);
    SetEnvironment("EVOENGINE_SHADER_CACHE_DIR", root_.string());
    application_ = std::make_unique<Application>();
    Shader::ClearInMemoryCompileCache();
    Shader::ResetCompileCacheStats();
  }

  ~ShaderBaselineScope() {
    Shader::ClearInMemoryCompileCache();
    application_.reset();
    if (previous_cache_directory_) {
      SetEnvironment("EVOENGINE_SHADER_CACHE_DIR", *previous_cache_directory_);
    } else {
      ClearEnvironment("EVOENGINE_SHADER_CACHE_DIR");
    }
    if (remove_root_) {
      std::error_code error;
      std::filesystem::remove_all(root_, error);
    }
  }

 private:
  std::filesystem::path root_;
  bool remove_root_ = false;
  std::optional<std::string> previous_cache_directory_;
  std::unique_ptr<Application> application_;
};

std::string ShaderGlobalDefinesForTests() {
  if (auto platform_defines = Platform::GetShaderGlobalDefines(); !platform_defines.empty()) {
    return platform_defines;
  }
  std::ostringstream stream;
  stream << "\n#define MAX_DIRECTIONAL_LIGHT_SIZE 4"
         << "\n#define MAX_KERNEL_AMOUNT " << Platform::Constants::max_kernel_amount
         << "\n#define MESHLET_MAX_VERTICES_SIZE " << Platform::Constants::meshlet_max_vertices_size
         << "\n#define MESHLET_MAX_TRIANGLES_SIZE " << Platform::Constants::meshlet_max_triangles_size
         << "\n#define MESHLET_MAX_INDICES_SIZE " << Platform::Constants::meshlet_max_triangles_size * 3
         << "\n#define SUBGROUP_SIZE 32"
         << "\n#define COMPUTE_SUBGROUP_COUNT 32"
         << "\n#define COMPUTE_WORK_GROUP_INVOCATIONS 1024"
         << "\n#define MAX_COMPUTE_WORK_GROUP_INVOCATIONS 1024"
         << "\n#define EXT_TASK_SUBGROUP_COUNT 1"
         << "\n#define EXT_MESH_SUBGROUP_COUNT 1"
         << "\n#define EXT_TASK_WORK_GROUP_INVOCATIONS 32"
         << "\n#define EE_SHADER_EXECUTION_REORDERING_SUPPORTED 0"
         << "\n#define EE_SHADER_FLOAT16_SUPPORTED 0\n";
  return stream.str();
}

std::optional<ShaderType> InferShaderType(const std::filesystem::path& path) {
  const auto generic_path = path.generic_string();
  if (generic_path.find("/Compute/") != std::string::npos)
    return ShaderType::Compute;
  if (generic_path.find("/Graphics/Vertex/") != std::string::npos)
    return ShaderType::Vertex;
  if (generic_path.find("/Graphics/Fragment/") != std::string::npos)
    return ShaderType::Fragment;
  if (generic_path.find("/Graphics/Mesh/") != std::string::npos)
    return ShaderType::Mesh;
  if (generic_path.find("/Graphics/Task/") != std::string::npos)
    return ShaderType::Task;
  if (generic_path.find("/RayTracing/RayGen/") != std::string::npos)
    return ShaderType::RayGen;
  if (generic_path.find("/RayTracing/Miss/") != std::string::npos)
    return ShaderType::Miss;
  if (generic_path.find("/RayTracing/ClosestHit/") != std::string::npos)
    return ShaderType::ClosestHit;
  if (generic_path.find("/RayTracing/AnyHit/") != std::string::npos)
    return ShaderType::AnyHit;
  return std::nullopt;
}

const char* ShaderTypeName(const ShaderType shader_type) {
  switch (shader_type) {
    case ShaderType::Vertex:
      return "vertex";
    case ShaderType::Task:
      return "task";
    case ShaderType::Mesh:
      return "mesh";
    case ShaderType::Fragment:
      return "fragment";
    case ShaderType::Compute:
      return "compute";
    case ShaderType::RayGen:
      return "raygen";
    case ShaderType::ClosestHit:
      return "closest_hit";
    case ShaderType::Miss:
      return "miss";
    case ShaderType::AnyHit:
      return "any_hit";
    default:
      return "unknown";
  }
}

std::vector<std::string> VariantDefines(const std::filesystem::path& path) {
  const auto name = path.filename().string();
  if (name == "SdfgiGatherAbi.slang")
    return {"#define EE_SDFGI_ABI_ONLY 1\n"};
  if (name == "SdfgiDirectLight.slang")
    return {"#define MODE_PROCESS_STATIC 1\n", "#define MODE_PROCESS_DYNAMIC 1\n"};
  if (name == "SdfgiIntegrate.slang")
    return {"#define MODE_PROCESS 1\n", "#define MODE_STORE 1\n", "#define MODE_SCROLL 1\n"};
  if (name == "SdfgiPreprocess.slang") {
    std::vector<std::string> variants;
    for (const auto mode : {"INITIALIZE_JUMP_FLOOD", "INITIALIZE_JUMP_FLOOD_HALF", "JUMPFLOOD", "JUMPFLOOD_OPTIMIZED",
                            "UPSCALE_JUMP_FLOOD", "OCCLUSION", "STORE", "SCROLL", "SCROLL_OCCLUSION"})
      variants.push_back(std::string("#define MODE_") + mode + " 1\n");
    return variants;
  }
  if (name == "HddagiGatherAbi.slang")
    return {"", "#define MODE_FULL_GATHER 1\n"};
  if (name == "HddagiDirectLight.slang")
    return {"", "#define MODE_PROCESS_STATIC 1\n"};
  if (name == "HddagiTransportStatus.slang")
    return {"", "#define MODE_BEGIN 1\n"};
  if (name == "HddagiReflectionFilter.slang")
    return {"", "#define MODE_VERTICAL 1\n"};
  if (name == "DeferredComputeLighting.slang")
    return {"", "#define EE_AUTOMATIC_SDFGI 1\n", "#define EE_AUTOMATIC_HDDAGI 1\n"};
  return {""};
}

std::vector<std::filesystem::path> CollectShaderEntryPoints(const std::filesystem::path& root) {
  std::vector<std::filesystem::path> paths;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    if (entry.is_regular_file() && entry.path().extension() == ".slang" && InferShaderType(entry.path())) {
      paths.emplace_back(entry.path());
    }
  }
  std::sort(paths.begin(), paths.end());
  return paths;
}

nlohmann::ordered_json ReflectionJson(const ShaderReflectionInfo& reflection) {
  nlohmann::ordered_json result;
  result["entry_point"] = reflection.entry_point;
  result["compute_thread_group_size"] = reflection.compute_thread_group_size;
  result["descriptor_bindings"] = nlohmann::ordered_json::array();
  for (const auto& binding : reflection.descriptor_bindings) {
    result["descriptor_bindings"].push_back({
        {"name", binding.name},
        {"set", binding.set},
        {"binding", binding.binding},
        {"descriptor_type", static_cast<uint32_t>(binding.descriptor_type)},
        {"descriptor_count", binding.descriptor_count},
        {"stage_flags", binding.stage_flags},
    });
  }
  result["push_constant_ranges"] = nlohmann::ordered_json::array();
  for (const auto& range : reflection.push_constant_ranges) {
    result["push_constant_ranges"].push_back({
        {"name", range.name},
        {"offset", range.offset},
        {"size", range.size},
        {"stage_flags", range.stage_flags},
    });
  }
  const auto stage_io_json = [](const std::vector<ShaderReflectionStageIo>& entries) {
    nlohmann::ordered_json values = nlohmann::ordered_json::array();
    for (const auto& entry : entries) {
      values.push_back({
          {"name", entry.name},
          {"semantic_name", entry.semantic_name},
          {"semantic_index", entry.semantic_index},
          {"location", entry.location},
      });
    }
    return values;
  };
  result["stage_inputs"] = stage_io_json(reflection.stage_inputs);
  result["stage_outputs"] = stage_io_json(reflection.stage_outputs);
  return result;
}

void CaptureArtifact(const std::filesystem::path& capture_root, const size_t index,
                     const std::filesystem::path& relative_path, const ShaderType shader_type,
                     const std::string& variant_defines, const std::string& source,
                     const std::vector<uint32_t>& binaries) {
  ShaderReflectionInfo reflection;
  std::string diagnostics;
  ASSERT_TRUE(Shader::ReflectSlang(shader_type, source, reflection, diagnostics,
                                   RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders") / relative_path))
      << relative_path.string() << "\n"
      << diagnostics;

  std::filesystem::create_directories(capture_root / "records");
  std::filesystem::create_directories(capture_root / "spirv");
  std::ostringstream stem;
  stem << std::setfill('0') << std::setw(4) << index;
  const auto record_path = capture_root / "records" / (stem.str() + ".json");
  const auto spirv_path = capture_root / "spirv" / (stem.str() + ".spv");

  std::ofstream spirv_stream(spirv_path, std::ios::binary | std::ios::trunc);
  spirv_stream.write(reinterpret_cast<const char*>(binaries.data()),
                     static_cast<std::streamsize>(binaries.size() * sizeof(uint32_t)));
  ASSERT_TRUE(spirv_stream.good()) << spirv_path.string();

  nlohmann::ordered_json record;
  record["path"] = relative_path.generic_string();
  record["stage"] = ShaderTypeName(shader_type);
  record["variant_defines"] = variant_defines;
  record["spirv"] = std::filesystem::relative(spirv_path, capture_root).generic_string();
  record["spirv_word_count"] = binaries.size();
  record["reflection"] = ReflectionJson(reflection);
  std::ofstream record_stream(record_path, std::ios::trunc);
  record_stream << record.dump(2) << '\n';
  ASSERT_TRUE(record_stream.good()) << record_path.string();
}
}  // namespace

TEST(ShaderBaseline, ProductionSdkEntryPointInventoryCompilesAndReflects) {
  ShaderBaselineScope scope;
  const auto shader_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders");
  Shader::RegisterShaderIncludePath(shader_root / "Modules");
  const auto shader_paths = CollectShaderEntryPoints(shader_root);
  ASSERT_FALSE(shader_paths.empty());
  const auto* capture_directory = std::getenv("EVOENGINE_SHADER_BASELINE_CAPTURE_DIR");
  const std::filesystem::path capture_root = capture_directory ? capture_directory : "";
  const std::string global_defines = ShaderGlobalDefinesForTests();

  size_t variant_index = 0;
  for (size_t index = 0; index < shader_paths.size(); ++index) {
    const auto& shader_path = shader_paths[index];
    const auto shader_type = InferShaderType(shader_path);
    ASSERT_TRUE(shader_type.has_value()) << shader_path.string();
    for (const auto& variant_defines : VariantDefines(std::filesystem::relative(shader_path, shader_root))) {
      const auto source = global_defines + variant_defines + ReadTextFile(shader_path);
      std::vector<uint32_t> binaries;
      ASSERT_TRUE(Shader::CompileToSpirv(*shader_type, source, binaries, shader_path)) << shader_path.string();
      ASSERT_FALSE(binaries.empty()) << shader_path.string();
      EXPECT_EQ(binaries.front(), 0x07230203u) << shader_path.string();
      if (capture_directory) {
        CaptureArtifact(capture_root, variant_index, std::filesystem::relative(shader_path, shader_root), *shader_type,
                        variant_defines, source, binaries);
      }
      ++variant_index;
    }
  }

  const auto stats = Shader::GetCompileCacheStats();
  nlohmann::ordered_json summary = {
      {"entry_point_count", shader_paths.size()},
      {"variant_count", variant_index},
      {"memory_hits", stats.memory_hits},
      {"disk_hits", stats.disk_hits},
      {"disk_misses", stats.disk_misses},
      {"compilations", stats.compilations},
      {"coalesced_waits", stats.coalesced_waits},
      {"corrupt_entries", stats.corrupt_entries},
      {"failures", stats.failures},
      {"native_slang_frontend_invocations", stats.native_slang_frontend_invocations},
  };
  std::cout << "EVOENGINE_SHADER_BASELINE_STATS " << summary.dump() << std::endl;
}
