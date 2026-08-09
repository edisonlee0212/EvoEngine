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

std::string StripShaderComments(const std::string& source) {
  std::string result = source;
  bool line_comment = false;
  bool block_comment = false;
  for (size_t index = 0; index < result.size(); ++index) {
    if (line_comment) {
      if (result[index] == '\n') {
        line_comment = false;
      } else {
        result[index] = ' ';
      }
    } else if (block_comment) {
      if (index + 1 < result.size() && result[index] == '*' && result[index + 1] == '/') {
        result[index++] = ' ';
        result[index] = ' ';
        block_comment = false;
      } else if (result[index] != '\n') {
        result[index] = ' ';
      }
    } else if (index + 1 < result.size() && result[index] == '/' && result[index + 1] == '/') {
      result[index++] = ' ';
      result[index] = ' ';
      line_comment = true;
    } else if (index + 1 < result.size() && result[index] == '/' && result[index + 1] == '*') {
      result[index++] = ' ';
      result[index] = ' ';
      block_comment = true;
    }
  }
  return result;
}

bool UsesCompatibilitySyntax(const std::string& source) {
  const auto uncommented = StripShaderComments(source);
  return uncommented.find("#extension GL_") != std::string::npos || uncommented.find("layout(") != std::string::npos ||
         uncommented.find("layout (") != std::string::npos ||
         uncommented.find("precision highp") != std::string::npos ||
         uncommented.find("readonly buffer") != std::string::npos ||
         uncommented.find("writeonly buffer") != std::string::npos;
}

bool EndsWith(const std::string_view value, const std::string_view suffix) {
  return value.size() >= suffix.size() && value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string VariantDefines(const std::filesystem::path& path) {
  const auto relative = path.generic_string();
  const std::string no_bindless = "\n#define EE_SKIP_PER_FRAME_BINDLESS_TEXTURES 1\n";
  const std::string material_no_bindless = no_bindless;
  const std::string fixed_lighting =
      no_bindless + "#define EE_RASTER_FIXED_LIGHTING_TEXTURES 1\n#define EE_RASTER_FIXED_LIGHTING_TEXTURE_SET 3\n";
  const std::string fixed_material_lighting =
      no_bindless + "#define EE_RASTER_FIXED_LIGHTING_TEXTURES 1\n#define EE_RASTER_FIXED_LIGHTING_TEXTURE_SET 4\n";

  if (relative.find("Graphics/Vertex/Standard/") != std::string::npos ||
      relative.find("Graphics/Task/Standard/") != std::string::npos ||
      relative.find("Graphics/Mesh/Standard/") != std::string::npos) {
    return no_bindless;
  }
  if (EndsWith(relative, "Graphics/Fragment/Standard/StandardDeferred.slang") ||
      EndsWith(relative, "Graphics/Fragment/Standard/SkinnedMotionVectors.slang") ||
      EndsWith(relative, "Graphics/Fragment/Standard/TransparentMotionVectors.slang")) {
    return material_no_bindless;
  }
  if (EndsWith(relative, "Graphics/Fragment/Standard/StandardDeferredLighting.slang") ||
      EndsWith(relative, "Graphics/Fragment/Standard/StandardDeferredLightingSceneCamera.slang") ||
      EndsWith(relative, "Graphics/Fragment/Standard/DDGIGatherTiming.slang")) {
    return fixed_lighting;
  }
  if (EndsWith(relative, "Graphics/Fragment/Standard/StandardTransparent.slang")) {
    return fixed_material_lighting;
  }
  return {};
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
  record["dialect"] = UsesCompatibilitySyntax(source) ? "compatibility" : "native";
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
  Shader::RegisterShaderIncludePath(shader_root / "Includes");
  Shader::RegisterShaderIncludePath(shader_root / "Modules");
  const auto shader_paths = CollectShaderEntryPoints(shader_root);
  ASSERT_FALSE(shader_paths.empty());
  const auto* capture_directory = std::getenv("EVOENGINE_SHADER_BASELINE_CAPTURE_DIR");
  const std::filesystem::path capture_root = capture_directory ? capture_directory : "";
  const std::string global_defines = ShaderGlobalDefinesForTests();

  for (size_t index = 0; index < shader_paths.size(); ++index) {
    const auto& shader_path = shader_paths[index];
    const auto shader_type = InferShaderType(shader_path);
    ASSERT_TRUE(shader_type.has_value()) << shader_path.string();
    const auto variant_defines = VariantDefines(std::filesystem::relative(shader_path, shader_root));
    const auto source = global_defines + variant_defines + ReadTextFile(shader_path);
    std::vector<uint32_t> binaries;
    ASSERT_TRUE(Shader::CompileToSpirv(*shader_type, source, binaries, shader_path)) << shader_path.string();
    ASSERT_FALSE(binaries.empty()) << shader_path.string();
    EXPECT_EQ(binaries.front(), 0x07230203u) << shader_path.string();
    if (capture_directory) {
      CaptureArtifact(capture_root, index, std::filesystem::relative(shader_path, shader_root), *shader_type,
                      variant_defines, source, binaries);
    }
  }

  const auto stats = Shader::GetCompileCacheStats();
  nlohmann::ordered_json summary = {
      {"entry_point_count", shader_paths.size()},
      {"memory_hits", stats.memory_hits},
      {"disk_hits", stats.disk_hits},
      {"disk_misses", stats.disk_misses},
      {"compilations", stats.compilations},
      {"coalesced_waits", stats.coalesced_waits},
      {"corrupt_entries", stats.corrupt_entries},
      {"failures", stats.failures},
      {"native_slang_frontend_invocations", stats.native_slang_frontend_invocations},
      {"compatibility_slang_frontend_invocations", stats.compatibility_slang_frontend_invocations},
      {"glslang_frontend_invocations", stats.glslang_frontend_invocations},
  };
  std::cout << "EVOENGINE_SHADER_BASELINE_STATS " << summary.dump() << std::endl;
}
