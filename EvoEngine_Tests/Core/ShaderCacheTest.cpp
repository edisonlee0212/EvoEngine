#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "Shader.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
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

class ShaderCacheScope {
 public:
  ShaderCacheScope() {
    if (const auto* previous = std::getenv("EVOENGINE_SHADER_CACHE_DIR"))
      previous_cache_directory_ = previous;
    const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = std::filesystem::temp_directory_path() / ("EvoEngineShaderCacheTest_" + std::to_string(suffix));
    std::filesystem::create_directories(root_);
    SetEnvironment("EVOENGINE_SHADER_CACHE_DIR", root_.string());
    application_ = std::make_unique<Application>();
    Shader::ClearInMemoryCompileCache();
    Shader::ResetCompileCacheStats();
  }

  ~ShaderCacheScope() {
    Shader::ClearInMemoryCompileCache();
    application_.reset();
    if (previous_cache_directory_)
      SetEnvironment("EVOENGINE_SHADER_CACHE_DIR", *previous_cache_directory_);
    else
      ClearEnvironment("EVOENGINE_SHADER_CACHE_DIR");
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] const std::filesystem::path& Root() const {
    return root_;
  }

 private:
  std::filesystem::path root_;
  std::optional<std::string> previous_cache_directory_;
  std::unique_ptr<Application> application_;
};

constexpr const char* kComputeShader = R"(
layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;
void main() {}
)";

constexpr const char* kComputeShaderGlobalDefines = R"(
#define SUBGROUP_SIZE 32
#define COMPUTE_SUBGROUP_COUNT 8
#define COMPUTE_WORK_GROUP_INVOCATIONS 256
#define MAX_COMPUTE_WORK_GROUP_INVOCATIONS 1024
)";

size_t CacheFileCount(const std::filesystem::path& root) {
  return static_cast<size_t>(std::count_if(std::filesystem::directory_iterator(root),
                                           std::filesystem::directory_iterator(), [](const auto& entry) {
                                             return entry.path().extension() == ".spvbin";
                                           }));
}

std::filesystem::path FirstCacheFile(const std::filesystem::path& root) {
  const auto entry = std::find_if(std::filesystem::directory_iterator(root), std::filesystem::directory_iterator(),
                                  [](const auto& candidate) {
                                    return candidate.path().extension() == ".spvbin";
                                  });
  return entry == std::filesystem::directory_iterator() ? std::filesystem::path() : entry->path();
}
}  // namespace

TEST(ShaderCache, UsesMemoryThenValidatedDiskEntry) {
  ShaderCacheScope scope;
  std::vector<uint32_t> first;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kComputeShader, first));
  ASSERT_FALSE(first.empty());
  auto stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.compilations, 1u);
  EXPECT_EQ(stats.disk_misses, 1u);

  std::vector<uint32_t> second;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kComputeShader, second));
  EXPECT_EQ(first, second);
  EXPECT_EQ(Shader::GetCompileCacheStats().memory_hits, 1u);

  Shader::ClearInMemoryCompileCache();
  Shader::ResetCompileCacheStats();
  std::vector<uint32_t> disk;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kComputeShader, disk));
  EXPECT_EQ(first, disk);
  stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.disk_hits, 1u);
  EXPECT_EQ(stats.compilations, 0u);
}

TEST(ShaderCache, ColdCompileReplacesCallerOutput) {
  ShaderCacheScope scope;
  std::vector<uint32_t> binaries{0x07230203u, 0xdeadbeefu};
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kComputeShader, binaries));
  ASSERT_GT(binaries.size(), 2u);
  EXPECT_EQ(binaries.front(), 0x07230203u);
  EXPECT_NE(binaries[1], 0xdeadbeefu);
}

TEST(ShaderCache, CoalescesConcurrentRequests) {
  ShaderCacheScope scope;
  auto* application = ApplicationContext::TryGet();
  ASSERT_NE(application, nullptr);
  constexpr size_t request_count = 16;
  std::vector<std::future<std::vector<uint32_t>>> requests;
  requests.reserve(request_count);
  for (size_t i = 0; i < request_count; ++i) {
    requests.emplace_back(std::async(std::launch::async, [application]() {
      ApplicationContextScope application_scope(*application);
      std::vector<uint32_t> binaries;
      if (!Shader::CompileToSpirv(ShaderType::Compute, kComputeShader, binaries))
        binaries.clear();
      return binaries;
    }));
  }
  const auto expected = requests.front().get();
  ASSERT_FALSE(expected.empty());
  for (size_t i = 1; i < requests.size(); ++i)
    EXPECT_EQ(requests[i].get(), expected);
  const auto stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.compilations, 1u);
  EXPECT_EQ(stats.memory_hits + stats.coalesced_waits, request_count - 1u);
}

TEST(ShaderCache, StageAndSourceChangesCreateDistinctEntries) {
  ShaderCacheScope scope;
  std::vector<uint32_t> compute;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kComputeShader, compute));
  std::vector<uint32_t> fragment;
  ASSERT_TRUE(Shader::CompileToSpirv(
      ShaderType::Fragment, "layout(location = 0) out vec4 color; void main() { color = vec4(1.0); }", fragment));
  std::vector<uint32_t> changed_source;
  ASSERT_TRUE(Shader::CompileToSpirv(
      ShaderType::Compute, "layout(local_size_x = 1) in; void main() { uint value = gl_GlobalInvocationID.x + 1u; }",
      changed_source));
  EXPECT_EQ(CacheFileCount(scope.Root()), 3u);
  EXPECT_EQ(Shader::GetCompileCacheStats().compilations, 3u);
}

TEST(ShaderCache, EcoSysLabPackedFungusEdgeShaderCompiles) {
  ShaderCacheScope scope;
  Shader::RegisterShaderIncludePath(
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Includes"));
  Shader::RegisterShaderIncludePath(RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes"));

  const auto shader_path = RepoPath(
      "EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Compute/DynamicStrands/Fungus/"
      "FungusDiffusion_edge.comp");
  const auto source = std::string(kComputeShaderGlobalDefines) + ReadTextFile(shader_path);

  std::vector<uint32_t> binaries;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries, shader_path));
  EXPECT_FALSE(binaries.empty());
}

TEST(ShaderCache, IncludeContentInvalidatesDeterministicKey) {
  ShaderCacheScope scope;
  const auto include_path = scope.Root() / "m5_shader_cache_include.glsl";
  {
    std::ofstream file(include_path);
    file << "float M5_INCLUDED_VALUE() { return 1.0; }\n";
  }
  Shader::RegisterShaderIncludePath(scope.Root());
  const std::string source = R"(
#extension GL_GOOGLE_include_directive : require
#include "m5_shader_cache_include.glsl"
layout(local_size_x = 1, local_size_y = 1, local_size_z = 1) in;
void main() { float value = M5_INCLUDED_VALUE(); }
)";
  std::vector<uint32_t> first;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, first));
  {
    std::ofstream file(include_path, std::ios::trunc);
    file << "float M5_INCLUDED_VALUE() { return 2.0; }\n";
  }
  Shader::ClearInMemoryCompileCache();
  Shader::ResetCompileCacheStats();
  std::vector<uint32_t> second;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, second));
  EXPECT_NE(first, second);
  EXPECT_EQ(Shader::GetCompileCacheStats().compilations, 1u);
  EXPECT_EQ(CacheFileCount(scope.Root()), 2u);
}

TEST(ShaderCache, CorruptEntryIsRecompiledWithoutThrowing) {
  ShaderCacheScope scope;
  std::vector<uint32_t> first;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kComputeShader, first));
  const auto cache_file = FirstCacheFile(scope.Root());
  ASSERT_FALSE(cache_file.empty());
  {
    std::ofstream file(cache_file, std::ios::binary | std::ios::trunc);
    file << "truncated";
  }
  Shader::ClearInMemoryCompileCache();
  Shader::ResetCompileCacheStats();
  std::vector<uint32_t> repaired;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kComputeShader, repaired));
  EXPECT_EQ(first, repaired);
  const auto stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.corrupt_entries, 1u);
  EXPECT_EQ(stats.compilations, 1u);
}

TEST(ShaderCache, FailedCompilationIsNotPublishedAndCanRetry) {
  ShaderCacheScope scope;
  constexpr const char* invalid_shader = "layout(local_size_x = 1) in; void main( {";
  std::vector<uint32_t> binaries;
  EXPECT_FALSE(Shader::CompileToSpirv(ShaderType::Compute, invalid_shader, binaries));
  EXPECT_EQ(CacheFileCount(scope.Root()), 0u);
  binaries.clear();
  EXPECT_FALSE(Shader::CompileToSpirv(ShaderType::Compute, invalid_shader, binaries));
  const auto stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.compilations, 2u);
  EXPECT_EQ(stats.failures, 2u);
}
