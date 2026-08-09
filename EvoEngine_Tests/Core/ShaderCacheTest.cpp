#include "EvoEngine_SDK_PCH.hpp"

#include "Application.hpp"
#include "ApplicationContext.hpp"
#include "Platform.hpp"
#include "PostProcessingStack.hpp"
#include "RenderInstanceStorage.hpp"
#include "Shader.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
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

void WriteTextFile(const std::filesystem::path& path, const std::string_view text) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  ASSERT_TRUE(file.good()) << path.string();
  file << text;
  ASSERT_TRUE(file.good()) << path.string();
}

std::string ShaderGlobalDefinesForTests(const bool force_shader_execution_reordering = false) {
  if (auto platform_defines = Platform::GetShaderGlobalDefines(); !platform_defines.empty()) {
    if (force_shader_execution_reordering) {
      constexpr std::string_view define = "#define EE_SHADER_EXECUTION_REORDERING_SUPPORTED ";
      if (const auto begin = platform_defines.find(define); begin != std::string::npos) {
        const auto value_begin = begin + define.size();
        const auto value_end = platform_defines.find_first_of("\r\n", value_begin);
        platform_defines.replace(value_begin, value_end - value_begin, "1");
      } else {
        platform_defines += "\n#define EE_SHADER_EXECUTION_REORDERING_SUPPORTED 1\n";
      }
    }
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
         << "\n#define EE_SHADER_EXECUTION_REORDERING_SUPPORTED "
         << static_cast<uint32_t>(force_shader_execution_reordering) << "\n#define EE_SHADER_FLOAT16_SUPPORTED 0\n";
  return stream.str();
}

uint32_t ExtractUnsignedShaderDefine(const std::string& source, const std::string& name, const uint32_t fallback) {
  std::istringstream stream(source);
  std::string directive;
  std::string define_name;
  uint32_t value = fallback;
  while (stream >> directive >> define_name) {
    if (directive == "#define" && define_name == name && stream >> value) {
      return value;
    }
    stream.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
  }
  return fallback;
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

constexpr const char* kSlangComputeShader = R"(
[shader("compute")]
[numthreads(1, 1, 1)]
void main() {}
)";

constexpr const char* kComputeShaderGlobalDefines = R"(
#define SUBGROUP_SIZE 32
#define COMPUTE_SUBGROUP_COUNT 8
#define COMPUTE_WORK_GROUP_INVOCATIONS 256
#define MAX_COMPUTE_WORK_GROUP_INVOCATIONS 1024
)";

constexpr const char* kSlangMatrixPushShader = R"(
struct MatrixPush
{
    float4x4 transform;
    float4 input;
};

[[vk::push_constant]]
ConstantBuffer<MatrixPush> constants;

[[vk::binding(0, 0)]]
RWStructuredBuffer<float4> outputBuffer;

[shader("compute")]
[numthreads(1, 1, 1)]
void main()
{
    outputBuffer[0] = mul(constants.transform, constants.input);
}
)";

constexpr const char* kSlangVertexIoShader = R"(
struct VertexInput
{
    [[vk::location(0)]]
    float3 position : POSITION;
};

struct VertexOutput
{
    float4 position : SV_Position;

    [[vk::location(0)]]
    float2 uv : TEXCOORD0;
};

[shader("vertex")]
VertexOutput main(VertexInput input)
{
    VertexOutput output;
    output.position = float4(input.position, 1.0);
    output.uv = input.position.xy;
    return output;
}
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

void RegisterDefaultShaderIncludePath() {
  Shader::RegisterShaderIncludePath(RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Includes"));
  Shader::RegisterShaderIncludePath(RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules"));
}

void RegisterEcoSysLabShaderIncludePath() {
  Shader::RegisterShaderIncludePath(
      RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders/Includes"));
}

std::optional<ShaderType> InferSlangStageFromPath(const std::filesystem::path& path) {
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

std::vector<std::filesystem::path> CollectSlangStageFiles(const std::filesystem::path& root) {
  std::vector<std::filesystem::path> shader_paths;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".slang")
      continue;
    if (InferSlangStageFromPath(entry.path()))
      shader_paths.emplace_back(entry.path());
  }
  std::sort(shader_paths.begin(), shader_paths.end());
  return shader_paths;
}

std::optional<ShaderType> InferGlslStageFromPath(const std::filesystem::path& path) {
  const auto extension = path.extension().string();
  if (extension == ".comp")
    return ShaderType::Compute;
  if (extension == ".frag")
    return ShaderType::Fragment;
  if (extension == ".mesh")
    return ShaderType::Mesh;
  if (extension == ".task")
    return ShaderType::Task;
  if (extension == ".vert")
    return ShaderType::Vertex;
  return std::nullopt;
}

bool HasUncommentedGlslMain(const std::filesystem::path& path) {
  std::ifstream file(path);
  std::string line;
  while (std::getline(file, line)) {
    const auto start = line.find_first_not_of(" \t");
    if (start != std::string::npos && line.compare(start, 2, "//") == 0)
      continue;
    if (line.find("main(") != std::string::npos || line.find("main (") != std::string::npos)
      return true;
  }
  return false;
}

std::vector<std::filesystem::path> CollectGlslStageFiles(const std::filesystem::path& root) {
  std::vector<std::filesystem::path> shader_paths;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file())
      continue;
    if (InferGlslStageFromPath(entry.path()) && HasUncommentedGlslMain(entry.path()))
      shader_paths.emplace_back(entry.path());
  }
  std::sort(shader_paths.begin(), shader_paths.end());
  return shader_paths;
}

bool SpirvDeclaresComputeDerivativeCapability(const std::vector<uint32_t>& words) {
  constexpr uint32_t kOpCapability = 17;
  constexpr uint32_t kComputeDerivativeGroupQuads = 5288;
  constexpr uint32_t kComputeDerivativeGroupLinear = 5350;
  for (size_t word_index = 5; word_index < words.size();) {
    const uint32_t instruction = words[word_index];
    const uint32_t word_count = instruction >> 16;
    const uint32_t op_code = instruction & 0xffffu;
    if (word_count == 0 || word_index + word_count > words.size()) {
      return false;
    }
    if (op_code == kOpCapability && word_count >= 2) {
      const uint32_t capability = words[word_index + 1];
      if (capability == kComputeDerivativeGroupQuads || capability == kComputeDerivativeGroupLinear) {
        return true;
      }
    }
    word_index += word_count;
  }
  return false;
}

struct SpirvMatrixDecorationCounts {
  size_t row_major = 0;
  size_t column_major = 0;
};

SpirvMatrixDecorationCounts CountSpirvMatrixStorageDecorations(const std::vector<uint32_t>& words) {
  constexpr uint32_t kOpMemberDecorate = 72;
  constexpr uint32_t kRowMajorDecoration = 4;
  constexpr uint32_t kColMajorDecoration = 5;
  SpirvMatrixDecorationCounts counts;
  for (size_t word_index = 5; word_index < words.size();) {
    const uint32_t instruction = words[word_index];
    const uint32_t word_count = instruction >> 16;
    const uint32_t op_code = instruction & 0xffffu;
    if (word_count == 0 || word_index + word_count > words.size()) {
      return counts;
    }
    if (op_code == kOpMemberDecorate && word_count >= 4) {
      const uint32_t decoration = words[word_index + 3];
      if (decoration == kRowMajorDecoration) {
        ++counts.row_major;
      } else if (decoration == kColMajorDecoration) {
        ++counts.column_major;
      }
    }
    word_index += word_count;
  }
  return counts;
}

std::optional<std::array<uint32_t, 3>> FindSpirvLocalSizeExecutionMode(const std::vector<uint32_t>& words) {
  constexpr uint32_t kOpExecutionMode = 16;
  constexpr uint32_t kLocalSizeExecutionMode = 17;
  for (size_t word_index = 5; word_index < words.size();) {
    const uint32_t instruction = words[word_index];
    const uint32_t word_count = instruction >> 16;
    const uint32_t op_code = instruction & 0xffffu;
    if (word_count == 0 || word_index + word_count > words.size()) {
      return std::nullopt;
    }
    if (op_code == kOpExecutionMode && word_count >= 6 && words[word_index + 2] == kLocalSizeExecutionMode) {
      return std::array<uint32_t, 3>{words[word_index + 3], words[word_index + 4], words[word_index + 5]};
    }
    word_index += word_count;
  }
  return std::nullopt;
}

void ExpectSlangStageFilesCompile(const std::filesystem::path& root, const std::string& defines = {}) {
  const auto shader_paths = CollectSlangStageFiles(root);
  ASSERT_FALSE(shader_paths.empty()) << root.string();
  for (const auto& shader_path : shader_paths) {
    const auto shader_type = InferSlangStageFromPath(shader_path);
    ASSERT_TRUE(shader_type.has_value()) << shader_path.string();
    const auto source = ShaderGlobalDefinesForTests() + defines + ReadTextFile(shader_path);
    std::vector<uint32_t> binaries;
    ASSERT_TRUE(Shader::CompileToSpirv(*shader_type, source, binaries, shader_path)) << shader_path.string();
    ASSERT_FALSE(binaries.empty()) << shader_path.string();
    EXPECT_EQ(binaries.front(), 0x07230203u) << shader_path.string();
  }
}

void ExpectGlslStageFilesCompile(const std::filesystem::path& root, const std::string& defines = {}) {
  const auto shader_paths = CollectGlslStageFiles(root);
  ASSERT_FALSE(shader_paths.empty()) << root.string();
  for (const auto& shader_path : shader_paths) {
    const auto shader_type = InferGlslStageFromPath(shader_path);
    ASSERT_TRUE(shader_type.has_value()) << shader_path.string();
    const auto source = ShaderGlobalDefinesForTests() + defines + ReadTextFile(shader_path);
    std::vector<uint32_t> binaries;
    ASSERT_TRUE(Shader::CompileToSpirv(*shader_type, source, binaries, shader_path)) << shader_path.string();
    ASSERT_FALSE(binaries.empty()) << shader_path.string();
    EXPECT_EQ(binaries.front(), 0x07230203u) << shader_path.string();
  }
}
}  // namespace

TEST(ShaderCache, ExplicitDialectsAreObservableAndStrict) {
  ShaderCacheScope scope;
  std::vector<uint32_t> binaries;
  const auto native_path = scope.Root() / "NativeCanary.slang";
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kSlangComputeShader, binaries, native_path,
                                     ShaderSourceDialect::NativeSlang));
  auto stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.native_slang_frontend_invocations, 1u);
  EXPECT_EQ(stats.compatibility_slang_frontend_invocations, 0u);
  EXPECT_EQ(stats.glslang_frontend_invocations, 0u);

  Shader::ResetCompileCacheStats();
  ASSERT_TRUE(Shader::CompileToSpirv(
      ShaderType::Compute,
      "// layout(local_size_x = 99) is compatibility syntax only in comments.\n" + std::string(kSlangComputeShader),
      binaries, scope.Root() / "CommentedCompatibilityToken.slang"));
  stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.native_slang_frontend_invocations, 1u);
  EXPECT_EQ(stats.compatibility_slang_frontend_invocations, 0u);

  Shader::ResetCompileCacheStats();
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kComputeShader, binaries, scope.Root() / "Compat.comp",
                                     ShaderSourceDialect::GlslCompatibility));
  stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.glslang_frontend_invocations, 1u);

  Shader::ResetCompileCacheStats();
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kComputeShader, binaries,
                                     scope.Root() / "CompatThroughSlang.slang",
                                     ShaderSourceDialect::GlslCompatibility));
  stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.compatibility_slang_frontend_invocations, 1u);
  EXPECT_EQ(stats.native_slang_frontend_invocations, 0u);

  ShaderReflectionInfo reflection;
  std::string diagnostics;
  EXPECT_FALSE(Shader::ReflectSlang(ShaderType::Compute, kComputeShader, reflection, diagnostics, native_path,
                                    ShaderSourceDialect::NativeSlang));
  EXPECT_NE(diagnostics.find("GLSL compatibility syntax"), std::string::npos) << diagnostics;
  diagnostics.clear();
  EXPECT_FALSE(Shader::ReflectSlang(ShaderType::Compute,
                                    "#include \"Legacy.slangh\"\n" + std::string(kSlangComputeShader), reflection,
                                    diagnostics, native_path, ShaderSourceDialect::NativeSlang));
  EXPECT_NE(diagnostics.find("may not use #include"), std::string::npos) << diagnostics;
  diagnostics.clear();
  EXPECT_FALSE(Shader::ReflectSlang(
      ShaderType::Compute,
      "// @evoengine-dialect native\nimport\tEvoEngine.ShaderMigration.Common;\n# include \"Legacy.slangh\"\n" +
          std::string(kSlangComputeShader),
      reflection, diagnostics, native_path));
  EXPECT_NE(diagnostics.find("may not use #include"), std::string::npos) << diagnostics;
}

TEST(ShaderCache, NativeImportCanariesCompileReflectAndUseTypedConfiguration) {
  ShaderCacheScope scope;
  const auto module_root = RepoPath("EvoEngine_Tests/ShaderMigration/Modules");
  Shader::RegisterShaderIncludePath(module_root);
  constexpr std::string_view import = "import EvoEngine.ShaderMigration.Common;\n";
  struct Case {
    ShaderType stage;
    const char* name;
    std::string source;
  };
  const std::vector<Case> cases = {
      {ShaderType::Compute, "Compute",
       R"(
struct DoubleTransform : IMigrationTransform
{
    float apply(float value) { return value * 2.0; }
};
[[vk::binding(0, 0)]] RWStructuredBuffer<float> outputBuffer;
[shader("compute")]
[numthreads(1, 1, 1)]
void main() { outputBuffer[0] = migrationApply(DoubleTransform(), 1.0); }
)"},
      {ShaderType::Vertex, "Vertex",
       R"(
[shader("vertex")]
float4 main(float3 position : POSITION) : SV_Position
{
    return float4(migrationCommon(position.x), position.yz, 1.0);
}
)"},
      {ShaderType::Fragment, "Fragment",
       R"(
[shader("fragment")]
float4 main() : SV_Target { return migrationCommon(0.0).xxxx; }
)"},
      {ShaderType::Task, "Task",
       R"(
struct MigrationTaskPayload { uint value; };
[shader("amplification")]
[numthreads(1, 1, 1)]
void main()
{
    MigrationTaskPayload payload;
    payload.value = uint(migrationCommon(0.0));
    DispatchMesh(1, 1, 1, payload);
}
)"},
      {ShaderType::Mesh, "Mesh",
       R"(
struct MigrationVertex { float4 position : SV_Position; };
[shader("mesh")]
[outputtopology("triangle")]
[numthreads(1, 1, 1)]
void main(OutputVertices<MigrationVertex, 3> vertices, OutputIndices<uint3, 1> triangles)
{
    SetMeshOutputCounts(3, 1);
    vertices[0].position = float4(-migrationCommon(0.0), -1.0, 0.0, 1.0);
    vertices[1].position = float4(1.0, -1.0, 0.0, 1.0);
    vertices[2].position = float4(0.0, 1.0, 0.0, 1.0);
    triangles[0] = uint3(0, 1, 2);
}
)"},
      {ShaderType::RayGen, "RayGen",
       R"([shader("raygeneration")] void main() { float value = migrationCommon(0.0); })"},
      {ShaderType::Miss, "Miss",
       R"(struct MigrationPayload { float value; }; [shader("miss")] void main(inout MigrationPayload payload) { payload.value = migrationCommon(0.0); })"},
      {ShaderType::AnyHit, "AnyHit",
       R"(struct MigrationPayload { float value; }; [shader("anyhit")] void main(inout MigrationPayload payload, BuiltInTriangleIntersectionAttributes attributes) { payload.value += migrationCommon(attributes.barycentrics.x); })"},
      {ShaderType::ClosestHit, "ClosestHit",
       R"(struct MigrationPayload { float value; }; [shader("closesthit")] void main(inout MigrationPayload payload, BuiltInTriangleIntersectionAttributes attributes) { payload.value = migrationCommon(attributes.barycentrics.y); })"},
  };

  std::vector<uint32_t> configured_compute;
  for (const auto& test_case : cases) {
    const auto path = module_root / (std::string("Canary") + test_case.name + ".slang");
    const std::string source = std::string(import) + test_case.source;
    std::vector<uint32_t> binaries;
    ASSERT_TRUE(Shader::CompileToSpirv(test_case.stage, source, binaries, path, ShaderSourceDialect::NativeSlang))
        << test_case.name;
    ASSERT_FALSE(binaries.empty()) << test_case.name;
    ShaderReflectionInfo reflection;
    std::string diagnostics;
    ASSERT_TRUE(
        Shader::ReflectSlang(test_case.stage, source, reflection, diagnostics, path, ShaderSourceDialect::NativeSlang))
        << test_case.name << '\n'
        << diagnostics;
    EXPECT_EQ(reflection.shader_type, test_case.stage) << test_case.name;
    if (test_case.stage == ShaderType::Compute) {
      configured_compute = binaries;
      ASSERT_EQ(reflection.descriptor_bindings.size(), 1u);
      EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{1, 1, 1}));
    }
  }

  std::string alternate_source = std::string(import) + cases.front().source;
  const auto scale = alternate_source.find("value * 2.0");
  ASSERT_NE(scale, std::string::npos);
  alternate_source.replace(scale, std::string_view("value * 2.0").size(), "value * 3.0");
  std::vector<uint32_t> alternate_compute;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, alternate_source, alternate_compute,
                                     module_root / "CanaryComputeAlternate.slang", ShaderSourceDialect::NativeSlang));
  EXPECT_NE(configured_compute, alternate_compute);
}

TEST(ShaderCache, NativeImportDependencyInvalidatesDiskCacheAndSupportsConcurrentRequests) {
  ShaderCacheScope scope;
  const auto module_root = scope.Root() / "Modules";
  const auto module_path = module_root / "EvoEngine" / "ShaderMigration" / "Mutable.slang";
  const auto entry_path = module_root / "MutableEntry.slang";
  Shader::RegisterShaderIncludePath(module_root);
  WriteTextFile(module_path, "public float mutableValue() { return 1.0; }\n");
  const std::string source = R"(
import EvoEngine.ShaderMigration.Mutable;
[[vk::binding(0, 0)]] RWStructuredBuffer<float> outputBuffer;
[shader("compute")]
[numthreads(1, 1, 1)]
void main() { outputBuffer[0] = mutableValue(); }
)";

  std::vector<uint32_t> first;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, first, entry_path, ShaderSourceDialect::NativeSlang));
  EXPECT_EQ(Shader::GetCompileCacheStats().native_slang_frontend_invocations, 1u);

  std::vector<uint32_t> memory;
  ASSERT_TRUE(
      Shader::CompileToSpirv(ShaderType::Compute, source, memory, entry_path, ShaderSourceDialect::NativeSlang));
  EXPECT_EQ(first, memory);
  EXPECT_EQ(Shader::GetCompileCacheStats().memory_hits, 1u);

  Shader::ClearInMemoryCompileCache();
  Shader::ResetCompileCacheStats();
  std::vector<uint32_t> disk;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, disk, entry_path, ShaderSourceDialect::NativeSlang));
  EXPECT_EQ(first, disk);
  EXPECT_EQ(Shader::GetCompileCacheStats().disk_hits, 1u);

  WriteTextFile(module_path, "public float mutableValue() { return 2.0; }\n");
  Shader::ClearInMemoryCompileCache();
  Shader::ResetCompileCacheStats();
  std::vector<uint32_t> changed;
  ASSERT_TRUE(
      Shader::CompileToSpirv(ShaderType::Compute, source, changed, entry_path, ShaderSourceDialect::NativeSlang));
  EXPECT_NE(first, changed);
  EXPECT_EQ(Shader::GetCompileCacheStats().native_slang_frontend_invocations, 1u);
  EXPECT_EQ(Shader::GetCompileCacheStats().disk_misses, 1u);

  WriteTextFile(module_path, "public float mutableValue() { return 3.0; }\n");
  Shader::ClearInMemoryCompileCache();
  Shader::ResetCompileCacheStats();
  auto* application = ApplicationContext::TryGet();
  ASSERT_NE(application, nullptr);
  std::array<std::future<std::vector<uint32_t>>, 4> futures;
  for (auto& future : futures) {
    future = std::async(std::launch::async, [&, application] {
      ApplicationContextScope application_scope(*application);
      std::vector<uint32_t> binaries;
      if (!Shader::CompileToSpirv(ShaderType::Compute, source, binaries, entry_path,
                                  ShaderSourceDialect::NativeSlang)) {
        binaries.clear();
      }
      return binaries;
    });
  }
  const auto concurrent = futures.front().get();
  ASSERT_FALSE(concurrent.empty());
  for (size_t i = 1; i < futures.size(); ++i) {
    EXPECT_EQ(futures[i].get(), concurrent);
  }
}

TEST(ShaderCache, RemainingSharedSamplingModulesCompileAndMatchNumericalContracts) {
  ShaderCacheScope scope;
  const auto module_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules");
  Shader::RegisterShaderIncludePath(module_root);
  const std::string source = R"(
import EvoEngine.Random;
import EvoEngine.VogelDisk;

[[vk::binding(0, 0)]] RWStructuredBuffer<float4> outputBuffer;

[shader("compute")]
[numthreads(1, 1, 1)]
void main()
{
    uint seed = 1u;
    outputBuffer[0] = float4(EE_PCG_RANDOM(seed), EE_PCG_RANDOM(seed), 0.0, 1.0);
    outputBuffer[1] = float4(EE_VOGEL_DISK_SAMPLE(3, 16, 0.25), 0.0, 1.0);
}
)";
  std::vector<uint32_t> binaries;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries, module_root / "PureModuleProbe.slang",
                                     ShaderSourceDialect::NativeSlang));
  ASSERT_FALSE(binaries.empty());

  uint32_t state = 1u;
  const uint32_t previous = state * 747796405u + 2891336453u;
  const uint32_t word = ((previous >> ((previous >> 28u) + 4u)) ^ previous) * 277803737u;
  state = previous;
  EXPECT_EQ((word >> 22u) ^ word, 2831084092u);
  EXPECT_EQ(state, 3639132858u);

  const float radius = std::sqrt(3.5f) / std::sqrt(16.0f);
  const float theta = 2.4f * 3.0f + 0.25f;
  EXPECT_NEAR(radius * std::cos(theta), 0.18384754f, 1e-6f);
  EXPECT_NEAR(radius * std::sin(theta), 0.43005824f, 1e-6f);
}

TEST(ShaderCache, SharedGpuAbiModuleMatchesHostLayoutsAndDescriptorBindings) {
  ShaderCacheScope scope;
  const auto module_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules");
  Shader::RegisterShaderIncludePath(module_root);
  const std::string source = R"(
import EvoEngine.PerFrame;

[[vk::binding(13, 0)]] RWStructuredBuffer<float4> outputBuffer;

[shader("compute")]
[numthreads(1, 1, 1)]
void main()
{
    static_assert(sizeof(RenderInfo, Std140DataLayout) == 5520, "RenderInfo layout");
    static_assert(sizeof(Environment, Std140DataLayout) == 64, "Environment layout");
    static_assert(sizeof(Camera, Std430DataLayout) == 976, "Camera layout");
    static_assert(sizeof(Instance, Std430DataLayout) == 96, "Instance layout");
    static_assert(sizeof(EeKernelBlock, Std140DataLayout) == 2048, "Kernel layout");
    static_assert(sizeof(DirectionalLight, Std430DataLayout) == 384, "DirectionalLight layout");
    static_assert(sizeof(PointLight, Std430DataLayout) == 480, "PointLight layout");
    static_assert(sizeof(SpotLight, Std430DataLayout) == 176, "SpotLight layout");
    static_assert(sizeof(GltfTextureInfo, ScalarDataLayout) == 40, "GltfTextureInfo layout");
    static_assert(sizeof(GltfShadeMaterial, ScalarDataLayout) == 288, "GltfShadeMaterial layout");

    float4 value = float4(EE_RENDER_INFO.shadow_split_0, EE_ENVIRONMENT.gamma,
                          EE_CAMERAS[0].resolution_x, float(EE_INSTANCES[0].entity_index));
    value += EE_KERNEL_BLOCK.uniform_kernel[0];
    value += EE_DIRECTIONAL_LIGHTS[0].diffuse + EE_POINT_LIGHTS[0].diffuse + EE_SPOT_LIGHTS[0].diffuse;
    value += EE_TEXTURE_2DS[0].SampleLevel(float2(0.5), 0.0);
    value += EE_CUBEMAPS[0].SampleLevel(float3(0.0, 0.0, 1.0), 0.0);
    value += EE_GLTF_MATERIALS[0].pbr_base_color_factor;
    value.xy += EE_GLTF_TEXTURE_INFOS[0].uv_transform[0];
    outputBuffer[0] = value;
}
)";

  ShaderReflectionInfo reflection;
  std::string diagnostics;
  ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Compute, source, reflection, diagnostics,
                                   module_root / "SharedGpuAbiProbe.slang", ShaderSourceDialect::NativeSlang))
      << diagnostics;
  ASSERT_EQ(reflection.descriptor_bindings.size(), 13u);

  const std::array<VkDescriptorType, 14> expected_types = {
      VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,         VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
      VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,         VK_DESCRIPTOR_TYPE_MAX_ENUM,
      VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,         VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
      VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,         VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
      VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,         VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
      VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,         VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
  };
  size_t reflected_index = 0;
  for (uint32_t binding = 0; binding < expected_types.size(); ++binding) {
    if (expected_types[binding] == VK_DESCRIPTOR_TYPE_MAX_ENUM)
      continue;
    ASSERT_LT(reflected_index, reflection.descriptor_bindings.size());
    const auto& reflected = reflection.descriptor_bindings[reflected_index++];
    EXPECT_EQ(reflected.set, 0u) << reflected.name;
    EXPECT_EQ(reflected.binding, binding) << reflected.name;
    EXPECT_EQ(reflected.descriptor_type, expected_types[binding]) << reflected.name;
    EXPECT_EQ(reflected.unbounded, binding == 9u || binding == 10u) << reflected.name;
    if (reflected.unbounded)
      EXPECT_EQ(reflected.descriptor_count, 0u) << reflected.name;
  }
  EXPECT_EQ(reflected_index, reflection.descriptor_bindings.size());
  EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{1, 1, 1}));
  EXPECT_TRUE(reflection.push_constant_ranges.empty());
  EXPECT_TRUE(reflection.stage_inputs.empty());
  EXPECT_TRUE(reflection.stage_outputs.empty());
}

TEST(ShaderCache, RayTracingFoundationModulesCompileAndReflectBindings) {
  ShaderCacheScope scope;
  const auto module_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Modules");
  Shader::RegisterShaderIncludePath(module_root);
  const std::string source = R"(
import EvoEngine.RayTracingBasic;
import EvoEngine.CameraRayTracingPayload;
import EvoEngine.PointCloudRayTracingPayload;
import EvoEngine.CameraRayGeometry;
import EvoEngine.GltfRayTracingMath;
import EvoEngine.CameraRayOutputs;

[[vk::binding(16, 2)]] RWStructuredBuffer<float4> outputBuffer;

[shader("compute")]
[numthreads(1, 1, 1)]
void main()
{
    static_assert(sizeof(Vertex) == 112, "Vertex ABI");
    static_assert(sizeof(CameraRayTracingPayload) == 52, "camera payload ABI");
    static_assert(sizeof(PointCloudRayTracingPayload) == 96, "point-cloud payload ABI");
    CameraRayTracingPayload cameraPayload = {};
    PointCloudRayTracingPayload pointCloudPayload = {};
    CameraRayOutputDiagnostics diagnostics = EE_CAMERA_INIT_RAY_OUTPUT_DIAGNOSTICS();
    float3 normal = EE_CAMERA_GEOMETRIC_NORMAL(float3(1.0, 0.0, 0.0), float3(0.0, 1.0, 0.0),
                                               float3(0.0, 0.0, 1.0));
    float3 sampleDirection = EE_GLTF_RT_SAMPLE_COSINE_HEMISPHERE(float2(0.25, 0.5));
    outputBuffer[0] = float4(normal + sampleDirection + diagnostics.normal + cameraPayload.shadow_transmission,
                             pointCloudPayload.hit_info.vertex_info1);
    outputBuffer[1] = float4(EE_VERTICES[0].position, float(EE_INDICES[0]));
}
)";

  ShaderReflectionInfo reflection;
  std::string diagnostics;
  ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Compute, source, reflection, diagnostics,
                                   module_root / "RayTracingFoundationProbe.slang", ShaderSourceDialect::NativeSlang))
      << diagnostics;
  ASSERT_EQ(reflection.descriptor_bindings.size(), 27u);
  EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{1, 1, 1}));

  std::vector<uint32_t> binaries;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries,
                                     module_root / "RayTracingFoundationProbe.slang",
                                     ShaderSourceDialect::NativeSlang));
  EXPECT_FALSE(binaries.empty());
}

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

TEST(ShaderCache, SlangComputeShaderCompilesToSpirv) {
  ShaderCacheScope scope;
  std::vector<uint32_t> binaries;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kSlangComputeShader, binaries));
  ASSERT_FALSE(binaries.empty());
  EXPECT_EQ(binaries.front(), 0x07230203u);
  const auto stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.compilations, 1u);
  EXPECT_EQ(stats.disk_misses, 1u);
  EXPECT_EQ(CacheFileCount(scope.Root()), 1u);
}

TEST(ShaderCache, SlangUsesMemoryThenValidatedDiskEntry) {
  ShaderCacheScope scope;
  std::vector<uint32_t> first;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kSlangComputeShader, first));
  ASSERT_FALSE(first.empty());
  auto stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.compilations, 1u);
  EXPECT_EQ(stats.disk_misses, 1u);

  std::vector<uint32_t> second;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kSlangComputeShader, second));
  EXPECT_EQ(first, second);
  EXPECT_EQ(Shader::GetCompileCacheStats().memory_hits, 1u);

  Shader::ClearInMemoryCompileCache();
  Shader::ResetCompileCacheStats();
  std::vector<uint32_t> disk;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kSlangComputeShader, disk));
  EXPECT_EQ(first, disk);
  stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.disk_hits, 1u);
  EXPECT_EQ(stats.compilations, 0u);
}

TEST(ShaderCache, SlangDiskHitSkipsFrontendAfterDependencyIndexIsPublished) {
  ShaderCacheScope scope;
  std::vector<uint32_t> first;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kSlangComputeShader, first));
  ASSERT_FALSE(first.empty());
  EXPECT_GT(Shader::GetCompileCacheStats().native_slang_frontend_invocations, 0u);

  Shader::ClearInMemoryCompileCache();
  Shader::ResetCompileCacheStats();
  std::vector<uint32_t> second;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kSlangComputeShader, second));
  EXPECT_EQ(first, second);

  const auto stats = Shader::GetCompileCacheStats();
  EXPECT_EQ(stats.disk_hits, 1u);
  EXPECT_EQ(stats.compilations, 0u);
  EXPECT_EQ(stats.native_slang_frontend_invocations, 0u);
}

TEST(ShaderCache, ProductionBloomSharedSlangPushConstantsMatchHostLayout) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const VkPushConstantRange downsampling_push_constant_range{
      VK_SHADER_STAGE_COMPUTE_BIT, 0, static_cast<uint32_t>(sizeof(Bloom::DownsamplingPushConstant))};
  const VkPushConstantRange upsampling_push_constant_range{
      VK_SHADER_STAGE_COMPUTE_BIT, 0, static_cast<uint32_t>(sizeof(Bloom::UpsamplingPushConstant))};
  const VkPushConstantRange compute_push_constant_range{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                                        static_cast<uint32_t>(sizeof(Bloom::ComputePushConstant))};

  auto sampling_layout = std::make_shared<DescriptorSetLayout>();
  sampling_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  sampling_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  const auto downsampling_path =
      RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing/BloomDownsampling.slang");
  const auto downsampling_source = ShaderGlobalDefinesForTests() + ReadTextFile(downsampling_path);
  const auto downsampling_validation =
      Shader::ValidateSlangPipelineLayout(ShaderType::Compute, downsampling_source, downsampling_path,
                                          {sampling_layout}, {downsampling_push_constant_range});
  const auto upsampling_path =
      RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing/BloomUpsampling.slang");
  const auto upsampling_source = ShaderGlobalDefinesForTests() + ReadTextFile(upsampling_path);
  const auto upsampling_validation = Shader::ValidateSlangPipelineLayout(
      ShaderType::Compute, upsampling_source, upsampling_path, {sampling_layout}, {upsampling_push_constant_range});

  auto copy_layout = std::make_shared<DescriptorSetLayout>();
  copy_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  copy_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  copy_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  const auto copy_path =
      RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing/BloomCopy.slang");
  const auto copy_source = ShaderGlobalDefinesForTests() + ReadTextFile(copy_path);
  const auto copy_validation = Shader::ValidateSlangPipelineLayout(ShaderType::Compute, copy_source, copy_path,
                                                                   {copy_layout}, {compute_push_constant_range});

  auto mix_layout = std::make_shared<DescriptorSetLayout>();
  mix_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  mix_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  mix_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  const auto mix_path =
      RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing/BloomMix.slang");
  const auto mix_source = ShaderGlobalDefinesForTests() + ReadTextFile(mix_path);
  const auto mix_validation = Shader::ValidateSlangPipelineLayout(ShaderType::Compute, mix_source, mix_path,
                                                                  {mix_layout}, {compute_push_constant_range});

  ASSERT_TRUE(downsampling_validation.success) << downsampling_validation.diagnostics;
  ASSERT_EQ(downsampling_validation.reflection.push_constant_ranges.size(), 1u);
  EXPECT_EQ(downsampling_validation.reflection.push_constant_ranges[0].size, downsampling_push_constant_range.size);
  ASSERT_TRUE(upsampling_validation.success) << upsampling_validation.diagnostics;
  ASSERT_EQ(upsampling_validation.reflection.push_constant_ranges.size(), 1u);
  EXPECT_EQ(upsampling_validation.reflection.push_constant_ranges[0].size, upsampling_push_constant_range.size);
  ASSERT_TRUE(copy_validation.success) << copy_validation.diagnostics;
  ASSERT_EQ(copy_validation.reflection.push_constant_ranges.size(), 1u);
  EXPECT_EQ(copy_validation.reflection.push_constant_ranges[0].size, compute_push_constant_range.size);
  ASSERT_TRUE(mix_validation.success) << mix_validation.diagnostics;
  ASSERT_EQ(mix_validation.reflection.push_constant_ranges.size(), 1u);
  EXPECT_EQ(mix_validation.reflection.push_constant_ranges[0].size, compute_push_constant_range.size);
}

TEST(ShaderCache, ProductionBlurSlangPushConstantsMatchHostLayout) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const VkPushConstantRange blur_push_constant_range{
      VK_SHADER_STAGE_COMPUTE_BIT, 0, static_cast<uint32_t>(sizeof(PostProcessingStack::BlurPushConstant))};
  const VkPushConstantRange ao_blur_push_constant_range{
      VK_SHADER_STAGE_COMPUTE_BIT, 0, static_cast<uint32_t>(sizeof(AmbientOcclusion::BlurPushConstant))};

  auto blur_layout = std::make_shared<DescriptorSetLayout>();
  blur_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  blur_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  const auto blur_path = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing/Blur.slang");
  const auto blur_source = ShaderGlobalDefinesForTests() + ReadTextFile(blur_path);
  const auto blur_validation = Shader::ValidateSlangPipelineLayout(ShaderType::Compute, blur_source, blur_path,
                                                                   {blur_layout}, {blur_push_constant_range});

  auto camera_gbuffer_layout = std::make_shared<DescriptorSetLayout>();
  camera_gbuffer_layout->PushDescriptorBinding(17, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                               VK_SHADER_STAGE_COMPUTE_BIT, 0);
  const auto ao_blur_path =
      RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing/AmbientOcclusionBlur.slang");
  const auto ao_blur_source = ShaderGlobalDefinesForTests() + ReadTextFile(ao_blur_path);
  const auto ao_blur_validation =
      Shader::ValidateSlangPipelineLayout(ShaderType::Compute, ao_blur_source, ao_blur_path,
                                          {blur_layout, camera_gbuffer_layout}, {ao_blur_push_constant_range});

  ASSERT_TRUE(blur_validation.success) << blur_validation.diagnostics;
  ASSERT_EQ(blur_validation.reflection.push_constant_ranges.size(), 1u);
  EXPECT_EQ(blur_validation.reflection.push_constant_ranges[0].size, blur_push_constant_range.size);
  ASSERT_TRUE(ao_blur_validation.success) << ao_blur_validation.diagnostics;
  ASSERT_EQ(ao_blur_validation.reflection.push_constant_ranges.size(), 1u);
  EXPECT_EQ(ao_blur_validation.reflection.push_constant_ranges[0].size, ao_blur_push_constant_range.size);
}

TEST(ShaderCache, ProductionTexturePassThroughSlangShadersMatchHostInterface) {
  ShaderCacheScope scope;
  const auto vertex_path =
      RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Graphics/Vertex/TexturePassThrough.slang");
  const auto fragment_path =
      RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Graphics/Fragment/TexturePassThrough.slang");
  const auto vertex_source = ShaderGlobalDefinesForTests() + ReadTextFile(vertex_path);
  const auto fragment_source = ShaderGlobalDefinesForTests() + ReadTextFile(fragment_path);
  ShaderReflectionStageIo position_input;
  position_input.semantic_name = "POSITION";
  position_input.location = 0;
  ShaderReflectionStageIo tex_coord_input;
  tex_coord_input.semantic_name = "TEXCOORD";
  tex_coord_input.location = 3;
  ShaderReflectionStageIo tex_coord_output;
  tex_coord_output.semantic_name = "TEXCOORD";
  tex_coord_output.location = 0;
  auto fragment_layout = std::make_shared<DescriptorSetLayout>();
  fragment_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT, 0);

  const auto vertex_validation =
      Shader::ValidateSlangPipelineLayout(ShaderType::Vertex, vertex_source, vertex_path, {}, {},
                                          std::vector{position_input, tex_coord_input}, std::vector{tex_coord_output});
  const auto fragment_validation =
      Shader::ValidateSlangPipelineLayout(ShaderType::Fragment, fragment_source, fragment_path, {fragment_layout}, {},
                                          std::vector{tex_coord_output}, std::vector<ShaderReflectionStageIo>{});
  std::vector<uint32_t> vertex_binaries;
  std::vector<uint32_t> fragment_binaries;

  ASSERT_TRUE(vertex_validation.success) << vertex_validation.diagnostics;
  ASSERT_TRUE(fragment_validation.success) << fragment_validation.diagnostics;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Vertex, vertex_source, vertex_binaries, vertex_path));
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Fragment, fragment_source, fragment_binaries, fragment_path));
  ASSERT_FALSE(vertex_binaries.empty());
  ASSERT_FALSE(fragment_binaries.empty());
}

TEST(ShaderCache, ProductionEnvironmentLightingSlangShadersMatchHostInterface) {
  ShaderCacheScope scope;
  const auto shader_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders");
  const VkPushConstantRange cubemap_process_range{VK_SHADER_STAGE_ALL, 0,
                                                  static_cast<uint32_t>(sizeof(glm::mat4) + sizeof(float))};
  auto fragment_sampler_layout = std::make_shared<DescriptorSetLayout>();
  fragment_sampler_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                                 VK_SHADER_STAGE_FRAGMENT_BIT, 0);
  ShaderReflectionStageIo position_input;
  position_input.semantic_name = "POSITION";
  position_input.location = 0;
  ShaderReflectionStageIo world_pos_io;
  world_pos_io.semantic_name = "TEXCOORD";
  world_pos_io.location = 0;
  ShaderReflectionStageIo tex_coord_io;
  tex_coord_io.semantic_name = "TEXCOORD";
  tex_coord_io.location = 0;

  struct Case {
    ShaderType shader_type;
    std::filesystem::path path;
    std::vector<std::shared_ptr<DescriptorSetLayout>> layouts;
    std::vector<VkPushConstantRange> ranges;
    std::optional<std::vector<ShaderReflectionStageIo>> inputs;
    std::optional<std::vector<ShaderReflectionStageIo>> outputs;
  };
  const std::array<Case, 5> cases = {{
      {ShaderType::Vertex,
       shader_root / "Graphics/Vertex/Lighting/CubemapProcess.slang",
       {},
       {cubemap_process_range},
       std::nullopt,
       std::nullopt},
      {ShaderType::Fragment,
       shader_root / "Graphics/Fragment/Lighting/EquirectangularMapToCubemap.slang",
       {fragment_sampler_layout},
       {cubemap_process_range},
       std::vector{world_pos_io},
       std::vector<ShaderReflectionStageIo>{}},
      {ShaderType::Fragment,
       shader_root / "Graphics/Fragment/Lighting/EnvironmentalMapIrradianceConvolution.slang",
       {fragment_sampler_layout},
       {},
       std::vector{world_pos_io},
       std::vector<ShaderReflectionStageIo>{}},
      {ShaderType::Fragment,
       shader_root / "Graphics/Fragment/Lighting/EnvironmentalMapPrefilter.slang",
       {fragment_sampler_layout},
       {cubemap_process_range},
       std::vector{world_pos_io},
       std::vector<ShaderReflectionStageIo>{}},
      {ShaderType::Fragment,
       shader_root / "Graphics/Fragment/Lighting/EnvironmentalMapBrdf.slang",
       {},
       {},
       std::vector{tex_coord_io},
       std::vector<ShaderReflectionStageIo>{}},
  }};

  for (const auto& test_case : cases) {
    const auto source = ShaderGlobalDefinesForTests() + ReadTextFile(test_case.path);
    const auto validation =
        Shader::ValidateSlangPipelineLayout(test_case.shader_type, source, test_case.path, test_case.layouts,
                                            test_case.ranges, test_case.inputs, test_case.outputs);
    std::vector<uint32_t> binaries;
    ASSERT_TRUE(validation.success) << test_case.path.string() << "\n" << validation.diagnostics;
    ASSERT_TRUE(Shader::CompileToSpirv(test_case.shader_type, source, binaries, test_case.path))
        << test_case.path.string();
    ASSERT_FALSE(binaries.empty()) << test_case.path.string();
    EXPECT_EQ(binaries.front(), 0x07230203u) << test_case.path.string();
    EXPECT_FALSE(SpirvDeclaresComputeDerivativeCapability(binaries)) << test_case.path.string();
  }
}

TEST(ShaderCache, ProductionSmaaSlangPresetVariantsCompileAndReflectInterfaces) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders");
  const std::array presets = {"SMAA_PRESET_LOW", "SMAA_PRESET_MEDIUM", "SMAA_PRESET_HIGH", "SMAA_PRESET_ULTRA"};
  struct Case {
    ShaderType shader_type;
    std::filesystem::path path;
    size_t descriptor_count;
    size_t varying_count;
  };
  const std::array<Case, 4> cases = {{
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/PostProcessing/SMAAEdge.slang", 0u, 4u},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/PostProcessing/SMAAEdge.slang", 1u, 4u},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/PostProcessing/SMAABlendWeight.slang", 0u, 5u},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/PostProcessing/SMAABlendWeight.slang", 3u, 5u},
  }};

  for (const auto* preset : presets) {
    for (const auto& test_case : cases) {
      const auto source = ShaderGlobalDefinesForTests() + "\n#define " + preset + " 1\n" + ReadTextFile(test_case.path);
      ShaderReflectionInfo reflection;
      std::string diagnostics;
      ASSERT_TRUE(Shader::ReflectSlang(test_case.shader_type, source, reflection, diagnostics, test_case.path))
          << preset << " " << test_case.path.string() << "\n"
          << diagnostics;
      ASSERT_EQ(reflection.push_constant_ranges.size(), 1u) << preset << " " << test_case.path.string();
      EXPECT_EQ(reflection.push_constant_ranges[0].size, sizeof(AntiAliasing::SmaaPushConstant));
      EXPECT_EQ(reflection.descriptor_bindings.size(), test_case.descriptor_count);
      EXPECT_EQ(test_case.shader_type == ShaderType::Vertex ? reflection.stage_outputs.size()
                                                            : reflection.stage_inputs.size(),
                test_case.varying_count);

      std::vector<uint32_t> binaries;
      ASSERT_TRUE(Shader::CompileToSpirv(test_case.shader_type, source, binaries, test_case.path))
          << preset << " " << test_case.path.string();
      ASSERT_FALSE(binaries.empty());
    }
  }
  EXPECT_EQ(Shader::GetCompileCacheStats().compatibility_slang_frontend_invocations, 0u);
}

TEST(ShaderCache, CameraRaygenSerCompileUsesExtInvocationReorder) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders");
  const auto shader_path = shader_root / "RayTracing/RayGen/Camera.slang";
  std::string header = ShaderGlobalDefinesForTests();
  const std::string disabled_define = "#define EE_SHADER_EXECUTION_REORDERING_SUPPORTED 0";
  const auto define_offset = header.find(disabled_define);
  ASSERT_NE(define_offset, std::string::npos);
  header.replace(define_offset, disabled_define.size(), "#define EE_SHADER_EXECUTION_REORDERING_SUPPORTED 1");

  std::vector<uint32_t> binaries;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::RayGen, header + ReadTextFile(shader_path), binaries, shader_path))
      << shader_path.string();
  ASSERT_FALSE(binaries.empty());
  EXPECT_EQ(binaries.front(), 0x07230203u);

  const std::string binary_bytes(reinterpret_cast<const char*>(binaries.data()), binaries.size() * sizeof(uint32_t));
  EXPECT_NE(binary_bytes.find("SPV_EXT_shader_invocation_reorder"), std::string::npos);
  EXPECT_EQ(binary_bytes.find("SPV_NV_shader_invocation_reorder"), std::string::npos);
}

TEST(ShaderCache, ProductionSdkSlangShaderInventoryCompiles) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders");
  const std::string no_bindless = "\n#define EE_SKIP_PER_FRAME_BINDLESS_TEXTURES 1\n";
  const std::string material_no_bindless = no_bindless;
  const std::string fixed_lighting =
      no_bindless + "#define EE_RASTER_FIXED_LIGHTING_TEXTURES 1\n#define EE_RASTER_FIXED_LIGHTING_TEXTURE_SET 3\n";
  const std::string fixed_material_lighting =
      no_bindless + "#define EE_RASTER_FIXED_LIGHTING_TEXTURES 1\n#define EE_RASTER_FIXED_LIGHTING_TEXTURE_SET 4\n";
  struct Case {
    ShaderType shader_type;
    std::filesystem::path path;
    std::string defines;
  };
  const std::vector<Case> cases = {
      {ShaderType::Compute, shader_root / "Compute/Trace.slang"},
      {ShaderType::Compute, shader_root / "Compute/RayQueryCamera.slang"},
      {ShaderType::Compute, shader_root / "Compute/DepthPyramid.slang"},
      {ShaderType::Compute, shader_root / "Compute/MotionVectors.slang"},
      {ShaderType::Compute, shader_root / "Compute/VolumetricClouds.slang"},
      {ShaderType::Compute, shader_root / "Compute/VolumetricCloudsComposite.slang"},
      {ShaderType::Compute, shader_root / "Compute/GaussianSplatCull.slang"},
      {ShaderType::Compute, shader_root / "Compute/GaussianSplatRadixUpsweep.slang"},
      {ShaderType::Compute, shader_root / "Compute/GaussianSplatRadixSpine.slang"},
      {ShaderType::Compute, shader_root / "Compute/GaussianSplatRadixDownsweep.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/TAAResolve.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/TAACopy.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/BloomDownsampling.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/BloomUpsampling.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/BloomCopy.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/BloomMix.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/SMAAPrepare.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/Blur.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/SSRReflect.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/SSRCombine.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/AmbientOcclusionGeometry.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/AmbientOcclusionBlur.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/ToneMappingHistogram.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/ToneMappingAutoExposure.slang"},
      {ShaderType::Compute, shader_root / "Compute/PostProcessing/ToneMapping.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Lighting/AtmosphereToCubemap.slang"},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/Lighting/AtmosphereToCubemap.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/GaussianSplat/GaussianSplat.slang"},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/GaussianSplat/GaussianSplat.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Lighting/PointLightShadowMap.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Lighting/SpotLightShadowMap.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Lighting/DirectionalLightShadowMap.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Lighting/PointLightShadowMapInstanced.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Lighting/SpotLightShadowMapInstanced.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Lighting/DirectionalLightShadowMapInstanced.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Lighting/PointLightShadowMapSkinned.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Lighting/SpotLightShadowMapSkinned.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Lighting/DirectionalLightShadowMapSkinned.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Standard/Standard.slang", no_bindless},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Standard/StandardInstanced.slang", no_bindless},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Standard/StandardSkinned.slang", no_bindless},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Standard/SkinnedMotionVectors.slang", no_bindless},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Standard/TransparentMotionVectors.slang", no_bindless},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/Empty.slang"},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/Standard/StandardDeferred.slang", material_no_bindless},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/Standard/StandardDeferredLighting.slang", fixed_lighting},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/Standard/StandardDeferredLightingSceneCamera.slang",
       fixed_lighting},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/Standard/StandardTransparent.slang",
       fixed_material_lighting},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/Standard/DDGIGatherTiming.slang", fixed_lighting},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/Standard/SkinnedMotionVectors.slang",
       material_no_bindless},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/Standard/TransparentMotionVectors.slang",
       material_no_bindless},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Gizmos/Gizmos.slang"},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/Gizmos/Gizmos.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Gizmos/GizmosNormalColored.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Gizmos/GizmosVertexColored.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/Gizmos/GizmosInstancedColored.slang"},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/Gizmos/GizmosColored.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/DDGI/DDGIProbeVisualization.slang"},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/DDGI/DDGIProbeVisualization.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/DDGI/DDGIProbeRayVisualization.slang"},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/DDGI/DDGIProbeRayVisualization.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/PostProcessing/SMAAEdge.slang"},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/PostProcessing/SMAAEdge.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/PostProcessing/SMAABlendWeight.slang"},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/PostProcessing/SMAABlendWeight.slang"},
      {ShaderType::Vertex, shader_root / "Graphics/Vertex/PostProcessing/SMAANeighborhood.slang"},
      {ShaderType::Fragment, shader_root / "Graphics/Fragment/PostProcessing/SMAANeighborhood.slang"},
      {ShaderType::Task, shader_root / "Graphics/Task/Standard/Standard.slang", no_bindless},
      {ShaderType::Task, shader_root / "Graphics/Task/Standard/StandardStrands.slang", no_bindless},
      {ShaderType::Task, shader_root / "Graphics/Task/Lighting/PointLightShadowMap.slang"},
      {ShaderType::Task, shader_root / "Graphics/Task/Lighting/SpotLightShadowMap.slang"},
      {ShaderType::Task, shader_root / "Graphics/Task/Lighting/DirectionalLightShadowMap.slang"},
      {ShaderType::Task, shader_root / "Graphics/Task/Lighting/StrandsShadowMap.slang"},
      {ShaderType::Task, shader_root / "Graphics/Task/Gizmos/GizmosStrands.slang"},
      {ShaderType::Mesh, shader_root / "Graphics/Mesh/Standard/Standard.slang", no_bindless},
      {ShaderType::Mesh, shader_root / "Graphics/Mesh/Standard/StandardMeshletColored.slang", no_bindless},
      {ShaderType::Mesh, shader_root / "Graphics/Mesh/Standard/StandardStrands.slang", no_bindless},
      {ShaderType::Mesh, shader_root / "Graphics/Mesh/Lighting/PointLightShadowMap.slang"},
      {ShaderType::Mesh, shader_root / "Graphics/Mesh/Lighting/SpotLightShadowMap.slang"},
      {ShaderType::Mesh, shader_root / "Graphics/Mesh/Lighting/DirectionalLightShadowMap.slang"},
      {ShaderType::Mesh, shader_root / "Graphics/Mesh/Lighting/PointLightStrandsShadowMap.slang"},
      {ShaderType::Mesh, shader_root / "Graphics/Mesh/Lighting/SpotLightStrandsShadowMap.slang"},
      {ShaderType::Mesh, shader_root / "Graphics/Mesh/Lighting/DirectionalLightStrandsShadowMap.slang"},
      {ShaderType::Mesh, shader_root / "Graphics/Mesh/Gizmos/GizmosStrands.slang"},
      {ShaderType::Mesh, shader_root / "Graphics/Mesh/GaussianSplat/GaussianSplat.slang"},
      {ShaderType::RayGen, shader_root / "RayTracing/RayGen/Camera.slang"},
      {ShaderType::Miss, shader_root / "RayTracing/Miss/Camera.slang"},
      {ShaderType::ClosestHit, shader_root / "RayTracing/ClosestHit/Camera.slang"},
      {ShaderType::AnyHit, shader_root / "RayTracing/AnyHit/Camera.slang"},
      {ShaderType::RayGen, shader_root / "RayTracing/RayGen/PointCloud.slang"},
      {ShaderType::Miss, shader_root / "RayTracing/Miss/PointCloud.slang"},
      {ShaderType::ClosestHit, shader_root / "RayTracing/ClosestHit/PointCloud.slang"},
      {ShaderType::RayGen, shader_root / "RayTracing/RayGen/DDGIProbeDiagnostics.slang"},
      {ShaderType::Miss, shader_root / "RayTracing/Miss/DDGIProbeDiagnostics.slang"},
      {ShaderType::ClosestHit, shader_root / "RayTracing/ClosestHit/DDGIProbeDiagnostics.slang"},
      {ShaderType::AnyHit, shader_root / "RayTracing/AnyHit/DDGIProbeDiagnostics.slang"},
  };

  for (const auto& test_case : cases) {
    const auto source = ShaderGlobalDefinesForTests() + test_case.defines + ReadTextFile(test_case.path);
    std::vector<uint32_t> binaries;
    ASSERT_TRUE(Shader::CompileToSpirv(test_case.shader_type, source, binaries, test_case.path))
        << test_case.path.string();
    ASSERT_FALSE(binaries.empty()) << test_case.path.string();
    EXPECT_EQ(binaries.front(), 0x07230203u) << test_case.path.string();
    if (test_case.shader_type == ShaderType::Compute) {
      EXPECT_FALSE(SpirvDeclaresComputeDerivativeCapability(binaries)) << test_case.path.string();
    }
  }
}

TEST(ShaderCache, ProductionEcoSysLabGlslShaderInventoryCompiles) {
  ShaderCacheScope scope;
  RegisterEcoSysLabShaderIncludePath();
  const auto sdk_shader_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders");
  const auto sdk_glsl_count =
      static_cast<size_t>(std::count_if(std::filesystem::recursive_directory_iterator(sdk_shader_root),
                                        std::filesystem::recursive_directory_iterator(), [](const auto& entry) {
                                          return entry.is_regular_file() && entry.path().extension() == ".glsl";
                                        }));
  EXPECT_EQ(sdk_glsl_count, 0u);
  const auto shader_root = RepoPath("EvoEngine_Packages/EcoSysLab/Internals/EcoSysLabResources/Shaders");
  ExpectGlslStageFilesCompile(shader_root);
}

TEST(ShaderCache, ProductionDdgiComputeSlangShadersMatchHostLayouts) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  auto ddgi_probe_update_layout = std::make_shared<DescriptorSetLayout>();
  ddgi_probe_update_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  ddgi_probe_update_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  ddgi_probe_update_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  ddgi_probe_update_layout->PushDescriptorBinding(3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  ddgi_probe_update_layout->PushDescriptorBinding(4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  ddgi_probe_update_layout->PushDescriptorBinding(5, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  auto ddgi_probe_relocation_layout = std::make_shared<DescriptorSetLayout>();
  ddgi_probe_relocation_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                      0);
  ddgi_probe_relocation_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT,
                                                      0);
  auto ddgi_probe_classification_layout = std::make_shared<DescriptorSetLayout>();
  ddgi_probe_classification_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                          VK_SHADER_STAGE_COMPUTE_BIT, 0);
  ddgi_probe_classification_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                          VK_SHADER_STAGE_COMPUTE_BIT, 0);
  auto ddgi_probe_variability_layout = std::make_shared<DescriptorSetLayout>();
  ddgi_probe_variability_layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                       0);
  ddgi_probe_variability_layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                                                       VK_SHADER_STAGE_COMPUTE_BIT, 0);
  ddgi_probe_variability_layout->PushDescriptorBinding(2, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT,
                                                       0);
  const VkPushConstantRange scroll_range{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                         static_cast<uint32_t>(sizeof(DdgiProbeScrollPushConstant))};
  const VkPushConstantRange update_range{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                         static_cast<uint32_t>(sizeof(DdgiProbeAtlasUpdatePushConstant))};
  const VkPushConstantRange relocation_range{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                             static_cast<uint32_t>(sizeof(DdgiProbeRelocationPushConstant))};
  const VkPushConstantRange classification_range{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                                 static_cast<uint32_t>(sizeof(DdgiProbeClassificationPushConstant))};
  const VkPushConstantRange variability_range{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                              static_cast<uint32_t>(sizeof(DdgiProbeVariabilityPushConstant))};
  struct Case {
    std::filesystem::path path;
    std::string defines;
    std::vector<std::shared_ptr<DescriptorSetLayout>> layouts;
    std::vector<VkPushConstantRange> ranges;
    std::array<uint32_t, 3> group_size;
  };
  const auto ddgi_path = [](const char* name) {
    return RepoPath(std::filesystem::path("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute") / name);
  };
  const std::array<Case, 10> cases = {{
      {ddgi_path("DDGIProbeScroll.slang"), {}, {ddgi_probe_update_layout}, {scroll_range}, {64, 1, 1}},
      {ddgi_path("DDGIProbeUpdate.slang"), {}, {nullptr, ddgi_probe_update_layout}, {update_range}, {64, 1, 1}},
      {ddgi_path("DDGIProbeUpdate.slang"),
       "\n#define EE_DDGI_PROBE_UPDATE_MODE 1\n#define EE_DDGI_PROBE_USE_SHARED_RAYS 0\n",
       {nullptr, ddgi_probe_update_layout},
       {update_range},
       {64, 1, 1}},
      {ddgi_path("DDGIProbeUpdate.slang"),
       "\n#define EE_DDGI_PROBE_UPDATE_MODE 2\n#define EE_DDGI_PROBE_USE_SHARED_RAYS 0\n",
       {nullptr, ddgi_probe_update_layout},
       {update_range},
       {64, 1, 1}},
      {ddgi_path("DDGIProbeUpdate.slang"),
       "\n#define EE_DDGI_PROBE_UPDATE_MODE 1\n#define EE_DDGI_PROBE_USE_SHARED_RAYS 1\n",
       {nullptr, ddgi_probe_update_layout},
       {update_range},
       {64, 1, 1}},
      {ddgi_path("DDGIProbeUpdate.slang"),
       "\n#define EE_DDGI_PROBE_UPDATE_MODE 2\n#define EE_DDGI_PROBE_USE_SHARED_RAYS 1\n",
       {nullptr, ddgi_probe_update_layout},
       {update_range},
       {64, 1, 1}},
      {ddgi_path("DDGIProbeRelocation.slang"), {}, {ddgi_probe_relocation_layout}, {relocation_range}, {32, 1, 1}},
      {ddgi_path("DDGIProbeClassification.slang"),
       {},
       {ddgi_probe_classification_layout},
       {classification_range},
       {32, 1, 1}},
      {ddgi_path("DDGIProbeVariabilityReduce.slang"),
       {},
       {ddgi_probe_variability_layout},
       {variability_range},
       {8, 8, 1}},
      {ddgi_path("DDGIProbeVariabilityExtraReduce.slang"),
       {},
       {ddgi_probe_variability_layout},
       {variability_range},
       {8, 8, 1}},
  }};

  for (const auto& test_case : cases) {
    const auto source = ShaderGlobalDefinesForTests() + test_case.defines + ReadTextFile(test_case.path);
    const auto validation = Shader::ValidateSlangPipelineLayout(ShaderType::Compute, source, test_case.path,
                                                                test_case.layouts, test_case.ranges);
    std::vector<uint32_t> binaries;
    ASSERT_TRUE(validation.success) << test_case.path.string() << "\n" << validation.diagnostics;
    EXPECT_EQ(validation.reflection.compute_thread_group_size, test_case.group_size) << test_case.path.string();
    ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries, test_case.path))
        << test_case.path.string();
    ASSERT_FALSE(binaries.empty()) << test_case.path.string();
    EXPECT_EQ(binaries.front(), 0x07230203u) << test_case.path.string();
  }
}

TEST(ShaderCache, ProductionTaaCopySlangReflectionMatchesHostLayout) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_path =
      RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing/TAACopy.slang");
  const auto source = ShaderGlobalDefinesForTests() + ReadTextFile(shader_path);
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, VK_SHADER_STAGE_COMPUTE_BIT, 0);

  const auto validation = Shader::ValidateSlangPipelineLayout(ShaderType::Compute, source, shader_path, {layout}, {});

  ASSERT_TRUE(validation.success) << validation.diagnostics;
  EXPECT_EQ(validation.reflection.compute_thread_group_size, (std::array<uint32_t, 3>{16, 16, 1}));
  ASSERT_EQ(validation.reflection.descriptor_bindings.size(), 2u);
  EXPECT_EQ(validation.reflection.descriptor_bindings[0].set, 0u);
  EXPECT_EQ(validation.reflection.descriptor_bindings[0].binding, 0u);
  EXPECT_EQ(validation.reflection.descriptor_bindings[0].descriptor_type, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
  EXPECT_EQ(validation.reflection.descriptor_bindings[1].set, 0u);
  EXPECT_EQ(validation.reflection.descriptor_bindings[1].binding, 1u);
  EXPECT_EQ(validation.reflection.descriptor_bindings[1].descriptor_type, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE);
  EXPECT_TRUE(validation.reflection.stage_inputs.empty());
  EXPECT_TRUE(validation.reflection.stage_outputs.empty());
}

TEST(ShaderCache, ProductionTaaResolveSlangThreadGroupMatchesHostDispatch) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_path =
      RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing/TAAResolve.slang");
  const auto source = ShaderGlobalDefinesForTests() + ReadTextFile(shader_path);

  ShaderReflectionInfo reflection;
  std::string diagnostics;
  ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Compute, source, reflection, diagnostics, shader_path)) << diagnostics;
  EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{8, 8, 1}));
  std::vector<uint32_t> binaries;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries, shader_path)) << diagnostics;
  ASSERT_FALSE(binaries.empty());
  const auto local_size = FindSpirvLocalSizeExecutionMode(binaries);
  ASSERT_TRUE(local_size.has_value());
  EXPECT_EQ(*local_size, (std::array<uint32_t, 3>{8, 8, 1}));

  for (const int use_tgsm : {0, 1}) {
    for (const int use_fp16 : {0, 1}) {
      const auto variant_source = ShaderGlobalDefinesForTests() + "\n#define EE_TAA_USE_TGSM " +
                                  std::to_string(use_tgsm) + "\n#define EE_TAA_USE_FP16 " + std::to_string(use_fp16) +
                                  "\n" + ReadTextFile(shader_path);
      binaries.clear();
      ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, variant_source, binaries, shader_path))
          << "TGSM=" << use_tgsm << ", FP16=" << use_fp16;
      ASSERT_FALSE(binaries.empty());
    }
  }
}

TEST(ShaderCache, ProductionMotionVectorsSlangThreadGroupMatchesHostDispatch) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_path = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/MotionVectors.slang");
  const auto source = ShaderGlobalDefinesForTests() + ReadTextFile(shader_path);

  ShaderReflectionInfo reflection;
  std::string diagnostics;
  ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Compute, source, reflection, diagnostics, shader_path)) << diagnostics;
  EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{16, 16, 1}));
  std::vector<uint32_t> binaries;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries, shader_path)) << diagnostics;
  ASSERT_FALSE(binaries.empty());
  const auto local_size = FindSpirvLocalSizeExecutionMode(binaries);
  ASSERT_TRUE(local_size.has_value());
  EXPECT_EQ(*local_size, (std::array<uint32_t, 3>{16, 16, 1}));
}

TEST(ShaderCache, ProductionRayQueryCameraSlangThreadGroupMatchesHostDispatch) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_path = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/RayQueryCamera.slang");
  const auto source = ShaderGlobalDefinesForTests() + ReadTextFile(shader_path);

  ShaderReflectionInfo reflection;
  std::string diagnostics;
  ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Compute, source, reflection, diagnostics, shader_path)) << diagnostics;
  EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{8, 8, 1}));
  std::vector<uint32_t> binaries;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries, shader_path)) << diagnostics;
  ASSERT_FALSE(binaries.empty());
  const auto local_size = FindSpirvLocalSizeExecutionMode(binaries);
  ASSERT_TRUE(local_size.has_value());
  EXPECT_EQ(*local_size, (std::array<uint32_t, 3>{8, 8, 1}));
}

TEST(ShaderCache, ProductionGaussianSplatComputeSlangThreadGroupsMatchHostDispatch) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute");
  struct Case {
    std::filesystem::path path;
    std::array<uint32_t, 3> group_size;
  };
  const std::array<Case, 4> cases = {{
      {shader_root / "GaussianSplatCull.slang", {256, 1, 1}},
      {shader_root / "GaussianSplatRadixUpsweep.slang", {512, 1, 1}},
      {shader_root / "GaussianSplatRadixSpine.slang", {1, 1, 1}},
      {shader_root / "GaussianSplatRadixDownsweep.slang", {512, 1, 1}},
  }};

  for (const auto& test_case : cases) {
    const auto source = ShaderGlobalDefinesForTests() + ReadTextFile(test_case.path);
    ShaderReflectionInfo reflection;
    std::string diagnostics;
    ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Compute, source, reflection, diagnostics, test_case.path))
        << test_case.path.string() << "\n"
        << diagnostics;
    EXPECT_EQ(reflection.compute_thread_group_size, test_case.group_size) << test_case.path.string();
    std::vector<uint32_t> binaries;
    ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries, test_case.path))
        << test_case.path.string() << "\n"
        << diagnostics;
    ASSERT_FALSE(binaries.empty()) << test_case.path.string();
    const auto local_size = FindSpirvLocalSizeExecutionMode(binaries);
    ASSERT_TRUE(local_size.has_value()) << test_case.path.string();
    EXPECT_EQ(*local_size, test_case.group_size) << test_case.path.string();
  }
}

TEST(ShaderCache, ProductionDepthPyramidSlangThreadGroupMatchesHostDispatch) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_path = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/DepthPyramid.slang");
  const auto source = ShaderGlobalDefinesForTests() + ReadTextFile(shader_path);

  ShaderReflectionInfo reflection;
  std::string diagnostics;
  ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Compute, source, reflection, diagnostics, shader_path)) << diagnostics;
  EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{16, 16, 1}));
  std::vector<uint32_t> binaries;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries, shader_path)) << diagnostics;
  ASSERT_FALSE(binaries.empty());
  const auto local_size = FindSpirvLocalSizeExecutionMode(binaries);
  ASSERT_TRUE(local_size.has_value());
  EXPECT_EQ(*local_size, (std::array<uint32_t, 3>{16, 16, 1}));
}

TEST(ShaderCache, ProductionAmbientOcclusionSlangThreadGroupsMatchHostDispatch) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing");
  const std::array<std::filesystem::path, 2> shader_paths = {
      shader_root / "AmbientOcclusionGeometry.slang",
      shader_root / "AmbientOcclusionBlur.slang",
  };

  for (const auto& shader_path : shader_paths) {
    const auto source = ShaderGlobalDefinesForTests() + ReadTextFile(shader_path);
    ShaderReflectionInfo reflection;
    std::string diagnostics;
    ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Compute, source, reflection, diagnostics, shader_path))
        << shader_path.string() << "\n"
        << diagnostics;
    EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{16, 16, 1})) << shader_path.string();
    std::vector<uint32_t> binaries;
    ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries, shader_path))
        << shader_path.string() << "\n"
        << diagnostics;
    ASSERT_FALSE(binaries.empty()) << shader_path.string();
    const auto local_size = FindSpirvLocalSizeExecutionMode(binaries);
    ASSERT_TRUE(local_size.has_value()) << shader_path.string();
    EXPECT_EQ(*local_size, (std::array<uint32_t, 3>{16, 16, 1})) << shader_path.string();
  }
}

TEST(ShaderCache, ProductionSmaaPrepareSlangThreadGroupMatchesHostDispatch) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_path =
      RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing/SMAAPrepare.slang");
  const auto source = ShaderGlobalDefinesForTests() + ReadTextFile(shader_path);

  ShaderReflectionInfo reflection;
  std::string diagnostics;
  ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Compute, source, reflection, diagnostics, shader_path)) << diagnostics;
  EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{8, 8, 1}));
  std::vector<uint32_t> binaries;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries, shader_path)) << diagnostics;
  ASSERT_FALSE(binaries.empty());
  const auto local_size = FindSpirvLocalSizeExecutionMode(binaries);
  ASSERT_TRUE(local_size.has_value());
  EXPECT_EQ(*local_size, (std::array<uint32_t, 3>{8, 8, 1}));
}

TEST(ShaderCache, ProductionToneMappingSlangThreadGroupMatchesHostDispatch) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_root = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing");
  const auto tone_mapping_path = shader_root / "ToneMapping.slang";
  const auto histogram_path = shader_root / "ToneMappingHistogram.slang";
  const auto auto_exposure_path = shader_root / "ToneMappingAutoExposure.slang";
  const auto tone_mapping_source = ShaderGlobalDefinesForTests() + ReadTextFile(tone_mapping_path);
  const auto histogram_source = ShaderGlobalDefinesForTests() + ReadTextFile(histogram_path);
  const auto auto_exposure_source = ShaderGlobalDefinesForTests() + ReadTextFile(auto_exposure_path);

  ShaderReflectionInfo reflection;
  std::string diagnostics;
  ASSERT_TRUE(
      Shader::ReflectSlang(ShaderType::Compute, tone_mapping_source, reflection, diagnostics, tone_mapping_path))
      << diagnostics;
  const uint32_t host_pixel_batch_size =
      ExtractUnsignedShaderDefine(tone_mapping_source, "COMPUTE_WORK_GROUP_INVOCATIONS", 1u);
  const uint32_t expected_work_group_size =
      ExtractUnsignedShaderDefine(tone_mapping_source, "SUBGROUP_SIZE", 1u) *
      ExtractUnsignedShaderDefine(tone_mapping_source, "COMPUTE_SUBGROUP_COUNT", 1u);
  ASSERT_GE(expected_work_group_size, host_pixel_batch_size);
  EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{expected_work_group_size, 1, 1}));
  std::vector<uint32_t> binaries;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, tone_mapping_source, binaries, tone_mapping_path))
      << diagnostics;
  ASSERT_FALSE(binaries.empty());
  auto local_size = FindSpirvLocalSizeExecutionMode(binaries);
  ASSERT_TRUE(local_size.has_value());
  EXPECT_EQ(*local_size, (std::array<uint32_t, 3>{expected_work_group_size, 1, 1}));

  reflection = {};
  diagnostics.clear();
  ASSERT_TRUE(Shader::ReflectSlang(ShaderType::Compute, histogram_source, reflection, diagnostics, histogram_path))
      << diagnostics;
  EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{16, 16, 1}));
  binaries.clear();
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, histogram_source, binaries, histogram_path)) << diagnostics;
  ASSERT_FALSE(binaries.empty());
  local_size = FindSpirvLocalSizeExecutionMode(binaries);
  ASSERT_TRUE(local_size.has_value());
  EXPECT_EQ(*local_size, (std::array<uint32_t, 3>{16, 16, 1}));

  reflection = {};
  diagnostics.clear();
  ASSERT_TRUE(
      Shader::ReflectSlang(ShaderType::Compute, auto_exposure_source, reflection, diagnostics, auto_exposure_path))
      << diagnostics;
  EXPECT_EQ(reflection.compute_thread_group_size, (std::array<uint32_t, 3>{1, 1, 1}));
  binaries.clear();
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, auto_exposure_source, binaries, auto_exposure_path))
      << diagnostics;
  ASSERT_FALSE(binaries.empty());
  local_size = FindSpirvLocalSizeExecutionMode(binaries);
  ASSERT_TRUE(local_size.has_value());
  EXPECT_EQ(*local_size, (std::array<uint32_t, 3>{1, 1, 1}));
}

TEST(ShaderCache, SlangPipelineLayoutValidationFailsOnDescriptorMismatch) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_path =
      RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/PostProcessing/TAACopy.slang");
  const auto source = ShaderGlobalDefinesForTests() + ReadTextFile(shader_path);
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  layout->PushDescriptorBinding(1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);

  const auto validation = Shader::ValidateSlangPipelineLayout(ShaderType::Compute, source, shader_path, {layout}, {});

  EXPECT_FALSE(validation.success);
  EXPECT_NE(validation.diagnostics.find("Descriptor type mismatch"), std::string::npos) << validation.diagnostics;
}

TEST(ShaderCache, SlangMatrixPushConstantReflectionMatchesGlmLayout) {
  ShaderCacheScope scope;
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  const VkPushConstantRange push_constant_range{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                                static_cast<uint32_t>(sizeof(glm::mat4) + sizeof(glm::vec4))};

  const auto validation = Shader::ValidateSlangPipelineLayout(ShaderType::Compute, kSlangMatrixPushShader, {}, {layout},
                                                              {push_constant_range});
  std::vector<uint32_t> binaries;

  ASSERT_TRUE(validation.success) << validation.diagnostics;
  ASSERT_EQ(validation.reflection.push_constant_ranges.size(), 1u);
  EXPECT_EQ(validation.reflection.push_constant_ranges[0].offset, 0u);
  EXPECT_EQ(validation.reflection.push_constant_ranges[0].size, push_constant_range.size);
  EXPECT_EQ(validation.reflection.compute_thread_group_size, (std::array<uint32_t, 3>{1, 1, 1}));
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kSlangMatrixPushShader, binaries));
  ASSERT_FALSE(binaries.empty());
  const auto matrix_decorations = CountSpirvMatrixStorageDecorations(binaries);
  EXPECT_GT(matrix_decorations.column_major, 0u);
  EXPECT_EQ(matrix_decorations.row_major, 0u);
}

TEST(ShaderCache, SlangCompatibilityMatricesUseGlmStorageLayout) {
  ShaderCacheScope scope;
  RegisterDefaultShaderIncludePath();
  const auto shader_path = RepoPath("EvoEngine_SDK/Internals/DefaultResources/Shaders/Compute/RayQueryCamera.slang");
  const auto source = ShaderGlobalDefinesForTests() + ReadTextFile(shader_path);
  std::vector<uint32_t> binaries;

  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, source, binaries, shader_path));
  ASSERT_FALSE(binaries.empty());
  const auto matrix_decorations = CountSpirvMatrixStorageDecorations(binaries);
  EXPECT_GT(matrix_decorations.column_major, 0u);
  EXPECT_EQ(matrix_decorations.row_major, 0u);
}

TEST(ShaderCache, SlangPipelineLayoutValidationFailsOnPushConstantMismatch) {
  ShaderCacheScope scope;
  auto layout = std::make_shared<DescriptorSetLayout>();
  layout->PushDescriptorBinding(0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT, 0);
  const VkPushConstantRange push_constant_range{VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                                static_cast<uint32_t>(sizeof(glm::mat4))};

  const auto validation = Shader::ValidateSlangPipelineLayout(ShaderType::Compute, kSlangMatrixPushShader, {}, {layout},
                                                              {push_constant_range});

  EXPECT_FALSE(validation.success);
  EXPECT_NE(validation.diagnostics.find("Missing push-constant range"), std::string::npos) << validation.diagnostics;
}

TEST(ShaderCache, SlangStageIoValidationFailsOnMismatch) {
  ShaderCacheScope scope;
  ShaderReflectionStageIo input;
  input.semantic_name = "POSITION";
  input.location = 0;
  ShaderReflectionStageIo output;
  output.semantic_name = "TEXCOORD";
  output.location = 0;
  ShaderReflectionStageIo wrong_input = input;
  wrong_input.location = 1;

  const auto valid = Shader::ValidateSlangPipelineLayout(ShaderType::Vertex, kSlangVertexIoShader, {}, {}, {},
                                                         std::vector{input}, std::vector{output});
  const auto invalid = Shader::ValidateSlangPipelineLayout(ShaderType::Vertex, kSlangVertexIoShader, {}, {}, {},
                                                           std::vector{wrong_input}, std::vector{output});

  ASSERT_TRUE(valid.success) << valid.diagnostics;
  ASSERT_EQ(valid.reflection.stage_inputs.size(), 1u);
  EXPECT_EQ(valid.reflection.stage_inputs[0].location, 0u);
  EXPECT_EQ(valid.reflection.stage_outputs.size(), 1u);
  EXPECT_EQ(valid.reflection.stage_outputs[0].location, 0u);
  EXPECT_FALSE(invalid.success);
  EXPECT_NE(invalid.diagnostics.find("stage input mismatch"), std::string::npos) << invalid.diagnostics;
}

TEST(ShaderCache, CoalescesConcurrentSlangRequests) {
  ShaderCacheScope scope;
  auto* application = ApplicationContext::TryGet();
  ASSERT_NE(application, nullptr);
  constexpr size_t request_count = 8;
  std::vector<std::future<std::vector<uint32_t>>> requests;
  requests.reserve(request_count);
  for (size_t i = 0; i < request_count; ++i) {
    requests.emplace_back(std::async(std::launch::async, [application]() {
      ApplicationContextScope application_scope(*application);
      std::vector<uint32_t> binaries;
      if (!Shader::CompileToSpirv(ShaderType::Compute, kSlangComputeShader, binaries))
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
      ShaderType::Fragment, "layout(location = 0) out float4 color; void main() { color = float4(1.0); }", fragment));
  std::vector<uint32_t> changed_source;
  ASSERT_TRUE(Shader::CompileToSpirv(
      ShaderType::Compute, "layout(local_size_x = 1) in; void main() { uint value = gl_GlobalInvocationID.x + 1u; }",
      changed_source));
  EXPECT_EQ(CacheFileCount(scope.Root()), 3u);
  EXPECT_EQ(Shader::GetCompileCacheStats().compilations, 3u);
}

TEST(ShaderCache, EcoSysLabGlslPackedFungusEdgeShaderCompiles) {
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
  const auto include_path = scope.Root() / "m5_shader_cache_include.slangh";
  {
    std::ofstream file(include_path);
    file << "float M5_INCLUDED_VALUE() { return 1.0; }\n";
  }
  Shader::RegisterShaderIncludePath(scope.Root());
  const std::string source = R"(
#extension GL_GOOGLE_include_directive : require
#include "m5_shader_cache_include.slangh"
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
  EXPECT_FALSE(second.empty());
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

TEST(ShaderCache, SlangCorruptEntryIsRecompiledWithoutThrowing) {
  ShaderCacheScope scope;
  std::vector<uint32_t> first;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kSlangComputeShader, first));
  const auto cache_file = FirstCacheFile(scope.Root());
  ASSERT_FALSE(cache_file.empty());
  {
    std::ofstream file(cache_file, std::ios::binary | std::ios::trunc);
    file << "truncated";
  }
  Shader::ClearInMemoryCompileCache();
  Shader::ResetCompileCacheStats();
  std::vector<uint32_t> repaired;
  ASSERT_TRUE(Shader::CompileToSpirv(ShaderType::Compute, kSlangComputeShader, repaired));
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
  EXPECT_EQ(stats.compilations, 0u);
  EXPECT_EQ(stats.failures, 2u);
}
