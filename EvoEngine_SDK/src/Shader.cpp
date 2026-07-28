#include "Shader.hpp"
#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <mutex>
#include <optional>
#include <thread>
#include "AssetManager.hpp"
#include "Console.hpp"
#include "PathUtils.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "ResourceLimits.h"
#include "SPIRV/GlslangToSpv.h"
#include "Serialization.hpp"
#include "ShaderLang.h"
#include "Utilities.hpp"
#include "slang-com-ptr.h"
#include "slang.h"

using namespace evo_engine;

namespace {
constexpr uint32_t kShaderCacheSchema = 11;
constexpr uint32_t kSlangDependencyCacheSchema = 1;
constexpr uint32_t kVulkanTarget = 13;
constexpr uint32_t kSpirvTarget = 14;
constexpr uint32_t kSpirvMagic = 0x07230203;
constexpr const char* kDefaultShaderEntryPoint = "main";
constexpr uint64_t kMaxSlangDependencyCount = 4096;
constexpr uint64_t kMaxSlangDependencyPathBytes = 32768;

struct ShaderCompileTarget {
  uint32_t slang_input_language = SLANG_SOURCE_LANGUAGE_SLANG;
};

struct ShaderCompileRequest {
  ShaderCompileTarget target = {};
  ShaderType shader_type = ShaderType::Unknown;
  std::string entry_point = kDefaultShaderEntryPoint;
  std::string global_defines;
  std::string source;
  std::filesystem::path path;
  std::set<std::filesystem::path> include_paths;
};

struct ShaderCompileResult {
  bool success = false;
  std::vector<uint32_t> binaries;
  std::string diagnostics;
};

std::filesystem::path GetShaderBinaryDirectory() {
  if (const char* path = std::getenv("EVOENGINE_SHADER_CACHE_DIR"); path && path[0] != '\0') {
    return path_utils::NormalizeAbsolutePath(path);
  }
  if (const auto executable_path = path_utils::CurrentExecutablePath();
      !executable_path.empty() && executable_path.has_parent_path()) {
    return executable_path.parent_path() / "ShaderBinaries";
  }
  return path_utils::NormalizeAbsolutePath("ShaderBinaries");
}

struct ShaderCacheKey {
  uint64_t low = 0;
  uint64_t high = 0;

  bool operator==(const ShaderCacheKey& other) const {
    return low == other.low && high == other.high;
  }
};

struct ShaderCacheKeyHasher {
  size_t operator()(const ShaderCacheKey& key) const {
    return static_cast<size_t>(key.low ^ (key.high + 0x9e3779b97f4a7c15ull + (key.low << 6u) + (key.low >> 2u)));
  }
};

struct ShaderCompileEntry {
  std::mutex mutex;
  std::condition_variable completed;
  bool done = false;
  bool success = false;
  std::vector<uint32_t> binaries;
};

std::mutex shader_compile_entries_mutex;
std::unordered_map<ShaderCacheKey, std::shared_ptr<ShaderCompileEntry>, ShaderCacheKeyHasher> shader_compile_entries;
std::mutex slang_mutex;
std::mutex glslang_mutex;
std::atomic<uint64_t> temporary_file_counter = 0;
std::atomic<uint64_t> memory_hit_count = 0;
std::atomic<uint64_t> disk_hit_count = 0;
std::atomic<uint64_t> disk_miss_count = 0;
std::atomic<uint64_t> compilation_count = 0;
std::atomic<uint64_t> coalesced_wait_count = 0;
std::atomic<uint64_t> corrupt_entry_count = 0;
std::atomic<uint64_t> failure_count = 0;
std::atomic<uint64_t> slang_frontend_count = 0;

std::string LowercaseExtension(const std::filesystem::path& path) {
  std::string extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return extension;
}

void HashBytes(uint64_t& hash, const void* data, const size_t size) {
  const auto* bytes = static_cast<const uint8_t*>(data);
  for (size_t i = 0; i < size; ++i) {
    hash ^= bytes[i];
    hash *= 1099511628211ull;
  }
}

std::set<std::filesystem::path> MakeShaderIncludePaths(const std::filesystem::path& path) {
  auto include_paths = Shader::GetRegisteredShaderIncludePaths();
  if (!path.empty() && path.has_parent_path()) {
    include_paths.emplace(path.parent_path());
  }
  return include_paths;
}

bool UsesCompatibilitySyntax(const std::string& source) {
  return source.find("#extension GL_") != std::string::npos || source.find("layout(") != std::string::npos ||
         source.find("layout (") != std::string::npos || source.find("precision highp") != std::string::npos ||
         source.find("readonly buffer") != std::string::npos || source.find("writeonly buffer") != std::string::npos;
}

bool IsSupportedSlangInputLanguage(const uint32_t input_language) {
  return input_language == SLANG_SOURCE_LANGUAGE_SLANG || input_language == SLANG_SOURCE_LANGUAGE_GLSL;
}

const char* SlangInputDialectName(const uint32_t input_language) {
  return input_language == SLANG_SOURCE_LANGUAGE_GLSL ? "compatibility" : "native";
}

bool IsGlslStageExtension(const std::string& extension) {
  return extension == ".vert" || extension == ".tesc" || extension == ".tese" || extension == ".geom" ||
         extension == ".frag" || extension == ".comp" || extension == ".task" || extension == ".mesh" ||
         extension == ".rgen" || extension == ".rmiss" || extension == ".rahit" || extension == ".rchit" ||
         extension == ".rint" || extension == ".rcall";
}

bool ShouldCompileWithGlslang(const ShaderCompileRequest& request) {
  return request.target.slang_input_language == SLANG_SOURCE_LANGUAGE_GLSL &&
         IsGlslStageExtension(LowercaseExtension(request.path));
}

const char* ShaderCompilerBackendName(const ShaderCompileRequest& request) {
  return ShouldCompileWithGlslang(request) ? "glslang" : "slang";
}

ShaderCompileRequest MakeSlangCompileRequest(const ShaderType shader_type, std::string source,
                                             const std::filesystem::path& path) {
  ShaderCompileRequest request;
  request.shader_type = shader_type;
  request.source = std::move(source);
  if (UsesCompatibilitySyntax(request.source)) {
    request.target.slang_input_language = SLANG_SOURCE_LANGUAGE_GLSL;
  }
  request.path = path;
  request.include_paths = MakeShaderIncludePaths(path);
  return request;
}

bool SourceDefinesEnabled(const std::string& source, const char* name) {
  std::istringstream stream(source);
  std::string directive;
  std::string define_name;
  std::string value;
  while (stream >> directive >> define_name) {
    if (directive == "#define" && define_name == name && stream >> value) {
      char* end = nullptr;
      const unsigned long parsed_value = std::strtoul(value.c_str(), &end, 0);
      if (end != value.c_str()) {
        return parsed_value != 0;
      }
      return value == "true" || value == "TRUE";
    }
    stream.ignore((std::numeric_limits<std::streamsize>::max)(), '\n');
  }
  return false;
}

bool RequiresShaderInvocationReorderCapability(const ShaderCompileRequest& request) {
  return SourceDefinesEnabled(request.global_defines, "EE_SHADER_EXECUTION_REORDERING_SUPPORTED") ||
         SourceDefinesEnabled(request.source, "EE_SHADER_EXECUTION_REORDERING_SUPPORTED");
}

void HashKeyBytes(ShaderCacheKey& key, const void* data, const size_t size) {
  HashBytes(key.low, data, size);
  HashBytes(key.high, data, size);
}

void HashKeyString(ShaderCacheKey& key, const std::string& value) {
  const auto size = static_cast<uint64_t>(value.size());
  HashKeyBytes(key, &size, sizeof(size));
  HashKeyBytes(key, value.data(), value.size());
}

std::string MakeShaderTargetProfileString(const ShaderCompileRequest& request) {
  std::ostringstream stream;
  stream << "vulkan_" << (kVulkanTarget / 10) << "_" << (kVulkanTarget % 10) << ";spirv_" << (kSpirvTarget / 10) << "_"
         << (kSpirvTarget % 10)
         << ";entry=" << (request.entry_point.empty() ? kDefaultShaderEntryPoint : request.entry_point);
  stream << ";compiler_backend=" << ShaderCompilerBackendName(request);
  stream << ";emit_spirv_directly;matrix_layout=row_major;scalar_layout=true;slang_input_dialect="
         << SlangInputDialectName(request.target.slang_input_language);
  stream << ";shader_invocation_reorder_ext="
         << (RequiresShaderInvocationReorderCapability(request) ? "true" : "false");
  return stream.str();
}

std::string MakeIncludePathSignature(const ShaderCompileRequest& request) {
  std::vector<std::string> paths;
  paths.reserve(request.include_paths.size());
  for (const auto& include_path : request.include_paths) {
    paths.emplace_back(include_path.lexically_normal().generic_string());
  }
  std::sort(paths.begin(), paths.end());
  std::ostringstream stream;
  for (const auto& path : paths) {
    stream << path << '\0';
  }
  return stream.str();
}

ShaderCacheKey MakeShaderCacheKey(const ShaderCompileRequest& request, const std::string& compiler_version,
                                  const std::string& source_signature, const std::string& dependency_signature = {}) {
  ShaderCacheKey key{14695981039346656037ull, 1099511628211ull ^ 0xd6e8feb86659fd93ull};
  const std::array<uint32_t, 3> descriptor = {kShaderCacheSchema, static_cast<uint32_t>(request.shader_type),
                                              request.target.slang_input_language};
  HashKeyBytes(key, descriptor.data(), descriptor.size() * sizeof(uint32_t));
  HashKeyString(key, compiler_version);
  HashKeyString(key, MakeShaderTargetProfileString(request));
  HashKeyString(key, request.entry_point.empty() ? kDefaultShaderEntryPoint : request.entry_point);
  HashKeyString(key, request.global_defines);
  HashKeyString(key, MakeIncludePathSignature(request));
  HashKeyString(key, source_signature);
  HashKeyString(key, dependency_signature);
  return key;
}

ShaderCacheKey MakePayloadChecksum(const std::vector<uint32_t>& binaries) {
  ShaderCacheKey key{14695981039346656037ull, 1099511628211ull ^ 0x94d049bb133111ebull};
  HashBytes(key.low, binaries.data(), binaries.size() * sizeof(uint32_t));
  HashBytes(key.high, binaries.data(), binaries.size() * sizeof(uint32_t));
  return key;
}

std::string ShaderCacheKeyString(const ShaderCacheKey& key) {
  std::ostringstream stream;
  stream << std::hex << std::setfill('0') << std::setw(16) << key.high << std::setw(16) << key.low;
  return stream.str();
}

class ShaderStagedLoadPayload final : public StagedAssetLoadPayload {
 public:
  std::string shader_code;
  unsigned shader_type = static_cast<unsigned>(ShaderType::Unknown);
};
}  // namespace

bool Shader::SaveInternal(const std::filesystem::path& path) const {
  try {
    if (LowercaseExtension(path) == ".eveshader") {
      YAML::Emitter out;
      out << YAML::BeginMap;
      out << YAML::Key << "shader_type" << YAML::Value << shader_type;
      out << YAML::Key << "shader_code" << YAML::Value << shader_code;
      out << YAML::EndMap;
      std::ofstream file_output(path.string());
      file_output << out.c_str();
      file_output.close();
    } else {
      std::ofstream file_output(path.string());
      file_output << shader_code.c_str();
      file_output.close();
    }
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to save!")
    return false;
  }
  return true;
}

bool Shader::LoadInternal(const std::filesystem::path& path) {
  if (!std::filesystem::exists(path)) {
    EVOENGINE_ERROR("Not exist!")
    return false;
  }
  try {
    const std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    const auto extension = LowercaseExtension(path);
    if (extension == ".eveshader") {
      const YAML::Node in = YAML::Load(string_stream.str());
      if (in["shader_code"])
        shader_code = in["shader_code"].as<std::string>();
      if (in["shader_type"])
        shader_type = in["shader_type"].as<unsigned>();
    } else {
      shader_type = static_cast<unsigned>(ShaderType::Unknown);
      shader_code = string_stream.str();
    }
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to load!")
    return false;
  }
  return true;
}

bool Shader::SupportsStagedLoading() const {
  return true;
}

std::shared_ptr<StagedAssetLoadPayload> Shader::LoadStagedPayloadInternal(const std::filesystem::path& path) const {
  if (!std::filesystem::exists(path)) {
    EVOENGINE_ERROR("Not exist!")
    return {};
  }
  try {
    const std::ifstream stream(path.string());
    std::stringstream string_stream;
    string_stream << stream.rdbuf();
    auto payload = std::make_shared<ShaderStagedLoadPayload>();
    const auto extension = LowercaseExtension(path);
    if (extension == ".eveshader") {
      const YAML::Node in = YAML::Load(string_stream.str());
      if (in["shader_code"]) {
        payload->shader_code = in["shader_code"].as<std::string>();
      }
      if (in["shader_type"]) {
        payload->shader_type = in["shader_type"].as<unsigned>();
      }
    } else {
      payload->shader_type = static_cast<unsigned>(ShaderType::Unknown);
      payload->shader_code = string_stream.str();
    }
    return payload;
  } catch (const std::exception& e) {
    EVOENGINE_ERROR("Failed to load staged shader payload: " + std::string(e.what()))
    return {};
  }
}

bool Shader::ApplyStagedPayloadInternal(const std::filesystem::path&,
                                        const std::shared_ptr<StagedAssetLoadPayload>& payload) {
  const auto shader_payload = std::dynamic_pointer_cast<ShaderStagedLoadPayload>(payload);
  if (!shader_payload) {
    return false;
  }
  shader_code = std::move(shader_payload->shader_code);
  shader_type = shader_payload->shader_type;
  return true;
}

bool Shader::RegisterAssetIoHandlers(const std::string& owner_name, const std::string& type_name) {
  return Serialization::RegisterAssetIoHandler<Shader>(
      [](const Shader& asset, const std::filesystem::path& path) {
        return asset.SaveInternal(path);
      },
      [](Shader& asset, const std::filesystem::path& path) {
        return asset.LoadInternal(path);
      },
      [](const Shader& asset, const std::filesystem::path&) {
        return asset.SupportsStagedLoading();
      },
      [](const Shader& asset, const std::filesystem::path& path) {
        return asset.LoadStagedPayloadInternal(path);
      },
      [](Shader& asset, const std::filesystem::path& path, const std::shared_ptr<StagedAssetLoadPayload>& payload) {
        return asset.ApplyStagedPayloadInternal(path, payload);
      },
      owner_name, type_name);
}

void Shader::RegisterShaderIncludePath(const std::filesystem::path& path) {
  Platform::GetInstance().RegisterShaderIncludePath(path);
}

std::set<std::filesystem::path> Shader::GetRegisteredShaderIncludePaths() {
  return Platform::GetInstance().GetRegisteredShaderIncludePaths();
}

bool Shader::Compiled() const {
  return shader_module != nullptr;
}

class GlslShaderIncluder : public glslang::TShader::Includer {
 public:
  explicit GlslShaderIncluder(std::set<std::filesystem::path> include_paths)
      : include_paths_(std::move(include_paths)) {
  }

  IncludeResult* includeSystem(const char* header_name, const char*, size_t) override;
  IncludeResult* includeLocal(const char* header_name, const char* includer_name, size_t inclusion_depth) override;
  void releaseInclude(IncludeResult*) override;

 private:
  std::set<std::filesystem::path> include_paths_;
  IncludeResult fail_result_ = IncludeResult("", "Header does not exist!", 0, nullptr);
  std::unordered_map<std::filesystem::path, std::shared_ptr<IncludeResult>> includes_;
  std::unordered_map<std::filesystem::path, std::string> sources_;
};

glslang::TShader::Includer::IncludeResult* GlslShaderIncluder::includeSystem(const char* header_name, const char*,
                                                                             size_t) {
  const std::filesystem::path requested_header(header_name);
  std::filesystem::path resolved_header_path;
  bool found = false;
  for (const auto& include_path : include_paths_) {
    const auto candidate = include_path / requested_header;
    if (std::filesystem::exists(candidate)) {
      resolved_header_path = std::filesystem::weakly_canonical(candidate);
      found = true;
      break;
    }
  }
  if (!found && std::filesystem::exists(requested_header)) {
    resolved_header_path = std::filesystem::weakly_canonical(requested_header);
    found = true;
  }
  if (!found) {
    return &fail_result_;
  }
  if (const auto existing = includes_.find(resolved_header_path); existing != includes_.end()) {
    return existing->second.get();
  }
  sources_[resolved_header_path] = FileUtils::LoadFileAsString(resolved_header_path);
  auto [inserted, success] = includes_.emplace(std::make_pair(
      resolved_header_path,
      std::make_shared<IncludeResult>(resolved_header_path.string(), sources_.at(resolved_header_path).data(),
                                      sources_.at(resolved_header_path).size(), nullptr)));
  if (!success) {
    return &fail_result_;
  }
  return inserted->second.get();
}

glslang::TShader::Includer::IncludeResult* GlslShaderIncluder::includeLocal(const char* header_name,
                                                                            const char* includer_name,
                                                                            const size_t inclusion_depth) {
  return includeSystem(header_name, includer_name, inclusion_depth);
}

void GlslShaderIncluder::releaseInclude(IncludeResult* result) {
  if (result == &fail_result_) {
    return;
  }
  std::filesystem::path resolved_header_path(result->headerName);
  sources_.erase(resolved_header_path);
  includes_.erase(resolved_header_path);
}

class GlslangProcessLifetime {
 public:
  GlslangProcessLifetime() {
    glslang::InitializeProcess();
  }

  ~GlslangProcessLifetime() {
    glslang::FinalizeProcess();
  }
};

void EnsureGlslangProcess() {
  static GlslangProcessLifetime lifetime;
  static_cast<void>(lifetime);
}

bool TryGetGlslangLanguage(const ShaderType shader_type, EShLanguage& language) {
  switch (shader_type) {
    case ShaderType::Task:
      language = EShLangTask;
      return true;
    case ShaderType::Mesh:
      language = EShLangMesh;
      return true;
    case ShaderType::Vertex:
      language = EShLangVertex;
      return true;
    case ShaderType::TessellationControl:
      language = EShLangTessControl;
      return true;
    case ShaderType::TessellationEvaluation:
      language = EShLangTessEvaluation;
      return true;
    case ShaderType::Geometry:
      language = EShLangGeometry;
      return true;
    case ShaderType::Fragment:
      language = EShLangFragment;
      return true;
    case ShaderType::Compute:
      language = EShLangCompute;
      return true;
    case ShaderType::RayGen:
      language = EShLangRayGen;
      return true;
    case ShaderType::Miss:
      language = EShLangMiss;
      return true;
    case ShaderType::AnyHit:
      language = EShLangAnyHit;
      return true;
    case ShaderType::ClosestHit:
      language = EShLangClosestHit;
      return true;
    case ShaderType::Intersection:
      language = EShLangIntersect;
      return true;
    case ShaderType::Callable:
      language = EShLangCallable;
      return true;
    case ShaderType::Unknown:
      return false;
  }
  return false;
}

void ConfigureGlslShader(glslang::TShader& shader, const EShLanguage language) {
  constexpr int default_version = 460;
  shader.setEnvClient(glslang::EShClientVulkan, glslang::EShTargetVulkan_1_3);
  shader.setEnvTarget(glslang::EshTargetSpv, glslang::EShTargetSpv_1_4);
  shader.setEnvInput(glslang::EShSourceGlsl, language, glslang::EShClientVulkan, default_version);
  shader.setEntryPoint(kDefaultShaderEntryPoint);
}

std::string MakeGlslangCompilerVersionString() {
  const auto compiler_version = glslang::GetVersion();
  std::ostringstream stream;
  stream << "glslang-" << compiler_version.major << "." << compiler_version.minor << "." << compiler_version.patch;
  if (compiler_version.flavor) {
    stream << "-" << compiler_version.flavor;
  }
  return stream.str();
}

bool PreprocessGlsl(const ShaderCompileRequest& request, std::string& preprocessed_source) {
  EShLanguage language;
  if (!TryGetGlslangLanguage(request.shader_type, language)) {
    EVOENGINE_ERROR("Unknown GLSL shader stage: " + request.path.string())
    return false;
  }
  EnsureGlslangProcess();
  const std::lock_guard glslang_lock(glslang_mutex);
  glslang::TShader shader(language);
  const std::string actual_code = std::string("#version 460\n") + request.source;
  const char* sources[] = {actual_code.c_str()};
  shader.setStrings(sources, 1);
  ConfigureGlslShader(shader, language);
  constexpr int default_version = 460;
  constexpr bool forward_compatible = false;
  constexpr auto message_flags = static_cast<EShMessages>(EShMsgSpvRules | EShMsgVulkanRules);
  GlslShaderIncluder includer(request.include_paths);
  if (!shader.preprocess(GetDefaultResources(), default_version, ECoreProfile, false, forward_compatible, message_flags,
                         &preprocessed_source, includer)) {
    EVOENGINE_ERROR("Failed to preprocess GLSL shader: " + request.path.string() + "\n" + shader.getInfoLog())
    return false;
  }
  return true;
}

bool CompilePreprocessedGlsl(const ShaderCompileRequest& request, const std::string& preprocessed_source,
                             std::vector<uint32_t>& binaries) {
  EShLanguage language;
  if (!TryGetGlslangLanguage(request.shader_type, language)) {
    return false;
  }
  EnsureGlslangProcess();
  const std::lock_guard glslang_lock(glslang_mutex);
  glslang::TShader shader(language);
  const char* sources[] = {preprocessed_source.c_str()};
  shader.setStrings(sources, 1);
  ConfigureGlslShader(shader, language);
  constexpr int default_version = 460;
  constexpr bool forward_compatible = false;
  constexpr auto message_flags = static_cast<EShMessages>(EShMsgSpvRules | EShMsgVulkanRules);
  GlslShaderIncluder includer(request.include_paths);
  if (!shader.parse(GetDefaultResources(), default_version, ECoreProfile, false, forward_compatible, message_flags,
                    includer)) {
    EVOENGINE_ERROR("Failed to parse GLSL shader: " + request.path.string() + "\n" + shader.getInfoLog())
    return false;
  }
  glslang::TProgram program;
  program.addShader(&shader);
  if (!program.link(message_flags)) {
    EVOENGINE_ERROR("Failed to link GLSL shader: " + request.path.string() + "\n" + program.getInfoLog())
    return false;
  }
  glslang::SpvOptions options{};
  options.generateDebugInfo = true;
  options.validate = true;
  spv::SpvBuildLogger logger;
  GlslangToSpv(*program.getIntermediate(language), binaries, &logger, &options);
  if (binaries.empty() || binaries.front() != kSpirvMagic) {
    EVOENGINE_ERROR("glslang produced invalid SPIR-V: " + request.path.string() + "\n" + logger.getAllMessages())
    return false;
  }
  return true;
}

template <typename T>
bool ReadBinaryValue(std::ifstream& stream, T& value) {
  stream.read(reinterpret_cast<char*>(&value), sizeof(T));
  return stream.good();
}

template <typename T>
void WriteBinaryValue(std::ofstream& stream, const T& value) {
  stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

enum class ShaderCacheLoadResult { Missing, Valid, Corrupt };

ShaderCacheLoadResult LoadShaderCacheEntry(const std::filesystem::path& cache_path, const ShaderCacheKey& key,
                                           const ShaderCompileRequest& request, std::vector<uint32_t>& binaries) {
  std::error_code error;
  if (!std::filesystem::exists(cache_path, error) || error) {
    return ShaderCacheLoadResult::Missing;
  }
  try {
    std::ifstream stream(cache_path, std::ios::binary);
    const std::array<char, 8> expected_magic = {'E', 'V', 'O', 'S', 'P', 'V', '1', '1'};
    std::array<char, 8> magic{};
    stream.read(magic.data(), magic.size());
    uint32_t schema = 0;
    uint32_t cached_shader_type = 0;
    uint32_t slang_input_language = 0;
    ShaderCacheKey cached_key{};
    ShaderCacheKey checksum{};
    uint64_t word_count = 0;
    if (!stream || magic != expected_magic || !ReadBinaryValue(stream, schema) ||
        !ReadBinaryValue(stream, cached_shader_type) || !ReadBinaryValue(stream, slang_input_language) ||
        !ReadBinaryValue(stream, cached_key.low) || !ReadBinaryValue(stream, cached_key.high) ||
        !ReadBinaryValue(stream, checksum.low) || !ReadBinaryValue(stream, checksum.high) ||
        !ReadBinaryValue(stream, word_count) || schema != kShaderCacheSchema ||
        cached_shader_type != static_cast<uint32_t>(request.shader_type) ||
        slang_input_language != request.target.slang_input_language || !(cached_key == key) || word_count == 0) {
      return ShaderCacheLoadResult::Corrupt;
    }
    constexpr uint64_t header_size = 60;
    const auto file_size = std::filesystem::file_size(cache_path);
    if (file_size < header_size || word_count > (file_size - header_size) / sizeof(uint32_t) ||
        file_size != header_size + word_count * sizeof(uint32_t)) {
      return ShaderCacheLoadResult::Corrupt;
    }
    binaries.resize(static_cast<size_t>(word_count));
    stream.read(reinterpret_cast<char*>(binaries.data()), static_cast<std::streamsize>(word_count * sizeof(uint32_t)));
    if (!stream || binaries.front() != kSpirvMagic || !(MakePayloadChecksum(binaries) == checksum)) {
      binaries.clear();
      return ShaderCacheLoadResult::Corrupt;
    }
    return ShaderCacheLoadResult::Valid;
  } catch (...) {
    binaries.clear();
    return ShaderCacheLoadResult::Corrupt;
  }
}

bool PublishShaderCacheEntry(const std::filesystem::path& cache_path, const ShaderCacheKey& key,
                             const ShaderCompileRequest& request, const std::vector<uint32_t>& binaries) {
  std::filesystem::path temporary_path;
  try {
    std::filesystem::create_directories(cache_path.parent_path());
    temporary_path = cache_path;
    temporary_path += ".tmp." + std::to_string(temporary_file_counter.fetch_add(1)) + "." +
                      std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    const std::array<char, 8> magic = {'E', 'V', 'O', 'S', 'P', 'V', '1', '1'};
    stream.write(magic.data(), magic.size());
    WriteBinaryValue(stream, kShaderCacheSchema);
    const auto type_value = static_cast<uint32_t>(request.shader_type);
    WriteBinaryValue(stream, type_value);
    WriteBinaryValue(stream, request.target.slang_input_language);
    WriteBinaryValue(stream, key.low);
    WriteBinaryValue(stream, key.high);
    const auto checksum = MakePayloadChecksum(binaries);
    WriteBinaryValue(stream, checksum.low);
    WriteBinaryValue(stream, checksum.high);
    const auto word_count = static_cast<uint64_t>(binaries.size());
    WriteBinaryValue(stream, word_count);
    stream.write(reinterpret_cast<const char*>(binaries.data()),
                 static_cast<std::streamsize>(binaries.size() * sizeof(uint32_t)));
    stream.flush();
    if (!stream) {
      stream.close();
      std::filesystem::remove(temporary_path);
      return false;
    }
    stream.close();
    std::error_code error;
    std::filesystem::rename(temporary_path, cache_path, error);
    if (error) {
      if (std::filesystem::exists(cache_path)) {
        std::filesystem::remove(temporary_path);
        return true;
      }
      std::filesystem::remove(temporary_path);
      return false;
    }
    return true;
  } catch (...) {
    std::error_code error;
    if (!temporary_path.empty())
      std::filesystem::remove(temporary_path, error);
    return false;
  }
}

bool ReadBinaryString(std::ifstream& stream, std::string& value) {
  uint64_t size = 0;
  if (!ReadBinaryValue(stream, size) || size > kMaxSlangDependencyPathBytes) {
    return false;
  }
  value.resize(static_cast<size_t>(size));
  if (size == 0) {
    return true;
  }
  stream.read(value.data(), static_cast<std::streamsize>(size));
  return stream.good();
}

void WriteBinaryString(std::ofstream& stream, const std::string& value) {
  const auto size = static_cast<uint64_t>(value.size());
  WriteBinaryValue(stream, size);
  if (size != 0) {
    stream.write(value.data(), static_cast<std::streamsize>(size));
  }
}

std::filesystem::path SlangDependencyCachePath(const ShaderCacheKey& source_key) {
  return GetShaderBinaryDirectory() / (ShaderCacheKeyString(source_key) + ".slangdeps");
}

bool LoadSlangDependencyCacheEntry(const ShaderCacheKey& source_key,
                                   std::vector<std::filesystem::path>& dependency_paths) {
  dependency_paths.clear();
  const auto cache_path = SlangDependencyCachePath(source_key);
  std::error_code error;
  if (!std::filesystem::exists(cache_path, error) || error) {
    return false;
  }
  try {
    std::ifstream stream(cache_path, std::ios::binary);
    const std::array<char, 8> expected_magic = {'E', 'V', 'O', 'S', 'L', 'D', '1', '0'};
    std::array<char, 8> magic{};
    stream.read(magic.data(), magic.size());
    uint32_t schema = 0;
    ShaderCacheKey cached_source_key{};
    uint64_t dependency_count = 0;
    if (!stream || magic != expected_magic || !ReadBinaryValue(stream, schema) ||
        !ReadBinaryValue(stream, cached_source_key.low) || !ReadBinaryValue(stream, cached_source_key.high) ||
        !ReadBinaryValue(stream, dependency_count) || schema != kSlangDependencyCacheSchema ||
        !(cached_source_key == source_key) || dependency_count > kMaxSlangDependencyCount) {
      return false;
    }
    dependency_paths.reserve(static_cast<size_t>(dependency_count));
    for (uint64_t dependency_index = 0; dependency_index < dependency_count; ++dependency_index) {
      std::string dependency_path;
      if (!ReadBinaryString(stream, dependency_path)) {
        dependency_paths.clear();
        return false;
      }
      dependency_paths.emplace_back(std::move(dependency_path));
    }
    return stream.peek() == std::ifstream::traits_type::eof();
  } catch (...) {
    dependency_paths.clear();
    return false;
  }
}

bool PublishSlangDependencyCacheEntry(const ShaderCacheKey& source_key,
                                      const std::vector<std::filesystem::path>& dependency_paths) {
  if (dependency_paths.size() > kMaxSlangDependencyCount) {
    return false;
  }
  const auto cache_path = SlangDependencyCachePath(source_key);
  std::filesystem::path temporary_path;
  try {
    std::filesystem::create_directories(cache_path.parent_path());
    temporary_path = cache_path;
    temporary_path += ".tmp." + std::to_string(temporary_file_counter.fetch_add(1)) + "." +
                      std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    const std::array<char, 8> magic = {'E', 'V', 'O', 'S', 'L', 'D', '1', '0'};
    stream.write(magic.data(), magic.size());
    WriteBinaryValue(stream, kSlangDependencyCacheSchema);
    WriteBinaryValue(stream, source_key.low);
    WriteBinaryValue(stream, source_key.high);
    const auto dependency_count = static_cast<uint64_t>(dependency_paths.size());
    WriteBinaryValue(stream, dependency_count);
    for (const auto& dependency_path : dependency_paths) {
      const auto serialized_path = dependency_path.lexically_normal().generic_string();
      if (serialized_path.size() > kMaxSlangDependencyPathBytes) {
        stream.close();
        std::filesystem::remove(temporary_path);
        return false;
      }
      WriteBinaryString(stream, serialized_path);
    }
    stream.flush();
    if (!stream) {
      stream.close();
      std::filesystem::remove(temporary_path);
      return false;
    }
    stream.close();
    std::error_code error;
    std::filesystem::rename(temporary_path, cache_path, error);
    if (error) {
      if (std::filesystem::exists(cache_path)) {
        std::filesystem::remove(temporary_path);
        return true;
      }
      std::filesystem::remove(temporary_path);
      return false;
    }
    return true;
  } catch (...) {
    std::error_code error;
    if (!temporary_path.empty())
      std::filesystem::remove(temporary_path, error);
    return false;
  }
}

template <typename CompileFunc>
void CompileShaderWithCache(const ShaderCompileRequest& request, const ShaderCacheKey& key, ShaderCompileResult& result,
                            CompileFunc&& compile_func) {
  std::shared_ptr<ShaderCompileEntry> entry;
  bool owns_request = false;
  {
    const std::lock_guard lock(shader_compile_entries_mutex);
    const auto [search, inserted] = shader_compile_entries.try_emplace(key, std::make_shared<ShaderCompileEntry>());
    entry = search->second;
    owns_request = inserted;
  }
  if (!owns_request) {
    std::unique_lock lock(entry->mutex);
    if (!entry->done) {
      coalesced_wait_count.fetch_add(1);
      entry->completed.wait(lock, [&]() {
        return entry->done;
      });
    } else {
      memory_hit_count.fetch_add(1);
    }
    result.success = entry->success;
    result.binaries = entry->binaries;
    return;
  }

  const auto cache_path = GetShaderBinaryDirectory() / (ShaderCacheKeyString(key) + ".spvbin");
  auto load_result = LoadShaderCacheEntry(cache_path, key, request, result.binaries);
  bool success = load_result == ShaderCacheLoadResult::Valid;
  if (success) {
    disk_hit_count.fetch_add(1);
  } else {
    disk_miss_count.fetch_add(1);
    if (load_result == ShaderCacheLoadResult::Corrupt) {
      corrupt_entry_count.fetch_add(1);
      std::error_code error;
      std::filesystem::remove(cache_path, error);
    }
    compilation_count.fetch_add(1);
    success = compile_func(result.binaries);
    if (success) {
      PublishShaderCacheEntry(cache_path, key, request, result.binaries);
    } else {
      failure_count.fetch_add(1);
    }
  }

  {
    const std::lock_guard lock(entry->mutex);
    entry->success = success;
    entry->binaries = result.binaries;
    entry->done = true;
  }
  entry->completed.notify_all();
  if (!success) {
    const std::lock_guard lock(shader_compile_entries_mutex);
    if (const auto search = shader_compile_entries.find(key);
        search != shader_compile_entries.end() && search->second == entry) {
      shader_compile_entries.erase(search);
    }
  }
  result.success = success;
}

bool TryLoadShaderCacheEntryOnly(const ShaderCompileRequest& request, const ShaderCacheKey& key,
                                 ShaderCompileResult& result) {
  std::shared_ptr<ShaderCompileEntry> entry;
  {
    const std::lock_guard lock(shader_compile_entries_mutex);
    if (const auto search = shader_compile_entries.find(key); search != shader_compile_entries.end()) {
      entry = search->second;
    }
  }
  if (entry) {
    std::unique_lock lock(entry->mutex);
    if (!entry->done) {
      coalesced_wait_count.fetch_add(1);
      entry->completed.wait(lock, [&]() {
        return entry->done;
      });
    } else {
      memory_hit_count.fetch_add(1);
    }
    result.success = entry->success;
    result.binaries = entry->binaries;
    if (entry->success) {
      return true;
    }
    {
      const std::lock_guard lock(shader_compile_entries_mutex);
      if (const auto search = shader_compile_entries.find(key);
          search != shader_compile_entries.end() && search->second == entry) {
        shader_compile_entries.erase(search);
      }
    }
    result.binaries.clear();
    return false;
  }

  const auto cache_path = GetShaderBinaryDirectory() / (ShaderCacheKeyString(key) + ".spvbin");
  auto load_result = LoadShaderCacheEntry(cache_path, key, request, result.binaries);
  if (load_result == ShaderCacheLoadResult::Valid) {
    disk_hit_count.fetch_add(1);
    result.success = true;
    auto disk_entry = std::make_shared<ShaderCompileEntry>();
    {
      const std::lock_guard lock(disk_entry->mutex);
      disk_entry->success = true;
      disk_entry->binaries = result.binaries;
      disk_entry->done = true;
    }
    {
      const std::lock_guard lock(shader_compile_entries_mutex);
      shader_compile_entries.try_emplace(key, std::move(disk_entry));
    }
    return true;
  }
  if (load_result == ShaderCacheLoadResult::Corrupt) {
    corrupt_entry_count.fetch_add(1);
    std::error_code error;
    std::filesystem::remove(cache_path, error);
  }
  result.binaries.clear();
  return false;
}

std::string BlobToString(slang::IBlob* blob) {
  if (!blob || !blob->getBufferPointer() || blob->getBufferSize() == 0) {
    return {};
  }
  return {static_cast<const char*>(blob->getBufferPointer()), blob->getBufferSize()};
}

std::string MakeSlangModuleName(const std::filesystem::path& path) {
  std::string name = path.empty() ? "evoengine_shader" : path.stem().string();
  if (name.empty()) {
    name = "evoengine_shader";
  }
  for (auto& c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '_') {
      c = '_';
    }
  }
  if (!std::isalpha(static_cast<unsigned char>(name.front())) && name.front() != '_') {
    name.insert(name.begin(), '_');
  }
  return name;
}

bool TryGetSlangStage(const ShaderType shader_type, SlangStage& stage) {
  switch (shader_type) {
    case ShaderType::Task:
      stage = SLANG_STAGE_AMPLIFICATION;
      return true;
    case ShaderType::Mesh:
      stage = SLANG_STAGE_MESH;
      return true;
    case ShaderType::Vertex:
      stage = SLANG_STAGE_VERTEX;
      return true;
    case ShaderType::TessellationControl:
      stage = SLANG_STAGE_HULL;
      return true;
    case ShaderType::TessellationEvaluation:
      stage = SLANG_STAGE_DOMAIN;
      return true;
    case ShaderType::Geometry:
      stage = SLANG_STAGE_GEOMETRY;
      return true;
    case ShaderType::Fragment:
      stage = SLANG_STAGE_FRAGMENT;
      return true;
    case ShaderType::Compute:
      stage = SLANG_STAGE_COMPUTE;
      return true;
    case ShaderType::RayGen:
      stage = SLANG_STAGE_RAY_GENERATION;
      return true;
    case ShaderType::Miss:
      stage = SLANG_STAGE_MISS;
      return true;
    case ShaderType::AnyHit:
      stage = SLANG_STAGE_ANY_HIT;
      return true;
    case ShaderType::ClosestHit:
      stage = SLANG_STAGE_CLOSEST_HIT;
      return true;
    case ShaderType::Intersection:
      stage = SLANG_STAGE_INTERSECTION;
      return true;
    case ShaderType::Callable:
      stage = SLANG_STAGE_CALLABLE;
      return true;
    case ShaderType::Unknown:
      return false;
  }
  return false;
}

const char* CompatibilityStageExtension(const ShaderType shader_type) {
  switch (shader_type) {
    case ShaderType::Task:
      return ".task";
    case ShaderType::Mesh:
      return ".mesh";
    case ShaderType::Vertex:
      return ".vert";
    case ShaderType::TessellationControl:
      return ".tesc";
    case ShaderType::TessellationEvaluation:
      return ".tese";
    case ShaderType::Geometry:
      return ".geom";
    case ShaderType::Fragment:
      return ".frag";
    case ShaderType::Compute:
      return ".comp";
    case ShaderType::RayGen:
      return ".rgen";
    case ShaderType::Miss:
      return ".rmiss";
    case ShaderType::AnyHit:
      return ".rahit";
    case ShaderType::ClosestHit:
      return ".rchit";
    case ShaderType::Intersection:
      return ".rint";
    case ShaderType::Callable:
      return ".rcall";
    case ShaderType::Unknown:
      return ".slang";
  }
  return ".slang";
}

std::string MakeSlangSourceDisplayPath(const ShaderCompileRequest& request, const std::string& module_name) {
  auto display_path = request.path.empty() ? std::filesystem::path(module_name) : request.path;
  if (request.target.slang_input_language == SLANG_SOURCE_LANGUAGE_GLSL) {
    display_path.replace_extension(CompatibilityStageExtension(request.shader_type));
  }
  return display_path.string();
}

slang::IGlobalSession* GetSlangGlobalSession(std::string& diagnostics) {
  static Slang::ComPtr<slang::IGlobalSession> global_session;
  const std::lock_guard slang_lock(slang_mutex);
  if (!global_session) {
    SlangGlobalSessionDesc session_desc = {};
    session_desc.enableGLSL = true;
    if (SLANG_FAILED(slang::createGlobalSession(&session_desc, global_session.writeRef()))) {
      diagnostics += "Failed to create Slang global session.\n";
      return nullptr;
    }
  }
  return global_session.get();
}

std::string MakeSlangCompilerVersionString(slang::IGlobalSession& global_session) {
  if (const auto* build_tag = global_session.getBuildTagString(); build_tag && build_tag[0] != '\0') {
    return build_tag;
  }
  return "unknown";
}

std::string MakeSlangCompilerDiagnosticHeader(slang::IGlobalSession& global_session,
                                              const ShaderCompileRequest& request) {
  std::ostringstream stream;
  stream << "Compiler backend: Slang";
  stream << " (" << MakeSlangCompilerVersionString(global_session) << ")";
  stream << ", Vulkan " << (kVulkanTarget / 10) << "." << (kVulkanTarget % 10) << ", SPIR-V " << (kSpirvTarget / 10)
         << "." << (kSpirvTarget % 10) << ", matrix layout: row-major, scalar layout: on, Slang input dialect: "
         << SlangInputDialectName(request.target.slang_input_language)
         << ", shader invocation reorder EXT: " << (RequiresShaderInvocationReorderCapability(request) ? "on" : "off")
         << "\n";
  return stream.str();
}

std::vector<std::filesystem::path> CollectSlangDependencyPaths(slang::IModule& module) {
  std::vector<std::filesystem::path> dependency_paths;
  const auto dependency_count = module.getDependencyFileCount();
  if (dependency_count > 0) {
    dependency_paths.reserve(static_cast<size_t>(dependency_count));
  }
  for (SlangInt32 dependency_index = 0; dependency_index < dependency_count; ++dependency_index) {
    if (const auto* dependency_path = module.getDependencyFilePath(dependency_index);
        dependency_path && dependency_path[0] != '\0') {
      dependency_paths.emplace_back(dependency_path);
    }
  }
  std::sort(dependency_paths.begin(), dependency_paths.end(), [](const auto& left, const auto& right) {
    return left.lexically_normal().generic_string() < right.lexically_normal().generic_string();
  });
  dependency_paths.erase(std::unique(dependency_paths.begin(), dependency_paths.end(),
                                     [](const auto& left, const auto& right) {
                                       return left.lexically_normal().generic_string() ==
                                              right.lexically_normal().generic_string();
                                     }),
                         dependency_paths.end());
  return dependency_paths;
}

std::string MakeSlangDependencySignature(const std::vector<std::filesystem::path>& dependency_paths) {
  std::ostringstream stream;
  for (const auto& dependency_path : dependency_paths) {
    std::error_code error;
    auto normalized_path = std::filesystem::weakly_canonical(dependency_path, error);
    if (error) {
      normalized_path = dependency_path.lexically_normal();
    }
    stream << normalized_path.generic_string() << '\0';
    error.clear();
    if (std::filesystem::is_regular_file(normalized_path, error) && !error) {
      error.clear();
      stream << std::filesystem::file_size(normalized_path, error) << '\0';
      if (!error) {
        stream << FileUtils::LoadFileAsString(normalized_path) << '\0';
      }
    } else {
      stream << "<missing>" << '\0';
    }
  }
  return stream.str();
}

std::string MakeSlangDependencySignature(slang::IModule& module) {
  return MakeSlangDependencySignature(CollectSlangDependencyPaths(module));
}

bool CreateSlangSession(const ShaderCompileRequest& request, slang::IGlobalSession& global_session,
                        Slang::ComPtr<slang::ISession>& session, std::string& diagnostics) {
  std::vector<slang::CompilerOptionEntry> session_options;
  session_options.push_back(
      {slang::CompilerOptionName::Language,
       {slang::CompilerOptionValueKind::Int, static_cast<int32_t>(request.target.slang_input_language)}});
  if (RequiresShaderInvocationReorderCapability(request)) {
    session_options.push_back({slang::CompilerOptionName::Capability,
                               {slang::CompilerOptionValueKind::String, 0, 0, "spvShaderInvocationReorderEXT"}});
  }
  slang::CompilerOptionEntry target_options[] = {
      {slang::CompilerOptionName::EmitSpirvDirectly, {slang::CompilerOptionValueKind::Int, 1}},
      {slang::CompilerOptionName::MatrixLayoutRow, {slang::CompilerOptionValueKind::Int, 1}},
      {slang::CompilerOptionName::GLSLForceScalarLayout, {slang::CompilerOptionValueKind::Int, 1}},
  };
  slang::TargetDesc target_desc;
  target_desc.format = SLANG_SPIRV;
  target_desc.profile = global_session.findProfile("spirv_1_4");
  target_desc.flags = SLANG_TARGET_FLAG_GENERATE_SPIRV_DIRECTLY;
  target_desc.compilerOptionEntries = target_options;
  target_desc.compilerOptionEntryCount = static_cast<uint32_t>(sizeof(target_options) / sizeof(target_options[0]));

  std::vector<std::string> include_path_strings;
  std::vector<const char*> include_paths;
  include_path_strings.reserve(request.include_paths.size());
  include_paths.reserve(request.include_paths.size());
  for (const auto& include_path : request.include_paths) {
    include_path_strings.emplace_back(include_path.string());
    include_paths.emplace_back(include_path_strings.back().c_str());
  }

  slang::SessionDesc session_desc;
  session_desc.targets = &target_desc;
  session_desc.targetCount = 1;
  session_desc.defaultMatrixLayoutMode = SLANG_MATRIX_LAYOUT_ROW_MAJOR;
  session_desc.allowGLSLSyntax = true;
  session_desc.searchPaths = include_paths.empty() ? nullptr : include_paths.data();
  session_desc.searchPathCount = static_cast<SlangInt>(include_paths.size());
  session_desc.compilerOptionEntries = session_options.data();
  session_desc.compilerOptionEntryCount = static_cast<uint32_t>(session_options.size());

  if (SLANG_FAILED(global_session.createSession(session_desc, session.writeRef()))) {
    diagnostics += "Failed to create Slang compile session.\n";
    return false;
  }
  return true;
}

Slang::ComPtr<slang::IModule> LoadSlangModule(const ShaderCompileRequest& request, slang::ISession& session,
                                              std::string& diagnostics, std::string& compiled_source) {
  Slang::ComPtr<slang::IBlob> diagnostic_blob;
  const std::string module_name = MakeSlangModuleName(request.path);
  const std::string display_path = MakeSlangSourceDisplayPath(request, module_name);
  compiled_source = request.global_defines + request.source;
  Slang::ComPtr<slang::IModule> module(session.loadModuleFromSourceString(
      module_name.c_str(), display_path.c_str(), compiled_source.c_str(), diagnostic_blob.writeRef()));
  diagnostics += BlobToString(diagnostic_blob);
  return module;
}

bool EmitSlangSpirv(const ShaderCompileRequest& request, slang::ISession& session, slang::IModule& module,
                    const SlangStage stage, std::string& diagnostics, std::vector<uint32_t>& binaries) {
  binaries.clear();
  Slang::ComPtr<slang::IBlob> diagnostic_blob;
  Slang::ComPtr<slang::IEntryPoint> entry_point;
  const char* entry_point_name = request.entry_point.empty() ? kDefaultShaderEntryPoint : request.entry_point.c_str();
  if (SLANG_FAILED(
          module.findAndCheckEntryPoint(entry_point_name, stage, entry_point.writeRef(), diagnostic_blob.writeRef()))) {
    diagnostics += BlobToString(diagnostic_blob);
    EVOENGINE_ERROR("Failed to find Slang entry point: " + request.path.string() + "\n" + diagnostics)
    return false;
  }

  slang::IComponentType* components[] = {&module, entry_point.get()};
  diagnostic_blob.setNull();
  Slang::ComPtr<slang::IComponentType> composed_program;
  if (SLANG_FAILED(session.createCompositeComponentType(components, sizeof(components) / sizeof(components[0]),
                                                        composed_program.writeRef(), diagnostic_blob.writeRef()))) {
    diagnostics += BlobToString(diagnostic_blob);
    EVOENGINE_ERROR("Failed to compose Slang program: " + request.path.string() + "\n" + diagnostics)
    return false;
  }

  diagnostic_blob.setNull();
  Slang::ComPtr<slang::IComponentType> linked_program;
  if (SLANG_FAILED(composed_program->link(linked_program.writeRef(), diagnostic_blob.writeRef()))) {
    diagnostics += BlobToString(diagnostic_blob);
    EVOENGINE_ERROR("Failed to link Slang program: " + request.path.string() + "\n" + diagnostics)
    return false;
  }

  diagnostic_blob.setNull();
  Slang::ComPtr<slang::IBlob> code;
  if (SLANG_FAILED(linked_program->getEntryPointCode(0, 0, code.writeRef(), diagnostic_blob.writeRef()))) {
    diagnostics += BlobToString(diagnostic_blob);
    EVOENGINE_ERROR("Failed to emit Slang SPIR-V: " + request.path.string() + "\n" + diagnostics)
    return false;
  }
  diagnostics += BlobToString(diagnostic_blob);
  if (!code || code->getBufferSize() == 0 || code->getBufferSize() % sizeof(uint32_t) != 0) {
    diagnostics += "Slang produced an invalid SPIR-V blob size.\n";
    return false;
  }
  const auto* words = static_cast<const uint32_t*>(code->getBufferPointer());
  const auto word_count = code->getBufferSize() / sizeof(uint32_t);
  binaries.assign(words, words + word_count);
  if (binaries.empty() || binaries.front() != kSpirvMagic) {
    diagnostics += "Slang output did not start with the SPIR-V magic number.\n";
    binaries.clear();
    return false;
  }
  return true;
}

VkShaderStageFlags SlangStageToVkStageFlags(const SlangStage stage) {
  switch (stage) {
    case SLANG_STAGE_VERTEX:
      return VK_SHADER_STAGE_VERTEX_BIT;
    case SLANG_STAGE_HULL:
      return VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT;
    case SLANG_STAGE_DOMAIN:
      return VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT;
    case SLANG_STAGE_GEOMETRY:
      return VK_SHADER_STAGE_GEOMETRY_BIT;
    case SLANG_STAGE_FRAGMENT:
      return VK_SHADER_STAGE_FRAGMENT_BIT;
    case SLANG_STAGE_COMPUTE:
      return VK_SHADER_STAGE_COMPUTE_BIT;
    case SLANG_STAGE_RAY_GENERATION:
      return VK_SHADER_STAGE_RAYGEN_BIT_KHR;
    case SLANG_STAGE_INTERSECTION:
      return VK_SHADER_STAGE_INTERSECTION_BIT_KHR;
    case SLANG_STAGE_ANY_HIT:
      return VK_SHADER_STAGE_ANY_HIT_BIT_KHR;
    case SLANG_STAGE_CLOSEST_HIT:
      return VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
    case SLANG_STAGE_MISS:
      return VK_SHADER_STAGE_MISS_BIT_KHR;
    case SLANG_STAGE_CALLABLE:
      return VK_SHADER_STAGE_CALLABLE_BIT_KHR;
    case SLANG_STAGE_AMPLIFICATION:
      return VK_SHADER_STAGE_TASK_BIT_EXT;
    case SLANG_STAGE_MESH:
      return VK_SHADER_STAGE_MESH_BIT_EXT;
    default:
      return 0;
  }
}

VkDescriptorType SlangBindingTypeToVkDescriptorType(const slang::BindingType binding_type,
                                                    const slang::ParameterCategory category) {
  const auto value = static_cast<uint32_t>(binding_type);
  const auto base = static_cast<slang::BindingType>(value & static_cast<uint32_t>(slang::BindingType::BaseMask));
  const bool mutable_binding = (value & static_cast<uint32_t>(slang::BindingType::MutableFlag)) != 0;
  switch (base) {
    case slang::BindingType::Sampler:
      return VK_DESCRIPTOR_TYPE_SAMPLER;
    case slang::BindingType::Texture:
      return mutable_binding || category == slang::ParameterCategory::UnorderedAccess
                 ? VK_DESCRIPTOR_TYPE_STORAGE_IMAGE
                 : VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;
    case slang::BindingType::CombinedTextureSampler:
      return VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    case slang::BindingType::ConstantBuffer:
    case slang::BindingType::ParameterBlock:
      return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    case slang::BindingType::TypedBuffer:
    case slang::BindingType::RawBuffer:
      return VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    case slang::BindingType::InputRenderTarget:
      return VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT;
    case slang::BindingType::RayTracingAccelerationStructure:
      return VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
    default:
      return VK_DESCRIPTOR_TYPE_MAX_ENUM;
  }
}

std::string VkDescriptorTypeName(const VkDescriptorType type) {
  switch (type) {
    case VK_DESCRIPTOR_TYPE_SAMPLER:
      return "sampler";
    case VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER:
      return "combined image sampler";
    case VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE:
      return "sampled image";
    case VK_DESCRIPTOR_TYPE_STORAGE_IMAGE:
      return "storage image";
    case VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER:
      return "uniform texel buffer";
    case VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER:
      return "storage texel buffer";
    case VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER:
      return "uniform buffer";
    case VK_DESCRIPTOR_TYPE_STORAGE_BUFFER:
      return "storage buffer";
    case VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT:
      return "input attachment";
    case VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR:
      return "acceleration structure";
    default:
      return "unknown";
  }
}

std::optional<uint32_t> TryConvertSlangSize(const size_t value) {
  if (value == SLANG_UNKNOWN_SIZE || value == SLANG_UNBOUNDED_SIZE ||
      value > static_cast<size_t>(std::numeric_limits<uint32_t>::max())) {
    return {};
  }
  return static_cast<uint32_t>(value);
}

std::optional<uint32_t> TryConvertSlangInt(const SlangInt value) {
  if (value < 0 || static_cast<uint64_t>(value) > std::numeric_limits<uint32_t>::max()) {
    return {};
  }
  return static_cast<uint32_t>(value);
}

std::string SlangVariableName(slang::VariableLayoutReflection* variable_layout) {
  const auto* name = variable_layout ? variable_layout->getName() : nullptr;
  return name ? name : "";
}

bool IsSystemValueSemantic(const std::string& semantic_name) {
  return semantic_name.size() > 3 && std::toupper(static_cast<unsigned char>(semantic_name[0])) == 'S' &&
         std::toupper(static_cast<unsigned char>(semantic_name[1])) == 'V' && semantic_name[2] == '_';
}

void CollectSlangDescriptorBindings(slang::TypeLayoutReflection* type_layout, const VkShaderStageFlags stage_flags,
                                    ShaderReflectionInfo& reflection, std::string& diagnostics) {
  if (!type_layout) {
    return;
  }
  const SlangInt set_count = type_layout->getDescriptorSetCount();
  for (SlangInt set_index = 0; set_index < set_count; ++set_index) {
    const auto set = TryConvertSlangInt(type_layout->getDescriptorSetSpaceOffset(set_index));
    if (!set) {
      diagnostics += "Slang reflection reported an unsupported descriptor set index.\n";
      continue;
    }
    const SlangInt range_count = type_layout->getDescriptorSetDescriptorRangeCount(set_index);
    for (SlangInt range_index = 0; range_index < range_count; ++range_index) {
      const auto binding =
          TryConvertSlangInt(type_layout->getDescriptorSetDescriptorRangeIndexOffset(set_index, range_index));
      const auto descriptor_count =
          TryConvertSlangInt(type_layout->getDescriptorSetDescriptorRangeDescriptorCount(set_index, range_index));
      if (!binding || !descriptor_count) {
        diagnostics += "Slang reflection reported an unsupported descriptor binding or count.\n";
        continue;
      }
      const auto binding_type = type_layout->getDescriptorSetDescriptorRangeType(set_index, range_index);
      const auto category = type_layout->getDescriptorSetDescriptorRangeCategory(set_index, range_index);
      const VkDescriptorType descriptor_type = SlangBindingTypeToVkDescriptorType(binding_type, category);
      if (descriptor_type == VK_DESCRIPTOR_TYPE_MAX_ENUM) {
        continue;
      }
      ShaderReflectionDescriptorBinding reflected;
      reflected.set = *set;
      reflected.binding = *binding;
      reflected.descriptor_count = *descriptor_count;
      reflected.descriptor_type = descriptor_type;
      reflected.stage_flags = stage_flags;
      reflection.descriptor_bindings.emplace_back(std::move(reflected));
    }
  }
}

void CollectSlangPushConstants(slang::VariableLayoutReflection* variable_layout, const VkShaderStageFlags stage_flags,
                               ShaderReflectionInfo& reflection, std::string& diagnostics) {
  if (!variable_layout) {
    return;
  }
  auto* type_layout = variable_layout->getTypeLayout();
  if (!type_layout) {
    return;
  }
  for (unsigned int category_index = 0; category_index < variable_layout->getCategoryCount(); ++category_index) {
    const auto category = variable_layout->getCategoryByIndex(category_index);
    if (category != slang::ParameterCategory::PushConstantBuffer) {
      continue;
    }
    const auto offset = TryConvertSlangSize(variable_layout->getOffset(category));
    auto* payload_layout = type_layout;
    if (type_layout->getKind() == slang::TypeReflection::Kind::ConstantBuffer && type_layout->getElementTypeLayout()) {
      payload_layout = type_layout->getElementTypeLayout();
    }
    auto size = TryConvertSlangSize(payload_layout->getSize(slang::ParameterCategory::Uniform));
    if ((!size || *size == 0) && payload_layout != type_layout) {
      size = TryConvertSlangSize(payload_layout->getSize(category));
    }
    if (!size || *size == 0) {
      size = TryConvertSlangSize(type_layout->getSize(slang::ParameterCategory::Uniform));
    }
    if (!size || *size == 0) {
      size = TryConvertSlangSize(type_layout->getSize(category));
    }
    if (!offset || !size || *size == 0) {
      diagnostics += "Slang reflection reported an unsupported push-constant range.\n";
      continue;
    }
    ShaderReflectionPushConstantRange reflected;
    reflected.name = SlangVariableName(variable_layout);
    reflected.offset = *offset;
    reflected.size = *size;
    reflected.stage_flags = stage_flags;
    reflection.push_constant_ranges.emplace_back(std::move(reflected));
  }
  for (unsigned int field_index = 0; field_index < type_layout->getFieldCount(); ++field_index) {
    CollectSlangPushConstants(type_layout->getFieldByIndex(field_index), stage_flags, reflection, diagnostics);
  }
}

void CollectSlangStageIo(slang::VariableLayoutReflection* variable_layout, const slang::ParameterCategory category,
                         std::vector<ShaderReflectionStageIo>& stage_io) {
  if (!variable_layout) {
    return;
  }
  auto* type_layout = variable_layout->getTypeLayout();
  if (!type_layout) {
    return;
  }
  if (type_layout->getFieldCount() == 0) {
    for (unsigned int category_index = 0; category_index < variable_layout->getCategoryCount(); ++category_index) {
      if (variable_layout->getCategoryByIndex(category_index) != category) {
        continue;
      }
      const auto location = TryConvertSlangSize(variable_layout->getOffset(category));
      if (!location) {
        continue;
      }
      ShaderReflectionStageIo reflected;
      reflected.name = SlangVariableName(variable_layout);
      if (const auto* semantic_name = variable_layout->getSemanticName()) {
        reflected.semantic_name = semantic_name;
      }
      if (IsSystemValueSemantic(reflected.semantic_name)) {
        continue;
      }
      reflected.semantic_index = static_cast<uint32_t>(variable_layout->getSemanticIndex());
      reflected.location = *location;
      stage_io.emplace_back(std::move(reflected));
    }
  }
  for (unsigned int field_index = 0; field_index < type_layout->getFieldCount(); ++field_index) {
    CollectSlangStageIo(type_layout->getFieldByIndex(field_index), category, stage_io);
  }
}

bool ReflectSlangRequest(const ShaderCompileRequest& request, ShaderReflectionInfo& reflection,
                         std::string& diagnostics) {
  reflection = {};
  diagnostics.clear();
  if (!IsSupportedSlangInputLanguage(request.target.slang_input_language)) {
    diagnostics = "Unsupported Slang reflection request.";
    return false;
  }
  SlangStage stage = SLANG_STAGE_NONE;
  if (!TryGetSlangStage(request.shader_type, stage)) {
    diagnostics = "Unknown Slang shader stage.";
    return false;
  }
  slang::IGlobalSession* global_session = GetSlangGlobalSession(diagnostics);
  if (!global_session) {
    return false;
  }
  diagnostics += MakeSlangCompilerDiagnosticHeader(*global_session, request);

  const std::lock_guard slang_lock(slang_mutex);
  Slang::ComPtr<slang::ISession> session;
  if (!CreateSlangSession(request, *global_session, session, diagnostics)) {
    return false;
  }

  std::string compiled_source;
  Slang::ComPtr<slang::IModule> module = LoadSlangModule(request, *session, diagnostics, compiled_source);
  if (!module) {
    EVOENGINE_ERROR("Failed to load Slang module for reflection: " + request.path.string() + "\n" + diagnostics)
    return false;
  }

  Slang::ComPtr<slang::IBlob> diagnostic_blob;
  Slang::ComPtr<slang::IEntryPoint> entry_point;
  const char* entry_point_name = request.entry_point.empty() ? kDefaultShaderEntryPoint : request.entry_point.c_str();
  if (SLANG_FAILED(module->findAndCheckEntryPoint(entry_point_name, stage, entry_point.writeRef(),
                                                  diagnostic_blob.writeRef()))) {
    diagnostics += BlobToString(diagnostic_blob);
    EVOENGINE_ERROR("Failed to find Slang entry point for reflection: " + request.path.string() + "\n" + diagnostics)
    return false;
  }

  slang::IComponentType* components[] = {module.get(), entry_point.get()};
  diagnostic_blob.setNull();
  Slang::ComPtr<slang::IComponentType> composed_program;
  if (SLANG_FAILED(session->createCompositeComponentType(components, sizeof(components) / sizeof(components[0]),
                                                         composed_program.writeRef(), diagnostic_blob.writeRef()))) {
    diagnostics += BlobToString(diagnostic_blob);
    EVOENGINE_ERROR("Failed to compose Slang program for reflection: " + request.path.string() + "\n" + diagnostics)
    return false;
  }

  diagnostic_blob.setNull();
  Slang::ComPtr<slang::IComponentType> linked_program;
  if (SLANG_FAILED(composed_program->link(linked_program.writeRef(), diagnostic_blob.writeRef()))) {
    diagnostics += BlobToString(diagnostic_blob);
    EVOENGINE_ERROR("Failed to link Slang program for reflection: " + request.path.string() + "\n" + diagnostics)
    return false;
  }

  diagnostic_blob.setNull();
  slang::ProgramLayout* program_layout = linked_program->getLayout(0, diagnostic_blob.writeRef());
  diagnostics += BlobToString(diagnostic_blob);
  if (!program_layout) {
    diagnostics += "Slang did not provide reflection layout.\n";
    return false;
  }
  auto* entry_point_layout = program_layout->findEntryPointByName(entry_point_name);
  if (!entry_point_layout && program_layout->getEntryPointCount() > 0) {
    entry_point_layout = program_layout->getEntryPointByIndex(0);
  }
  if (!entry_point_layout) {
    diagnostics += "Slang reflection did not provide an entry-point layout.\n";
    return false;
  }

  reflection.shader_type = request.shader_type;
  reflection.entry_point = entry_point_name;
  const VkShaderStageFlags stage_flags = SlangStageToVkStageFlags(entry_point_layout->getStage());
  if (request.shader_type == ShaderType::Compute) {
    SlangUInt group_size[3] = {};
    entry_point_layout->getComputeThreadGroupSize(3, group_size);
    reflection.compute_thread_group_size = {static_cast<uint32_t>(group_size[0]), static_cast<uint32_t>(group_size[1]),
                                            static_cast<uint32_t>(group_size[2])};
  }

  if (auto* global_params = program_layout->getGlobalParamsVarLayout()) {
    CollectSlangDescriptorBindings(global_params->getTypeLayout(), stage_flags, reflection, diagnostics);
    CollectSlangPushConstants(global_params, stage_flags, reflection, diagnostics);
  }
  CollectSlangStageIo(entry_point_layout->getVarLayout(), slang::ParameterCategory::VaryingInput,
                      reflection.stage_inputs);
  CollectSlangStageIo(entry_point_layout->getResultVarLayout(), slang::ParameterCategory::VaryingOutput,
                      reflection.stage_outputs);

  std::sort(reflection.descriptor_bindings.begin(), reflection.descriptor_bindings.end(),
            [](const auto& left, const auto& right) {
              return std::tie(left.set, left.binding, left.name) < std::tie(right.set, right.binding, right.name);
            });
  std::sort(reflection.push_constant_ranges.begin(), reflection.push_constant_ranges.end(),
            [](const auto& left, const auto& right) {
              return std::tie(left.offset, left.size, left.name) < std::tie(right.offset, right.size, right.name);
            });
  std::sort(reflection.stage_inputs.begin(), reflection.stage_inputs.end(), [](const auto& left, const auto& right) {
    return std::tie(left.location, left.name) < std::tie(right.location, right.name);
  });
  std::sort(reflection.stage_outputs.begin(), reflection.stage_outputs.end(), [](const auto& left, const auto& right) {
    return std::tie(left.location, left.name) < std::tie(right.location, right.name);
  });
  return true;
}

bool PushConstantRangeCovers(const VkPushConstantRange& host_range,
                             const ShaderReflectionPushConstantRange& reflected) {
  const uint64_t reflected_begin = reflected.offset;
  const uint64_t reflected_end = static_cast<uint64_t>(reflected.offset) + reflected.size;
  const uint64_t host_begin = host_range.offset;
  const uint64_t host_end = static_cast<uint64_t>(host_range.offset) + host_range.size;
  return host_begin <= reflected_begin && host_end >= reflected_end &&
         (host_range.stageFlags & reflected.stage_flags) == reflected.stage_flags;
}

bool StageIoMatches(const ShaderReflectionStageIo& expected, const ShaderReflectionStageIo& reflected) {
  if (expected.location != reflected.location || expected.semantic_index != reflected.semantic_index) {
    return false;
  }
  return expected.semantic_name.empty() || expected.semantic_name == reflected.semantic_name;
}

void ValidateStageIoList(const std::optional<std::vector<ShaderReflectionStageIo>>& expected,
                         const std::vector<ShaderReflectionStageIo>& reflected, const char* label,
                         std::ostringstream& diagnostics) {
  if (!expected) {
    return;
  }
  if (expected->size() != reflected.size()) {
    diagnostics << "Slang " << label << " count mismatch: expected " << expected->size() << ", reflected "
                << reflected.size() << ".\n";
    return;
  }
  std::vector<ShaderReflectionStageIo> sorted_expected = *expected;
  std::sort(sorted_expected.begin(), sorted_expected.end(), [](const auto& left, const auto& right) {
    return std::tie(left.location, left.semantic_name, left.semantic_index) <
           std::tie(right.location, right.semantic_name, right.semantic_index);
  });
  for (size_t i = 0; i < sorted_expected.size(); ++i) {
    if (!StageIoMatches(sorted_expected[i], reflected[i])) {
      diagnostics << "Slang " << label << " mismatch at index " << i << ": expected location "
                  << sorted_expected[i].location << " semantic " << sorted_expected[i].semantic_name
                  << sorted_expected[i].semantic_index << ", reflected location " << reflected[i].location
                  << " semantic " << reflected[i].semantic_name << reflected[i].semantic_index << ".\n";
    }
  }
}

ShaderCacheKey MakeSlangSourceCacheKey(const ShaderCompileRequest& request, const std::string& compiler_version,
                                       const std::string& source) {
  return MakeShaderCacheKey(request, compiler_version, source);
}

ShaderCacheKey MakeSlangBinaryCacheKey(const ShaderCompileRequest& request, const std::string& compiler_version,
                                       const std::string& source,
                                       const std::vector<std::filesystem::path>& dependency_paths) {
  return MakeShaderCacheKey(request, compiler_version, source, MakeSlangDependencySignature(dependency_paths));
}

ShaderCompileResult CompileSlang(const ShaderCompileRequest& request) {
  ShaderCompileResult result;
  if (!IsSupportedSlangInputLanguage(request.target.slang_input_language)) {
    result.diagnostics = "Unsupported Slang compile request.";
    failure_count.fetch_add(1);
    return result;
  }
  SlangStage stage = SLANG_STAGE_NONE;
  if (!TryGetSlangStage(request.shader_type, stage)) {
    result.diagnostics = "Unknown Slang shader stage.";
    failure_count.fetch_add(1);
    return result;
  }

  slang::IGlobalSession* global_session = GetSlangGlobalSession(result.diagnostics);
  if (!global_session) {
    failure_count.fetch_add(1);
    return result;
  }
  result.diagnostics += MakeSlangCompilerDiagnosticHeader(*global_session, request);
  const auto compiler_version = MakeSlangCompilerVersionString(*global_session);
  const auto source = request.global_defines + request.source;
  const auto source_key = MakeSlangSourceCacheKey(request, compiler_version, source);

  std::vector<std::filesystem::path> dependency_paths;
  if (LoadSlangDependencyCacheEntry(source_key, dependency_paths)) {
    const auto key = MakeSlangBinaryCacheKey(request, compiler_version, source, dependency_paths);
    if (TryLoadShaderCacheEntryOnly(request, key, result)) {
      return result;
    }
  }

  const std::lock_guard slang_lock(slang_mutex);
  Slang::ComPtr<slang::ISession> session;
  if (!CreateSlangSession(request, *global_session, session, result.diagnostics)) {
    failure_count.fetch_add(1);
    return result;
  }

  std::string frontend_source;
  slang_frontend_count.fetch_add(1);
  Slang::ComPtr<slang::IModule> module = LoadSlangModule(request, *session, result.diagnostics, frontend_source);
  if (!module) {
    EVOENGINE_ERROR("Failed to load Slang module: " + request.path.string() + "\n" + result.diagnostics)
    failure_count.fetch_add(1);
    return result;
  }

  dependency_paths = CollectSlangDependencyPaths(*module);
  const auto key = MakeSlangBinaryCacheKey(request, compiler_version, source, dependency_paths);
  CompileShaderWithCache(request, key, result, [&](std::vector<uint32_t>& binaries) {
    return EmitSlangSpirv(request, *session, *module, stage, result.diagnostics, binaries);
  });
  if (result.success) {
    PublishSlangDependencyCacheEntry(source_key, dependency_paths);
  }
  return result;
}

ShaderCompileResult CompileGlsl(const ShaderCompileRequest& request) {
  ShaderCompileResult result;
  std::string preprocessed_source;
  if (!PreprocessGlsl(request, preprocessed_source)) {
    failure_count.fetch_add(1);
    return result;
  }
  const auto key = MakeShaderCacheKey(request, MakeGlslangCompilerVersionString(), preprocessed_source);
  CompileShaderWithCache(request, key, result, [&](std::vector<uint32_t>& binaries) {
    return CompilePreprocessedGlsl(request, preprocessed_source, binaries);
  });
  return result;
}

ShaderCompileResult CompileShaderToSpirv(const ShaderCompileRequest& request) {
  if (ShouldCompileWithGlslang(request)) {
    return CompileGlsl(request);
  }
  return CompileSlang(request);
}

bool CompileShaderToSpirv(const ShaderCompileRequest& request, std::vector<uint32_t>& binaries) {
  binaries.clear();
  ShaderCompileResult result = CompileShaderToSpirv(request);
  if (!result.success) {
    return false;
  }
  binaries = std::move(result.binaries);
  return true;
}

bool Shader::CompileToSpirv(const ShaderType shader_type, const std::string& source, std::vector<uint32_t>& binaries,
                            const std::filesystem::path& path) {
  return CompileShaderToSpirv(MakeSlangCompileRequest(shader_type, source, path), binaries);
}

bool Shader::ReflectSlang(const ShaderType shader_type, const std::string& source, ShaderReflectionInfo& reflection,
                          std::string& diagnostics, const std::filesystem::path& path) {
  return ReflectSlangRequest(MakeSlangCompileRequest(shader_type, source, path), reflection, diagnostics);
}

ShaderPipelineLayoutValidation Shader::ValidateSlangPipelineLayout(
    const ShaderType shader_type, const std::string& source, const std::filesystem::path& path,
    const std::vector<std::shared_ptr<DescriptorSetLayout>>& descriptor_set_layouts,
    const std::vector<VkPushConstantRange>& push_constant_ranges,
    const std::optional<std::vector<ShaderReflectionStageIo>>& stage_inputs,
    const std::optional<std::vector<ShaderReflectionStageIo>>& stage_outputs) {
  ShaderPipelineLayoutValidation validation;
  if (!ReflectSlang(shader_type, source, validation.reflection, validation.diagnostics, path)) {
    validation.success = false;
    return validation;
  }
  std::ostringstream diagnostics;
  for (const auto& reflected : validation.reflection.descriptor_bindings) {
    if (reflected.set >= descriptor_set_layouts.size() || !descriptor_set_layouts[reflected.set]) {
      diagnostics << "Missing descriptor set " << reflected.set << " for Slang binding '" << reflected.name << "'.\n";
      continue;
    }
    const auto& bindings = descriptor_set_layouts[reflected.set]->GetDescriptorBindings();
    const auto host = bindings.find(reflected.binding);
    if (host == bindings.end()) {
      diagnostics << "Missing descriptor binding set " << reflected.set << ", binding " << reflected.binding
                  << " for Slang binding '" << reflected.name << "'.\n";
      continue;
    }
    const auto& host_binding = host->second.binding;
    if (host_binding.descriptorType != reflected.descriptor_type) {
      diagnostics << "Descriptor type mismatch for set " << reflected.set << ", binding " << reflected.binding
                  << ": Slang expects " << VkDescriptorTypeName(reflected.descriptor_type) << ", host declares "
                  << VkDescriptorTypeName(host_binding.descriptorType) << ".\n";
    }
    if (host_binding.descriptorCount < reflected.descriptor_count) {
      diagnostics << "Descriptor count mismatch for set " << reflected.set << ", binding " << reflected.binding
                  << ": Slang expects " << reflected.descriptor_count << ", host declares "
                  << host_binding.descriptorCount << ".\n";
    }
    if ((host_binding.stageFlags & reflected.stage_flags) != reflected.stage_flags) {
      diagnostics << "Descriptor stage mismatch for set " << reflected.set << ", binding " << reflected.binding
                  << ".\n";
    }
  }
  for (const auto& reflected : validation.reflection.push_constant_ranges) {
    const auto covered =
        std::any_of(push_constant_ranges.begin(), push_constant_ranges.end(), [&](const VkPushConstantRange& range) {
          return PushConstantRangeCovers(range, reflected);
        });
    if (!covered) {
      diagnostics << "Missing push-constant range '" << reflected.name << "' at offset " << reflected.offset
                  << " with size " << reflected.size << ".\n";
    }
  }
  ValidateStageIoList(stage_inputs, validation.reflection.stage_inputs, "stage input", diagnostics);
  ValidateStageIoList(stage_outputs, validation.reflection.stage_outputs, "stage output", diagnostics);
  validation.diagnostics += diagnostics.str();
  validation.success = diagnostics.str().empty();
  return validation;
}

ShaderCompileCacheStats Shader::GetCompileCacheStats() {
  return {memory_hit_count.load(),     disk_hit_count.load(),      disk_miss_count.load(), compilation_count.load(),
          coalesced_wait_count.load(), corrupt_entry_count.load(), failure_count.load(),   slang_frontend_count.load()};
}

void Shader::ResetCompileCacheStats() {
  memory_hit_count.store(0);
  disk_hit_count.store(0);
  disk_miss_count.store(0);
  compilation_count.store(0);
  coalesced_wait_count.store(0);
  corrupt_entry_count.store(0);
  failure_count.store(0);
  slang_frontend_count.store(0);
}

void Shader::ClearInMemoryCompileCache() {
  const std::lock_guard lock(shader_compile_entries_mutex);
  shader_compile_entries.clear();
}

bool Shader::TryCompile(const ShaderType target_shader_type, const std::filesystem::path& path) {
  return TryCompile(target_shader_type, "", path);
}

bool Shader::TryCompile(const ShaderType target_shader_type, const std::string& header,
                        const std::filesystem::path& path) {
  std::stringstream shader_code_stream;
  shader_code_stream << header;
  shader_code_stream << FileUtils::LoadFileAsString(path);
  shader_type = static_cast<unsigned>(target_shader_type);
  shader_code = shader_code_stream.str();
  return TryCompile(path);
}

bool Shader::TryCompile(const ShaderType target_shader_type, const std::string& target_shader_code) {
  shader_type = static_cast<unsigned>(target_shader_type);
  shader_code = target_shader_code;
  return TryCompile();
}

bool Shader::TryCompile(const std::filesystem::path& path) {
  if (!Platform::Initialized())
    return false;
  VkShaderModuleCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  std::vector<uint32_t> binaries;
  if (CompileShaderToSpirv(MakeSlangCompileRequest(static_cast<ShaderType>(shader_type), shader_code, path),
                           binaries)) {
    create_info.pCode = binaries.data();
    create_info.codeSize = binaries.size() * sizeof(uint32_t);
    shader_module = std::make_unique<ShaderModule>(create_info);
    version_++;
    return true;
  }
  return false;
}

const std::unique_ptr<ShaderModule>& Shader::GetShaderModule() const {
  return shader_module;
}

ShaderType Shader::GetShaderType() const {
  return static_cast<ShaderType>(shader_type);
}

const std::string& Shader::PeekShaderCode() const {
  return shader_code;
}

std::string& Shader::RefShaderCode() {
  return shader_code;
}

unsigned& Shader::RefShaderType() {
  return shader_type;
}

std::shared_ptr<Shader> Shader::CreateTemporary(const ShaderType target_shader_type,
                                                const std::string& target_shader_code) {
  const auto ret_val = AssetManager::CreateTemporaryAsset<Shader>();
  ret_val->TryCompile(target_shader_type, target_shader_code);
  return ret_val;
}

std::shared_ptr<Shader> Shader::CreateTemporary(const ShaderType target_shader_type, const std::string& header,
                                                const std::filesystem::path& path) {
  const auto ret_val = AssetManager::CreateTemporaryAsset<Shader>();
  ret_val->TryCompile(target_shader_type, header, path);
  return ret_val;
}

std::shared_ptr<Shader> Shader::CreateTemporary(const ShaderType target_shader_type,
                                                const std::filesystem::path& path) {
  const auto ret_val = AssetManager::CreateTemporaryAsset<Shader>();
  ret_val->TryCompile(target_shader_type, path);
  return ret_val;
}
