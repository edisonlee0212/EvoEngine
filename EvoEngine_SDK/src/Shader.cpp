#include "Shader.hpp"
#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <iomanip>
#include <mutex>
#include <thread>
#include "AssetManager.hpp"
#include "Console.hpp"
#include "Platform.hpp"
#include "ProjectManager.hpp"
#include "ResourceLimits.h"
#include "SPIRV/GlslangToSpv.h"
#include "Serialization.hpp"
#include "ShaderLang.h"
#include "Utilities.hpp"

using namespace evo_engine;

namespace {
constexpr uint32_t kShaderCacheSchema = 3;
constexpr uint32_t kVulkanTarget = 13;
constexpr uint32_t kSpirvTarget = 14;
constexpr uint32_t kSpirvMagic = 0x07230203;

std::filesystem::path GetShaderBinaryDirectory() {
  if (const char* path = std::getenv("EVOENGINE_SHADER_CACHE_DIR"); path && path[0] != '\0') {
    return path;
  }
  return "./ShaderBinaries";
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
std::mutex glslang_mutex;
std::atomic<uint64_t> temporary_file_counter = 0;
std::atomic<uint64_t> memory_hit_count = 0;
std::atomic<uint64_t> disk_hit_count = 0;
std::atomic<uint64_t> disk_miss_count = 0;
std::atomic<uint64_t> compilation_count = 0;
std::atomic<uint64_t> coalesced_wait_count = 0;
std::atomic<uint64_t> corrupt_entry_count = 0;
std::atomic<uint64_t> failure_count = 0;

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

void HashBytes(uint64_t& hash, const void* data, const size_t size) {
  const auto* bytes = static_cast<const uint8_t*>(data);
  for (size_t i = 0; i < size; ++i) {
    hash ^= bytes[i];
    hash *= 1099511628211ull;
  }
}

ShaderCacheKey MakeShaderCacheKey(const ShaderType shader_type, const std::string& preprocessed_source) {
  ShaderCacheKey key{14695981039346656037ull, 1099511628211ull ^ 0xd6e8feb86659fd93ull};
  const auto compiler_version = glslang::GetVersion();
  const std::array<uint32_t, 8> descriptor = {kShaderCacheSchema,
                                              static_cast<uint32_t>(shader_type),
                                              kVulkanTarget,
                                              kSpirvTarget,
                                              1u,
                                              static_cast<uint32_t>(compiler_version.major),
                                              static_cast<uint32_t>(compiler_version.minor),
                                              static_cast<uint32_t>(compiler_version.patch)};
  HashBytes(key.low, descriptor.data(), descriptor.size() * sizeof(uint32_t));
  HashBytes(key.high, descriptor.data(), descriptor.size() * sizeof(uint32_t));
  if (compiler_version.flavor) {
    const auto flavor_size = std::strlen(compiler_version.flavor);
    HashBytes(key.low, compiler_version.flavor, flavor_size);
    HashBytes(key.high, compiler_version.flavor, flavor_size);
  }
  HashBytes(key.low, preprocessed_source.data(), preprocessed_source.size());
  HashBytes(key.high, preprocessed_source.data(), preprocessed_source.size());
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
    if (path.extension() == ".eveshader") {
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
    if (path.extension() == ".eveshader") {
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
    if (path.extension() == ".eveshader") {
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

  // Note "local" vs. "system" is not an "either/or": "local" is an
  // extra thing to do over "system". Both might get called, as per
  // the C++ specification.
  //
  // For the "system" or <>-style includes; search the "system" paths.
  IncludeResult* includeSystem(const char* header_name, const char* includer_name, size_t inclusion_depth) override;

  // For the "local"-only aspect of a "" include. Should not search in the
  // "system" paths, because on returning a failure, the parser will
  // call includeSystem() to look in the "system" locations.
  IncludeResult* includeLocal(const char* header_name, const char* includer_name, size_t inclusion_depth) override;

  void releaseInclude(IncludeResult*) override;

 private:
  std::set<std::filesystem::path> include_paths_;
  IncludeResult fail_result_ = IncludeResult("", "Header does not exist!", 0, nullptr);
  std::unordered_map<std::filesystem::path, std::shared_ptr<IncludeResult>> includes_;
  std::unordered_map<std::filesystem::path, std::string> sources_;
};

glslang::TShader::Includer::IncludeResult* GlslShaderIncluder::includeSystem(const char* header_name,
                                                                             const char* includer_name,
                                                                             size_t inclusion_depth) {
  std::filesystem::path resolved_header_path;
  const std::filesystem::path temp(header_name);
  bool found = false;
  for (const auto& dir : include_paths_) {
    if (std::filesystem::exists(dir / temp)) {
      found = true;
      resolved_header_path = std::filesystem::weakly_canonical(dir / temp);
      break;
    }
  }
  if (!found && std::filesystem::exists(temp)) {
    resolved_header_path = std::filesystem::weakly_canonical(temp);
    found = true;
  }
  if (const auto it = includes_.find(resolved_header_path); it != includes_.end()) {
    return it->second.get();
  }
  if (found) {
    sources_[resolved_header_path] = FileUtils::LoadFileAsString(resolved_header_path);
  } else {
    return &fail_result_;
  }
  auto [it, b] = includes_.emplace(std::make_pair(
      resolved_header_path,
      std::make_shared<IncludeResult>(resolved_header_path.string(), sources_.at(resolved_header_path).data(),
                                      sources_.at(resolved_header_path).size(), nullptr)));
  if (!b) {
    EVOENGINE_ERROR("Failed to insert IncludeResult into std::map!");
    return &fail_result_;
  }
  return it->second.get();
}

glslang::TShader::Includer::IncludeResult* GlslShaderIncluder::includeLocal(const char* header_name,
                                                                            const char* includer_name,
                                                                            const size_t inclusion_depth) {
  return includeSystem(header_name, includer_name, inclusion_depth);
}

void GlslShaderIncluder::releaseInclude(IncludeResult* result) {
  if (const auto it = sources_.find(result->headerName); it != sources_.end()) {
    sources_.erase(it);
  }
  if (const auto it = includes_.find(result->headerName); it != includes_.end()) {
    includes_.erase(it);
  }
}

bool TryGetShaderLanguage(const ShaderType shader_type, EShLanguage& language) {
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

void ConfigureShader(glslang::TShader& shader, const EShLanguage language) {
  constexpr int default_version = 460;
  shader.setEnvClient(glslang::EShClientVulkan, glslang::EShTargetVulkan_1_3);
  shader.setEnvTarget(glslang::EshTargetSpv, glslang::EShTargetSpv_1_4);
  shader.setEnvInput(glslang::EShSourceGlsl, language, glslang::EShClientVulkan, default_version);
  shader.setEntryPoint("main");
}

bool PreprocessGlsl(const ShaderType shader_type, const std::string& source, std::string& preprocessed_source,
                    const std::filesystem::path& path) {
  EShLanguage language;
  if (!TryGetShaderLanguage(shader_type, language)) {
    EVOENGINE_ERROR("Unknown type shader!")
    return false;
  }
  EnsureGlslangProcess();
  const std::lock_guard glslang_lock(glslang_mutex);
  glslang::TShader shader(language);
  const std::string actual_code = std::string("#version 460\n") + source;
  const char* sources[] = {actual_code.c_str()};
  shader.setStrings(sources, 1);
  ConfigureShader(shader, language);
  constexpr int default_version = 460;
  constexpr bool forward_compatible = false;
  constexpr auto message_flags = static_cast<EShMessages>(EShMsgSpvRules | EShMsgVulkanRules);
  GlslShaderIncluder includer(Shader::GetRegisteredShaderIncludePaths());
  if (!shader.preprocess(GetDefaultResources(), default_version, ECoreProfile, false, forward_compatible, message_flags,
                         &preprocessed_source, includer)) {
    EVOENGINE_ERROR("Failed to preprocess shader: " + path.string() + "\n" + std::string(shader.getInfoLog()))
    return false;
  }
  return true;
}

bool CompilePreprocessedGlsl(const ShaderType shader_type, const std::string& preprocessed_source,
                             std::vector<uint32_t>& binaries, const std::filesystem::path& path) {
  EShLanguage language;
  if (!TryGetShaderLanguage(shader_type, language)) {
    return false;
  }
  EnsureGlslangProcess();
  const std::lock_guard glslang_lock(glslang_mutex);
  glslang::TShader shader(language);
  const char* sources[] = {preprocessed_source.c_str()};
  shader.setStrings(sources, 1);
  ConfigureShader(shader, language);
  constexpr int default_version = 460;
  constexpr bool forward_compatible = false;
  constexpr auto message_flags = static_cast<EShMessages>(EShMsgSpvRules | EShMsgVulkanRules);
  GlslShaderIncluder includer(Shader::GetRegisteredShaderIncludePaths());
  if (!shader.parse(GetDefaultResources(), default_version, ECoreProfile, false, forward_compatible, message_flags,
                    includer)) {
    EVOENGINE_ERROR("Failed to parse shader: " + path.string() + "\n" + std::string(shader.getInfoLog()))
    return false;
  }
  glslang::TProgram program;
  program.addShader(&shader);
  if (!program.link(message_flags)) {
    EVOENGINE_ERROR("Failed to link shader: " + path.string() + "\n" + std::string(program.getInfoLog()))
    return false;
  }
  glslang::SpvOptions options{};
  options.generateDebugInfo = true;
  options.validate = true;
  spv::SpvBuildLogger logger;
  GlslangToSpv(*program.getIntermediate(language), binaries, &logger, &options);
  return !binaries.empty() && binaries.front() == kSpirvMagic;
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
                                           const ShaderType shader_type, std::vector<uint32_t>& binaries) {
  std::error_code error;
  if (!std::filesystem::exists(cache_path, error) || error) {
    return ShaderCacheLoadResult::Missing;
  }
  try {
    std::ifstream stream(cache_path, std::ios::binary);
    const std::array<char, 8> expected_magic = {'E', 'V', 'O', 'S', 'P', 'V', '3', '\0'};
    std::array<char, 8> magic{};
    stream.read(magic.data(), magic.size());
    uint32_t schema = 0;
    uint32_t cached_shader_type = 0;
    uint32_t vulkan_target = 0;
    uint32_t spirv_target = 0;
    ShaderCacheKey cached_key{};
    ShaderCacheKey checksum{};
    uint64_t word_count = 0;
    if (!stream || magic != expected_magic || !ReadBinaryValue(stream, schema) ||
        !ReadBinaryValue(stream, cached_shader_type) || !ReadBinaryValue(stream, vulkan_target) ||
        !ReadBinaryValue(stream, spirv_target) || !ReadBinaryValue(stream, cached_key.low) ||
        !ReadBinaryValue(stream, cached_key.high) || !ReadBinaryValue(stream, checksum.low) ||
        !ReadBinaryValue(stream, checksum.high) || !ReadBinaryValue(stream, word_count) ||
        schema != kShaderCacheSchema || cached_shader_type != static_cast<uint32_t>(shader_type) ||
        vulkan_target != kVulkanTarget || spirv_target != kSpirvTarget || !(cached_key == key) || word_count == 0) {
      return ShaderCacheLoadResult::Corrupt;
    }
    constexpr uint64_t header_size = 64;
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
                             const ShaderType shader_type, const std::vector<uint32_t>& binaries) {
  std::filesystem::path temporary_path;
  try {
    std::filesystem::create_directories(cache_path.parent_path());
    temporary_path = cache_path;
    temporary_path += ".tmp." + std::to_string(temporary_file_counter.fetch_add(1)) + "." +
                      std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id()));
    std::ofstream stream(temporary_path, std::ios::binary | std::ios::trunc);
    const std::array<char, 8> magic = {'E', 'V', 'O', 'S', 'P', 'V', '3', '\0'};
    stream.write(magic.data(), magic.size());
    WriteBinaryValue(stream, kShaderCacheSchema);
    const auto type_value = static_cast<uint32_t>(shader_type);
    WriteBinaryValue(stream, type_value);
    WriteBinaryValue(stream, kVulkanTarget);
    WriteBinaryValue(stream, kSpirvTarget);
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

bool CompileGlsl(const ShaderType shader_type, const std::string& source, std::vector<uint32_t>& binaries,
                 const std::filesystem::path& path) {
  binaries.clear();
  std::string preprocessed_source;
  if (!PreprocessGlsl(shader_type, source, preprocessed_source, path)) {
    failure_count.fetch_add(1);
    return false;
  }
  const auto key = MakeShaderCacheKey(shader_type, preprocessed_source);
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
    binaries = entry->binaries;
    return entry->success;
  }

  const auto cache_path = GetShaderBinaryDirectory() / (ShaderCacheKeyString(key) + ".spvbin");
  auto load_result = LoadShaderCacheEntry(cache_path, key, shader_type, binaries);
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
    success = CompilePreprocessedGlsl(shader_type, preprocessed_source, binaries, path);
    if (success) {
      PublishShaderCacheEntry(cache_path, key, shader_type, binaries);
    } else {
      failure_count.fetch_add(1);
    }
  }

  {
    const std::lock_guard lock(entry->mutex);
    entry->success = success;
    entry->binaries = binaries;
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
  return success;
}

bool Shader::CompileToSpirv(const ShaderType shader_type, const std::string& source, std::vector<uint32_t>& binaries,
                            const std::filesystem::path& path) {
  return CompileGlsl(shader_type, source, binaries, path);
}

ShaderCompileCacheStats Shader::GetCompileCacheStats() {
  return {memory_hit_count.load(),     disk_hit_count.load(),      disk_miss_count.load(), compilation_count.load(),
          coalesced_wait_count.load(), corrupt_entry_count.load(), failure_count.load()};
}

void Shader::ResetCompileCacheStats() {
  memory_hit_count.store(0);
  disk_hit_count.store(0);
  disk_miss_count.store(0);
  compilation_count.store(0);
  coalesced_wait_count.store(0);
  corrupt_entry_count.store(0);
  failure_count.store(0);
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
  if (CompileGlsl(static_cast<ShaderType>(shader_type), shader_code, binaries, path)) {
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
