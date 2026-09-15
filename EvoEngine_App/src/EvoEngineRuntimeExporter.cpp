#include <Windows.h>
#include <bcrypt.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#pragma comment(lib, "bcrypt.lib")

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace {
constexpr int kSchema = 1;
constexpr wchar_t kStagingMarker[] = L".evoengine-runtime-export-staging.json";

struct FileInfo {
  fs::path path;
  uintmax_t size = 0;
  std::string sha256;

  bool operator==(const FileInfo& other) const {
    return size == other.size && sha256 == other.sha256;
  }
};

std::string Utf8(const fs::path& path) {
  return path.generic_u8string();
}

std::string LowerAscii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool IsWithin(const fs::path& path, const fs::path& parent) {
  auto part = path.begin();
  for (const auto& expected : parent) {
    if (part == path.end() || CompareStringOrdinal(part->c_str(), -1, expected.c_str(), -1, TRUE) != CSTR_EQUAL) {
      return false;
    }
    ++part;
  }
  return true;
}

bool IsReparsePoint(const fs::path& path) {
  const DWORD attributes = GetFileAttributesW(path.c_str());
  return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_REPARSE_POINT) != 0;
}

void RequireNoReparseAncestors(fs::path path) {
  while (!path.empty()) {
    if (IsReparsePoint(path)) {
      throw std::runtime_error("Output must not traverse a symlink or junction: " + Utf8(path));
    }
    const auto parent = path.parent_path();
    if (parent == path) {
      break;
    }
    path = parent;
  }
}

void RequireRegularTreeEntry(const fs::path& path) {
  if (IsReparsePoint(path) || fs::is_symlink(fs::symlink_status(path))) {
    throw std::runtime_error("Symlinks and junctions are not allowed: " + Utf8(path));
  }
}

fs::path RelativePath(const std::string& value, const std::string& label, const bool allow_root = false) {
  if (value.empty()) {
    if (allow_root) {
      return fs::path(".");
    }
    throw std::runtime_error(label + " must not be empty.");
  }
  const fs::path path = fs::u8path(value);
  if (path.has_root_path()) {
    throw std::runtime_error(label + " must be relative.");
  }
  for (const auto& part : path) {
    if (part == "" || part == "." || part == ".." || part.native().find(L':') != std::wstring::npos) {
      throw std::runtime_error(label + " contains an invalid path component.");
    }
  }
  return path;
}

json ReadJson(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("Cannot read JSON: " + Utf8(path));
  }
  json value;
  stream >> value;
  if (!value.is_object()) {
    throw std::runtime_error("Expected a JSON object: " + Utf8(path));
  }
  return value;
}

void WriteJson(const fs::path& path, const json& value) {
  std::ofstream stream(path, std::ios::binary);
  if (!stream || !(stream << value.dump(2) << '\n')) {
    throw std::runtime_error("Cannot write JSON: " + Utf8(path));
  }
}

std::string Sha256(const fs::path& path) {
  BCRYPT_ALG_HANDLE algorithm = nullptr;
  BCRYPT_HASH_HANDLE hash = nullptr;
  DWORD object_size = 0;
  DWORD result_size = 0;
  std::vector<unsigned char> object;
  std::array<unsigned char, 32> digest{};
  const auto close = [&] {
    if (hash) {
      BCryptDestroyHash(hash);
    }
    if (algorithm) {
      BCryptCloseAlgorithmProvider(algorithm, 0);
    }
  };
  if (BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, 0) < 0 ||
      BCryptGetProperty(algorithm, BCRYPT_OBJECT_LENGTH, reinterpret_cast<PUCHAR>(&object_size), sizeof(object_size),
                        &result_size, 0) < 0) {
    close();
    throw std::runtime_error("Cannot initialize SHA-256.");
  }
  object.resize(object_size);
  if (BCryptCreateHash(algorithm, &hash, object.data(), object_size, nullptr, 0, 0) < 0) {
    close();
    throw std::runtime_error("Cannot initialize SHA-256 state.");
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    close();
    throw std::runtime_error("Cannot hash file: " + Utf8(path));
  }
  std::vector<char> buffer(64 * 1024);
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto size = stream.gcount();
    if (size > 0 && BCryptHashData(hash, reinterpret_cast<PUCHAR>(buffer.data()), static_cast<ULONG>(size), 0) < 0) {
      close();
      throw std::runtime_error("Cannot hash file: " + Utf8(path));
    }
  }
  if (!stream.eof() || BCryptFinishHash(hash, digest.data(), static_cast<ULONG>(digest.size()), 0) < 0) {
    close();
    throw std::runtime_error("Cannot finish hashing file: " + Utf8(path));
  }
  close();
  constexpr char hex[] = "0123456789abcdef";
  std::string value;
  value.reserve(digest.size() * 2);
  for (const auto byte : digest) {
    value.push_back(hex[byte >> 4]);
    value.push_back(hex[byte & 0xf]);
  }
  return value;
}

std::set<std::string> SnapshotAssetTree(const fs::path& assets) {
  std::set<std::string> entries;
  for (const auto& entry : fs::recursive_directory_iterator(assets)) {
    RequireRegularTreeEntry(entry.path());
    const auto relative = Utf8(entry.path().lexically_relative(assets));
    if (entry.is_directory())
      entries.emplace(LowerAscii(relative + "/"));
    else if (entry.is_regular_file())
      entries.emplace(LowerAscii(relative));
    else
      throw std::runtime_error("Unsupported project Assets entry: " + Utf8(entry.path()));
  }
  return entries;
}

std::map<std::string, FileInfo> SnapshotProject(const fs::path& project_file) {
  const auto root = project_file.parent_path();
  const auto assets = root / "Assets";
  if (!fs::is_regular_file(project_file) || project_file.extension() != ".eveproj" || !fs::is_directory(assets)) {
    throw std::runtime_error("Source project requires an existing .eveproj file and Assets directory.");
  }
  RequireRegularTreeEntry(project_file);
  RequireRegularTreeEntry(assets);
  std::vector<fs::path> files{project_file};
  for (const auto& entry : fs::recursive_directory_iterator(assets)) {
    RequireRegularTreeEntry(entry.path());
    if (entry.is_regular_file()) {
      files.push_back(entry.path());
    }
  }
  std::map<std::string, FileInfo> snapshot;
  for (const auto& file : files) {
    const auto relative = Utf8(file.lexically_relative(root));
    snapshot.emplace(LowerAscii(relative), FileInfo{fs::u8path(relative), fs::file_size(file), Sha256(file)});
  }
  return snapshot;
}

void ValidateCapturedInventory(const json& request, const std::map<std::string, FileInfo>& snapshot) {
  if (!request.contains("source_inventory")) {
    return;
  }
  const auto& inventory = request.at("source_inventory");
  if (!inventory.is_array()) {
    throw std::runtime_error("source_inventory must be an array.");
  }
  std::map<std::string, FileInfo> captured;
  for (const auto& item : inventory) {
    const auto relative = RelativePath(item.at("path").get<std::string>(), "source_inventory.path");
    FileInfo info{relative, item.at("size").get<uintmax_t>(), LowerAscii(item.at("sha256").get<std::string>())};
    if (!captured.emplace(LowerAscii(Utf8(relative)), std::move(info)).second) {
      throw std::runtime_error("source_inventory contains duplicate paths.");
    }
  }
  if (captured != snapshot) {
    throw std::runtime_error("Source project does not match the captured inventory.");
  }
}

void ValidateIdentity(const json& editor, const json& runtime) {
  constexpr std::array<const char*, 6> keys = {"sdk_source_id", "compiler_id", "compiler_version",
                                               "configuration", "platform",    "architecture"};
  if (editor.value("with_editor", false) != true || runtime.value("with_editor", true) != false) {
    throw std::runtime_error("Editor and runtime identities must be editor and runtime variants respectively.");
  }
  for (const auto* key : keys) {
    if (!editor.contains(key) || !editor.at(key).is_string() || editor.at(key).get<std::string>().empty() ||
        editor.at(key) != runtime.at(key)) {
      throw std::runtime_error(std::string("Editor/runtime native identity mismatch: ") + key);
    }
  }
}

std::string SanitizeName(const std::string& value) {
  std::string result;
  result.reserve(value.size());
  for (const unsigned char c : value) {
    result.push_back(c < 32 || std::string_view("<>:\"/\\|?*").find(static_cast<char>(c)) != std::string_view::npos
                         ? '_'
                         : static_cast<char>(c));
  }
  while (!result.empty() && (result.back() == ' ' || result.back() == '.')) {
    result.pop_back();
  }
  if (result.empty()) {
    throw std::runtime_error("application_name does not contain a usable Windows filename.");
  }
  static const std::set<std::string> reserved = {"con",  "prn",  "aux",  "nul",  "com1", "com2", "com3", "com4",
                                                 "com5", "com6", "com7", "com8", "com9", "lpt1", "lpt2", "lpt3",
                                                 "lpt4", "lpt5", "lpt6", "lpt7", "lpt8", "lpt9"};
  if (reserved.count(LowerAscii(result.substr(0, result.find('.'))))) {
    result.insert(result.begin(), '_');
  }
  return result;
}

struct PackageInfo {
  std::string name;
  std::string source_id;
  std::vector<std::string> dependencies;
  std::vector<fs::path> files;
  std::vector<fs::path> resources;
};

std::map<std::string, PackageInfo> ReadPackages(const json& metadata) {
  std::map<std::string, PackageInfo> packages;
  for (const auto& item : metadata.at("packages")) {
    PackageInfo package;
    package.name = item.at("name").get<std::string>();
    package.source_id = item.at("source_id").get<std::string>();
    package.dependencies = item.at("dependencies").get<std::vector<std::string>>();
    for (const auto* key : {"library", "manifest"}) {
      package.files.push_back(RelativePath(item.at(key).get<std::string>(), std::string("package.") + key));
    }
    if (!item.at("pdb").is_null()) {
      package.files.push_back(RelativePath(item.at("pdb").get<std::string>(), "package.pdb"));
    }
    for (const auto& resource : item.at("resources")) {
      const auto path = RelativePath(resource.get<std::string>(), "package.resources", true);
      if (path == ".") {
        throw std::runtime_error("Package resource root '.' cannot be filtered safely.");
      }
      package.resources.push_back(path);
    }
    if (!packages.emplace(LowerAscii(package.name), std::move(package)).second) {
      throw std::runtime_error("Runtime template contains duplicate packages.");
    }
  }
  return packages;
}

std::set<std::string> ValidateSelectedPackages(const json& request,
                                               const std::map<std::string, PackageInfo>& packages) {
  const auto& loaded = request.at("loaded_packages");
  if (!loaded.is_array()) {
    throw std::runtime_error("loaded_packages must be an array.");
  }
  std::set<std::string> selected;
  for (const auto& item : loaded) {
    const auto name = item.at("name").get<std::string>();
    const auto key = LowerAscii(name);
    const auto found = packages.find(key);
    if (found == packages.end() || found->second.name != name ||
        found->second.source_id != item.at("source_id").get<std::string>() || !selected.insert(key).second) {
      throw std::runtime_error("Loaded package does not match the runtime template: " + name);
    }
  }
  for (const auto& key : selected) {
    for (const auto& dependency : packages.at(key).dependencies) {
      if (!selected.count(LowerAscii(dependency))) {
        throw std::runtime_error("Loaded package set is missing dependency " + dependency + " required by " +
                                 packages.at(key).name);
      }
    }
  }
  return selected;
}

std::map<std::string, FileInfo> VerifyTemplate(const fs::path& root, const json& metadata) {
  RequireRegularTreeEntry(root);
  std::map<std::string, FileInfo> inventory;
  for (const auto& item : metadata.at("files")) {
    const auto relative = RelativePath(item.at("path").get<std::string>(), "template.files.path");
    const auto key = LowerAscii(Utf8(relative));
    const auto source = root / relative;
    RequireRegularTreeEntry(source);
    FileInfo info{relative, item.at("size").get<uintmax_t>(), LowerAscii(item.at("sha256").get<std::string>())};
    if (!fs::is_regular_file(source) || fs::file_size(source) != info.size || Sha256(source) != info.sha256 ||
        !inventory.emplace(key, std::move(info)).second) {
      throw std::runtime_error("Runtime template inventory mismatch: " + Utf8(relative));
    }
  }
  return inventory;
}

bool UnderRoot(const fs::path& path, const fs::path& root) {
  const auto path_key = LowerAscii(Utf8(path));
  auto root_key = LowerAscii(Utf8(root));
  return path_key == root_key ||
         (path_key.size() > root_key.size() && path_key.compare(0, root_key.size(), root_key) == 0 &&
          path_key[root_key.size()] == '/');
}

bool IncludeTemplateFile(const fs::path& path, const std::map<std::string, PackageInfo>& packages,
                         const std::set<std::string>& selected) {
  const auto key = LowerAscii(Utf8(path));
  bool package_file = false;
  bool selected_file = false;
  bool package_resource = false;
  bool selected_resource = false;
  for (const auto& [package_key, package] : packages) {
    for (const auto& file : package.files) {
      if (key == LowerAscii(Utf8(file))) {
        package_file = true;
        selected_file = selected_file || selected.count(package_key) != 0;
      }
    }
    for (const auto& root : package.resources) {
      if (UnderRoot(path, root)) {
        package_resource = true;
        selected_resource = selected_resource || selected.count(package_key) != 0;
      }
    }
  }
  return (!package_file || selected_file) && (!package_resource || selected_resource);
}

fs::path CreateStaging(const fs::path& output) {
  const auto parent = output.parent_path();
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto stamp = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    const auto candidate = parent / (L"." + output.filename().wstring() + L".tmp-" +
                                     std::to_wstring(GetCurrentProcessId()) + L"-" + std::to_wstring(stamp + attempt));
    if (fs::create_directory(candidate)) {
      WriteJson(candidate / kStagingMarker, {{"schema_version", 1}, {"kind", "EvoEngineRuntimeExportStaging"}});
      return candidate;
    }
  }
  throw std::runtime_error("Cannot create export staging directory.");
}

void CleanupStaging(const fs::path& staging, const fs::path& output_parent) {
  if (staging.parent_path() == output_parent && fs::is_regular_file(staging / kStagingMarker)) {
    std::error_code error;
    fs::remove_all(staging, error);
  }
}

void CopyFile(const fs::path& source, const fs::path& destination) {
  RequireRegularTreeEntry(source);
  fs::create_directories(destination.parent_path());
  fs::copy_file(source, destination, fs::copy_options::none);
}

void CopyAssets(const fs::path& source, const fs::path& destination) {
  fs::create_directories(destination);
  for (const auto& entry : fs::recursive_directory_iterator(source)) {
    RequireRegularTreeEntry(entry.path());
    const auto target = destination / entry.path().lexically_relative(source);
    if (entry.is_directory()) {
      fs::create_directories(target);
    } else if (entry.is_regular_file()) {
      CopyFile(entry.path(), target);
    } else {
      throw std::runtime_error("Unsupported project Assets entry: " + Utf8(entry.path()));
    }
  }
}

json OutputInventory(const fs::path& root) {
  std::vector<fs::path> files;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    RequireRegularTreeEntry(entry.path());
    if (entry.is_regular_file() && entry.path().filename() != kStagingMarker &&
        entry.path().filename() != L"build-report.json") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  json inventory = json::array();
  for (const auto& file : files) {
    inventory.push_back(
        {{"path", Utf8(file.lexically_relative(root))}, {"size", fs::file_size(file)}, {"sha256", Sha256(file)}});
  }
  return inventory;
}

void Export(const fs::path& request_path, const fs::path& template_path, const fs::path& output_path) {
  const auto request_file = fs::weakly_canonical(request_path);
  const auto template_root = fs::weakly_canonical(template_path);
  const auto output = fs::absolute(output_path).lexically_normal();
  RequireNoReparseAncestors(output);
  if (IsReparsePoint(output) || (fs::exists(output) && (!fs::is_directory(output) || !fs::is_empty(output)))) {
    throw std::runtime_error("Output must be absent or an empty regular directory.");
  }
  const auto request = ReadJson(request_file);
  const auto metadata = ReadJson(template_root / "template.json");
  if (request.value("schema_version", 0) != kSchema || metadata.value("schema_version", 0) != kSchema) {
    throw std::runtime_error("Unsupported export request or template schema.");
  }
  if (!metadata.contains("host") || !metadata.at("host").is_object()) {
    throw std::runtime_error("Runtime template does not contain a native host.");
  }
  ValidateIdentity(request.at("editor_identity"), metadata.at("identity"));
  const auto project_file = fs::weakly_canonical(fs::u8path(request.at("project").get<std::string>()));
  const auto project_root = project_file.parent_path();
  if (IsWithin(output, project_root)) {
    throw std::runtime_error("Output must be outside the source project.");
  }
  const auto source_snapshot = SnapshotProject(project_file);
  const auto source_asset_tree = SnapshotAssetTree(project_root / "Assets");
  ValidateCapturedInventory(request, source_snapshot);
  const auto template_inventory = VerifyTemplate(template_root, metadata);
  const auto packages = ReadPackages(metadata);
  const auto& identity_packages = metadata.at("identity").at("packages");
  for (const auto& [_, package] : packages) {
    if (!identity_packages.contains(package.name) || identity_packages.at(package.name) != package.source_id) {
      throw std::runtime_error("Runtime package metadata disagrees with native identity: " + package.name);
    }
  }
  const auto selected = ValidateSelectedPackages(request, packages);

  const auto host_relative = RelativePath(metadata.at("host").at("executable").get<std::string>(), "host.executable");
  if (!template_inventory.count(LowerAscii(Utf8(host_relative)))) {
    throw std::runtime_error("Runtime host is absent from the verified template inventory.");
  }
  const auto application_name = request.at("application_name").get<std::string>();
  if (application_name.empty() || request.at("startup_scene_handle").get<uint64_t>() == 0) {
    throw std::runtime_error("Export requires an application name and nonzero startup scene handle.");
  }
  const auto file_name = SanitizeName(application_name);
  const auto output_parent = output.parent_path();
  fs::create_directories(output_parent);
  const auto staging = CreateStaging(output);
  try {
    for (const auto& [_, info] : template_inventory) {
      if (!IncludeTemplateFile(info.path, packages, selected)) {
        continue;
      }
      const auto destination =
          LowerAscii(Utf8(info.path)) == LowerAscii(Utf8(host_relative)) ? fs::u8path(file_name + ".exe") : info.path;
      CopyFile(template_root / info.path, staging / destination);
      const auto copied = staging / destination;
      if (fs::file_size(copied) != info.size || Sha256(copied) != info.sha256) {
        throw std::runtime_error("Runtime template changed while copying: " + Utf8(info.path));
      }
    }
    const auto exported_project = staging / "Project";
    CopyAssets(project_root / "Assets", exported_project / "Assets");
    if (SnapshotAssetTree(exported_project / "Assets") != source_asset_tree) {
      throw std::runtime_error("Copied Assets tree differs from the captured source inventory.");
    }
    for (const auto& [_, info] : source_snapshot) {
      if (info.path.begin()->native() == L"Assets") {
        const auto copied = exported_project / info.path;
        if (!fs::is_regular_file(copied) || fs::file_size(copied) != info.size || Sha256(copied) != info.sha256) {
          throw std::runtime_error("Copied asset differs from the captured source: " + Utf8(info.path));
        }
      }
    }
    const auto project_name = file_name + ".eveproj";
    json project = {{"application_name", application_name},
                    {"start_scene_handle", request.at("startup_scene_handle")},
                    {"startup_runtime_packages", json::array()}};
    for (const auto& item : request.at("loaded_packages")) {
      project["startup_runtime_packages"].push_back(item.at("name"));
    }
    WriteJson(exported_project / fs::u8path(project_name), project);

    json runtime = request.value("runtime_config", json::object());
    if (!runtime.is_object()) {
      throw std::runtime_error("runtime_config must be an object.");
    }
    for (const auto* reserved : {"schema_version", "identity", "application_name", "project", "packages"}) {
      if (runtime.contains(reserved)) {
        throw std::runtime_error(std::string("runtime_config contains reserved key: ") + reserved);
      }
    }
    runtime["schema_version"] = 1;
    runtime["identity"] = metadata.at("identity");
    runtime["application_name"] = application_name;
    runtime["project"] = "Project/" + project_name;
    runtime["packages"] = request.at("loaded_packages");
    WriteJson(staging / "runtime.yaml", runtime);

    if (SnapshotProject(project_file) != source_snapshot ||
        SnapshotAssetTree(project_root / "Assets") != source_asset_tree) {
      throw std::runtime_error("Source project changed while the runtime application was being exported.");
    }
    json report = {{"schema_version", 1},
                   {"application_name", application_name},
                   {"template_id", metadata.at("template_id")},
                   {"identity", metadata.at("identity")},
                   {"packages", request.at("loaded_packages")},
                   {"files", OutputInventory(staging)}};
    WriteJson(staging / "build-report.json", report);

    if (SnapshotProject(project_file) != source_snapshot ||
        SnapshotAssetTree(project_root / "Assets") != source_asset_tree) {
      throw std::runtime_error("Source project changed before the runtime application was published.");
    }

    RequireNoReparseAncestors(output);
    if (fs::exists(output)) {
      if (IsReparsePoint(output) || !fs::is_directory(output) || !fs::is_empty(output) || !fs::remove(output)) {
        throw std::runtime_error("Output changed before publication; existing files were preserved.");
      }
    }
    const auto publish_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!MoveFileExW(staging.c_str(), output.c_str(), 0)) {
      const auto error = GetLastError();
      if ((error != ERROR_ACCESS_DENIED && error != ERROR_SHARING_VIOLATION) || fs::exists(output) ||
          std::chrono::steady_clock::now() >= publish_deadline) {
        throw std::runtime_error("Cannot publish export without overwriting the destination (Windows error " +
                                 std::to_string(error) + ").");
      }
      Sleep(100);
      RequireNoReparseAncestors(output);
      if (SnapshotProject(project_file) != source_snapshot ||
          SnapshotAssetTree(project_root / "Assets") != source_asset_tree) {
        throw std::runtime_error("Source project changed before the runtime application was published.");
      }
    }
    fs::remove(output / kStagingMarker);
  } catch (...) {
    CleanupStaging(staging, output_parent);
    throw;
  }
}

std::map<std::wstring, fs::path> ParseArguments(const int argc, wchar_t** argv) {
  std::map<std::wstring, fs::path> values;
  for (int index = 1; index < argc; index += 2) {
    if (index + 1 >= argc || (std::wstring(argv[index]) != L"--request" && std::wstring(argv[index]) != L"--template" &&
                              std::wstring(argv[index]) != L"--output")) {
      throw std::runtime_error("Usage: EvoEngineRuntimeExporter --request FILE --template DIR --output DIR");
    }
    if (!values.emplace(argv[index], fs::path(argv[index + 1])).second) {
      throw std::runtime_error("Duplicate command-line argument.");
    }
  }
  if (values.size() != 3) {
    throw std::runtime_error("Usage: EvoEngineRuntimeExporter --request FILE --template DIR --output DIR");
  }
  return values;
}
}  // namespace

int wmain(const int argc, wchar_t** argv) {
  try {
    const auto arguments = ParseArguments(argc, argv);
    Export(arguments.at(L"--request"), arguments.at(L"--template"), arguments.at(L"--output"));
    std::wcout << L"Runtime application exported to " << fs::absolute(arguments.at(L"--output")) << std::endl;
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Runtime export failed: " << error.what() << std::endl;
    return 1;
  }
}
