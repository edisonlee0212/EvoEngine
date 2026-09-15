#include "NativeLibrary.hpp"
#include <atomic>
#include <cctype>
#include <fstream>
#include "Console.hpp"
#include "RuntimePaths.hpp"

#if defined(_WIN32)
#  include <bcrypt.h>
#  pragma comment(lib, "bcrypt.lib")
#else
#  include <dlfcn.h>
#endif

using namespace evo_engine;

bool evo_engine::native_library::OpenLibrary(const std::filesystem::path& path, void*& handle) {
#if defined(_WIN32)
  handle = LoadLibraryW(path.wstring().c_str());
  if (!handle) {
    EVOENGINE_ERROR("Failed to load runtime package library: " + path.string() +
                    ". Windows error: " + std::to_string(GetLastError()))
    return false;
  }
#else
  handle = dlopen(path.string().c_str(), RTLD_NOW);
  if (!handle) {
    EVOENGINE_ERROR("Failed to load runtime package library: " + path.string() + ". " + dlerror())
    return false;
  }
#endif
  return true;
}

void evo_engine::native_library::CloseLibrary(void* handle) {
  if (!handle)
    return;
#if defined(_WIN32)
  FreeLibrary(static_cast<HMODULE>(handle));
#else
  dlclose(handle);
#endif
}

void* evo_engine::native_library::GetSymbol(void* handle, const char* name) {
  if (!handle)
    return nullptr;
#if defined(_WIN32)
  return reinterpret_cast<void*>(GetProcAddress(static_cast<HMODULE>(handle), name));
#else
  return dlsym(handle, name);
#endif
}

std::filesystem::path evo_engine::native_library::CreateShadowCopy(const std::filesystem::path& source) {
  if (runtime_paths::IsStrict()) {
    return source;
  }
#if defined(_WIN32)
  static std::atomic<uint64_t> next_copy_index{0};
  const auto copy_index = ++next_copy_index;

  std::error_code ec;
  const auto shadow_root =
      std::filesystem::temp_directory_path(ec) / "EvoEnginePackageShadow" / std::to_string(GetCurrentProcessId());
  if (ec) {
    EVOENGINE_ERROR("Failed to find temporary directory for runtime package shadow copy: " + ec.message())
    return {};
  }
  std::filesystem::create_directories(shadow_root, ec);
  if (ec) {
    EVOENGINE_ERROR("Failed to create runtime package shadow directory: " + shadow_root.string())
    return {};
  }

  const auto timestamp =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
          .count();
  const auto shadow_directory =
      shadow_root / (source.stem().string() + "_" + std::to_string(timestamp) + "_" + std::to_string(copy_index));
  std::filesystem::create_directories(shadow_directory, ec);
  if (ec) {
    EVOENGINE_ERROR("Failed to create runtime package shadow directory: " + shadow_directory.string())
    return {};
  }

  const auto shadow_path = shadow_directory / source.filename();
  std::filesystem::copy_file(source, shadow_path, std::filesystem::copy_options::overwrite_existing, ec);
  if (ec) {
    EVOENGINE_ERROR("Failed to shadow-copy runtime package " + source.string() + ": " + ec.message())
    return {};
  }
  return shadow_path;
#else
  return source;
#endif
}

bool evo_engine::native_library::VerifyLibraryHash(const std::filesystem::path& path,
                                                   const std::string& expected_sha256) {
#if defined(_WIN32)
  BCRYPT_ALG_HANDLE algorithm = nullptr;
  BCRYPT_HASH_HANDLE hash = nullptr;
  DWORD object_size = 0;
  DWORD hash_size = 0;
  DWORD result_size = 0;
  if (BCryptOpenAlgorithmProvider(&algorithm, BCRYPT_SHA256_ALGORITHM, nullptr, 0) < 0 ||
      BCryptGetProperty(algorithm, BCRYPT_OBJECT_LENGTH, reinterpret_cast<PUCHAR>(&object_size), sizeof(object_size),
                        &result_size, 0) < 0 ||
      BCryptGetProperty(algorithm, BCRYPT_HASH_LENGTH, reinterpret_cast<PUCHAR>(&hash_size), sizeof(hash_size),
                        &result_size, 0) < 0) {
    if (algorithm)
      BCryptCloseAlgorithmProvider(algorithm, 0);
    EVOENGINE_ERROR("Failed to initialize SHA-256 validation for runtime package: " + path.string())
    return false;
  }
  std::vector<UCHAR> object(object_size);
  std::vector<UCHAR> digest(hash_size);
  if (BCryptCreateHash(algorithm, &hash, object.data(), object_size, nullptr, 0, 0) < 0) {
    BCryptCloseAlgorithmProvider(algorithm, 0);
    EVOENGINE_ERROR("Failed to initialize SHA-256 hash for runtime package: " + path.string())
    return false;
  }
  std::ifstream stream(path, std::ios::binary);
  std::vector<char> buffer(64 * 1024);
  while (stream) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto size = stream.gcount();
    if (size > 0 && BCryptHashData(hash, reinterpret_cast<PUCHAR>(buffer.data()), static_cast<ULONG>(size), 0) < 0) {
      BCryptDestroyHash(hash);
      BCryptCloseAlgorithmProvider(algorithm, 0);
      EVOENGINE_ERROR("Failed to hash runtime package library: " + path.string())
      return false;
    }
  }
  const bool read_success = stream.eof();
  const bool hash_success = read_success && BCryptFinishHash(hash, digest.data(), hash_size, 0) >= 0;
  BCryptDestroyHash(hash);
  BCryptCloseAlgorithmProvider(algorithm, 0);
  if (!hash_success) {
    EVOENGINE_ERROR("Failed to read or finish hashing runtime package library: " + path.string())
    return false;
  }
  constexpr char hex[] = "0123456789abcdef";
  std::string actual_sha256;
  actual_sha256.reserve(digest.size() * 2);
  for (const auto value : digest) {
    actual_sha256.push_back(hex[value >> 4]);
    actual_sha256.push_back(hex[value & 0xf]);
  }
  std::string expected = expected_sha256;
  std::transform(expected.begin(), expected.end(), expected.begin(), [](const unsigned char value) {
    return static_cast<char>(std::tolower(value));
  });
  if (actual_sha256 != expected) {
    EVOENGINE_ERROR("Runtime package library hash mismatch: " + path.string())
    return false;
  }
  return true;
#else
  // Native package templates are currently produced for Windows only.
  (void)path;
  (void)expected_sha256;
  return true;
#endif
}
