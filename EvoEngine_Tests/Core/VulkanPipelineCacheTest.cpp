#include "EvoEngine_SDK_PCH.hpp"

#include "VulkanPipelineCache.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

using namespace evo_engine;

namespace {
class PipelineCacheFileScope {
 public:
  PipelineCacheFileScope() {
    root_ =
        std::filesystem::temp_directory_path() /
        ("EvoEnginePipelineCacheTest_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root_);
  }

  ~PipelineCacheFileScope() {
    std::error_code error;
    std::filesystem::remove_all(root_, error);
  }

  [[nodiscard]] std::filesystem::path Path() const {
    return root_ / "pipeline.bin";
  }

 private:
  std::filesystem::path root_;
};

PipelineCacheIdentity TestIdentity() {
  PipelineCacheIdentity identity;
  identity.vendor_id = 0x10de;
  identity.device_id = 0x2684;
  identity.driver_version = 0x12345678;
  identity.api_version = VK_API_VERSION_1_3;
  for (size_t i = 0; i < identity.uuid.size(); ++i)
    identity.uuid[i] = static_cast<uint8_t>(i + 1u);
  return identity;
}

template <typename T>
void WritePayloadValue(std::vector<uint8_t>& payload, const size_t offset, const T value) {
  std::memcpy(payload.data() + offset, &value, sizeof(T));
}

std::vector<uint8_t> TestPayload(const PipelineCacheIdentity& identity, const uint8_t suffix = 0) {
  std::vector<uint8_t> payload(48, suffix);
  WritePayloadValue(payload, 0, uint32_t{32});
  WritePayloadValue(payload, 4, uint32_t{VK_PIPELINE_CACHE_HEADER_VERSION_ONE});
  WritePayloadValue(payload, 8, identity.vendor_id);
  WritePayloadValue(payload, 12, identity.device_id);
  std::copy(identity.uuid.begin(), identity.uuid.end(), payload.begin() + 16);
  return payload;
}
}  // namespace

TEST(VulkanPipelineCache, ValidatedFileRoundTripsAndAtomicallyReplaces) {
  PipelineCacheFileScope scope;
  const auto identity = TestIdentity();
  const auto first = TestPayload(identity, 0x11);
  ASSERT_TRUE(VulkanPipelineCache::PublishFile(scope.Path(), identity, first));
  std::vector<uint8_t> loaded;
  EXPECT_EQ(VulkanPipelineCache::LoadFile(scope.Path(), identity, loaded), PipelineCacheLoadResult::Valid);
  EXPECT_EQ(loaded, first);

  const auto replacement = TestPayload(identity, 0x22);
  ASSERT_TRUE(VulkanPipelineCache::PublishFile(scope.Path(), identity, replacement));
  EXPECT_EQ(VulkanPipelineCache::LoadFile(scope.Path(), identity, loaded), PipelineCacheLoadResult::Valid);
  EXPECT_EQ(loaded, replacement);
}

TEST(VulkanPipelineCache, RejectsCorruptAndIncompatibleFilesBeforeDriverUse) {
  PipelineCacheFileScope scope;
  const auto identity = TestIdentity();
  ASSERT_TRUE(VulkanPipelineCache::PublishFile(scope.Path(), identity, TestPayload(identity)));
  {
    std::fstream stream(scope.Path(), std::ios::binary | std::ios::in | std::ios::out);
    stream.seekp(-1, std::ios::end);
    const char corrupt = '\x7f';
    stream.write(&corrupt, 1);
  }
  std::vector<uint8_t> loaded;
  EXPECT_EQ(VulkanPipelineCache::LoadFile(scope.Path(), identity, loaded), PipelineCacheLoadResult::Corrupt);
  EXPECT_TRUE(loaded.empty());

  ASSERT_TRUE(VulkanPipelineCache::PublishFile(scope.Path(), identity, TestPayload(identity)));
  auto other_driver = identity;
  ++other_driver.driver_version;
  EXPECT_EQ(VulkanPipelineCache::LoadFile(scope.Path(), other_driver, loaded), PipelineCacheLoadResult::Incompatible);
}

TEST(VulkanPipelineCache, ValidatesRawHeaderAndEnforcesReferenceSizeCap) {
  PipelineCacheFileScope scope;
  const auto identity = TestIdentity();
  auto payload = TestPayload(identity);
  EXPECT_TRUE(VulkanPipelineCache::IsRawPayloadCompatible(payload, identity));
  ++payload[16];
  EXPECT_FALSE(VulkanPipelineCache::IsRawPayloadCompatible(payload, identity));

  {
    std::ofstream stream(scope.Path(), std::ios::binary | std::ios::trunc);
    stream.seekp(static_cast<std::streamoff>(VulkanPipelineCache::kMaxCacheFileBytes));
    stream.put('\0');
  }
  std::vector<uint8_t> loaded;
  EXPECT_EQ(VulkanPipelineCache::LoadFile(scope.Path(), identity, loaded), PipelineCacheLoadResult::Oversized);
}

TEST(VulkanPipelineCache, ClassifiesDeferredCompletionAndSynchronousRetry) {
  EXPECT_TRUE(VulkanPipelineCache::IsRayTracingCreateSuccess(VK_SUCCESS));
  EXPECT_TRUE(VulkanPipelineCache::IsRayTracingCreateSuccess(VK_OPERATION_NOT_DEFERRED_KHR));
  EXPECT_FALSE(VulkanPipelineCache::ShouldRetryRayTracingSynchronously(VK_OPERATION_NOT_DEFERRED_KHR));
  EXPECT_FALSE(VulkanPipelineCache::ShouldRetryRayTracingSynchronously(VK_ERROR_DEVICE_LOST));
  EXPECT_TRUE(VulkanPipelineCache::ShouldRetryRayTracingSynchronously(VK_ERROR_INITIALIZATION_FAILED));
  EXPECT_TRUE(VulkanPipelineCache::IsIncompleteDeferredOperation(VK_NOT_READY));
  EXPECT_FALSE(VulkanPipelineCache::IsIncompleteDeferredOperation(VK_SUCCESS));
  EXPECT_FALSE(VulkanPipelineCache::IsIncompleteDeferredOperation(VK_ERROR_OUT_OF_HOST_MEMORY));
}
