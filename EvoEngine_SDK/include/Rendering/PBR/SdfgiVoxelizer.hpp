// Godot _render_sdfgi adapter, 34d06658a85845111a50db9e485ec4a0701d4298.
// See docs/licenses/Godot-MIT.txt.
#pragma once

#include "SdfgiResources.hpp"
#include "SdfgiScene.hpp"

namespace evo_engine {

// Explicit diagnostic readback; copies are recorded between rasterization and the next scratch clear.
class EVOENGINE_API SdfgiVoxelDebug {
 public:
  uint32_t cascade = 0;
  uint32_t slice = 64;
  bool recorded = false;
  std::array<std::shared_ptr<Buffer>, 4> planes;
  SdfgiVoxelDebug(uint32_t cascade, uint32_t slice);
  void Record(VkCommandBuffer command, const SdfgiResources& resources);
  [[nodiscard]] std::array<std::vector<uint32_t>, 4> Read() const;
  void StoreToPng(const std::filesystem::path& path) const;
  [[nodiscard]] uint64_t AllocationBytes() const;
};

class EVOENGINE_API SdfgiVoxelFrame : public std::enable_shared_from_this<SdfgiVoxelFrame> {
 public:
  struct Draw {
    SdfgiContributor contributor;
    SdfgiVoxelPushConstant constants;
    uint32_t first_index = 0;
    uint32_t index_count = 0;
  };
  struct Region {
    SdfgiPendingRegion pending;
    std::array<SdfgiVoxelData, 3> constants;
    std::array<std::shared_ptr<DescriptorSet>, 3> sets;
  };

  std::vector<Draw> draws;
  std::vector<Region> regions;
  std::map<std::string, std::shared_ptr<Buffer>> buffers;
  std::shared_ptr<DescriptorSet> scene_set;
  std::shared_ptr<Buffer> vertex_buffer;
  std::shared_ptr<Buffer> index_buffer;
  std::vector<SdfgiTextureInput> textures;
  std::vector<GltfShadeMaterial> material_data;
  std::vector<GltfTextureInfo> texture_data;
  std::shared_ptr<SdfgiVoxelDebug> debug;
  BufferUploadBatch input_uploads;
  BufferUploadArena uploads{64 * 1024};

  static std::shared_ptr<SdfgiVoxelFrame> Create(const SdfgiResources& resources,
                                                 const SdfgiContributorRegistry& contributors,
                                                 const std::vector<SdfgiCascade>& cascades,
                                                 const std::vector<SdfgiPendingRegion>& pending);
  void AddPasses(RenderGraph& graph, RenderGraphResourceRegistry& registry,
                 const std::shared_ptr<SdfgiResources>& resources);
  [[nodiscard]] uint64_t AllocationBytes() const;
};

}  // namespace evo_engine
