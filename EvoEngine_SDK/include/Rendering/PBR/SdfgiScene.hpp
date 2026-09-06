// Godot SDFGI input/placement adapter, 34d06658a85845111a50db9e485ec4a0701d4298.
// See docs/licenses/Godot-MIT.txt and docs/sdfgi.md.
#pragma once

#include "Bound.hpp"
#include "GltfMaterial.hpp"
#include "SdfgiSettings.hpp"
#include "SdfgiTypes.hpp"

#include <map>
#include <memory>
#include <optional>
#include <set>

namespace evo_engine {

class Scene;
class Mesh;
class Material;
class Texture2D;
class Cubemap;
class Image;
class ImageView;
class Sampler;
struct ResolvedEnvironmentalLighting;

struct EVOENGINE_API SdfgiCascade {
  float cell_size = 0;
  glm::ivec3 position{0};
  glm::ivec3 size{128};
  glm::ivec3 dirty_regions{0};
  bool full_redraw = true;

  [[nodiscard]] Bound WorldBounds(float y_mult) const;
};

struct SdfgiPendingRegion {
  uint32_t cascade = 0;
  glm::ivec3 offset{0};
  glm::ivec3 size{128};
  Bound world_bounds;
};

EVOENGINE_API float SdfgiYMultiplier(SdfgiSettings::VerticalScale scale);
// Returns a diagnostic before modifying the field if coordinates cannot fit the reference integer grid.
EVOENGINE_API std::string UpdateSdfgiCascades(const SdfgiSettings& settings, const glm::vec3& anchor,
                                              std::vector<SdfgiCascade>& cascades);
EVOENGINE_API std::vector<SdfgiPendingRegion> GetSdfgiPendingRegions(const std::vector<SdfgiCascade>& cascades,
                                                                     float y_mult);
EVOENGINE_API SdfgiCascadeBlock BuildSdfgiCascadeBlock(const std::vector<SdfgiCascade>& cascades);

struct EVOENGINE_API SdfgiTextureInput {
  uint64_t asset_id = 0;
  uint32_t version = 0;
  uint64_t content_signature = 0;
  std::shared_ptr<Texture2D> texture;
  std::shared_ptr<Image> image;
  std::shared_ptr<ImageView> image_view;
  std::shared_ptr<Sampler> sampler;
  GltfTextureInfo mapping;
  bool samples_linear_srgb = false;

  [[nodiscard]] bool operator==(const SdfgiTextureInput& other) const;
};

struct EVOENGINE_API SdfgiMaterialInput {
  glm::vec4 base_color{1};
  glm::vec3 emission{0};
  SdfgiTextureInput base_texture;
  SdfgiTextureInput emission_texture;
  bool masked = false;
  bool double_sided = false;
  float alpha_cutoff = 0.5f;
  uint32_t cull_mode = 0;

  [[nodiscard]] bool SameCoverage(const SdfgiMaterialInput& other) const;
  [[nodiscard]] bool SamePayload(const SdfgiMaterialInput& other) const;
};

EVOENGINE_API SdfgiMaterialInput SnapshotSdfgiMaterial(const std::shared_ptr<Material>& material);

enum class SdfgiExclusion : uint32_t { None, Disabled, Dynamic, Deforming, Forward, Unsupported, Empty, InvalidBounds };
EVOENGINE_API const char* GetSdfgiExclusionName(SdfgiExclusion reason);

// A renderer may be instanced by more than one LOD-group entity; neither array indices nor camera LOD are identity.
using SdfgiContributorId = std::pair<uint64_t, uint64_t>;

struct SdfgiContributor {
  SdfgiContributorId id{};
  std::shared_ptr<Mesh> mesh;
  uint64_t mesh_id = 0;
  uint32_t geometry_version = 0;
  uint32_t vertex_count = 0;
  uint32_t triangle_count = 0;
  glm::mat4 transform{1};
  Bound world_bounds;
  SdfgiMaterialInput material;
  SdfgiExclusion exclusion = SdfgiExclusion::None;
};

enum SdfgiChangeFlags : uint32_t {
  SdfgiAdded = 1,
  SdfgiRemoved = 2,
  SdfgiTransformChanged = 4,
  SdfgiGeometryChanged = 8,
  SdfgiCoverageChanged = 16,
  SdfgiPayloadChanged = 32,
  SdfgiUncertainBounds = 64,
};

struct SdfgiContributorChange {
  SdfgiContributorId id{};
  uint32_t flags = 0;
  std::optional<SdfgiContributor> before;
  std::optional<SdfgiContributor> after;
};

struct SdfgiLightInput {
  enum class Type : uint32_t { Directional, Point, Spot };
  uint64_t id = 0;
  Type type = Type::Directional;
  bool dynamic = true;
  bool casts_shadow = false;
  glm::vec3 color{0};
  glm::vec3 position{0};
  glm::vec3 direction{0, 0, -1};
  glm::vec3 attenuation{1, 0, 0};
  float range = 0;
  float cos_inner = 1;
  float cos_outer = 1;
  Bound world_bounds;
};

struct SdfgiSkyInput {
  std::shared_ptr<Cubemap> cubemap;
  uint64_t map_id = 0;
  uint32_t map_version = 0;
  uint32_t cubemap_version = 0;
  bool constant_color = false;
  glm::vec3 color{0};
  float gamma = 2.2f;
  float rotation = 0;
  float energy = 1;
};

// Apply after cascade/classification filtering. Returns the number excluded by the reference capacity.
EVOENGINE_API uint32_t BoundSdfgiLightList(std::vector<SdfgiLightInput>& lights, bool dynamic);

struct SdfgiSceneSnapshot {
  std::vector<SdfgiContributor> contributors;
  std::vector<SdfgiLightInput> lights;
  SdfgiSkyInput sky;
  std::map<SdfgiExclusion, uint32_t> excluded;
};

EVOENGINE_API SdfgiSceneSnapshot SnapshotSdfgiScene(const std::shared_ptr<Scene>& scene,
                                                    const ResolvedEnvironmentalLighting& lighting);

struct EVOENGINE_API SdfgiContributorRegistry {
  std::map<SdfgiContributorId, SdfgiContributor> entries;
  std::vector<SdfgiContributorChange> changes;
  std::set<SdfgiContributorId> invalid_bounds;

  void Update(const std::vector<SdfgiContributor>& snapshot);
  [[nodiscard]] std::vector<uint32_t> AffectedCascades(const std::vector<SdfgiCascade>& cascades, float y_mult) const;
};

}  // namespace evo_engine
