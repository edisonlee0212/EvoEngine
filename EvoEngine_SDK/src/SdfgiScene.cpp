// Placement/region logic adapted from Godot renderer_rd/environment/gi.cpp::SDFGI::{create,update,
// get_pending_region_data,update_cascades}, 34d06658a85845111a50db9e485ec4a0701d4298.
// Scene/material/light snapshots are the EvoEngine host adapter. See docs/licenses/Godot-MIT.txt.
#include "SdfgiScene.hpp"

#include "Cubemap.hpp"
#include "EnvironmentalMap.hpp"
#include "GaussianSplatRenderer.hpp"
#include "Lights.hpp"
#include "LodGroup.hpp"
#include "Material.hpp"
#include "MeshRenderer.hpp"
#include "Particles.hpp"
#include "ResolvedEnvironmentalLighting.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "SkinnedMeshRenderer.hpp"
#include "StrandsRenderer.hpp"

#include <algorithm>
#include <cmath>
#include <set>

using namespace evo_engine;

namespace {
bool Finite(const glm::vec3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool ValidBound(const Bound& bound) {
  return Finite(bound.min) && Finite(bound.max) && glm::all(glm::lessThanEqual(bound.min, bound.max));
}

bool SameBound(const Bound& a, const Bound& b) {
  return a.min == b.min && a.max == b.max;
}

SdfgiTextureInput SnapshotTexture(const std::shared_ptr<Material>& material, const uint16_t slot) {
  SdfgiTextureInput result;
  if (slot == 0 || slot >= material->material_data.texture_infos.size())
    return result;
  result.mapping = material->material_data.texture_infos[slot];
  // Bindless allocation indices are not material identity and must not dirty unrelated contributors.
  result.mapping.index = -1;
  if (slot < material->PeekTextureRefs().size())
    result.asset_id = material->PeekTextureRefs()[slot].GetAssetHandle().GetValue();
  result.texture = material->GetTexture(slot);
  if (result.texture) {
    result.version = result.texture->GetVersion();
    result.image = result.texture->GetImage();
    const auto& storage = result.texture->PeekTexture2DStorage();
    result.image_view = storage.image_view;
    result.sampler = storage.sampler;
    if (!TextureStorage::TryGetTexture2DContentSignature(result.texture->GetTextureStorageIndex(),
                                                         result.content_signature))
      result.content_signature = 0;
    result.samples_linear_srgb = result.texture->SamplesLinearSrgb();
  }
  return result;
}
}  // namespace

float evo_engine::SdfgiYMultiplier(const SdfgiSettings::VerticalScale scale) {
  switch (scale) {
    case SdfgiSettings::VerticalScale::Percent50:
      return 2.0f;
    case SdfgiSettings::VerticalScale::Percent75:
      return 1.5f;
    default:
      return 1.0f;
  }
}

uint32_t evo_engine::BoundSdfgiLightList(std::vector<SdfgiLightInput>& lights, const bool dynamic) {
  // Godot pre_process_gi supplies directionals first; stable host identities resolve capacity selection.
  std::sort(lights.begin(), lights.end(), [](const auto& a, const auto& b) {
    return std::tie(a.type, a.id) < std::tie(b.type, b.id);
  });
  const auto count = lights.size();
  lights.resize(std::min(count, dynamic ? size_t(128) : size_t(1024)));
  return static_cast<uint32_t>(count - lights.size());
}

Bound SdfgiCascade::WorldBounds(const float y_mult) const {
  const glm::vec3 scale = cell_size * glm::vec3(1, 1 / y_mult, 1);
  return {glm::vec3(position - size / 2) * scale, glm::vec3(position + size / 2) * scale};
}

std::string evo_engine::UpdateSdfgiCascades(const SdfgiSettings& settings, const glm::vec3& anchor,
                                            std::vector<SdfgiCascade>& cascades) {
  if (const auto failure = settings.Validate(); !failure.empty())
    return failure;
  if (!std::isfinite(std::ldexp(settings.min_cell_size, settings.cascade_count + 6)) ||
      !std::isfinite(1 / settings.min_cell_size))
    return "Cell sizes exceed SDFGI floating-point coordinate range";
  const glm::vec3 world_position = anchor * glm::vec3(1, SdfgiYMultiplier(settings.vertical_scale), 1);
  const glm::vec3 cells = world_position / settings.min_cell_size;
  if (!Finite(cells) || glm::any(glm::greaterThan(glm::abs(glm::dvec3(cells)), glm::dvec3(INT32_MAX - 256))))
    return "Anchor exceeds SDFGI reference integer-grid range";
  if (cascades.empty()) {
    float cell_size = settings.min_cell_size;
    for (uint32_t i = 0; i < settings.cascade_count; ++i) {
      SdfgiCascade cascade;
      cascade.cell_size = cell_size;
      cascade.size = settings.GridSize();
      cascade.position = glm::ivec3(glm::floor(world_position / (cell_size * 8) + glm::vec3(0.5f))) * 8;
      cascades.push_back(cascade);
      cell_size *= 2;
    }
    return {};
  }
  for (auto& cascade : cascades) {
    cascade.dirty_regions = glm::ivec3(0);
    cascade.full_redraw = false;
    // Godot truncates here, unlike the floor(x + 0.5) used at creation.
    const glm::ivec3 cell_position(world_position / cascade.cell_size);
    for (int axis = 0; axis < 3; ++axis) {
      // Algebraically identical to the reference eight-cell while loops, without overflow or long teleport loops.
      const int64_t delta = static_cast<int64_t>(cell_position[axis]) - cascade.position[axis];
      const int64_t shift = std::abs(delta) > 4 ? ((std::abs(delta) - 4 + 7) / 8) * 8 * (delta < 0 ? -1 : 1) : 0;
      cascade.position[axis] = static_cast<int32_t>(cascade.position[axis] + shift);
      if (std::abs(shift) >= cascade.size[axis]) {
        cascade.full_redraw = true;
        break;
      }
      cascade.dirty_regions[axis] = static_cast<int32_t>(-shift);
    }
    if (!cascade.full_redraw) {
      uint32_t safe_volume = 1;
      for (int axis = 0; axis < 3; ++axis)
        safe_volume *= cascade.size[axis] - std::abs(cascade.dirty_regions[axis]);
      cascade.full_redraw = cascade.size.x * cascade.size.y * cascade.size.z - safe_volume > safe_volume / 2;
    }
    if (cascade.full_redraw)
      cascade.dirty_regions = glm::ivec3(0);
  }
  return {};
}

std::vector<SdfgiPendingRegion> evo_engine::GetSdfgiPendingRegions(const std::vector<SdfgiCascade>& cascades,
                                                                   const float y_mult) {
  std::vector<SdfgiPendingRegion> regions;
  for (uint32_t i = 0; i < cascades.size(); ++i) {
    const auto& cascade = cascades[i];
    const auto append = [&](const glm::ivec3& from, const glm::ivec3& to) {
      const glm::vec3 scale = cascade.cell_size * glm::vec3(1, 1 / y_mult, 1);
      const glm::vec3 min = glm::vec3(from - cascade.size / 2 + cascade.position) * scale;
      regions.push_back({i, from, to - from, {min, min + glm::vec3(to - from) * scale}});
    };
    if (cascade.full_redraw) {
      append(glm::ivec3(0), cascade.size);
      continue;
    }
    for (int axis = 0; axis < 3; ++axis) {
      if (cascade.dirty_regions[axis] == 0)
        continue;
      glm::ivec3 from(0), to(cascade.size);
      if (cascade.dirty_regions[axis] > 0)
        to[axis] = cascade.dirty_regions[axis];
      else
        from[axis] += cascade.size[axis] + cascade.dirty_regions[axis];
      for (int previous = 0; previous < axis; ++previous) {
        if (cascade.dirty_regions[previous] > 0)
          from[previous] += cascade.dirty_regions[previous];
        else if (cascade.dirty_regions[previous] < 0)
          to[previous] += cascade.dirty_regions[previous];
      }
      append(from, to);
    }
  }
  return regions;
}

SdfgiCascadeBlock evo_engine::BuildSdfgiCascadeBlock(const std::vector<SdfgiCascade>& cascades) {
  SdfgiCascadeBlock result{};
  for (size_t i = 0; i < cascades.size(); ++i) {
    const auto& cascade = cascades[i];
    auto& data = result.data[i];
    for (int axis = 0; axis < 3; ++axis) {
      data.offset[axis] = (cascade.position[axis] - cascade.size[axis] / 2) * cascade.cell_size;
      data.probe_world_offset[axis] = cascade.position[axis] / 8;
    }
    data.to_cell = 1 / cascade.cell_size;
  }
  return result;
}

bool SdfgiTextureInput::operator==(const SdfgiTextureInput& other) const {
  return asset_id == other.asset_id && version == other.version && content_signature == other.content_signature &&
         image == other.image && image_view == other.image_view && sampler == other.sampler &&
         mapping == other.mapping && samples_linear_srgb == other.samples_linear_srgb;
}

bool SdfgiMaterialInput::SameCoverage(const SdfgiMaterialInput& other) const {
  return masked == other.masked && double_sided == other.double_sided && cull_mode == other.cull_mode &&
         (!masked || (alpha_cutoff == other.alpha_cutoff && base_color.a == other.base_color.a &&
                      base_texture == other.base_texture));
}

bool SdfgiMaterialInput::SamePayload(const SdfgiMaterialInput& other) const {
  return glm::vec3(base_color) == glm::vec3(other.base_color) && emission == other.emission &&
         base_texture == other.base_texture && emission_texture == other.emission_texture;
}

SdfgiMaterialInput evo_engine::SnapshotSdfgiMaterial(const std::shared_ptr<Material>& material) {
  const auto& source = material->material_data.shade_material;
  SdfgiMaterialInput result;
  result.base_color = source.pbr_base_color_factor;
  result.emission = source.emissive_factor;
  result.base_texture = SnapshotTexture(material, source.pbr_base_color_texture);
  if (result.emission != glm::vec3(0))
    result.emission_texture = SnapshotTexture(material, source.emissive_texture);
  result.masked = source.alpha_mode == static_cast<int32_t>(GltfAlphaMode::Mask);
  result.double_sided = source.double_sided != 0;
  result.alpha_cutoff = source.alpha_cutoff;
  result.cull_mode = material->draw_settings.cull_mode;
  return result;
}

const char* evo_engine::GetSdfgiExclusionName(const SdfgiExclusion reason) {
  switch (reason) {
    case SdfgiExclusion::None:
      return "eligible";
    case SdfgiExclusion::Disabled:
      return "disabled";
    case SdfgiExclusion::Dynamic:
      return "dynamic receiver only";
    case SdfgiExclusion::Deforming:
      return "deforming receiver only";
    case SdfgiExclusion::Forward:
      return "forward fallback";
    case SdfgiExclusion::Unsupported:
      return "unsupported contributor";
    case SdfgiExclusion::Empty:
      return "missing or empty mesh/material";
    case SdfgiExclusion::InvalidBounds:
      return "invalid contributor bounds";
  }
  return "unknown";
}

SdfgiSceneSnapshot evo_engine::SnapshotSdfgiScene(const std::shared_ptr<Scene>& scene,
                                                  const ResolvedEnvironmentalLighting& lighting) {
  SdfgiSceneSnapshot result;
  std::map<uint64_t, SdfgiMaterialInput> materials;
  const auto collect_mesh = [&](const Entity transform_owner, const std::shared_ptr<MeshRenderer>& renderer,
                                const bool group_enabled) {
    SdfgiContributor input;
    input.id = {renderer->GetHandle().GetValue(), scene->GetEntityHandle(transform_owner).GetValue()};
    const auto owner = renderer->GetOwner();
    if (!group_enabled || !scene->IsEntityEnabled(transform_owner) || !scene->IsEntityEnabled(owner) ||
        !renderer->IsEnabled())
      input.exclusion = SdfgiExclusion::Disabled;
    else if (!scene->IsEntityStatic(owner) || !scene->IsEntityStatic(transform_owner))
      input.exclusion = SdfgiExclusion::Dynamic;
    else {
      input.mesh = renderer->mesh.Get<Mesh>();
      const auto material = renderer->material.Get<Material>();
      if (!input.mesh || !material || input.mesh->PeekVertices().empty() || input.mesh->PeekTriangles().empty())
        input.exclusion = SdfgiExclusion::Empty;
      else if (!input.mesh->PeekMorphTargets().empty() || !renderer->PeekMorphWeights().empty())
        input.exclusion = SdfgiExclusion::Deforming;
      else if (ClassifyGltfRasterMaterial(material->material_data.shade_material, material->draw_settings.blending) ==
               GltfRasterMaterialClass::Forward)
        input.exclusion = SdfgiExclusion::Forward;
      else if (material->draw_settings.polygon_mode != VK_POLYGON_MODE_FILL)
        input.exclusion = SdfgiExclusion::Unsupported;
      else {
        input.mesh_id = input.mesh->GetHandle().GetValue();
        input.geometry_version = input.mesh->GetVersion();
        input.vertex_count = static_cast<uint32_t>(input.mesh->PeekVertices().size());
        input.triangle_count = static_cast<uint32_t>(input.mesh->PeekTriangles().size());
        input.transform = scene->GetDataComponent<GlobalTransform>(transform_owner).value;
        input.world_bounds = input.mesh->GetBound();
        const bool valid_local_bound = ValidBound(input.world_bounds);
        input.world_bounds.ApplyTransform(input.transform);
        if (!valid_local_bound || !ValidBound(input.world_bounds))
          input.exclusion = SdfgiExclusion::InvalidBounds;
        const auto material_id = material->GetHandle().GetValue();
        auto found = materials.find(material_id);
        if (found == materials.end())
          found = materials.emplace(material_id, SnapshotSdfgiMaterial(material)).first;
        input.material = found->second;
      }
    }
    if (input.exclusion != SdfgiExclusion::None)
      ++result.excluded[input.exclusion];
    result.contributors.push_back(std::move(input));
  };
  std::set<uint64_t> lod_renderers;
  if (const auto owners = scene->UnsafeGetPrivateComponentOwnersList<LodGroup>()) {
    for (const auto owner : *owners) {
      const auto group = scene->GetOrSetPrivateComponent<LodGroup>(owner).lock();
      for (size_t level = 0; level < group->lods.size(); ++level) {
        for (auto& ref : group->lods[level].renderers) {
          if (const auto renderer = ref.Get<MeshRenderer>()) {
            lod_renderers.insert(renderer->GetHandle().GetValue());
            if (level == 0)
              collect_mesh(owner, renderer, group->IsEnabled());
          }
        }
      }
    }
  }
  if (const auto owners = scene->UnsafeGetPrivateComponentOwnersList<MeshRenderer>()) {
    for (const auto owner : *owners) {
      const auto renderer = scene->GetOrSetPrivateComponent<MeshRenderer>(owner).lock();
      if (lod_renderers.count(renderer->GetHandle().GetValue()) == 0)
        collect_mesh(owner, renderer, true);
    }
  }
  const auto count_excluded = [&](auto* component, const SdfgiExclusion reason) {
    using T = std::remove_pointer_t<decltype(component)>;
    if (const auto owners = scene->UnsafeGetPrivateComponentOwnersList<T>()) {
      for (const auto owner : *owners)
        if (scene->IsEntityEnabled(owner) && scene->GetOrSetPrivateComponent<T>(owner).lock()->IsEnabled())
          ++result.excluded[reason];
    }
  };
  count_excluded(static_cast<SkinnedMeshRenderer*>(nullptr), SdfgiExclusion::Deforming);
  count_excluded(static_cast<Particles*>(nullptr), SdfgiExclusion::Forward);
  count_excluded(static_cast<StrandsRenderer*>(nullptr), SdfgiExclusion::Forward);
  count_excluded(static_cast<GaussianSplatRenderer*>(nullptr), SdfgiExclusion::Forward);
  const auto collect_lights = [&](auto* component, const SdfgiLightInput::Type type) {
    using T = std::remove_pointer_t<decltype(component)>;
    if (const auto owners = scene->UnsafeGetPrivateComponentOwnersList<T>()) {
      for (const auto owner : *owners) {
        const auto light = scene->GetOrSetPrivateComponent<T>(owner).lock();
        if (!scene->IsEntityEnabled(owner) || !light->IsEnabled())
          continue;
        SdfgiLightInput input;
        input.id = light->GetHandle().GetValue();
        input.type = type;
        input.dynamic = type == SdfgiLightInput::Type::Directional || !scene->IsEntityStatic(owner);
        input.casts_shadow = light->cast_shadow;
        input.color = light->diffuse * light->diffuse_brightness;
        const auto transform = scene->GetDataComponent<GlobalTransform>(owner);
        input.position = transform.GetPosition();
        input.direction = glm::normalize(transform.GetRotation() * glm::vec3(0, 0, -1));
        if constexpr (!std::is_same_v<T, DirectionalLight>) {
          input.attenuation = {light->constant, light->linear, light->quadratic};
          input.range = light->range > 0 ? light->range : light->GetFarPlane();
        }
        if constexpr (std::is_same_v<T, SpotLight>) {
          input.cos_inner = glm::cos(glm::radians(light->inner_degrees));
          input.cos_outer = glm::cos(glm::radians(light->outer_degrees));
        }
        if constexpr (!std::is_same_v<T, DirectionalLight>) {
          input.world_bounds = {-glm::vec3(input.range), glm::vec3(input.range)};
          if constexpr (std::is_same_v<T, SpotLight>)
            if (input.cos_outer >= 0) {
              const float width = glm::sin(glm::radians(light->outer_degrees)) * input.range;
              input.world_bounds = {{-width, -width, -input.range}, {width, width, 0}};
            }
          input.world_bounds.ApplyTransform(transform.value);
        }
        result.lights.push_back(input);
      }
    }
  };
  collect_lights(static_cast<DirectionalLight*>(nullptr), SdfgiLightInput::Type::Directional);
  collect_lights(static_cast<PointLight*>(nullptr), SdfgiLightInput::Type::Point);
  collect_lights(static_cast<SpotLight*>(nullptr), SdfgiLightInput::Type::Spot);
  std::sort(result.contributors.begin(), result.contributors.end(), [](const auto& a, const auto& b) {
    return a.id < b.id;
  });
  std::sort(result.lights.begin(), result.lights.end(), [](const auto& a, const auto& b) {
    return std::make_pair(a.type, a.id) < std::make_pair(b.type, b.id);
  });
  const auto& source = lighting.indirect_environment_source;
  auto& sky = result.sky;
  sky.constant_color = source.kind == ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::Color;
  sky.color = source.color;
  sky.gamma = source.gamma;
  sky.rotation = source.rotation;
  sky.energy = std::max(lighting.environment_lighting_intensity, 0.0f);
  if (!sky.constant_color) {
    auto map_ref = source.environmental_map;
    const auto map = source.kind == ResolvedEnvironmentalLighting::IndirectEnvironmentSourceKind::EngineDefault
                         ? Resources::GetInstance().GetDefaultEnvironmentalMap()
                         : map_ref.Get<EnvironmentalMap>();
    if (map) {
      sky.map_id = map->GetHandle().GetValue();
      sky.map_version = map->GetVersion();
      sky.cubemap = map->environment_cubemap.Get<Cubemap>();
      if (sky.cubemap)
        sky.cubemap_version = sky.cubemap->GetVersion();
    }
  }
  return result;
}

void SdfgiContributorRegistry::Update(const std::vector<SdfgiContributor>& snapshot) {
  std::map<SdfgiContributorId, SdfgiContributor> next;
  std::set<SdfgiContributorId> invalid;
  changes.clear();
  for (const auto& input : snapshot) {
    if (input.exclusion == SdfgiExclusion::InvalidBounds ||
        (input.exclusion == SdfgiExclusion::None && !ValidBound(input.world_bounds))) {
      invalid.insert(input.id);
      if (!invalid_bounds.count(input.id))
        changes.push_back({input.id, SdfgiUncertainBounds, {}, {}});
    } else if (input.exclusion == SdfgiExclusion::None)
      next.emplace(input.id, input);
  }
  invalid_bounds = std::move(invalid);
  for (const auto& [id, input] : next) {
    const auto old = entries.find(id);
    if (old == entries.end()) {
      changes.push_back({id, SdfgiAdded, {}, input});
      continue;
    }
    const auto& before = old->second;
    uint32_t flags = 0;
    if (input.transform != before.transform || !SameBound(input.world_bounds, before.world_bounds))
      flags |= SdfgiTransformChanged;
    if (input.mesh_id != before.mesh_id || input.geometry_version != before.geometry_version ||
        input.vertex_count != before.vertex_count || input.triangle_count != before.triangle_count)
      flags |= SdfgiGeometryChanged;
    if (!input.material.SameCoverage(before.material))
      flags |= SdfgiCoverageChanged;
    if (!input.material.SamePayload(before.material))
      flags |= SdfgiPayloadChanged;
    if (flags)
      changes.push_back({id, flags, before, input});
  }
  for (const auto& [id, input] : entries)
    if (next.count(id) == 0)
      changes.push_back({id, SdfgiRemoved, input, {}});
  std::sort(changes.begin(), changes.end(), [](const auto& a, const auto& b) {
    return a.id < b.id;
  });
  entries = std::move(next);
}

std::vector<uint32_t> SdfgiContributorRegistry::AffectedCascades(const std::vector<SdfgiCascade>& cascades,
                                                                 const float y_mult) const {
  std::vector<uint32_t> result(cascades.size());
  for (size_t i = 0; i < cascades.size(); ++i) {
    const auto bounds = cascades[i].WorldBounds(y_mult);
    const auto intersects = [&](const std::optional<SdfgiContributor>& contributor) {
      return contributor && (!ValidBound(contributor->world_bounds) ||
                             (glm::all(glm::lessThanEqual(bounds.min, contributor->world_bounds.max)) &&
                              glm::all(glm::greaterThanEqual(bounds.max, contributor->world_bounds.min))));
    };
    for (const auto& change : changes)
      if ((change.flags & SdfgiUncertainBounds) || intersects(change.before) || intersects(change.after))
        result[i] |= change.flags;
  }
  return result;
}
