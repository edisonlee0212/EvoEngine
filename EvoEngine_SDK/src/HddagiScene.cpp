// Godot HDDAGI entering regions and two-cell raster border, da1410fa3516d08cc31b6e86bd6673b9ce776316.
// See docs/licenses/Godot-MIT.txt. Region unions and contributor edits are the host adapter.
#include "HddagiScene.hpp"
#include <algorithm>
#include <cmath>

using namespace evo_engine;

std::string evo_engine::BuildHddagiUpdatePlan(const GiProbeSettings& probes, const HddagiSettings& settings,
                                              const glm::vec3 anchor, const std::vector<SdfgiCascade>& previous,
                                              const std::vector<SdfgiContributorChange>& changes, const bool force_full,
                                              HddagiUpdatePlan& output) {
  if (const auto failure = settings.Validate(probes); !failure.empty())
    return failure;
  SdfgiSettings placement;
  placement.probe_spacing_cells = 8;
  placement = DeriveSdfgiSettings(probes, placement);
  HddagiUpdatePlan plan;
  bool edited_history = false;
  plan.cascades = previous;
  if (const auto failure = UpdateSdfgiCascades(placement, anchor, plan.cascades); !failure.empty())
    return failure;
  const float y_mult = SdfgiYMultiplier(probes.vertical_scale);
  const auto pending = GetSdfgiPendingRegions(plan.cascades, y_mult);
  for (uint32_t cascade = 0; cascade < plan.cascades.size(); ++cascade) {
    const auto& field = plan.cascades[cascade];
    plan.scroll.push_back(previous.empty() ? glm::ivec3(0) : field.position - previous[cascade].position);
    const auto dimensions = field.size / 8;
    const size_t count = dimensions.x * dimensions.y * dimensions.z;
    std::vector<uint8_t> dirty(count, force_full || field.full_redraw ? 1 : 0);
    if (force_full || field.full_redraw)
      plan.reset_history_cascades |= 1u << cascade;
    const auto index = [&](const glm::ivec3 cell) {
      return cell.x + dimensions.x * (cell.y + dimensions.y * cell.z);
    };
    const auto mark = [&](const glm::ivec3 begin, const glm::ivec3 end) {
      for (int z = begin.z; z < end.z; ++z)
        for (int y = begin.y; y < end.y; ++y)
          for (int x = begin.x; x < end.x; ++x)
            dirty[index({x, y, z})] = 1;
    };
    for (const auto& region : pending)
      if (region.cascade == cascade)
        mark(region.offset / 8, (region.offset + region.size) / 8);
    for (const auto& change : changes) {
      if (change.flags & SdfgiUncertainBounds) {
        edited_history = true;
        plan.reset_history_cascades |= 1u << cascade;
        std::fill(dirty.begin(), dirty.end(), 1);
        break;
      }
      for (const auto* contributor : {&change.before, &change.after}) {
        if (!*contributor)
          continue;
        const auto& bounds = contributor->value().world_bounds;
        glm::dvec3 begin, end;
        for (int axis = 0; axis < 3; ++axis) {
          const double scale = (axis == 1 ? y_mult : 1.0) / field.cell_size;
          const double origin = field.position[axis] - field.size[axis] / 2;
          begin[axis] = double(bounds.min[axis]) * scale - origin;
          end[axis] = double(bounds.max[axis]) * scale - origin;
        }
        if (glm::any(glm::lessThan(end, glm::dvec3(0))) ||
            glm::any(glm::greaterThanEqual(begin, glm::dvec3(field.size))))
          continue;
        plan.reset_history_cascades |= 1u << cascade;
        begin = glm::clamp(begin, glm::dvec3(0), glm::dvec3(field.size));
        edited_history = true;
        end = glm::clamp(end, glm::dvec3(0), glm::dvec3(field.size));
        mark(glm::ivec3(glm::floor(begin / 8.0)), glm::min(dimensions, glm::ivec3(glm::floor(end / 8.0)) + 1));
      }
    }
    const auto changed = std::count(dirty.begin(), dirty.end(), uint8_t(1));
    plan.region_count += changed;
    if (changed == count)
      plan.full_cascades |= 1u << cascade;
    const auto box_dirty = [&](const glm::ivec3 begin, const glm::ivec3 end) {
      for (int z = begin.z; z < end.z; ++z)
        for (int y = begin.y; y < end.y; ++y)
          for (int x = begin.x; x < end.x; ++x)
            if (!dirty[index({x, y, z})])
              return false;
      return true;
    };
    const auto region = [&](const glm::ivec3 begin, const glm::ivec3 end, const int border) {
      SdfgiPendingRegion result;
      result.cascade = cascade;
      result.offset = glm::max(begin * 8 - border, glm::ivec3(0));
      result.size = glm::min(end * 8 + border, field.size) - result.offset;
      const glm::vec3 scale = field.cell_size * glm::vec3(1, 1 / y_mult, 1);
      result.world_bounds.min = glm::vec3(field.position - field.size / 2 + result.offset) * scale;
      result.world_bounds.max = result.world_bounds.min + glm::vec3(result.size) * scale;
      return result;
    };
    for (int z = 0; z < dimensions.z; ++z)
      for (int y = 0; y < dimensions.y; ++y)
        for (int x = 0; x < dimensions.x; ++x) {
          const glm::ivec3 begin(x, y, z);
          if (!dirty[index(begin)])
            continue;
          auto end = begin + 1;
          while (end.x < dimensions.x && dirty[index({end.x, y, z})])
            ++end.x;
          while (end.y < dimensions.y && box_dirty({x, end.y, z}, {end.x, end.y + 1, z + 1}))
            ++end.y;
          while (end.z < dimensions.z && box_dirty({x, y, end.z}, {end.x, end.y, end.z + 1}))
            ++end.z;
          plan.regions.push_back({region(begin, end, 0), region(begin, end, 1), region(begin, end, 2)});
          for (int iz = z; iz < end.z; ++iz)
            for (int iy = y; iy < end.y; ++iy)
              for (int ix = x; ix < end.x; ++ix)
                dirty[index({ix, iy, iz})] = 0;
        }
    if (!(plan.full_cascades & (1u << cascade)))
      for (int axis = 0; axis < 3; ++axis) {
        if (plan.scroll[cascade][axis] == 0)
          continue;
        const auto border = [&](const int width) {
          SdfgiPendingRegion result;
          result.cascade = cascade;
          result.size = field.size;
          result.size[axis] = width;
          if (plan.scroll[cascade][axis] < 0)
            result.offset[axis] = field.size[axis] - width;
          const glm::vec3 scale = field.cell_size * glm::vec3(1, 1 / y_mult, 1);
          result.world_bounds.min = glm::vec3(field.position - field.size / 2 + result.offset) * scale;
          result.world_bounds.max = result.world_bounds.min + glm::vec3(result.size) * scale;
          return result;
        };
        SdfgiPendingRegion core;
        core.cascade = cascade;
        core.size = glm::ivec3(0);
        plan.regions.push_back({core, border(1), border(2)});
      }
  }
  // Probe paths can cross cascades, including paths that previously missed the edited geometry.
  if (edited_history)
    plan.reset_history_cascades = (1u << plan.cascades.size()) - 1u;
  output = std::move(plan);
  return {};
}
