#include "TreePhysicsLod.hpp"
#include <algorithm>
#include <cmath>
#include <glm/gtx/quaternion.hpp>
#include <numeric>

using namespace eco_sys_lab_package;
namespace {
using Rest = DynamicTreeSkeleton::GpuRestState;
using Level = TreePhysicsLodLevel;
using Group = std::vector<uint32_t>;
glm::quat Quaternion(const glm::vec4& q) {
  return {q.w, q.x, q.y, q.z};
}
glm::vec4 Vector(const glm::quat& q) {
  return {q.x, q.y, q.z, q.w};
}
glm::vec3 Position(const Rest& rest, uint32_t v) {
  return glm::vec3(rest.vertices[v].position_inverse_mass);
}
glm::quat Frame(const Rest& rest, const Group& group) {
  const auto& first = rest.rods[group.front()];
  const auto q = Quaternion(first.rest_rotation);
  const auto direction =
      glm::normalize(Position(rest, rest.rods[group.back()].links.y) - Position(rest, first.links.x));
  return glm::normalize(glm::rotation(q * glm::vec3(1, 0, 0), direction) * q);
}
float Error(const Rest& rest, const Group& group) {
  if (group.size() == 1)
    return 0;
  const auto a = Position(rest, rest.rods[group.front()].links.x);
  const auto b = Position(rest, rest.rods[group.back()].links.y);
  const auto delta = b - a;
  const float squared = glm::dot(delta, delta);
  if (squared < 1e-12f)
    return INFINITY;
  float error = 0;
  for (auto index : group) {
    const auto& rod = rest.rods[index];
    for (auto vertex : {rod.links.x, rod.links.y}) {
      const auto p = Position(rest, vertex);
      const auto closest = a + glm::clamp(glm::dot(p - a, delta) / squared, 0.0f, 1.0f) * delta;
      error = std::max(error, glm::length(p - closest) / std::max(rod.length_scale_mass_radius.w, 1e-6f));
    }
  }
  return error;
}
float SpanError(const Rest& rest, const Group& group) {
  if (group.size() == 1)
    return 0;
  float radius = INFINITY, original_length = 0;
  for (auto index : group) {
    radius = std::min(radius, rest.rods[index].length_scale_mass_radius.w);
    original_length = std::max(original_length, rest.rods[index].length_scale_mass_radius.x);
  }
  const float length =
      glm::length(Position(rest, rest.rods[group.back()].links.y) - Position(rest, rest.rods[group.front()].links.x));
  const float radial_span = length / (4 * radius);
  const float sampling_span = std::max(0.0f, length / original_length - 1) / 6;
  return std::min(radial_span * radial_span, sampling_span * sampling_span);
}
bool CanMerge(const Rest& rest, const Group& group) {
  const auto a = Position(rest, rest.rods[group.front()].links.x);
  const auto b = Position(rest, rest.rods[group.back()].links.y);
  if (glm::length2(b - a) < 1e-12f)
    return false;
  const auto frame = Frame(rest, group);
  float min_radius = INFINITY, max_radius = 0;
  for (auto index : group) {
    const auto& rod = rest.rods[index];
    const float radius = rod.length_scale_mass_radius.w;
    if (rod.links.w || rod.length_scale_mass_radius.y <= 0 || radius <= 0)
      return false;
    min_radius = std::min(min_radius, radius);
    max_radius = std::max(max_radius, radius);
    if (std::abs(glm::dot(frame, Quaternion(rod.rest_rotation))) < std::cos(glm::radians(5.0f)))
      return false;
  }
  return max_radius <= 1.25f * min_radius && glm::length(b - a) <= 128.0f * min_radius;
}
void Adjacency(Rest& rest) {
  std::vector<std::vector<uint32_t>> vertices(rest.vertices.size()), rods(rest.rods.size());
  for (uint32_t i = 0; i < rest.rods.size(); ++i) {
    vertices[rest.rods[i].links.x].push_back(i);
    vertices[rest.rods[i].links.y].push_back(i);
  }
  for (uint32_t i = 0; i < rest.angles.size(); ++i) {
    rods[rest.angles[i].rods.x].push_back(i);
    rods[rest.angles[i].rods.y].push_back(i);
  }
  for (uint32_t i = 0; i < vertices.size(); ++i) {
    rest.vertices[i].incident_rods = {static_cast<uint32_t>(rest.vertex_rods.size()),
                                      static_cast<uint32_t>(vertices[i].size()), 0, 0};
    rest.vertex_rods.insert(rest.vertex_rods.end(), vertices[i].begin(), vertices[i].end());
  }
  for (uint32_t i = 0; i < rods.size(); ++i) {
    rest.rods[i].incident_angles = {static_cast<uint32_t>(rest.rod_angles.size()),
                                    static_cast<uint32_t>(rods[i].size()), 0, 0};
    rest.rod_angles.insert(rest.rod_angles.end(), rods[i].begin(), rods[i].end());
  }
}
Level MakeLevel(const Rest& source, std::vector<Group> groups) {
  Level level;
  level.groups = std::move(groups);
  level.rest.initial_model = source.initial_model;
  level.rest.density = source.density;
  level.vertices.resize(source.vertices.size());
  level.rods.resize(source.rods.size());
  std::vector<uint32_t> retained(source.vertices.size(), UINT32_MAX);
  const auto retain = [&](uint32_t vertex) {
    if (retained[vertex] != UINT32_MAX)
      return;
    retained[vertex] = static_cast<uint32_t>(level.rest.vertices.size());
    level.original_vertices.push_back(vertex);
    level.rest.vertices.push_back(source.vertices[vertex]);
    level.vertices[vertex].links.y = retained[vertex];
  };
  for (const auto& group : level.groups) {
    retain(source.rods[group.front()].links.x);
    retain(source.rods[group.back()].links.y);
  }
  std::vector<bool> connected(source.vertices.size(), false);
  for (const auto& rod : source.rods)
    connected[rod.links.x] = connected[rod.links.y] = true;
  for (uint32_t i = 0; i < source.vertices.size(); ++i)
    if (!connected[i])
      retain(i);
  for (uint32_t i = 0; i < level.groups.size(); ++i) {
    const auto& group = level.groups[i];
    auto rod = source.rods[group.front()];
    rod.links.x = retained[rod.links.x];
    rod.links.y = retained[source.rods[group.back()].links.y];
    const auto a = glm::vec3(level.rest.vertices[rod.links.x].position_inverse_mass);
    const auto b = glm::vec3(level.rest.vertices[rod.links.y].position_inverse_mass);
    const float length = glm::length(b - a);
    const auto frame = group.size() == 1 ? Quaternion(rod.rest_rotation) : Frame(source, group);
    float arc = 0, compliance = 0, mass = 0, radius_length = 0;
    for (auto index : group) {
      const auto& r = source.rods[index];
      arc += r.length_scale_mass_radius.x;
      compliance += r.length_scale_mass_radius.x / std::max(r.length_scale_mass_radius.y, 1e-20f);
      mass += r.length_scale_mass_radius.z;
      radius_length += r.length_scale_mass_radius.x * r.length_scale_mass_radius.w;
      level.rods[index].links.x = i;
      level.rods[index].relative_rotation =
          group.size() == 1 ? glm::vec4(0, 0, 0, 1) : Vector(glm::inverse(frame) * Quaternion(r.rest_rotation));
    }
    if (group.size() > 1)
      rod.length_scale_mass_radius = {length, length / compliance, mass, radius_length / arc};
    rod.rotation = rod.rest_rotation = Vector(frame);
    level.rest.rods.push_back(rod);
    float distance = 0;
    for (auto index : group) {
      const auto& r = source.rods[index];
      for (auto vertex : {r.links.x, r.links.y}) {
        const float t = (distance + (vertex == r.links.y ? r.length_scale_mass_radius.x : 0)) / arc;
        auto& map = level.vertices[vertex];
        if (map.links.y == UINT32_MAX) {
          map.links.x = i;
          map.offset_t = glm::vec4(glm::inverse(frame) * (Position(source, vertex) - glm::mix(a, b, t)), t);
        }
      }
      distance += r.length_scale_mass_radius.x;
    }
    level.approximation_error = std::max(level.approximation_error, Error(source, group));
    level.span_error = std::max(level.span_error, SpanError(source, group));
  }
  std::vector<float> masses(level.rest.vertices.size(), 0);
  for (uint32_t i = 0; i < source.vertices.size(); ++i) {
    const float inverse_mass = source.vertices[i].position_inverse_mass.w;
    if (inverse_mass <= 0)
      continue;
    const auto& map = level.vertices[i];
    if (map.links.y != UINT32_MAX)
      masses[map.links.y] += 1.0f / inverse_mass;
    else {
      const auto& rod = level.rest.rods[map.links.x];
      masses[rod.links.x] += (1 - map.offset_t.w) / inverse_mass;
      masses[rod.links.y] += map.offset_t.w / inverse_mass;
    }
  }
  for (uint32_t i = 0; i < masses.size(); ++i)
    if (level.rest.vertices[i].position_inverse_mass.w > 0)
      level.rest.vertices[i].position_inverse_mass.w = 1.0f / std::max(masses[i], 1e-20f);
  std::vector<float> internal_compliance(level.groups.size(), 0);
  for (const auto& angle : source.angles) {
    const auto a = level.rods[angle.rods.x].links.x, b = level.rods[angle.rods.y].links.x;
    if (a == b)
      internal_compliance[a] += 1.0f / std::max(angle.bend_scale.x, 1e-20f);
  }
  for (const auto& original : source.angles) {
    const auto a = level.rods[original.rods.x].links.x, b = level.rods[original.rods.y].links.x;
    if (a == b)
      continue;
    auto angle = original;
    const auto& first = level.rest.rods[a];
    const auto& second = level.rest.rods[b];
    angle.rods = {a, b, 0, 0};
    angle.rest_relative = Vector(glm::inverse(Quaternion(first.rest_rotation)) * Quaternion(second.rest_rotation));
    if (level.groups[a].size() > 1 || level.groups[b].size() > 1)
      angle.bend_scale.x =
          original.bend_scale.x > 0
              ? 1.0f / (1.0f / original.bend_scale.x + 0.5f * (internal_compliance[a] + internal_compliance[b]))
              : 0;
    level.rest.angles.push_back(angle);
  }
  level.rest.node_rods = source.node_rods;
  for (auto& index : level.rest.node_rods)
    if (index != UINT32_MAX)
      index = level.rods[index].links.x;
  Adjacency(level.rest);
  return level;
}
}  // namespace

std::vector<TreePhysicsLodLevel> eco_sys_lab_package::BuildTreePhysicsLod(const Rest& original) {
  std::vector<Level> levels;
  if (original.rods.empty())
    return levels;
  std::vector<Group> groups;
  for (uint32_t i = 0; i < original.rods.size(); ++i)
    groups.push_back({i});
  levels.push_back(MakeLevel(original, groups));
  levels.front().rest = original;
  levels.front().original_vertices.resize(original.vertices.size());
  std::iota(levels.front().original_vertices.begin(), levels.front().original_vertices.end(), 0u);
  for (uint32_t i = 0; i < original.vertices.size(); ++i)
    levels.front().vertices[i].links = {UINT32_MAX, i, 0, 0};
  std::vector<bool> bend_joint(original.vertices.size(), false);
  for (const auto& angle : original.angles) {
    const auto& a = original.rods[angle.rods.x];
    const auto& b = original.rods[angle.rods.y];
    if (a.links.y == b.links.x && angle.bend_scale.x > 0)
      bend_joint[a.links.y] = true;
  }
  float budget = 0;
  while (true) {
    float next_budget = INFINITY;
    std::vector<uint32_t> outgoing(original.vertices.size(), UINT32_MAX), degree(original.vertices.size(), 0);
    for (uint32_t i = 0; i < groups.size(); ++i) {
      const auto start = original.rods[groups[i].front()].links.x, end = original.rods[groups[i].back()].links.y;
      outgoing[start] = i;
      ++degree[start];
      ++degree[end];
    }
    std::vector<bool> used(groups.size(), false);
    std::vector<Group> next;
    for (uint32_t i = 0; i < groups.size(); ++i) {
      if (used[i])
        continue;
      Group group = groups[i];
      used[i] = true;
      const auto joint = original.rods[group.back()].links.y;
      const auto child = outgoing[joint];
      if (bend_joint[joint] && degree[joint] == 2 && child != UINT32_MAX && !used[child] &&
          original.vertices[joint].position_inverse_mass.w > 0) {
        Group merged = group;
        merged.insert(merged.end(), groups[child].begin(), groups[child].end());
        if (CanMerge(original, merged)) {
          const float error = std::max(Error(original, merged), SpanError(original, merged));
          if (error <= budget) {
            group = std::move(merged);
            used[child] = true;
          } else
            next_budget = std::min(next_budget, error);
        }
      }
      next.push_back(std::move(group));
    }
    if (next.size() == groups.size()) {
      if (!std::isfinite(next_budget))
        break;
      budget = std::max(next_budget, std::exp2(std::ceil(std::log2(next_budget))));
      continue;
    }
    groups = std::move(next);
    auto level = MakeLevel(original, groups);
    level.approximation_error = std::max(level.approximation_error, levels.back().approximation_error);
    level.span_error = std::max(level.span_error, levels.back().span_error);
    levels.push_back(std::move(level));
  }
  return levels;
}
