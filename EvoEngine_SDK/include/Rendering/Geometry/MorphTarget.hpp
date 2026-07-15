#pragma once

#include "Vertex.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace evo_engine {

struct MorphTarget {
  std::string name;
  std::vector<glm::vec3> position_deltas;
  std::vector<glm::vec3> normal_deltas;
  std::vector<glm::vec3> tangent_deltas;
};

template <typename VertexType>
[[nodiscard]] std::vector<VertexType> BuildMorphedVertices(const std::vector<VertexType>& base_vertices,
                                                           const std::vector<MorphTarget>& morph_targets,
                                                           const std::vector<float>& base_weights,
                                                           const std::vector<float>& weights) {
  auto result = base_vertices;
  bool morph_normals = false;
  bool morph_tangents = false;
  for (size_t target_index = 0; target_index < morph_targets.size(); target_index++) {
    const auto base_weight = target_index < base_weights.size() ? base_weights[target_index] : 0.0f;
    const auto weight = target_index < weights.size() ? weights[target_index] : base_weight;
    const auto delta_weight = weight - base_weight;
    if (delta_weight == 0.0f) {
      continue;
    }
    const auto& target = morph_targets[target_index];
    for (size_t vertex_index = 0; vertex_index < result.size(); vertex_index++) {
      if (vertex_index < target.position_deltas.size()) {
        result[vertex_index].position += target.position_deltas[vertex_index] * delta_weight;
      }
      if (vertex_index < target.normal_deltas.size()) {
        result[vertex_index].normal += target.normal_deltas[vertex_index] * delta_weight;
        morph_normals = true;
      }
      if (vertex_index < target.tangent_deltas.size()) {
        result[vertex_index].tangent += target.tangent_deltas[vertex_index] * delta_weight;
        morph_tangents = true;
      }
    }
  }
  for (auto& vertex : result) {
    const auto normalize = [](const glm::vec3 value) {
      const auto length_squared = glm::dot(value, value);
      return std::isfinite(length_squared) && length_squared > 0.0f ? value * glm::inversesqrt(length_squared) : value;
    };
    if (morph_normals) {
      vertex.normal = normalize(vertex.normal);
    }
    if (morph_tangents) {
      vertex.tangent = normalize(vertex.tangent);
    }
  }
  return result;
}

template <typename VertexType>
[[nodiscard]] bool MorphVertexStreamsMatch(const std::vector<VertexType>& lhs, const std::vector<VertexType>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  const auto matches = [](const glm::vec3& first, const glm::vec3& second) {
    const auto delta = first - second;
    return glm::dot(delta, delta) <= 1.0e-12f;
  };
  for (size_t index = 0; index < lhs.size(); index++) {
    if (!matches(lhs[index].position, rhs[index].position) || !matches(lhs[index].normal, rhs[index].normal) ||
        !matches(lhs[index].tangent, rhs[index].tangent)) {
      return false;
    }
  }
  return true;
}

template <typename VertexType>
[[nodiscard]] std::vector<VertexType> ComposeMorphBaseVertices(const std::vector<VertexType>& evaluated_vertices,
                                                               const std::vector<VertexType>& neutral_vertices,
                                                               const std::vector<MorphTarget>& morph_targets) {
  auto result = evaluated_vertices;
  const auto has_stream = [&](const auto stream) {
    return std::any_of(morph_targets.begin(), morph_targets.end(), [&](const MorphTarget& target) {
      return !(target.*stream).empty();
    });
  };
  const bool positions = has_stream(&MorphTarget::position_deltas);
  const bool normals = has_stream(&MorphTarget::normal_deltas);
  const bool tangents = has_stream(&MorphTarget::tangent_deltas);
  for (size_t index = 0; index < result.size(); index++) {
    if (positions) {
      result[index].position = neutral_vertices.at(index).position;
    }
    if (normals) {
      result[index].normal = neutral_vertices.at(index).normal;
    }
    if (tangents) {
      result[index].tangent = neutral_vertices.at(index).tangent;
    }
  }
  return result;
}

inline void RemapMorphTargets(std::vector<MorphTarget>& morph_targets,
                              const std::vector<uint32_t>& source_vertex_indices) {
  const auto remap = [&](std::vector<glm::vec3>& values) {
    if (values.empty()) {
      return;
    }
    const auto source = values;
    values.resize(source_vertex_indices.size());
    for (size_t index = 0; index < source_vertex_indices.size(); index++) {
      values[index] = source.at(source_vertex_indices[index]);
    }
  };
  for (auto& target : morph_targets) {
    remap(target.position_deltas);
    remap(target.normal_deltas);
    remap(target.tangent_deltas);
  }
}

}  // namespace evo_engine
