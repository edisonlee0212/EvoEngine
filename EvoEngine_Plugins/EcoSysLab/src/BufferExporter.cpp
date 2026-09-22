#include "BufferExporter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace eco_sys_lab_plugin;

namespace {

struct ExactX0Key {
  uint32_t x = 0;
  uint32_t y = 0;
  uint32_t z = 0;

  static ExactX0Key From(const glm::vec3& v) {
    ExactX0Key key;
    std::memcpy(&key.x, &v.x, sizeof(float));
    std::memcpy(&key.y, &v.y, sizeof(float));
    std::memcpy(&key.z, &v.z, sizeof(float));
    return key;
  }

  bool operator==(const ExactX0Key& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct ExactX0KeyHash {
  size_t operator()(const ExactX0Key& key) const {
    size_t h = static_cast<size_t>(key.x);
    h ^= static_cast<size_t>(key.y) + 0x9e3779b9 + (h << 6) + (h >> 2);
    h ^= static_cast<size_t>(key.z) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

class UnionFind {
 public:
  explicit UnionFind(const size_t n) : parent_(n), rank_(n, 0) {
    for (size_t i = 0; i < n; ++i) {
      parent_[i] = i;
    }
  }

  size_t Find(size_t i) {
    while (parent_[i] != i) {
      parent_[i] = parent_[parent_[i]];
      i = parent_[i];
    }
    return i;
  }

  void Unite(size_t a, size_t b) {
    a = Find(a);
    b = Find(b);
    if (a == b) {
      return;
    }
    if (rank_[a] < rank_[b]) {
      std::swap(a, b);
    }
    parent_[b] = a;
    if (rank_[a] == rank_[b]) {
      ++rank_[a];
    }
  }

 private:
  std::vector<size_t> parent_;
  std::vector<size_t> rank_;
};

bool AreSegmentsStillConnected(const int segment_a, const int segment_b,
                               const std::vector<DynamicStrands::GpuSegment>& segments,
                               const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
                               const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  if (segment_a == segment_b) {
    return true;
  }
  if (segment_a < 0 || segment_b < 0 || static_cast<size_t>(segment_a) >= segment_data_list.size() ||
      static_cast<size_t>(segment_b) >= segment_data_list.size() ||
      static_cast<size_t>(segment_a) >= segments.size() || static_cast<size_t>(segment_b) >= segments.size()) {
    return false;
  }

  const auto& data_a = segment_data_list[static_cast<size_t>(segment_a)];
  for (const int pair_handle : data_a.pair_handles) {
    if (pair_handle < 0 || static_cast<size_t>(pair_handle) >= segment_pairs.size()) {
      continue;
    }
    const auto& pair = segment_pairs[static_cast<size_t>(pair_handle)];
    const int other = (pair.segment0_handle == segment_a)   ? pair.segment1_handle
                      : (pair.segment1_handle == segment_a) ? pair.segment0_handle
                                                            : -1;
    if (other != segment_b) {
      continue;
    }

    // Neighbor in the rod-element graph. Strand (prev/next) pairs use connectivity_integrity;
    // lateral bundle pairs use bend_twist_bundle_integrity.
    const auto& seg_a = segments[static_cast<size_t>(segment_a)];
    const auto& seg_b = segments[static_cast<size_t>(segment_b)];
    const bool strand_adjacent = seg_a.prev_handle == segment_b || seg_a.next_handle == segment_b ||
                                 seg_b.prev_handle == segment_a || seg_b.next_handle == segment_a;
    if (strand_adjacent) {
      return pair.connectivity_integrity > 0.f;
    }
    return pair.bend_twist_bundle_integrity > 0.f;
  }
  return false;
}

struct SegmentPairSheetKey {
  int owner_segment_index = -1;
  int segment_pair_index = -1;

  bool operator==(const SegmentPairSheetKey& other) const {
    return owner_segment_index == other.owner_segment_index &&
           segment_pair_index == other.segment_pair_index;
  }
};

struct SegmentPairSheetKeyHash {
  size_t operator()(const SegmentPairSheetKey& key) const {
    size_t h = static_cast<size_t>(key.owner_segment_index + 1);
    h ^= static_cast<size_t>(key.segment_pair_index + 2) + 0x9e3779b9 + (h << 6) + (h >> 2);
    return h;
  }
};

void AddTriangleVertices(std::unordered_set<size_t>& dst,
                         const DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle& triangle) {
  dst.insert(triangle.vertex_index0);
  dst.insert(triangle.vertex_index1);
  dst.insert(triangle.vertex_index2);
}

struct SimilarityTransform3D {
  glm::vec3 center_before{0.f};
  glm::vec3 center_after{0.f};
  glm::mat3 rotation{1.f};
  float scale = 1.f;
  bool translation_only = true;

  glm::vec3 Apply(const glm::vec3& x) const {
    return scale * (rotation * (x - center_before)) + center_after;
  }
};

enum class SimilarityFitMode {
  Empty,
  TranslationOnlyTooFewPoints,
  TranslationOnlyLowVariance,
  TranslationOnlySvdFailed,
  TranslationOnlyScaleOutOfRange,
  TranslationOnlyHighError,
  Similarity,
};

struct SimilarityFitResult {
  SimilarityTransform3D transform;
  SimilarityFitMode mode = SimilarityFitMode::Empty;
};

struct SmoothingPropagationStats {
  size_t sheet_fit_count = 0;
  size_t sheets_skipped_no_matched = 0;
  size_t fallback_count = 0;
  size_t similarity_count = 0;
  size_t identity_rotation_scale_count = 0;
  size_t similarity_identity_rotation_scale_count = 0;
  size_t too_few_points = 0;
  size_t low_variance = 0;
  size_t svd_failed = 0;
  size_t scale_out_of_range = 0;
  size_t high_error = 0;
  size_t unmatched_vertices_updated = 0;
  size_t min_pair_count = std::numeric_limits<size_t>::max();
  size_t max_pair_count = 0;
  size_t sheets_with_at_least_4_matched = 0;
  size_t segment_pair_triangle_count = 0;
  size_t segment_pair_vertex_count = 0;
  size_t segment_pair_matched_vertex_count = 0;
  size_t segment_pair_unmatched_vertex_count = 0;
  size_t segment_pair_sheet_count = 0;
  size_t unmatched_segment_sheet_count = 0;

  void RecordSheetFit(const SimilarityFitResult& fit, const size_t pair_count) {
    ++sheet_fit_count;
    min_pair_count = std::min(min_pair_count, pair_count);
    max_pair_count = std::max(max_pair_count, pair_count);
    if (pair_count >= 4) {
      ++sheets_with_at_least_4_matched;
    }
    if (fit.mode == SimilarityFitMode::Similarity) {
      ++similarity_count;
    } else if (fit.mode != SimilarityFitMode::Empty) {
      ++fallback_count;
      switch (fit.mode) {
        case SimilarityFitMode::TranslationOnlyTooFewPoints:
          ++too_few_points;
          break;
        case SimilarityFitMode::TranslationOnlyLowVariance:
          ++low_variance;
          break;
        case SimilarityFitMode::TranslationOnlySvdFailed:
          ++svd_failed;
          break;
        case SimilarityFitMode::TranslationOnlyScaleOutOfRange:
          ++scale_out_of_range;
          break;
        case SimilarityFitMode::TranslationOnlyHighError:
          ++high_error;
          break;
        default:
          break;
      }
    }

    if (IsNearIdentityRotationAndScale(fit.transform)) {
      ++identity_rotation_scale_count;
      if (fit.mode == SimilarityFitMode::Similarity) {
        ++similarity_identity_rotation_scale_count;
      }
    }
  }

  void LogSummary() const {
    if (sheet_fit_count == 0 && sheets_skipped_no_matched == 0) {
      return;
    }
    std::cout << "ApplySmoothing propagation: " << sheet_fit_count << " sheet fit(s), " << sheets_skipped_no_matched
              << " sheet(s) skipped (no matched control points), " << fallback_count
              << " fallback (translation-only), " << similarity_count << " similarity fit(s), "
              << identity_rotation_scale_count << " with identity rotation and scale=1";
    if (similarity_count > 0) {
      std::cout << " (" << similarity_identity_rotation_scale_count
                << " of similarity fits are numerically identity)";
    }
    std::cout << ", " << unmatched_vertices_updated << " unmatched vertex update(s)";
    if (sheet_fit_count > 0) {
      std::cout << ", matched-pair range [" << min_pair_count << ", " << max_pair_count << "], "
                << sheets_with_at_least_4_matched << " sheet(s) with >= 4 matched point(s)";
    }
    std::cout << std::endl;
    std::cout << "  segment-pair sheets: " << segment_pair_sheet_count << ", unmatched segment sheets: "
              << unmatched_segment_sheet_count << ", segment-pair triangles=" << segment_pair_triangle_count
              << ", vertices=" << segment_pair_vertex_count << " (matched=" << segment_pair_matched_vertex_count
              << ", unmatched=" << segment_pair_unmatched_vertex_count << ")" << std::endl;
    if (fallback_count > 0) {
      std::cout << "  fallback reasons: too_few_points=" << too_few_points << ", low_variance=" << low_variance
                << ", svd_failed=" << svd_failed << ", scale_out_of_range=" << scale_out_of_range
                << ", high_error=" << high_error << std::endl;
    }
  }

 private:
  static bool IsNearIdentityRotation(const glm::mat3& rotation, const float epsilon = 1e-4f) {
    const glm::mat3 identity(1.f);
    for (int column = 0; column < 3; ++column) {
      for (int row = 0; row < 3; ++row) {
        if (std::abs(rotation[column][row] - identity[column][row]) > epsilon) {
          return false;
        }
      }
    }
    return true;
  }

  static bool IsNearIdentityRotationAndScale(const SimilarityTransform3D& transform, const float rotation_epsilon = 1e-4f,
                                             const float scale_epsilon = 1e-4f) {
    return std::abs(transform.scale - 1.f) <= scale_epsilon && IsNearIdentityRotation(transform.rotation, rotation_epsilon);
  }
};

SimilarityTransform3D MakeTranslationOnlyTransform(const glm::vec3& center_before, const glm::vec3& center_after) {
  SimilarityTransform3D transform;
  transform.center_before = center_before;
  transform.center_after = center_after;
  transform.rotation = glm::mat3(1.f);
  transform.scale = 1.f;
  transform.translation_only = true;
  return transform;
}

bool SymmetricEigen3x3(glm::dmat3& matrix, glm::dmat3& eigenvectors, glm::dvec3& eigenvalues) {
  eigenvectors = glm::dmat3(1.0);
  for (int sweep = 0; sweep < 32; ++sweep) {
    int p = 0;
    int q = 1;
    double max_off_diagonal = std::abs(matrix[1][0]);
    if (std::abs(matrix[2][0]) > max_off_diagonal) {
      p = 0;
      q = 2;
      max_off_diagonal = std::abs(matrix[2][0]);
    }
    if (std::abs(matrix[2][1]) > max_off_diagonal) {
      p = 1;
      q = 2;
      max_off_diagonal = std::abs(matrix[2][1]);
    }
    if (max_off_diagonal < 1e-15) {
      break;
    }

    const double app = matrix[p][p];
    const double aqq = matrix[q][q];
    const double apq = matrix[q][p];
    const double tau = (aqq - app) / (2.0 * apq);
    const double t = (tau >= 0.0 ? 1.0 : -1.0) / (std::abs(tau) + std::sqrt(1.0 + tau * tau));
    const double c = 1.0 / std::sqrt(1.0 + t * t);
    const double s = t * c;

    matrix[p][p] = app - t * apq;
    matrix[q][q] = aqq + t * apq;
    matrix[q][p] = 0.0;
    matrix[p][q] = 0.0;

    for (int k = 0; k < 3; ++k) {
      if (k == p || k == q) {
        continue;
      }
      const double akp = matrix[p][k];
      const double akq = matrix[q][k];
      matrix[p][k] = c * akp - s * akq;
      matrix[q][k] = s * akp + c * akq;
      matrix[k][p] = matrix[p][k];
      matrix[k][q] = matrix[q][k];
    }

    for (int k = 0; k < 3; ++k) {
      const double vip = eigenvectors[p][k];
      const double viq = eigenvectors[q][k];
      eigenvectors[p][k] = c * vip - s * viq;
      eigenvectors[q][k] = s * vip + c * viq;
    }
  }

  eigenvalues = glm::dvec3(matrix[0][0], matrix[1][1], matrix[2][2]);
  return std::isfinite(eigenvalues.x) && std::isfinite(eigenvalues.y) && std::isfinite(eigenvalues.z);
}

bool Svd3x3(const glm::dmat3& matrix, glm::dmat3& u, glm::dvec3& singular_values, glm::dmat3& v) {
  const glm::dmat3 normal_matrix = glm::transpose(matrix) * matrix;
  glm::dmat3 v_candidate = glm::dmat3(1.0);
  glm::dvec3 eigenvalues{0.0};
  glm::dmat3 normal_copy = normal_matrix;
  if (!SymmetricEigen3x3(normal_copy, v_candidate, eigenvalues)) {
    return false;
  }

  singular_values = glm::dvec3(std::sqrt(std::max(0.0, eigenvalues.x)), std::sqrt(std::max(0.0, eigenvalues.y)),
                               std::sqrt(std::max(0.0, eigenvalues.z)));
  v = v_candidate;

  u = glm::dmat3(1.0);
  for (int i = 0; i < 3; ++i) {
    const double sigma = singular_values[i];
    if (sigma < 1e-12) {
      continue;
    }
    const glm::dvec3 column = matrix * glm::dvec3(v[0][i], v[1][i], v[2][i]) / sigma;
    u[0][i] = column.x;
    u[1][i] = column.y;
    u[2][i] = column.z;
  }

  for (int i = 0; i < 3; ++i) {
    if (singular_values[i] >= 1e-12) {
      continue;
    }
    glm::dvec3 fallback{0.0, 0.0, 0.0};
    fallback[i] = 1.0;
    for (int j = 0; j < i; ++j) {
      if (singular_values[j] < 1e-12) {
        continue;
      }
      const glm::dvec3 uj(u[0][j], u[1][j], u[2][j]);
      fallback -= glm::dot(fallback, uj) * uj;
    }
    const double length = glm::length(fallback);
    if (length < 1e-12) {
      fallback = glm::dvec3(1.0, 0.0, 0.0);
      for (int j = 0; j < i; ++j) {
        if (singular_values[j] < 1e-12) {
          continue;
        }
        const glm::dvec3 uj(u[0][j], u[1][j], u[2][j]);
        fallback -= glm::dot(fallback, uj) * uj;
      }
    }
    const glm::dvec3 column = glm::normalize(fallback);
    u[0][i] = column.x;
    u[1][i] = column.y;
    u[2][i] = column.z;
  }

  return std::isfinite(singular_values.x) && std::isfinite(singular_values.y) && std::isfinite(singular_values.z);
}

SimilarityFitResult FitSimilarityTransform(const std::vector<std::pair<glm::vec3, glm::vec3>>& pairs) {
  SimilarityFitResult result;
  if (pairs.empty()) {
    return result;
  }

  glm::dvec3 mean_before{0.0};
  glm::dvec3 mean_after{0.0};
  for (const auto& [before, after] : pairs) {
    mean_before += glm::dvec3(before);
    mean_after += glm::dvec3(after);
  }
  mean_before /= static_cast<double>(pairs.size());
  mean_after /= static_cast<double>(pairs.size());
  result.transform.center_before = glm::vec3(mean_before);
  result.transform.center_after = glm::vec3(mean_after);

  if (pairs.size() < 3) {
    result.transform = MakeTranslationOnlyTransform(result.transform.center_before, result.transform.center_after);
    result.mode = SimilarityFitMode::TranslationOnlyTooFewPoints;
    return result;
  }

  glm::dmat3 covariance{0.0};
  double source_variance = 0.0;
  glm::dvec3 min_before{std::numeric_limits<double>::max()};
  glm::dvec3 max_before{std::numeric_limits<double>::lowest()};
  for (const auto& [before, after] : pairs) {
    const glm::dvec3 centered_before = glm::dvec3(before) - mean_before;
    const glm::dvec3 centered_after = glm::dvec3(after) - mean_after;
    covariance += glm::transpose(glm::outerProduct(centered_after, centered_before));
    source_variance += glm::dot(centered_before, centered_before);
    min_before = glm::min(min_before, glm::dvec3(before));
    max_before = glm::max(max_before, glm::dvec3(before));
  }

  if (source_variance < 1e-12) {
    result.transform = MakeTranslationOnlyTransform(result.transform.center_before, result.transform.center_after);
    result.mode = SimilarityFitMode::TranslationOnlyLowVariance;
    return result;
  }

  glm::dmat3 u;
  glm::dmat3 v;
  glm::dvec3 singular_values;
  if (!Svd3x3(covariance, u, singular_values, v)) {
    result.transform = MakeTranslationOnlyTransform(result.transform.center_before, result.transform.center_after);
    result.mode = SimilarityFitMode::TranslationOnlySvdFailed;
    return result;
  }

  const glm::dmat3 v_transpose = glm::transpose(v);
  const double reflection_sign = glm::determinant(u * v_transpose) < 0.0 ? -1.0 : 1.0;
  glm::dmat3 correction(1.0);
  correction[2][2] = reflection_sign;
  const glm::dmat3 rotation = u * correction * v_transpose;
  const double scale = (singular_values.x + singular_values.y + reflection_sign * singular_values.z) / source_variance;
  if (!std::isfinite(scale) || scale < 0.25 || scale > 4.0) {
    result.transform = MakeTranslationOnlyTransform(result.transform.center_before, result.transform.center_after);
    result.mode = SimilarityFitMode::TranslationOnlyScaleOutOfRange;
    return result;
  }

  result.transform.rotation = glm::mat3(rotation);
  result.transform.scale = static_cast<float>(scale);
  result.transform.translation_only = false;

  const glm::dvec3 extent = max_before - min_before;
  const double reference_extent = std::max({extent.x, extent.y, extent.z, 1e-6});
  const double max_error_tolerance = 0.05 * reference_extent;
  const double max_error_tolerance_sq = max_error_tolerance * max_error_tolerance;
  for (const auto& [before, after] : pairs) {
    const glm::vec3 predicted = result.transform.Apply(before);
    const glm::vec3 error = predicted - after;
    if (static_cast<double>(glm::dot(error, error)) > max_error_tolerance_sq) {
      result.transform = MakeTranslationOnlyTransform(result.transform.center_before, result.transform.center_after);
      result.mode = SimilarityFitMode::TranslationOnlyHighError;
      return result;
    }
  }

  result.mode = SimilarityFitMode::Similarity;
  return result;
}

std::vector<std::pair<glm::vec3, glm::vec3>> BuildSeamPairsFromVertices(
    const std::unordered_set<size_t>& vertex_indices, const std::vector<glm::vec3>& x_before,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices) {
  std::vector<std::pair<glm::vec3, glm::vec3>> pairs;
  pairs.reserve(vertex_indices.size());
  for (const size_t vertex_index : vertex_indices) {
    pairs.emplace_back(x_before[vertex_index], vertices[vertex_index].x);
  }
  return pairs;
}

std::unordered_set<size_t> CollectMatchedVertices(const std::unordered_set<size_t>& vertex_indices,
                                                  const std::vector<uint8_t>& was_smoothed) {
  std::unordered_set<size_t> matched_vertices;
  for (const size_t vertex_index : vertex_indices) {
    if (vertex_index < was_smoothed.size() && was_smoothed[vertex_index]) {
      matched_vertices.insert(vertex_index);
    }
  }
  return matched_vertices;
}

std::optional<SimilarityFitResult> FitSheetTransform(const std::unordered_set<size_t>& matched_vertices,
                                                     const std::vector<glm::vec3>& x_before,
                                                     const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                                                     SmoothingPropagationStats& stats) {
  if (matched_vertices.empty()) {
    ++stats.sheets_skipped_no_matched;
    return std::nullopt;
  }
  const std::vector<std::pair<glm::vec3, glm::vec3>> pairs =
      BuildSeamPairsFromVertices(matched_vertices, x_before, vertices);
  const SimilarityFitResult fit = FitSimilarityTransform(pairs);
  stats.RecordSheetFit(fit, pairs.size());
  return fit;
}

size_t ApplyTransformToUnmatchedVertices(std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                                         const std::vector<glm::vec3>& x_before,
                                         const std::vector<uint8_t>& was_smoothed,
                                         const std::unordered_set<size_t>& vertex_indices,
                                         const SimilarityTransform3D& transform) {
  size_t updated_count = 0;
  for (const size_t vertex_index : vertex_indices) {
    if (vertex_index >= was_smoothed.size() || was_smoothed[vertex_index]) {
      continue;
    }
    vertices[vertex_index].x = transform.Apply(x_before[vertex_index]);
    ++updated_count;
  }
  return updated_count;
}

size_t ApplyNearestTransformToUnmatchedVertices(
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices, const std::vector<glm::vec3>& x_before,
    const std::vector<uint8_t>& was_smoothed, const std::unordered_set<size_t>& vertex_indices,
    const std::vector<SimilarityTransform3D>& transforms) {
  if (transforms.empty()) {
    return 0;
  }

  size_t updated_count = 0;
  for (const size_t vertex_index : vertex_indices) {
    if (vertex_index >= was_smoothed.size() || was_smoothed[vertex_index]) {
      continue;
    }

    const glm::vec3 query = x_before[vertex_index];
    float best_distance_sq = std::numeric_limits<float>::max();
    const SimilarityTransform3D* best_transform = &transforms.front();
    for (const SimilarityTransform3D& transform : transforms) {
      const glm::vec3 delta = query - transform.center_before;
      const float distance_sq = glm::dot(delta, delta);
      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best_transform = &transform;
      }
    }
    vertices[vertex_index].x = best_transform->Apply(query);
    ++updated_count;
  }
  return updated_count;
}

void PropagateSmoothingToUnmatchedVertices(
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<glm::vec3>& x_before,
    const std::vector<uint8_t>& was_smoothed,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles) {
  SmoothingPropagationStats stats;

  struct SegmentPairSheetGroup {
    size_t triangle_count = 0;
    std::unordered_set<size_t> vertices;
  };

  std::unordered_map<SegmentPairSheetKey, SegmentPairSheetGroup, SegmentPairSheetKeyHash> segment_pair_sheet_groups;
  std::unordered_map<int, std::unordered_set<size_t>> unmatched_segment_vertices;
  std::unordered_map<int, std::vector<SimilarityTransform3D>> pair_sheet_transforms_by_owner;

  for (const auto& triangle : triangles) {
    if (triangle.segment_pair_index >= 0) {
      const SegmentPairSheetKey sheet_key{static_cast<int>(vertices[triangle.vertex_index0].segment_index),
                                          triangle.segment_pair_index};
      SegmentPairSheetGroup& group = segment_pair_sheet_groups[sheet_key];
      ++group.triangle_count;
      AddTriangleVertices(group.vertices, triangle);
      continue;
    }

    const int owner_segment_index = static_cast<int>(vertices[triangle.vertex_index0].segment_index);
    AddTriangleVertices(unmatched_segment_vertices[owner_segment_index], triangle);
  }

  for (const auto& [sheet_key, group] : segment_pair_sheet_groups) {
    ++stats.segment_pair_sheet_count;
    stats.segment_pair_triangle_count += group.triangle_count;
    stats.segment_pair_vertex_count += group.vertices.size();

    const std::unordered_set<size_t> matched_vertices = CollectMatchedVertices(group.vertices, was_smoothed);
    stats.segment_pair_matched_vertex_count += matched_vertices.size();
    stats.segment_pair_unmatched_vertex_count += group.vertices.size() - matched_vertices.size();

    const std::optional<SimilarityFitResult> fit = FitSheetTransform(matched_vertices, x_before, vertices, stats);
    if (!fit.has_value()) {
      continue;
    }

    pair_sheet_transforms_by_owner[sheet_key.owner_segment_index].push_back(fit->transform);
    stats.unmatched_vertices_updated +=
        ApplyTransformToUnmatchedVertices(vertices, x_before, was_smoothed, group.vertices, fit->transform);
  }

  for (const auto& [owner_segment_index, segment_vertices] : unmatched_segment_vertices) {
    ++stats.unmatched_segment_sheet_count;

    std::vector<SimilarityTransform3D> transforms =
        pair_sheet_transforms_by_owner[owner_segment_index];
    const std::unordered_set<size_t> matched_vertices = CollectMatchedVertices(segment_vertices, was_smoothed);
    if (const std::optional<SimilarityFitResult> fit = FitSheetTransform(matched_vertices, x_before, vertices, stats);
        fit.has_value()) {
      transforms.push_back(fit->transform);
    }

    stats.unmatched_vertices_updated += ApplyNearestTransformToUnmatchedVertices(
        vertices, x_before, was_smoothed, segment_vertices, transforms);
  }

  stats.LogSummary();
}

constexpr int kNeighborMatGrey = 0;
constexpr int kNeighborMatBrown = 1;
constexpr int kNeighborMatRed = 2;
constexpr int kNeighborMatGreen = 3;

bool NeighborConnectivityDebugEnabled() {
  return DsKineticVoronoiMeshing::render_settings.segment_meshlet_render_parameters.debug_neighbor_connectivity;
}

int NeighborConnectivityMaterialId(const DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle& triangle,
                                   const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs) {
  if (triangle.neighbor_segment_index == -2) {
    return kNeighborMatBrown;
  }
  if (triangle.neighbor_segment_index == -1 || triangle.neighbor_segment_index == -3) {
    return kNeighborMatGrey;
  }
  if (triangle.segment_pair_index < 0 ||
      static_cast<size_t>(triangle.segment_pair_index) >= segment_pairs.size()) {
    return kNeighborMatRed;
  }
  const auto& pair = segment_pairs[static_cast<size_t>(triangle.segment_pair_index)];
  if (pair.connectivity_integrity <= 0.f && pair.bend_twist_bundle_integrity <= 0.f) {
    return kNeighborMatRed;
  }
  return kNeighborMatGreen;
}

void WriteNeighborConnectivityDebugMtl(const std::filesystem::path& mtl_path) {
  std::ofstream file(mtl_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open neighbor-connectivity debug MTL file");
  }
  file << "newmtl debug_grey\n";
  file << "Ka 0.55 0.55 0.55\n";
  file << "Kd 0.55 0.55 0.55\n";
  file << "Ks 0.0 0.0 0.0\n";
  file << "d 1.0\n\n";
  file << "newmtl debug_brown\n";
  file << "Ka 0.45 0.28 0.12\n";
  file << "Kd 0.45 0.28 0.12\n";
  file << "Ks 0.0 0.0 0.0\n";
  file << "d 1.0\n\n";
  file << "newmtl debug_red\n";
  file << "Ka 1.0 0.0 0.0\n";
  file << "Kd 1.0 0.0 0.0\n";
  file << "Ks 0.0 0.0 0.0\n";
  file << "d 1.0\n\n";
  file << "newmtl debug_green\n";
  file << "Ka 0.0 1.0 0.0\n";
  file << "Kd 0.0 1.0 0.0\n";
  file << "Ks 0.0 0.0 0.0\n";
  file << "d 1.0\n";
}

void WriteExportedMesh(const std::filesystem::path& path, kinDS::VoronoiMesh mesh, kinDS::ObjWriteOptions options,
                       const bool neighbor_connectivity_debug) {
  options.framework_compatible = !neighbor_connectivity_debug;
  kinDS::ObjExporter::writeMesh(mesh, path, options);
  if (neighbor_connectivity_debug) {
    std::filesystem::path mtl_path = path;
    mtl_path.replace_extension(".mtl");
    WriteNeighborConnectivityDebugMtl(mtl_path);
  }
}

}  // namespace

bool MeshletObjExport::enable_smoothing = false;
bool MeshletObjExport::per_meshlet_objects = false;
MeshletObjExport::VisualizationColorMode MeshletObjExport::visualization_color_mode =
    MeshletObjExport::VisualizationColorMode::Segments;
MeshletObjExport::VisualizationObjectGrouping MeshletObjExport::visualization_object_grouping =
    MeshletObjExport::VisualizationObjectGrouping::Combined;
MeshletObjExport::VisualizationObjectGrouping MeshletObjExport::intersection_visualization_object_grouping =
    MeshletObjExport::VisualizationObjectGrouping::IntersectionMeshes;

std::vector<MeshletObjExport::MeshGroup> BuildMeshGroupsBySegmentIndex(
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles) {
  using Triangle = DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle;

  std::unordered_map<unsigned int, std::vector<size_t>> triangle_indices_by_segment;
  triangle_indices_by_segment.reserve(triangles.size() / 4 + 1);
  for (size_t tri_index = 0; tri_index < triangles.size(); ++tri_index) {
    const unsigned int segment_index = vertices[triangles[tri_index].vertex_index0].segment_index;
    triangle_indices_by_segment[segment_index].push_back(tri_index);
  }

  std::vector<unsigned int> segment_indices;
  segment_indices.reserve(triangle_indices_by_segment.size());
  for (const auto& [segment_index, _] : triangle_indices_by_segment) {
    segment_indices.push_back(segment_index);
  }
  std::sort(segment_indices.begin(), segment_indices.end());

  std::vector<MeshletObjExport::MeshGroup> groups;
  groups.reserve(segment_indices.size());
  for (const unsigned int segment_index : segment_indices) {
    const auto& tri_indices = triangle_indices_by_segment[segment_index];
    MeshletObjExport::MeshGroup group;
    group.name = "meshlet_" + std::to_string(segment_index);

    std::unordered_map<unsigned int, unsigned int> vertex_remap;
    vertex_remap.reserve(tri_indices.size() * 2);
    group.vertices.reserve(tri_indices.size());
    group.triangles.reserve(tri_indices.size());

    const auto remap_vertex = [&](const unsigned int old_index) -> unsigned int {
      const auto found = vertex_remap.find(old_index);
      if (found != vertex_remap.end()) {
        return found->second;
      }
      const unsigned int new_index = static_cast<unsigned int>(group.vertices.size());
      group.vertices.push_back(vertices[old_index]);
      vertex_remap.emplace(old_index, new_index);
      return new_index;
    };

    for (const size_t tri_index : tri_indices) {
      const Triangle& src = triangles[tri_index];
      Triangle dst = src;
      dst.vertex_index0 = remap_vertex(src.vertex_index0);
      dst.vertex_index1 = remap_vertex(src.vertex_index1);
      dst.vertex_index2 = remap_vertex(src.vertex_index2);
      group.triangles.push_back(dst);
    }
    groups.push_back(std::move(group));
  }
  return groups;
}

void WriteCombinedMeshGroups(const std::filesystem::path& path, const std::vector<MeshletObjExport::MeshGroup>& groups,
                             const std::vector<DynamicStrands::GpuSegment>& segments, const double uv_height_factor,
                             const double uv_circum_factor, const float fracture_distance,
                             const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
                             const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list,
                             const bool smooth_each_group) {
  if (groups.empty()) {
    throw std::runtime_error("WriteCombinedMeshGroups: no mesh groups to export");
  }

  const bool neighbor_connectivity_debug = NeighborConnectivityDebugEnabled();

  kinDS::VoronoiMesh combined;
  kinDS::ObjExportGpuAttributes combined_attrs;
  bool initialized = false;

  for (const auto& group : groups) {
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex> export_vertices = group.vertices;
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle> export_triangles = group.triangles;
    if (smooth_each_group && MeshletObjExport::enable_smoothing) {
      MeshletObjExport::ApplySmoothing(export_vertices, export_triangles, segments, segment_pairs, segment_data_list);
    }

    kinDS::VoronoiMesh part = MeshletObjExport::ToVoronoiMesh(export_vertices, export_triangles, fracture_distance,
                                                              neighbor_connectivity_debug, segment_pairs);
    kinDS::ObjExportGpuAttributes part_attrs =
        MeshletObjExport::BuildGpuAttributes(export_vertices, export_triangles, segments, uv_height_factor);

    if (!initialized) {
      combined = std::move(part);
      combined.setGroupOffsets({0});
      combined.setGroupNames({group.name});
      combined_attrs = std::move(part_attrs);
      initialized = true;
    } else {
      combined.startNewGroup(group.name);
      combined += part;
      MeshletObjExport::AppendGpuAttributes(combined_attrs, part_attrs);
    }
  }

  kinDS::ObjWriteOptions options;
  options.uv_height_factor = uv_height_factor;
  options.uv_circum_factor = uv_circum_factor;
  options.write_obj_groups = true;
  options.gpu_attributes = std::move(combined_attrs);
  WriteExportedMesh(path, std::move(combined), options, neighbor_connectivity_debug);
}

void PlyExporter::ExportAscii(const std::filesystem::path& path,
                              const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                              const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                              double uv_height_factor, double uv_circum_factor) {
  std::ofstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open PLY file for writing");
  }

  WriteHeader(file, vertices.size(), triangles.size());
  WriteVertices(file, vertices);
  WriteFaces(file, triangles, uv_height_factor, uv_circum_factor);

  file.close();
}

void PlyExporter::WriteHeader(std::ofstream& file, size_t vertex_count, size_t face_count) {
  file << "ply\n";
  file << "format ascii 1.0\n";

  // Material convention
  file << "comment material 0 bark\n";
  file << "comment material 1 interior\n";

  // Vertices
  file << "element vertex " << vertex_count << "\n";
  file << "property float x\n";
  file << "property float y\n";
  file << "property float z\n";

  // Faces
  file << "element face " << face_count << "\n";
  file << "property list uchar int vertex_indices\n";
  file << "property list uchar float corner_normals\n";
  file << "property list uchar float corner_uvs\n";
  file << "property int material_id\n";

  file << "end_header\n";
}

void PlyExporter::WriteVertices(std::ofstream& file,
                                const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices) {
  for (const auto& v : vertices) {
    file << v.x.x << " " << v.x.y << " " << v.x.z << "\n";
  }
}

void PlyExporter::WriteFaces(std::ofstream& file,
                             const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                             double uv_height_factor, double uv_circum_factor) {
  for (const auto& t : triangles) {
    const int material_id = (t.neighbor_segment_index == -2) ? 0 : 1;

    // Vertex indices
    file << "3 " << t.vertex_index0 << " " << t.vertex_index1 << " " << t.vertex_index2 << " ";

    // Corner normals (3 * vec3)
    file << "9 ";
    for (int i = 0; i < 3; ++i) {
      file << t.normal[i].x << " " << t.normal[i].y << " " << t.normal[i].z << " ";
    }

    // Corner UVs (3 * vec3)
    file << "6 ";
    for (int i = 0; i < 3; ++i) {
      glm::vec4 uv = t.uv[i];

      if (material_id == 0) {
        uv.x *= uv_circum_factor;
        uv.y *= uv_height_factor;
      } else {
        uv.z *= uv_height_factor;
      }

      file << uv.x << " " << uv.y << " " << uv.z << " ";
    }

    // Material
    file << material_id << "\n";
  }
}

kinDS::VoronoiMesh MeshletObjExport::ToVoronoiMesh(
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles, const float fracture_distance,
    const bool neighbor_connectivity_debug, const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs) {
  kinDS::VoronoiMesh mesh(neighbor_connectivity_debug ? std::vector<std::string>{"debug_grey", "debug_brown", "debug_red",
                                                                                "debug_green"}
                                                      : std::vector<std::string>{"bark", "interior"},
                          kinDS::PerTriangleCorner);

  for (const auto& v : vertices) {
    const glm::vec3 p = v.x - fracture_distance * v.shift;
    mesh.addVertex(glm::dvec3(p.x, p.y, p.z));
  }

  for (const auto& t : triangles) {
    const int material_id =
        neighbor_connectivity_debug
            ? NeighborConnectivityMaterialId(t, segment_pairs)
            : ((t.neighbor_segment_index == -2) ? 0 : 1);
    const size_t uv0 = mesh.addUV(glm::dvec3(t.uv[0].x, t.uv[0].y, t.uv[0].z));
    const size_t uv1 = mesh.addUV(glm::dvec3(t.uv[1].x, t.uv[1].y, t.uv[1].z));
    const size_t uv2 = mesh.addUV(glm::dvec3(t.uv[2].x, t.uv[2].y, t.uv[2].z));
    mesh.addTriangle(t.vertex_index0, t.vertex_index1, t.vertex_index2, uv0, uv1, uv2, material_id);
    for (int i = 0; i < 3; ++i) {
      mesh.addNormal(glm::dvec3(t.normal[i].x, t.normal[i].y, t.normal[i].z));
    }
  }

  return mesh;
}

kinDS::ObjExportGpuAttributes MeshletObjExport::BuildGpuAttributes(
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
    const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor) {
  kinDS::ObjExportGpuAttributes attrs;
  const size_t n = vertices.size();
  attrs.color.resize(n);
  attrs.boundary_distance.resize(n);
  attrs.profile_position.resize(n);
  attrs.profile_polar_coordinate.resize(n);
  attrs.HC.resize(n);
  attrs.HL.resize(n);
  attrs.RW.resize(n);
  attrs.RB.resize(n);
  attrs.moisture.resize(n);
  attrs.position0.resize(n);
  attrs.direction0.resize(n);
  attrs.root_distance.resize(n);
  attrs.uv_3.assign(n, 0.0);
  attrs.has_neighbor.resize(triangles.size());

  for (size_t i = 0; i < n; ++i) {
    const unsigned int segment_index = vertices[i].segment_index;
    if (segment_index >= segments.size()) {
      continue;
    }
    const auto& seg = segments[segment_index];
    attrs.color[i] = glm::dvec4(seg.color.x, seg.color.y, seg.color.z, seg.color.w);
    attrs.boundary_distance[i] = seg.boundary_distance;
    attrs.profile_position[i] = glm::dvec2(seg.profile_position.x, seg.profile_position.y);
    attrs.profile_polar_coordinate[i] = glm::dvec2(seg.profile_polar_coordinate.x, seg.profile_polar_coordinate.y);
    attrs.HC[i] = seg.HC;
    attrs.HL[i] = seg.HL;
    attrs.RW[i] = seg.RW;
    attrs.RB[i] = seg.RB;
    attrs.moisture[i] = seg.moisture;

    const glm::vec3 p0 = seg.particle0.x0;
    const glm::vec3 p1 = seg.particle1.x0;
    const glm::vec3 avg = 0.5f * (p0 + p1);
    attrs.position0[i] = glm::dvec3(avg.x, avg.y, avg.z);
    const glm::vec3 direction = glm::normalize(p1 - p0);
    attrs.direction0[i] = glm::dvec3(direction.x, direction.y, direction.z);
    attrs.root_distance[i] = 0.5 * (seg.particle0.root_distance + seg.particle1.root_distance);
  }

  for (size_t tri_index = 0; tri_index < triangles.size(); ++tri_index) {
    const auto& tri = triangles[tri_index];
    attrs.has_neighbor[tri_index] = tri.neighbor_segment_index >= 0;
    const bool is_bark = (tri.neighbor_segment_index == -2);
    const std::array<unsigned int, 3> v_idxs = {tri.vertex_index0, tri.vertex_index1, tri.vertex_index2};
    for (int c = 0; c < 3; ++c) {
      const glm::vec4& uv = tri.uv[c];
      const double value =
          is_bark ? static_cast<double>(uv.y) * uv_height_factor : static_cast<double>(uv.z) * uv_height_factor;
      if (v_idxs[c] < attrs.uv_3.size()) {
        attrs.uv_3[v_idxs[c]] = value;
      }
    }
  }

  return attrs;
}

void MeshletObjExport::AppendGpuAttributes(kinDS::ObjExportGpuAttributes& dst,
                                           const kinDS::ObjExportGpuAttributes& src) {
  auto append = [](auto& d, const auto& s) {
    d.insert(d.end(), s.begin(), s.end());
  };
  append(dst.color, src.color);
  append(dst.boundary_distance, src.boundary_distance);
  append(dst.profile_position, src.profile_position);
  append(dst.profile_polar_coordinate, src.profile_polar_coordinate);
  append(dst.HC, src.HC);
  append(dst.HL, src.HL);
  append(dst.RW, src.RW);
  append(dst.RB, src.RB);
  append(dst.moisture, src.moisture);
  append(dst.position0, src.position0);
  append(dst.direction0, src.direction0);
  append(dst.root_distance, src.root_distance);
  append(dst.uv_3, src.uv_3);
  append(dst.has_neighbor, src.has_neighbor);
}

void MeshletObjExport::ApplySmoothing(
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
    const std::vector<DynamicStrands::GpuSegment>& segments,
    const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
    const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  if (vertices.size() < 2) {
    return;
  }

  std::vector<glm::vec3> x_before(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    x_before[i] = vertices[i].x;
  }
  std::vector<uint8_t> was_smoothed(vertices.size(), 0);

  std::unordered_map<ExactX0Key, std::vector<size_t>, ExactX0KeyHash> groups_by_x0;
  groups_by_x0.reserve(vertices.size());
  for (size_t i = 0; i < vertices.size(); ++i) {
    groups_by_x0[ExactX0Key::From(vertices[i].x0)].push_back(i);
  }

  for (auto& [x0_key, member_indices] : groups_by_x0) {
    (void)x0_key;
    const size_t member_count = member_indices.size();
    if (member_count < 2) {
      continue;
    }

    UnionFind uf(member_count);
    for (size_t i = 0; i < member_count; ++i) {
      const int segment_i = static_cast<int>(vertices[member_indices[i]].segment_index);
      for (size_t j = i + 1; j < member_count; ++j) {
        const int segment_j = static_cast<int>(vertices[member_indices[j]].segment_index);
        if (AreSegmentsStillConnected(segment_i, segment_j, segments, segment_pairs, segment_data_list)) {
          uf.Unite(i, j);
        }
      }
    }

    std::unordered_map<size_t, std::vector<size_t>> components;
    components.reserve(member_count);
    for (size_t i = 0; i < member_count; ++i) {
      components[uf.Find(i)].push_back(member_indices[i]);
    }

    for (const auto& [root, component] : components) {
      (void)root;
      if (component.size() < 2) {
        continue;
      }
      glm::vec3 mean_x(0.f);
      for (const size_t vertex_index : component) {
        mean_x += vertices[vertex_index].x;
      }
      const float inv = 1.f / static_cast<float>(component.size());
      mean_x *= inv;
      for (const size_t vertex_index : component) {
        vertices[vertex_index].x = mean_x;
        was_smoothed[vertex_index] = 1;
      }
    }
  }

  PropagateSmoothingToUnmatchedVertices(vertices, x_before, was_smoothed, triangles);
}

void MeshletObjExport::ExportObj(const std::filesystem::path& path,
                                 const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                                 const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                                 const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor,
                                 double uv_circum_factor, float fracture_distance,
                                 const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
                                 const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex> export_vertices = vertices;
  std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle> export_triangles = triangles;
  if (enable_smoothing) {
    ApplySmoothing(export_vertices, export_triangles, segments, segment_pairs, segment_data_list);
  }

  if (per_meshlet_objects) {
    WriteCombinedMeshGroups(path, BuildMeshGroupsBySegmentIndex(export_vertices, export_triangles), segments,
                            uv_height_factor, uv_circum_factor, fracture_distance, segment_pairs, segment_data_list,
                            false);
    return;
  }

  const bool neighbor_connectivity_debug = NeighborConnectivityDebugEnabled();
  kinDS::VoronoiMesh mesh =
      ToVoronoiMesh(export_vertices, export_triangles, fracture_distance, neighbor_connectivity_debug, segment_pairs);
  kinDS::ObjWriteOptions options;
  options.uv_height_factor = uv_height_factor;
  options.uv_circum_factor = uv_circum_factor;
  options.write_obj_groups = false;
  options.gpu_attributes = BuildGpuAttributes(export_vertices, export_triangles, segments, uv_height_factor);
  WriteExportedMesh(path, std::move(mesh), options, neighbor_connectivity_debug);
}

void MeshletObjExport::ExportObjCombined(const std::filesystem::path& path, const std::vector<MeshGroup>& groups,
                                         const std::vector<DynamicStrands::GpuSegment>& segments,
                                         double uv_height_factor, double uv_circum_factor, float fracture_distance,
                                         const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
                                         const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  WriteCombinedMeshGroups(path, groups, segments, uv_height_factor, uv_circum_factor, fracture_distance,
                          segment_pairs, segment_data_list, true);
}

namespace {

struct Rgb8Key {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;

  static Rgb8Key From(const glm::vec4& c) {
    auto to_u8 = [](float v) -> uint8_t {
      return static_cast<uint8_t>(glm::clamp(static_cast<int>(std::lround(static_cast<double>(v) * 255.0)), 0, 255));
    };
    return Rgb8Key{to_u8(c.x), to_u8(c.y), to_u8(c.z)};
  }

  bool operator==(const Rgb8Key& other) const {
    return r == other.r && g == other.g && b == other.b;
  }
};

struct Rgb8KeyHash {
  size_t operator()(const Rgb8Key& key) const {
    return (static_cast<size_t>(key.r) << 16) | (static_cast<size_t>(key.g) << 8) | static_cast<size_t>(key.b);
  }
};

/// Push mid-saturation RGB toward a more vivid albedo for ray-traced MTL materials.
glm::dvec3 VibrantAlbedoFromRgb8(const Rgb8Key& key) {
  glm::dvec3 rgb(static_cast<double>(key.r) / 255.0, static_cast<double>(key.g) / 255.0,
                 static_cast<double>(key.b) / 255.0);
  const double max_c = std::max({rgb.x, rgb.y, rgb.z});
  const double min_c = std::min({rgb.x, rgb.y, rgb.z});
  if (max_c <= 1e-8) {
    return rgb;
  }
  // HSV-style saturation boost: keep hue/value, raise chroma toward full saturation.
  constexpr double kSaturationBoost = 1.75;
  const double value = max_c;
  const double saturation = (max_c - min_c) / max_c;
  if (saturation <= 1e-8) {
    return rgb;
  }
  const double boosted_s = std::min(1.0, saturation * kSaturationBoost);
  const double scale = boosted_s / saturation;
  rgb = value + (rgb - glm::dvec3(value)) * scale;
  return glm::clamp(rgb, 0.0, 1.0);
}

enum class VisualizationColorSource {
  SegmentColor,
  StrandColor,
};

glm::vec4 VisualizationColorForSegment(const DynamicStrands::GpuSegment& segment,
                                       const std::vector<DynamicStrands::GpuSegment>& segments,
                                       const VisualizationColorSource color_source) {
  if (color_source == VisualizationColorSource::StrandColor) {
    // Matches Segments.mesh Strand color mode: color = segments[strand_handle].color
    if (segment.strand_handle >= 0 && static_cast<size_t>(segment.strand_handle) < segments.size()) {
      return segments[static_cast<size_t>(segment.strand_handle)].color;
    }
  }
  return segment.color;
}

VisualizationColorSource ColorSourceFromMode(const MeshletObjExport::VisualizationColorMode color_mode) {
  return color_mode == MeshletObjExport::VisualizationColorMode::Strands ? VisualizationColorSource::StrandColor
                                                                         : VisualizationColorSource::SegmentColor;
}

int VisualizationGroupId(const unsigned int segment_index, const std::vector<DynamicStrands::GpuSegment>& segments,
                         const VisualizationColorSource color_source) {
  if (segment_index >= segments.size()) {
    return static_cast<int>(segment_index);
  }
  if (color_source == VisualizationColorSource::StrandColor) {
    const int strand_handle = segments[segment_index].strand_handle;
    return strand_handle >= 0 ? strand_handle : static_cast<int>(segment_index);
  }
  return static_cast<int>(segment_index);
}

std::vector<MeshletObjExport::MeshGroup> BuildMeshGroupsForVisualization(
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
    const std::vector<DynamicStrands::GpuSegment>& segments, const VisualizationColorSource color_source,
    const std::string& name_prefix = {}) {
  using Triangle = DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle;

  std::unordered_map<int, std::vector<size_t>> triangle_indices_by_group;
  triangle_indices_by_group.reserve(triangles.size() / 4 + 1);
  for (size_t tri_index = 0; tri_index < triangles.size(); ++tri_index) {
    const unsigned int segment_index = vertices[triangles[tri_index].vertex_index0].segment_index;
    const int group_id = VisualizationGroupId(segment_index, segments, color_source);
    triangle_indices_by_group[group_id].push_back(tri_index);
  }

  std::vector<int> group_ids;
  group_ids.reserve(triangle_indices_by_group.size());
  for (const auto& [group_id, _] : triangle_indices_by_group) {
    group_ids.push_back(group_id);
  }
  std::sort(group_ids.begin(), group_ids.end());

  const char* id_prefix =
      color_source == VisualizationColorSource::StrandColor ? "strand_" : "segment_";

  std::vector<MeshletObjExport::MeshGroup> groups;
  groups.reserve(group_ids.size());
  for (const int group_id : group_ids) {
    const auto& tri_indices = triangle_indices_by_group[group_id];
    MeshletObjExport::MeshGroup group;
    group.name = name_prefix + id_prefix + std::to_string(group_id);

    std::unordered_map<unsigned int, unsigned int> vertex_remap;
    vertex_remap.reserve(tri_indices.size() * 2);
    group.vertices.reserve(tri_indices.size());
    group.triangles.reserve(tri_indices.size());

    const auto remap_vertex = [&](const unsigned int old_index) -> unsigned int {
      const auto found = vertex_remap.find(old_index);
      if (found != vertex_remap.end()) {
        return found->second;
      }
      const unsigned int new_index = static_cast<unsigned int>(group.vertices.size());
      group.vertices.push_back(vertices[old_index]);
      vertex_remap.emplace(old_index, new_index);
      return new_index;
    };

    for (const size_t tri_index : tri_indices) {
      const Triangle& src = triangles[tri_index];
      Triangle dst = src;
      dst.vertex_index0 = remap_vertex(src.vertex_index0);
      dst.vertex_index1 = remap_vertex(src.vertex_index1);
      dst.vertex_index2 = remap_vertex(src.vertex_index2);
      group.triangles.push_back(dst);
    }
    groups.push_back(std::move(group));
  }
  return groups;
}

class SolidColorPalette {
 public:
  int IdForColor(const glm::vec4& color) {
    const Rgb8Key key = Rgb8Key::From(color);
    const auto found = index_by_color_.find(key);
    if (found != index_by_color_.end()) {
      return static_cast<int>(found->second);
    }
    const size_t index = names_.size();
    index_by_color_.emplace(key, index);
    names_.push_back("solid_" + std::to_string(index));
    kd_.push_back(VibrantAlbedoFromRgb8(key));
    return static_cast<int>(index);
  }

  const std::vector<std::string>& Names() const {
    return names_;
  }
  const std::vector<glm::dvec3>& Kd() const {
    return kd_;
  }

 private:
  std::unordered_map<Rgb8Key, size_t, Rgb8KeyHash> index_by_color_;
  std::vector<std::string> names_;
  std::vector<glm::dvec3> kd_;
};

void WriteSolidColoredVisualizationGroups(const std::filesystem::path& path,
                                          const std::vector<MeshletObjExport::MeshGroup>& groups,
                                          const std::vector<DynamicStrands::GpuSegment>& segments,
                                          const VisualizationColorSource color_source, const bool per_face_colors,
                                          const double uv_height_factor, const double uv_circum_factor,
                                          const float fracture_distance) {
  if (groups.empty()) {
    throw std::runtime_error("WriteSolidColoredVisualizationGroups: no mesh groups to export");
  }

  size_t total_vertices = 0;
  size_t total_triangles = 0;
  size_t non_empty_groups = 0;
  for (const auto& group : groups) {
    if (group.vertices.empty() || group.triangles.empty()) {
      continue;
    }
    total_vertices += group.vertices.size();
    total_triangles += group.triangles.size();
    ++non_empty_groups;
  }
  if (non_empty_groups == 0) {
    throw std::runtime_error("WriteSolidColoredVisualizationGroups: all mesh groups were empty");
  }

  // Pass 1: hash unique RGB8 colors into a dense material palette (O(faces) expected).
  SolidColorPalette palette;
  std::vector<int> uniform_material_ids(groups.size(), -1);
  for (size_t group_index = 0; group_index < groups.size(); ++group_index) {
    const auto& group = groups[group_index];
    if (group.vertices.empty() || group.triangles.empty()) {
      continue;
    }
    if (per_face_colors) {
      for (const auto& t : group.triangles) {
        glm::vec4 color(0.8f, 0.8f, 0.8f, 1.0f);
        const unsigned int segment_index = group.vertices[t.vertex_index0].segment_index;
        if (segment_index < segments.size()) {
          color = VisualizationColorForSegment(segments[segment_index], segments, color_source);
        }
        palette.IdForColor(color);
      }
    } else {
      glm::vec4 color(0.8f, 0.8f, 0.8f, 1.0f);
      const unsigned int segment_index = group.vertices.front().segment_index;
      if (segment_index < segments.size()) {
        color = VisualizationColorForSegment(segments[segment_index], segments, color_source);
      }
      uniform_material_ids[group_index] = palette.IdForColor(color);
    }
  }

  // Pass 2: append everything into one mesh — no VoronoiMesh::operator+= (avoids per-merge string scans).
  kinDS::VoronoiMesh combined(palette.Names(), kinDS::PerTriangleCorner);
  combined.getVertices().reserve(total_vertices);
  combined.getTriangles().reserve(total_triangles * 3);
  combined.getNormals().reserve(total_triangles * 3);
  combined.getUVs().reserve(total_triangles * 3);
  combined.getUVIndices().reserve(total_triangles * 3);
  combined.getMaterialIDs().reserve(total_triangles);

  std::vector<size_t> group_offsets;
  std::vector<std::string> group_names;
  group_offsets.reserve(non_empty_groups);
  group_names.reserve(non_empty_groups);

  for (size_t group_index = 0; group_index < groups.size(); ++group_index) {
    const auto& group = groups[group_index];
    if (group.vertices.empty() || group.triangles.empty()) {
      continue;
    }

    group_offsets.push_back(combined.getTriangleCount());
    group_names.push_back(group.name);

    const size_t vertex_base = combined.getVertexCount();
    for (const auto& v : group.vertices) {
      const glm::vec3 p = v.x - fracture_distance * v.shift;
      combined.addVertex(glm::dvec3(p.x, p.y, p.z));
    }

    const int uniform_material_id = uniform_material_ids[group_index];
    for (const auto& t : group.triangles) {
      int material_id = uniform_material_id;
      if (per_face_colors) {
        glm::vec4 color(0.8f, 0.8f, 0.8f, 1.0f);
        const unsigned int segment_index = group.vertices[t.vertex_index0].segment_index;
        if (segment_index < segments.size()) {
          color = VisualizationColorForSegment(segments[segment_index], segments, color_source);
        }
        material_id = palette.IdForColor(color);
      }

      const size_t uv0 = combined.addUV(glm::dvec3(t.uv[0].x, t.uv[0].y, t.uv[0].z));
      const size_t uv1 = combined.addUV(glm::dvec3(t.uv[1].x, t.uv[1].y, t.uv[1].z));
      const size_t uv2 = combined.addUV(glm::dvec3(t.uv[2].x, t.uv[2].y, t.uv[2].z));
      combined.addTriangle(vertex_base + t.vertex_index0, vertex_base + t.vertex_index1, vertex_base + t.vertex_index2,
                           uv0, uv1, uv2, material_id);
      for (int i = 0; i < 3; ++i) {
        combined.addNormal(glm::dvec3(t.normal[i].x, t.normal[i].y, t.normal[i].z));
      }
    }
  }

  combined.setGroupOffsets(group_offsets);
  combined.setGroupNames(group_names);

  kinDS::ObjWriteOptions options;
  options.uv_height_factor = uv_height_factor;
  options.uv_circum_factor = uv_circum_factor;
  options.write_obj_groups = true;
  options.framework_compatible = false;
  options.material_kd_colors = palette.Kd();
  kinDS::ObjExporter::writeMesh(combined, path, options);
}

}  // namespace

void MeshletObjExport::ExportVisualizationObj(
    const std::filesystem::path& path, const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
    const std::vector<DynamicStrands::GpuSegment>& segments, const VisualizationColorMode color_mode,
    const VisualizationObjectGrouping object_grouping, double uv_height_factor, double uv_circum_factor,
    float fracture_distance, const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
    const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex> export_vertices = vertices;
  std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle> export_triangles = triangles;
  if (enable_smoothing) {
    ApplySmoothing(export_vertices, export_triangles, segments, segment_pairs, segment_data_list);
  }

  const VisualizationColorSource color_source = ColorSourceFromMode(color_mode);
  const bool by_highlight = object_grouping == VisualizationObjectGrouping::ByHighlight;

  std::vector<MeshGroup> groups;
  if (by_highlight) {
    groups = BuildMeshGroupsForVisualization(export_vertices, export_triangles, segments, color_source);
  } else {
    MeshGroup combined;
    combined.name = "mesh";
    combined.vertices = std::move(export_vertices);
    combined.triangles = std::move(export_triangles);
    if (!combined.vertices.empty() && !combined.triangles.empty()) {
      groups.push_back(std::move(combined));
    }
  }

  WriteSolidColoredVisualizationGroups(path, groups, segments, color_source, !by_highlight, uv_height_factor,
                                       uv_circum_factor, fracture_distance);
}

void MeshletObjExport::ExportVisualizationObjCombined(
    const std::filesystem::path& path, const std::vector<MeshGroup>& groups,
    const std::vector<DynamicStrands::GpuSegment>& segments, const VisualizationColorMode color_mode,
    const VisualizationObjectGrouping object_grouping, double uv_height_factor, double uv_circum_factor,
    float fracture_distance, const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
    const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list) {
  if (groups.empty()) {
    throw std::runtime_error("ExportVisualizationObjCombined: no mesh groups to export");
  }

  std::vector<MeshGroup> export_groups;
  export_groups.reserve(groups.size());

  const VisualizationColorSource color_source = ColorSourceFromMode(color_mode);
  const bool by_highlight = object_grouping == VisualizationObjectGrouping::ByHighlight;
  const bool per_face_colors = !by_highlight;

  for (const auto& group : groups) {
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex> export_vertices = group.vertices;
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle> export_triangles = group.triangles;
    if (enable_smoothing) {
      ApplySmoothing(export_vertices, export_triangles, segments, segment_pairs, segment_data_list);
    }

    if (!by_highlight) {
      MeshGroup out = group;
      out.vertices = std::move(export_vertices);
      out.triangles = std::move(export_triangles);
      if (!out.vertices.empty() && !out.triangles.empty()) {
        export_groups.push_back(std::move(out));
      }
      continue;
    }

    const std::string prefix = group.name.empty() ? std::string() : (group.name + "_");
    auto subdivided = BuildMeshGroupsForVisualization(export_vertices, export_triangles, segments, color_source, prefix);
    for (auto& part : subdivided) {
      export_groups.push_back(std::move(part));
    }
  }

  WriteSolidColoredVisualizationGroups(path, export_groups, segments, color_source, per_face_colors, uv_height_factor,
                                       uv_circum_factor, fracture_distance);
}
