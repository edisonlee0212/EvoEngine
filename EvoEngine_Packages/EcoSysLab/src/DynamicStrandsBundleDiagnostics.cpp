#include "DynamicStrandsBundleDiagnostics.hpp"

#include "DsConstraints.hpp"

#include <fstream>

using namespace eco_sys_lab_package;
using namespace evo_engine;

namespace {
float SegmentMass(const DynamicStrands::GpuSegment& segment) {
  return glm::max(segment.original_mass + segment.extra_mass, 0.f);
}

glm::vec3 SegmentCenter(const DynamicStrands::GpuSegment& segment) {
  return (segment.particle0.x + segment.particle1.x) * .5f;
}

glm::vec3 SegmentRestCenter(const DynamicStrands::GpuSegment& segment) {
  return (segment.particle0.x0 + segment.particle1.x0) * .5f;
}

glm::quat FitRotation(const std::vector<const DynamicStrands::GpuSegment*>& members) {
  glm::vec3 rest_center{};
  glm::vec3 current_center{};
  float weight_sum = 0.f;
  for (const auto* segment : members) {
    const float weight = glm::max(SegmentMass(*segment), 1e-6f) * .5f;
    rest_center += weight * (segment->particle0.x0 + segment->particle1.x0);
    current_center += weight * (segment->particle0.x + segment->particle1.x);
    weight_sum += 2.f * weight;
  }
  if (weight_sum <= 0.f)
    return glm::quat(1.f, 0.f, 0.f, 0.f);
  rest_center /= weight_sum;
  current_center /= weight_sum;

  glm::mat3 covariance(0.f);
  for (const auto* segment : members) {
    const float weight = glm::max(SegmentMass(*segment), 1e-6f) * .5f;
    covariance +=
        weight * glm::outerProduct(segment->particle0.x - current_center, segment->particle0.x0 - rest_center);
    covariance +=
        weight * glm::outerProduct(segment->particle1.x - current_center, segment->particle1.x0 - rest_center);
  }
  float norm_squared = 0.f;
  for (int column = 0; column < 3; ++column)
    norm_squared += glm::dot(covariance[column], covariance[column]);
  if (norm_squared <= 1e-16f)
    return glm::quat(1.f, 0.f, 0.f, 0.f);

  glm::mat3 rotation = covariance / glm::sqrt(norm_squared);
  for (int iteration = 0; iteration < 8; ++iteration) {
    if (glm::abs(glm::determinant(rotation)) <= 1e-8f)
      return glm::quat(1.f, 0.f, 0.f, 0.f);
    rotation = .5f * (rotation + glm::inverse(glm::transpose(rotation)));
  }
  if (glm::determinant(rotation) < 0.f)
    rotation[2] = -rotation[2];
  return glm::normalize(glm::quat_cast(rotation));
}

uint64_t BufferBytes(const std::shared_ptr<Buffer>& buffer) {
  return buffer ? buffer->GetSize() : 0;
}

GpuTimestampStats BundleGpuStats(const BundleSolverMode mode) {
  GpuTimestampStats stats;
  const auto history = Platform::GetGpuTimestampFrameHistory();
  const size_t begin = history.size() > 120 ? history.size() - 120 : 0;
  for (size_t frame_index = begin; frame_index < history.size(); ++frame_index) {
    double frame_milliseconds = 0.0;
    bool found = false;
    for (const auto& sample : history[frame_index].samples) {
      const auto& pass = sample.metadata.stable_pass_id;
      const bool matches =
          mode == BundleSolverMode::Legacy
              ? pass == "EcoSysLab.DynamicStrands.Bundle.Legacy"
              : pass == "EcoSysLab.DynamicStrands.Bundle.PairSolveGather" ||
                    (mode == BundleSolverMode::Hybrid && (pass == "EcoSysLab.DynamicStrands.Bundle.TopologyRebuild" ||
                                                          pass == "EcoSysLab.DynamicStrands.Bundle.SliceFitApply" ||
                                                          pass == "EcoSysLab.DynamicStrands.Bundle.CoarseEdgeSolve"));
      if (matches) {
        frame_milliseconds += sample.duration_milliseconds;
        found = true;
      }
    }
    if (found)
      stats.AddSample(frame_milliseconds);
  }
  return stats;
}
}  // namespace

BundleMomentum eco_sys_lab_package::CalculateBundleMomentum(const DynamicStrands& dynamic_strands) {
  BundleMomentum result;
  glm::vec3 center_of_mass{};
  float mass_sum = 0.f;
  for (const auto& segment : dynamic_strands.segments) {
    const float mass = SegmentMass(segment);
    mass_sum += mass;
    center_of_mass += mass * SegmentCenter(segment);
    result.linear += mass * (segment.particle0.v + segment.particle1.v) * .5f;
  }
  if (mass_sum > 0.f)
    center_of_mass /= mass_sum;
  for (const auto& segment : dynamic_strands.segments) {
    const float mass = SegmentMass(segment);
    const glm::vec3 momentum = mass * (segment.particle0.v + segment.particle1.v) * .5f;
    result.angular += glm::cross(SegmentCenter(segment) - center_of_mass, momentum) +
                      glm::mat3(segment.inertia_w) * segment.angular_v;
  }
  return result;
}

BundleExperimentDiagnostics eco_sys_lab_package::CaptureBundleExperimentDiagnostics(
    const std::string& experiment, const DynamicStrands& dynamic_strands, const DsBundle& bundle,
    const BundleMomentum& reference_momentum) {
  BundleExperimentDiagnostics result;
  result.experiment = experiment;
  result.mode = bundle.solver_settings.mode;
  result.segment_count = static_cast<uint32_t>(dynamic_strands.segments.size());
  result.pair_count = static_cast<uint32_t>(dynamic_strands.segment_pairs.size());

  if (!dynamic_strands.segments.empty()) {
    float minimum_root_distance = std::numeric_limits<float>::max();
    float maximum_root_distance = std::numeric_limits<float>::lowest();
    for (const auto& segment : dynamic_strands.segments) {
      const float root_distance = .5f * (segment.particle0.root_distance + segment.particle1.root_distance);
      minimum_root_distance = glm::min(minimum_root_distance, root_distance);
      maximum_root_distance = glm::max(maximum_root_distance, root_distance);
    }
    const float target_root_distance = .5f * (minimum_root_distance + maximum_root_distance);
    std::unordered_map<int32_t, std::pair<float, uint32_t>> nodes;
    for (const auto& segment : dynamic_strands.segments) {
      auto& node = nodes[segment.node_handle];
      node.first += .5f * (segment.particle0.root_distance + segment.particle1.root_distance);
      ++node.second;
    }
    float best_distance = std::numeric_limits<float>::max();
    uint32_t best_members = 0;
    for (const auto& [node_handle, node] : nodes) {
      const float distance = glm::abs(node.first / static_cast<float>(node.second) - target_root_distance);
      if (distance < best_distance || (distance == best_distance && node.second > best_members)) {
        best_distance = distance;
        best_members = node.second;
        result.cross_section_node_handle = node_handle;
      }
    }

    std::vector<const DynamicStrands::GpuSegment*> members;
    glm::vec3 rest_center{};
    glm::vec3 current_center{};
    float weight_sum = 0.f;
    for (const auto& segment : dynamic_strands.segments) {
      if (segment.node_handle != result.cross_section_node_handle)
        continue;
      members.emplace_back(&segment);
      const float weight = glm::max(SegmentMass(segment), 1e-6f);
      rest_center += weight * SegmentRestCenter(segment);
      current_center += weight * SegmentCenter(segment);
      weight_sum += weight;
    }
    if (weight_sum > 0.f) {
      rest_center /= weight_sum;
      current_center /= weight_sum;
      result.center_translation = current_center - rest_center;
      result.cross_section_rotation = FitRotation(members);
      float farthest_squared = -1.f;
      for (const auto* segment : members) {
        const glm::vec3 offset = SegmentRestCenter(*segment) - rest_center;
        const float distance_squared = glm::dot(offset, offset);
        if (distance_squared > farthest_squared) {
          farthest_squared = distance_squared;
          result.far_side_displacement = SegmentCenter(*segment) - SegmentRestCenter(*segment);
        }
      }
    }
  }

  double residual_squared = 0.0;
  uint64_t residual_count = 0;
  for (size_t pair_index = dynamic_strands.connection_segment_pair_size;
       pair_index < dynamic_strands.segment_pairs.size(); ++pair_index) {
    const auto& pair = dynamic_strands.segment_pairs[pair_index];
    if (pair.bend_twist_bundle_integrity <= 0.f)
      continue;
    const auto& segment0 = dynamic_strands.segments[pair.segment0_handle];
    const auto& segment1 = dynamic_strands.segments[pair.segment1_handle];
    const glm::vec3 residual0 =
        SegmentCenter(segment1) + segment1.q * glm::vec3(pair.segment0_offset) - SegmentCenter(segment0);
    const glm::vec3 residual1 =
        SegmentCenter(segment0) + segment0.q * glm::vec3(pair.segment1_offset) - SegmentCenter(segment1);
    residual_squared += glm::dot(residual0, residual0) + glm::dot(residual1, residual1);
    residual_count += 2;
  }
  if (residual_count > 0)
    result.constraint_rms = static_cast<float>(glm::sqrt(residual_squared / static_cast<double>(residual_count)));

  const auto momentum = CalculateBundleMomentum(dynamic_strands);
  result.linear_momentum_residual = momentum.linear - reference_momentum.linear;
  result.angular_momentum_residual = momentum.angular - reference_momentum.angular;
  if (bundle.solver_settings.mode == BundleSolverMode::Legacy) {
    const uint64_t calculate_apply_passes =
        2ull * bundle.skip_size *
        (static_cast<uint64_t>(bundle.enable_bundle_position) + static_cast<uint64_t>(bundle.enable_bundle_rotation) +
         static_cast<uint64_t>(bundle.enable_bend_twist) + static_cast<uint64_t>(bundle.enable_stretch_shear));
    result.bundle_dispatches_per_projection =
        bundle.sub_iteration * (calculate_apply_passes + static_cast<uint64_t>(bundle.enable_connections));
  } else {
    result.bundle_dispatches_per_projection =
        2ull * bundle.solver_settings.pair_iterations + static_cast<uint64_t>(bundle.enable_connections);
    if (bundle.solver_settings.mode == BundleSolverMode::Hybrid)
      result.bundle_dispatches_per_projection += 2ull + bundle.solver_settings.coarse_iterations;
  }
  result.strand_buffer_bytes =
      BufferBytes(dynamic_strands.device_strands_buffer) + BufferBytes(dynamic_strands.device_nodes_buffer) +
      BufferBytes(dynamic_strands.device_segments_buffer) +
      BufferBytes(dynamic_strands.device_segment_particle0_buffer) +
      BufferBytes(dynamic_strands.device_segment_particle1_buffer) +
      BufferBytes(dynamic_strands.device_segment_pairs_buffer) +
      BufferBytes(dynamic_strands.device_segment_data_list_buffer) +
      BufferBytes(dynamic_strands.device_segment_connection_handles_buffer) +
      BufferBytes(dynamic_strands.device_hashed_grid_elements_buffer) +
      BufferBytes(dynamic_strands.device_hashed_grid_cell_starts_buffer) +
      BufferBytes(dynamic_strands.device_foliage_buffer) + BufferBytes(bundle.coupled_pair_state_buffer) +
      BufferBytes(bundle.coupled_pair_correction_buffer) + BufferBytes(bundle.base_slice_buffer) +
      BufferBytes(bundle.slice_member_buffer) + BufferBytes(bundle.slice_range_buffer) +
      BufferBytes(bundle.segment_slice_buffer) + BufferBytes(bundle.slice_transform_buffer) +
      BufferBytes(bundle.slice_count_buffer) + BufferBytes(bundle.slice_dispatch_buffer) +
      BufferBytes(bundle.coarse_candidate_buffer) + BufferBytes(bundle.coarse_edge_buffer) +
      BufferBytes(bundle.coarse_edge_count_buffer);
  const auto gpu_stats = BundleGpuStats(bundle.solver_settings.mode);
  result.gpu_sample_count = gpu_stats.sample_count;
  result.gpu_median_milliseconds = gpu_stats.MedianMilliseconds();
  result.gpu_p95_milliseconds = gpu_stats.PercentileMilliseconds(.95);
  return result;
}

void BundleExperimentDiagnostics::Save(const std::filesystem::path& path) const {
  std::filesystem::create_directories(path.parent_path());
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "experiment" << YAML::Value << experiment;
  out << YAML::Key << "solver_mode" << YAML::Value << static_cast<int>(mode);
  out << YAML::Key << "segment_count" << YAML::Value << segment_count;
  out << YAML::Key << "pair_count" << YAML::Value << pair_count;
  out << YAML::Key << "cross_section_node_handle" << YAML::Value << cross_section_node_handle;
  out << YAML::Key << "cross_section_rotation_xyzw" << YAML::Value << YAML::Flow << YAML::BeginSeq
      << cross_section_rotation.x << cross_section_rotation.y << cross_section_rotation.z << cross_section_rotation.w
      << YAML::EndSeq;
  out << YAML::Key << "center_translation" << YAML::Value << center_translation;
  out << YAML::Key << "far_side_displacement" << YAML::Value << far_side_displacement;
  out << YAML::Key << "constraint_rms" << YAML::Value << constraint_rms;
  out << YAML::Key << "linear_momentum_residual" << YAML::Value << linear_momentum_residual;
  out << YAML::Key << "angular_momentum_residual" << YAML::Value << angular_momentum_residual;
  out << YAML::Key << "bundle_dispatches_per_projection" << YAML::Value << bundle_dispatches_per_projection;
  out << YAML::Key << "strand_buffer_bytes" << YAML::Value << strand_buffer_bytes;
  out << YAML::Key << "gpu_sample_count" << YAML::Value << gpu_sample_count;
  out << YAML::Key << "gpu_median_milliseconds" << YAML::Value << gpu_median_milliseconds;
  out << YAML::Key << "gpu_p95_milliseconds" << YAML::Value << gpu_p95_milliseconds;
  out << YAML::EndMap;
  std::ofstream stream(path);
  stream << out.c_str();
}
