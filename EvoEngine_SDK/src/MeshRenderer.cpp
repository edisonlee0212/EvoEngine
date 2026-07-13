#include "MeshRenderer.hpp"

#include "Platform.hpp"

using namespace evo_engine;

namespace {
bool MorphWeightsMatch(const std::vector<float>& lhs, const std::vector<float>& rhs) {
  return lhs.size() == rhs.size() && std::equal(lhs.begin(), lhs.end(), rhs.begin());
}

std::vector<float> ResolveMorphWeights(const Mesh& mesh, const std::vector<float>& overrides) {
  auto result = mesh.GetDefaultMorphWeights();
  for (size_t index = 0; index < std::min(result.size(), overrides.size()); index++) {
    result[index] = overrides[index];
  }
  return result;
}

Bound CalculateBound(const std::vector<Vertex>& vertices) {
  Bound result{};
  if (vertices.empty()) {
    return result;
  }
  result.min = vertices.front().position;
  result.max = result.min;
  for (const auto& vertex : vertices) {
    result.min = glm::min(result.min, vertex.position);
    result.max = glm::max(result.max, vertex.position);
  }
  return result;
}
}  // namespace

void MeshRenderer::SetMorphWeights(const std::vector<float>& weights) {
  if (!MorphWeightsMatch(morph_weights_, weights)) {
    morph_weights_ = weights;
    morph_weights_version_++;
  }
}

const std::vector<float>& MeshRenderer::PeekMorphWeights() const {
  return morph_weights_;
}

void MeshRenderer::UpdateRayTracingGeometry() {
  if (pending_ray_tracing_submission_state_) {
    if (pending_ray_tracing_submission_state_->status == FrameSubmissionState::Status::Pending) {
      return;
    }
    if (pending_ray_tracing_submission_state_->status == FrameSubmissionState::Status::Submitted) {
      ray_tracing_morph_weights_ = std::move(pending_ray_tracing_morph_weights_);
      ray_tracing_payload_retry_required_ = false;
    } else {
      ray_tracing_payload_retry_required_ = true;
    }
    pending_ray_tracing_submission_state_.reset();
    pending_ray_tracing_morph_weights_.clear();
  }
  const auto clear_ray_tracing_geometry = [&]() {
    if (!ray_tracing_meshlet_range_ && !ray_tracing_triangle_range_ && !ray_tracing_blas_ &&
        ray_tracing_mesh_handle_ == 0) {
      return;
    }
    GeometryStorage::FreeMesh(GetHandle());
    ray_tracing_meshlet_range_.reset();
    ray_tracing_triangle_range_.reset();
    ray_tracing_blas_.reset();
    ray_tracing_packed_source_vertex_indices_.clear();
    ray_tracing_morph_weights_.clear();
    pending_ray_tracing_morph_weights_.clear();
    pending_ray_tracing_submission_state_.reset();
    ray_tracing_payload_retry_required_ = false;
    ray_tracing_mesh_handle_ = Handle(0);
    ray_tracing_geometry_version_ = 0;
    ray_tracing_bound_ = {};
  };
  const auto mesh_asset = mesh.Get<Mesh>();
  if (!Platform::RayAccelerationStructureEnabled() || !mesh_asset || mesh_asset->PeekMorphTargets().empty()) {
    clear_ray_tracing_geometry();
    return;
  }
  const auto weights = ResolveMorphWeights(*mesh_asset, morph_weights_);
  if (MorphWeightsMatch(weights, mesh_asset->GetDefaultMorphWeights())) {
    clear_ray_tracing_geometry();
    return;
  }
  const auto mesh_handle = mesh_asset->GetHandle();
  const auto geometry_version = mesh_asset->GetVersion();
  const bool topology_changed = !ray_tracing_blas_ || ray_tracing_mesh_handle_ != mesh_handle ||
                                ray_tracing_geometry_version_ != geometry_version ||
                                ray_tracing_packed_source_vertex_indices_.empty();
  if (!topology_changed && !ray_tracing_payload_retry_required_ &&
      MorphWeightsMatch(ray_tracing_morph_weights_, weights)) {
    return;
  }

  const auto morphed_vertices = mesh_asset->BuildMorphedVertices(weights);
  ray_tracing_bound_ = CalculateBound(morphed_vertices);
  if (topology_changed) {
    GeometryStorage::FreeMesh(GetHandle());
    ray_tracing_meshlet_range_ = std::make_shared<RangeDescriptor>();
    ray_tracing_triangle_range_ = std::make_shared<RangeDescriptor>();
    auto packed_vertices = morphed_vertices;
    auto triangles = mesh_asset->PeekTriangles();
    GeometryStorage::AllocateMesh(GetHandle(), packed_vertices, triangles, ray_tracing_meshlet_range_,
                                  ray_tracing_triangle_range_, &ray_tracing_packed_source_vertex_indices_);
    ray_tracing_blas_ = std::make_shared<BottomLevelAccelerationStructure>(packed_vertices, triangles, true);
    ray_tracing_mesh_handle_ = mesh_handle;
    ray_tracing_geometry_version_ = geometry_version;
    ray_tracing_morph_weights_ = weights;
    ray_tracing_payload_retry_required_ = false;
    return;
  }

  std::vector<Vertex> packed_vertices;
  packed_vertices.reserve(ray_tracing_packed_source_vertex_indices_.size());
  for (const auto source_index : ray_tracing_packed_source_vertex_indices_) {
    packed_vertices.emplace_back(morphed_vertices.at(source_index));
  }
  GeometryStorage::UpdateMeshVertices(ray_tracing_meshlet_range_, packed_vertices);
  pending_ray_tracing_submission_state_ = ray_tracing_blas_->UpdateVertices(packed_vertices);
  pending_ray_tracing_morph_weights_ = weights;
}

void MeshRenderer::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
  ray_tracing_meshlet_range_.reset();
  ray_tracing_triangle_range_.reset();
  ray_tracing_blas_.reset();
  ray_tracing_packed_source_vertex_indices_.clear();
  ray_tracing_morph_weights_.clear();
  pending_ray_tracing_morph_weights_.clear();
  pending_ray_tracing_submission_state_.reset();
  ray_tracing_bound_ = {};
  ray_tracing_payload_retry_required_ = false;
  ray_tracing_mesh_handle_ = Handle(0);
  ray_tracing_geometry_version_ = 0;
}
void MeshRenderer::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(mesh);
  list.push_back(material);
}
void MeshRenderer::OnDestroy() {
  if (ray_tracing_meshlet_range_ || ray_tracing_triangle_range_ || ray_tracing_blas_) {
    GeometryStorage::FreeMesh(GetHandle());
  }
  ray_tracing_meshlet_range_.reset();
  ray_tracing_triangle_range_.reset();
  ray_tracing_blas_.reset();
  ray_tracing_packed_source_vertex_indices_.clear();
  morph_weights_.clear();
  ray_tracing_morph_weights_.clear();
  pending_ray_tracing_morph_weights_.clear();
  pending_ray_tracing_submission_state_.reset();
  ray_tracing_payload_retry_required_ = false;
  ray_tracing_mesh_handle_ = Handle(0);
  ray_tracing_geometry_version_ = 0;
  morph_weights_version_ = 0;
  ray_tracing_bound_ = {};
  mesh.Clear();
  material.Clear();

  cast_shadow = true;
}
