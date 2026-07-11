#include "SkinnedMeshRenderer.hpp"
#include "Platform.hpp"
using namespace evo_engine;

namespace {
bool BoneMatricesMatch(const std::vector<glm::mat4>& lhs, const std::vector<glm::mat4>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (size_t matrix_index = 0; matrix_index < lhs.size(); matrix_index++) {
    for (int column = 0; column < 4; column++) {
      for (int row = 0; row < 4; row++) {
        if (lhs[matrix_index][column][row] != rhs[matrix_index][column][row]) {
          return false;
        }
      }
    }
  }
  return true;
}
}  // namespace

void SkinnedMeshRenderer::UpdateBoneMatrices() {
  const auto scene = GetScene();
  const auto tmp = this->animator.Get<Animator>();
  if (!tmp)
    return;
  const auto tmp_mesh = skinned_mesh.Get<SkinnedMesh>();
  if (!tmp_mesh)
    return;
  if (rag_doll_) {
    if (rag_doll_freeze)
      return;

    bone_matrices->value.resize(tmp_mesh->bone_animator_indices.size());
    for (int i = 0; i < bound_entities_.size(); i++) {
      auto entity = bound_entities_[i].Get();
      if (entity.GetIndex() != 0) {
        rag_doll_transform_chain_[i] =
            scene->GetDataComponent<GlobalTransform>(entity).value * tmp->offset_matrices_[i];
      }
    }
    for (int i = 0; i < tmp_mesh->bone_animator_indices.size(); i++) {
      bone_matrices->value[i] = rag_doll_transform_chain_[tmp_mesh->bone_animator_indices[i]];
    }
  } else {
    if (tmp->bone_size_ == 0)
      return;
    bone_matrices->value.resize(tmp_mesh->bone_animator_indices.size());
    for (int i = 0; i < tmp_mesh->bone_animator_indices.size(); i++) {
      bone_matrices->value[i] = tmp->transform_chain_[tmp_mesh->bone_animator_indices[i]];
    }
  }
}

void SkinnedMeshRenderer::UpdateRayTracingGeometry() {
  if (pending_ray_tracing_submission_state_) {
    if (pending_ray_tracing_submission_state_->status == FrameSubmissionState::Status::Pending) {
      return;
    }
    if (pending_ray_tracing_submission_state_->status == FrameSubmissionState::Status::Submitted) {
      ray_tracing_bone_matrices_ = std::move(pending_ray_tracing_bone_matrices_);
      ray_tracing_payload_retry_required_ = false;
    } else {
      ray_tracing_payload_retry_required_ = true;
    }
    pending_ray_tracing_submission_state_.reset();
    pending_ray_tracing_bone_matrices_.clear();
  }
  const auto clear_ray_tracing_geometry = [&]() {
    if (!ray_tracing_meshlet_range_ && !ray_tracing_triangle_range_ && !ray_tracing_blas_ &&
        ray_tracing_bone_matrices_.empty() && ray_tracing_geometry_version_ == 0) {
      return;
    }
    GeometryStorage::FreeMesh(GetHandle());
    ray_tracing_meshlet_range_.reset();
    ray_tracing_triangle_range_.reset();
    ray_tracing_blas_.reset();
    ray_tracing_packed_source_vertex_indices_.clear();
    ray_tracing_bone_matrices_.clear();
    pending_ray_tracing_bone_matrices_.clear();
    pending_ray_tracing_submission_state_.reset();
    ray_tracing_payload_retry_required_ = false;
    ray_tracing_geometry_version_ = 0;
  };
  if (!Platform::RayTracingEnabled() || !bone_matrices) {
    clear_ray_tracing_geometry();
    return;
  }
  const auto mesh = skinned_mesh.Get<SkinnedMesh>();
  if (!mesh || mesh->skinned_vertices_.empty() || mesh->skinned_triangles_.empty() || bone_matrices->value.empty()) {
    clear_ray_tracing_geometry();
    return;
  }
  const auto geometry_version = mesh->GetVersion();
  const bool topology_changed = !ray_tracing_blas_ || ray_tracing_geometry_version_ != geometry_version ||
                                ray_tracing_packed_source_vertex_indices_.empty();
  if (!topology_changed && !ray_tracing_payload_retry_required_ &&
      BoneMatricesMatch(ray_tracing_bone_matrices_, bone_matrices->value)) {
    return;
  }

  if (topology_changed) {
    GeometryStorage::FreeMesh(GetHandle());
    ray_tracing_meshlet_range_ = std::make_shared<RangeDescriptor>();
    ray_tracing_triangle_range_ = std::make_shared<RangeDescriptor>();
    auto vertices = BuildSkinnedRayTracingVertices(mesh->skinned_vertices_, bone_matrices->value);
    auto triangles = mesh->skinned_triangles_;
    GeometryStorage::AllocateMesh(GetHandle(), vertices, triangles, ray_tracing_meshlet_range_,
                                  ray_tracing_triangle_range_, &ray_tracing_packed_source_vertex_indices_);
    ray_tracing_blas_ = std::make_shared<BottomLevelAccelerationStructure>(vertices, triangles, true);
    ray_tracing_geometry_version_ = geometry_version;
    ray_tracing_bone_matrices_ = bone_matrices->value;
    ray_tracing_payload_retry_required_ = false;
    return;
  }

  const auto packed_vertices = BuildSkinnedRayTracingVertices(mesh->skinned_vertices_, bone_matrices->value,
                                                              ray_tracing_packed_source_vertex_indices_);
  GeometryStorage::UpdateMeshVertices(ray_tracing_meshlet_range_, packed_vertices);
  pending_ray_tracing_submission_state_ = ray_tracing_blas_->UpdateVertices(packed_vertices);
  pending_ray_tracing_bone_matrices_ = bone_matrices->value;
}

void SkinnedMeshRenderer::OnCreate() {
  bone_matrices = std::make_shared<BoneMatrices>();
  SetEnabled(true);
}
void SkinnedMeshRenderer::PostCloneAction(const std::shared_ptr<IPrivateComponent>& target) {
}
void SkinnedMeshRenderer::Relink(const std::unordered_map<Handle, Handle>& map, const std::shared_ptr<Scene>& scene) {
  animator.Relink(map, scene);
  for (auto& i : bound_entities_) {
    i.Relink(map);
  }
}
void SkinnedMeshRenderer::CollectAssetRef(std::vector<AssetRef>& list) {
  list.push_back(skinned_mesh);
  list.push_back(material);
}
bool SkinnedMeshRenderer::RagDoll() const {
  return rag_doll_;
}

const std::vector<glm::mat4>& SkinnedMeshRenderer::PeekRagDollTransformChain() const {
  return rag_doll_transform_chain_;
}

std::vector<glm::mat4>& SkinnedMeshRenderer::RefRagDollTransformChain() {
  return rag_doll_transform_chain_;
}

const std::vector<EntityRef>& SkinnedMeshRenderer::PeekRagDollBoundEntities() const {
  return bound_entities_;
}

std::vector<EntityRef>& SkinnedMeshRenderer::RefRagDollBoundEntities() {
  return bound_entities_;
}

Entity SkinnedMeshRenderer::GetRagDollBoundEntity(const int index) {
  if (index < 0 || index >= bound_entities_.size()) {
    return {};
  }
  return bound_entities_[index].Get();
}

void SkinnedMeshRenderer::SetRagDoll(bool value) {
  const auto amt = animator.Get<Animator>();
  if (value && !amt) {
    EVOENGINE_ERROR("Failed! No animator!");
    return;
  }
  rag_doll_ = value;
  if (rag_doll_) {
    const auto scene = GetScene();
    // Resize entities
    bound_entities_.resize(amt->transform_chain_.size());
    // Copy current transform chain
    rag_doll_transform_chain_ = amt->transform_chain_;
    const auto ltw = scene->GetDataComponent<GlobalTransform>(GetOwner()).value;
    for (auto& i : rag_doll_transform_chain_) {
      i = ltw * i;
    }
  }
}

void SkinnedMeshRenderer::SetRagDollState(const bool value) {
  rag_doll_ = value;
}

void SkinnedMeshRenderer::SetRagDollBoundEntity(int index, const Entity& entity, bool reset_transform) {
  if (!rag_doll_) {
    EVOENGINE_ERROR("Not ragdoll!");
    return;
  }
  if (index >= bound_entities_.size()) {
    EVOENGINE_ERROR("Index exceeds limit!");
    return;
  }
  if (const auto scene = GetScene(); scene->IsEntityValid(entity)) {
    if (const auto amt = animator.Get<Animator>()) {
      if (reset_transform) {
        GlobalTransform global_transform;
        global_transform.value = rag_doll_transform_chain_[index] * glm::inverse(amt->offset_matrices_[index]);
        scene->SetDataComponent(entity, global_transform);
      }
    }
    bound_entities_[index] = entity;
  }
}

void SkinnedMeshRenderer::ClearRagDollBoundEntity(const int index) {
  if (index < 0 || index >= bound_entities_.size()) {
    EVOENGINE_ERROR("Index exceeds limit!");
    return;
  }
  bound_entities_[index].Clear();
}

void SkinnedMeshRenderer::SetRagDollBoundEntities(const std::vector<Entity>& entities, bool reset_transform) {
  if (!rag_doll_) {
    EVOENGINE_ERROR("Not ragdoll!");
    return;
  }
  for (int i = 0; i < entities.size(); i++) {
    SetRagDollBoundEntity(i, entities[i], reset_transform);
  }
}
size_t SkinnedMeshRenderer::GetRagDollBoneSize() const {
  if (!rag_doll_) {
    EVOENGINE_ERROR("Not ragdoll!");
    return 0;
  }
  return bound_entities_.size();
}
void SkinnedMeshRenderer::OnDestroy() {
  GeometryStorage::FreeMesh(GetHandle());
  ray_tracing_meshlet_range_.reset();
  ray_tracing_triangle_range_.reset();
  ray_tracing_blas_.reset();
  ray_tracing_packed_source_vertex_indices_.clear();
  ray_tracing_bone_matrices_.clear();
  pending_ray_tracing_bone_matrices_.clear();
  pending_ray_tracing_submission_state_.reset();
  ray_tracing_payload_retry_required_ = false;
  ray_tracing_geometry_version_ = 0;
  rag_doll_transform_chain_.clear();
  bound_entities_.clear();
  animator.Clear();
  bone_matrices.reset();
  skinned_mesh.Clear();
  material.Clear();
  rag_doll_ = false;
  rag_doll_freeze = false;
  cast_shadow = true;
}
