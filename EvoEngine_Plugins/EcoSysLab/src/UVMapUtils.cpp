#include "UVMapUtils.hpp"

using namespace eco_sys_lab_plugin;

UVMapUtils::UVMapUtils() {
}

UVMapUtils::~UVMapUtils() {
}

bool UVMapUtils::IsValidPipeParam(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t) {
  const auto& pipe = strand_model.strand_model_skeleton.data.strand_group.PeekStrand(pipe_handle);
  return pipe.PeekStrandSegmentHandles().size() > glm::floor(t);
}

float UVMapUtils::GetPipePolar(const Particle2D<CellParticlePhysicsData>& p0,
                               const Particle2D<CellParticlePhysicsData>& p1, float t) {
  float a0 = p0.GetPolarPosition().y;
  float a1 = p1.GetPolarPosition().y;

  float interpolation_param = fmod(t, 1.0f);

  // we will just assume that the difference cannot exceed 180 degrees
  if (a1 < a0) {
    std::swap(a0, a1);
    interpolation_param = 1 - interpolation_param;
  }

  float angle;

  if (a1 - a0 > glm::pi<float>()) {
    // rotation wraps around
    angle =
        fmod((a0 + 2 * glm::pi<float>()) * (1 - interpolation_param) + a1 * interpolation_param, 2 * glm::pi<float>());

    if (angle > glm::pi<float>()) {
      angle -= 2 * glm::pi<float>();
    }
  } else {
    angle = a0 * (1 - interpolation_param) + a1 * interpolation_param;

    if (angle > glm::pi<float>()) {
      angle -= 2 * glm::pi<float>();
    }
  }

  return angle;
}

float UVMapUtils::GetPipePolar(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t) {
  // cheap interpolation, maybe improve this later ?
  const auto& p0_ptr = GetStartParticle(strand_model, pipe_handle, std::floor(t));
  const auto& p1_ptr = GetEndParticle(strand_model, pipe_handle, std::floor(t));

  if (!p0_ptr || !p1_ptr)
  {
    return 0.0f;
  }

  if (IsValidPipeParam(strand_model, pipe_handle, std::ceil(t))) {
    const auto& p1 = GetStartParticle(strand_model, pipe_handle, std::ceil(t));
  }

  return GetPipePolar(*p0_ptr, *p1_ptr, t);
}

const Particle2D<CellParticlePhysicsData>* UVMapUtils::GetEndParticle(const StrandModel& strand_model,
                                                                      const StrandHandle& pipe_handle, size_t index) {
  if (!IsValidPipeParam(strand_model, pipe_handle, index)) {
    EVOENGINE_ERROR("Strand " << pipe_handle << " does not exist at " << index);
  }

  const auto& skeleton = strand_model.strand_model_skeleton;

  return GetEndParticle(skeleton, pipe_handle, index);
}

const Particle2D<CellParticlePhysicsData>* UVMapUtils::GetEndParticle(const StrandModelSkeleton& skeleton,
                                                                      const StrandHandle& pipe_handle, size_t index) {
  const auto& pipe = skeleton.data.strand_group.PeekStrand(pipe_handle);

  if (pipe.PeekStrandSegmentHandles().size() <= index) {
    EVOENGINE_ERROR("Strand segment index out of bounds:" << index << "/" << pipe.PeekStrandSegmentHandles().size());
    return nullptr;
  }

  StrandSegmentHandle seg_handle = pipe.PeekStrandSegmentHandles()[index];
  auto& pipe_segment_data = skeleton.data.strand_group.PeekStrandSegmentData(seg_handle);
  if (pipe_segment_data.profile_particle_handle == -1){
    return nullptr;
  }

  const auto& node = skeleton.PeekNode(pipe_segment_data.node_handle);
  const auto& start_profile = node.data.profile;
  const auto& end_particle = start_profile.PeekParticle(pipe_segment_data.profile_particle_handle);

  return &end_particle;
}

const Particle2D<CellParticlePhysicsData>* UVMapUtils::GetStartParticle(const StrandModel& strand_model,
                                                                        const StrandHandle& pipe_handle, size_t index) {
  if (!IsValidPipeParam(strand_model, pipe_handle, index)) {
    EVOENGINE_ERROR("Strand " << pipe_handle << " does not exist at " << index);
  }

  const auto& skeleton = strand_model.strand_model_skeleton;

  return GetStartParticle(skeleton, pipe_handle, index);
}

const Particle2D<CellParticlePhysicsData>* UVMapUtils::GetStartParticle(const StrandModelSkeleton& skeleton,
                                                                        const StrandHandle& pipe_handle, size_t index) {
  const auto& pipe = skeleton.data.strand_group.PeekStrand(pipe_handle);

  if (pipe.PeekStrandSegmentHandles().size() <= index) {
    EVOENGINE_ERROR("Strand segment index out of bounds:" << index << "/" << pipe.PeekStrandSegmentHandles().size());
    return nullptr;
  }

  const auto seg_handle = pipe.PeekStrandSegmentHandles()[index];
  auto& strand_segment_data = skeleton.data.strand_group.PeekStrandSegmentData(seg_handle);
  if (strand_segment_data.profile_particle_handle == -1) {
    return nullptr;
  }
  const auto& node = skeleton.PeekNode(strand_segment_data.node_handle);
  const auto& start_profile = node.data.profile;
  const auto& start_particle = start_profile.PeekParticle(strand_segment_data.profile_particle_handle);

  return &start_particle;
}