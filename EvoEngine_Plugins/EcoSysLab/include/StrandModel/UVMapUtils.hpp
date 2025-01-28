#pragma once

#include "StrandModel.hpp"

namespace eco_sys_lab_plugin {

class UVMapUtils {
 public:
  UVMapUtils();
  ~UVMapUtils();

  static bool IsValidPipeParam(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t);

  static float GetPipePolar(const Particle2D<CellParticlePhysicsData>& p0,
                            const Particle2D<CellParticlePhysicsData>& p1, float t);

  static float GetPipePolar(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t);

  static const Particle2D<CellParticlePhysicsData>* GetEndParticle(const StrandModel& strand_model,
                                                                   const StrandHandle& pipe_handle, size_t index);

  static const Particle2D<CellParticlePhysicsData>* GetEndParticle(const StrandModelSkeleton& skeleton,
                                                                   const StrandHandle& pipe_handle, size_t index);

  static const Particle2D<CellParticlePhysicsData>* GetStartParticle(const StrandModel& strand_model,
                                                                     const StrandHandle& pipe_handle, size_t index);

  static const Particle2D<CellParticlePhysicsData>* GetStartParticle(const StrandModelSkeleton& skeleton,
                                                                     const StrandHandle& pipe_handle, size_t index);
};

}  // namespace eco_sys_lab_plugin
