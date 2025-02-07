
#pragma once

#include "StrandModel.hpp"

namespace eco_sys_lab_plugin {

/**
 * @brief Utility class for UV mapping operations related to strand models.
 */
class UVMapUtils {
 public:
  /**
   * @brief Default constructor for UVMapUtils.
   */
  UVMapUtils();

  /**
   * @brief Destructor for UVMapUtils.
   */
  ~UVMapUtils();

  /**
   * @brief Checks if the given pipe parameter is valid within the specified strand model.
   *
   * @param strand_model The strand model containing the pipe.
   * @param pipe_handle Handle to the pipe within the strand model.
   * @param t Parameter along the pipe's length (0 to 1).
   * @return True if the parameter is valid, false otherwise.
   */
  static bool IsValidPipeParam(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t);

  /**
   * @brief Computes the polar coordinate along a pipe segment defined by two particles.
   *
   * @param p0 The first particle of the segment.
   * @param p1 The second particle of the segment.
   * @param t Parameter along the segment's length (0 to 1).
   * @return The computed polar coordinate.
   */
  static float GetPipePolar(const Particle2D<CellParticlePhysicsData>& p0,
                            const Particle2D<CellParticlePhysicsData>& p1, float t);

  /**
   * @brief Computes the polar coordinate along a specified pipe within the strand model.
   *
   * @param strand_model The strand model containing the pipe.
   * @param pipe_handle Handle to the pipe within the strand model.
   * @param t Parameter along the pipe's length (0 to 1).
   * @return The computed polar coordinate.
   */
  static float GetPipePolar(const StrandModel& strand_model, const StrandHandle& pipe_handle, float t);

  /**
   * @brief Retrieves the end particle of a pipe within the strand model.
   *
   * @param strand_model The strand model containing the pipe.
   * @param pipe_handle Handle to the pipe within the strand model.
   * @param index The index of the particle (0 for first end, 1 for second end).
   * @return Pointer to the end particle, or nullptr if not found.
   */
  static const Particle2D<CellParticlePhysicsData>* GetEndParticle(const StrandModel& strand_model,
                                                                   const StrandHandle& pipe_handle, size_t index);

  /**
   * @brief Retrieves the end particle of a pipe within the strand model skeleton.
   *
   * @param skeleton The strand model skeleton containing the pipe.
   * @param pipe_handle Handle to the pipe within the skeleton.
   * @param index The index of the particle (0 for first end, 1 for second end).
   * @return Pointer to the end particle, or nullptr if not found.
   */
  static const Particle2D<CellParticlePhysicsData>* GetEndParticle(const StrandModelSkeleton& skeleton,
                                                                   const StrandHandle& pipe_handle, size_t index);

  /**
   * @brief Retrieves the start particle of a pipe within the strand model.
   *
   * @param strand_model The strand model containing the pipe.
   * @param pipe_handle Handle to the pipe within the strand model.
   * @param index The index of the particle (0 for first start, 1 for second start).
   * @return Pointer to the start particle, or nullptr if not found.
   */
  static const Particle2D<CellParticlePhysicsData>* GetStartParticle(const StrandModel& strand_model,
                                                                     const StrandHandle& pipe_handle, size_t index);

  /**
   * @brief Retrieves the start particle of a pipe within the strand model skeleton.
   *
   * @param skeleton The strand model skeleton containing the pipe.
   * @param pipe_handle Handle to the pipe within the skeleton.
   * @param index The index of the particle (0 for first start, 1 for second start).
   * @return Pointer to the start particle, or nullptr if not found.
   */
  static const Particle2D<CellParticlePhysicsData>* GetStartParticle(const StrandModelSkeleton& skeleton,
                                                                     const StrandHandle& pipe_handle, size_t index);
};

}  // namespace eco_sys_lab_plugin
