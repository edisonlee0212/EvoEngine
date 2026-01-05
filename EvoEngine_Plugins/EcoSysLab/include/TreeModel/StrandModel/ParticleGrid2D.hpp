#pragma once
#include "CellGrid.hpp"
#include "ProfileConstraints.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/**
 * @typedef ParticleHandle
 * @brief Integer type representing a handle to a particle.
 */
typedef int ParticleHandle;

/**
 * @class ParticleCell
 * @brief Represents a storage unit for particles in a 2D grid.
 */
class ParticleCell {
  template <typename PD>
  friend class StrandModelProfile;
  friend class ParticleGrid2D;

  static constexpr size_t cell_capacity = 4;                   ///< Maximum number of particles per cell.
  static constexpr size_t max_cell_index = cell_capacity - 1;  ///< Maximum valid cell index.

  size_t atom_count_ = 0;                            ///< Number of particles currently stored in the cell.
  ParticleHandle atom_handles_[cell_capacity] = {};  ///< Array storing the handles of contained particles.

 public:
  glm::vec2 target = glm::vec2(0.0f);  ///< Target position for the particles in the cell.

  /**
   * @brief Registers a particle handle in the cell.
   * @param handle The handle of the particle to register.
   */
  void RegisterParticle(ParticleHandle handle);

  /**
   * @brief Clears all particles from the cell.
   */
  void Clear();

  /**
   * @brief Unregisters a particle from the cell.
   * @param handle The handle of the particle to remove.
   */
  void UnregisterParticle(ParticleHandle handle);
};

/**
 * @class ParticleGrid2D
 * @brief Represents a 2D grid structure for managing particle positions.
 */
class ParticleGrid2D {
  glm::vec2 min_bound_ = glm::vec2(0.0f);  ///< Minimum boundary of the grid.
  glm::vec2 max_bound_ = glm::vec2(0.0f);  ///< Maximum boundary of the grid.
  float cell_size_ = 1.0f;                 ///< Size of each cell in the grid.
  glm::ivec2 resolution_ = {0, 0};         ///< Grid resolution in terms of number of cells.
  std::vector<ParticleCell> cells_{};      ///< Storage for all the cells in the grid.

  template <typename PD>
  friend class StrandModelProfile;

 public:
  /**
   * @brief Applies boundary constraints to the grid.
   * @param profile_boundaries The boundary constraints to apply.
   */
  void ApplyBoundaries(const ProfileConstraints& profile_boundaries);

  /**
   * @brief Default constructor.
   */
  ParticleGrid2D() = default;

  /**
   * @brief Resets the grid with new parameters.
   * @param cell_size The size of each cell.
   * @param min_bound The minimum bound of the grid.
   * @param resolution The resolution of the grid.
   */
  void Reset(float cell_size, const glm::vec2& min_bound, const glm::ivec2& resolution);

  /**
   * @brief Resets the grid with new parameters.
   * @param cell_size The size of each cell.
   * @param min_bound The minimum bound of the grid.
   * @param max_bound The maximum bound of the grid.
   */
  void Reset(float cell_size, const glm::vec2& min_bound, const glm::vec2& max_bound);

  /**
   * @brief Registers a particle at a given position.
   * @param position The position of the particle.
   * @param handle The handle of the particle.
   */
  void RegisterParticle(const glm::vec2& position, ParticleHandle handle);

  /**
   * @brief Computes the grid coordinate corresponding to a position.
   * @param position The position to convert.
   * @return The coordinate in the grid.
   */
  [[nodiscard]] glm::ivec2 GetCoordinate(const glm::vec2& position) const;

  /**
   * @brief Computes the grid coordinate corresponding to a cell index.
   * @param index The cell index.
   * @return The coordinate in the grid.
   */
  [[nodiscard]] glm::ivec2 GetCoordinate(unsigned index) const;

  /**
   * @brief Retrieves a reference to the cell at a given position.
   * @param position The position to query.
   * @return Reference to the corresponding ParticleCell.
   */
  [[nodiscard]] ParticleCell& RefCell(const glm::vec2& position);

  /**
   * @brief Retrieves a reference to the cell at a given coordinate.
   * @param coordinate The grid coordinate to query.
   * @return Reference to the corresponding ParticleCell.
   */
  [[nodiscard]] ParticleCell& RefCell(const glm::ivec2& coordinate);

  /**
   * @brief Provides read-only access to all the cells.
   * @return A const reference to the vector of cells.
   */
  [[nodiscard]] const std::vector<ParticleCell>& PeekCells() const;

  /**
   * @brief Computes the world position corresponding to a grid coordinate.
   * @param coordinate The grid coordinate.
   * @return The corresponding world position.
   */
  [[nodiscard]] glm::vec2 GetPosition(const glm::ivec2& coordinate) const;

  /**
   * @brief Computes the world position corresponding to a cell index.
   * @param index The index of the cell.
   * @return The corresponding world position.
   */
  [[nodiscard]] glm::vec2 GetPosition(unsigned index) const;

  /**
   * @brief Clears the grid of all particles.
   */
  void Clear();
};
}  // namespace eco_sys_lab_plugin