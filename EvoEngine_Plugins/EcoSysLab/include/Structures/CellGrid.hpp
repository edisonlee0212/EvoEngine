#pragma once
#include "Jobs.hpp"

namespace eco_sys_lab_plugin {
using namespace evo_engine;

/* Coordinate system

The cell position is its center.
Each cell is dx wide.

                                <-dx ->
                                -------------------------
                                |     |     |     |     |
                                |  x  |  x  |  x  |  x  |
                                |     |     |     |     |
                                -------------------------
                                   |     |     |     |
                                   |     |     |     |
X-Coordinate:   -- 0 --- 1 --- 2 --- 3 -----

The "min_bound_" stores the lower left corner of the lower left cell.
I.e. for min_bound_ = (0, 0) and resolution_ = (2, 2), and m_size = 1,
the cell centers are at 0.5 and 1.5.

*/
/**
 * @brief Represents a 2D grid of cells with procedural positioning and access methods.
 *
 * @tparam CellData The data type stored in each cell.
 */
template <typename CellData>
class CellGrid {
  glm::vec2 min_bound_ = glm::vec2(0.0f);  ///< The minimum boundary of the grid.
  glm::vec2 max_bound_ = glm::vec2(0.0f);  ///< The maximum boundary of the grid.
  float cell_size_ = 1.0f;                 ///< The size of each cell.
  glm::ivec2 resolution_ = {0, 0};         ///< The resolution of the grid (number of cells in x and y directions).
  std::vector<CellData> cells_{};          ///< The storage for grid cells.

 public:
  /**
   * @brief Default destructor.
   */
  virtual ~CellGrid() = default;

  /**
   * @brief Gets the minimum boundary of the grid.
   * @return The minimum boundary as a glm::vec2.
   */
  [[nodiscard]] glm::vec2 GetMinBound() const;

  /**
   * @brief Gets the maximum boundary of the grid.
   * @return The maximum boundary as a glm::vec2.
   */
  [[nodiscard]] glm::vec2 GetMaxBound() const;

  /**
   * @brief Gets the size of a cell.
   * @return The size of a single cell.
   */
  [[nodiscard]] float GetCellSize() const;

  /**
   * @brief Gets the resolution of the grid.
   * @return The grid resolution as a glm::ivec2.
   */
  [[nodiscard]] glm::ivec2 GetResolution() const;

  /**
   * @brief Default constructor.
   */
  CellGrid() = default;

  /**
   * @brief Resets the grid with new parameters.
   * @param cell_size The size of each cell.
   * @param min_bound The minimum boundary of the grid.
   * @param resolution The resolution of the grid.
   */
  void Reset(float cell_size, const glm::vec2& min_bound, const glm::ivec2& resolution);

  /**
   * @brief Resets the grid using minimum and maximum bounds.
   * @param cell_size The size of each cell.
   * @param min_bound The minimum boundary of the grid.
   * @param max_bound The maximum boundary of the grid.
   */
  void Reset(float cell_size, const glm::vec2& min_bound, const glm::vec2& max_bound);

  /**
   * @brief Calculates the grid coordinates for a given position.
   * @param position The position in world space.
   * @return The grid coordinates corresponding to the position.
   */
  [[nodiscard]] glm::ivec2 GetCoordinate(const glm::vec2& position) const;

  /**
   * @brief Retrieves the grid coordinates from a cell index.
   * @param index The index of the cell.
   * @return The grid coordinates as glm::ivec2.
   */
  [[nodiscard]] glm::ivec2 GetCoordinate(unsigned index) const;

  /**
   * @brief Accesses a cell reference based on a position.
   * @param position The position in world space.
   * @return A reference to the cell at the given position.
   */
  [[nodiscard]] CellData& RefCell(const glm::vec2& position);

  /**
   * @brief Accesses a cell reference based on grid coordinates.
   * @param coordinate The coordinate of the cell.
   * @return A reference to the cell at the given coordinate.
   */
  [[nodiscard]] CellData& RefCell(const glm::ivec2& coordinate);

  /**
   * @brief Accesses a cell reference based on its index.
   * @param index The index of the cell.
   * @return A reference to the cell at the given index.
   */
  [[nodiscard]] CellData& RefCell(unsigned index);

  /**
   * @brief Provides read-only access to all grid cells.
   * @return A const reference to the vector of cells.
   */
  [[nodiscard]] const std::vector<CellData>& PeekCells() const;

  /**
   * @brief Provides a modifiable reference to all grid cells.
   * @return A reference to the vector of cells.
   */
  [[nodiscard]] std::vector<CellData>& RefCells();

  /**
   * @brief Computes the position of a cell from its grid coordinates.
   * @param coordinate The grid coordinates.
   * @return The world-space position of the cell center.
   */
  [[nodiscard]] glm::vec2 GetPosition(const glm::ivec2& coordinate) const;

  /**
   * @brief Computes the position of a cell from its index.
   * @param index The index of the cell.
   * @return The world-space position of the cell center.
   */
  [[nodiscard]] glm::vec2 GetPosition(unsigned index) const;

  /**
   * @brief Iterates over cells within a given radius of a position and applies a function to each.
   * @param position The central position in world space.
   * @param radius The search radius.
   * @param func The function to apply to each found cell.
   */
  void ForEach(const glm::vec2& position, float radius, const std::function<void(CellData& data)>& func);

  /**
   * @brief Clears the cell data.
   */
  virtual void Clear() = 0;
};

template <typename CellData>
glm::vec2 CellGrid<CellData>::GetMinBound() const {
  return min_bound_;
}

template <typename CellData>
glm::vec2 CellGrid<CellData>::GetMaxBound() const {
  return max_bound_;
}

template <typename CellData>
float CellGrid<CellData>::GetCellSize() const {
  return cell_size_;
}

template <typename CellData>
glm::ivec2 CellGrid<CellData>::GetResolution() const {
  return resolution_;
}

template <typename CellData>
void CellGrid<CellData>::Reset(const float cell_size, const glm::vec2& min_bound, const glm::ivec2& resolution) {
  resolution_ = resolution;
  cell_size_ = cell_size;
  min_bound_ = min_bound;
  max_bound_ = min_bound + cell_size * glm::vec2(resolution);
  cells_.resize(resolution.x * resolution.y);
}

template <typename CellData>
void CellGrid<CellData>::Reset(const float cell_size, const glm::vec2& min_bound, const glm::vec2& max_bound) {
  Reset(cell_size, min_bound,
        glm::ivec2(glm::ceil((max_bound.x - min_bound.x) / cell_size) + 1,
                   glm::ceil((max_bound.y - min_bound.y) / cell_size) + 1));
}

template <typename CellData>
glm::ivec2 CellGrid<CellData>::GetCoordinate(const glm::vec2& position) const {
  const auto coordinate =
      glm::ivec2(floor((position.x - min_bound_.x) / cell_size_), floor((position.y - min_bound_.y) / cell_size_));
  assert(coordinate.x < resolution_.x && coordinate.y < resolution_.y);
  return coordinate;
}

template <typename CellData>
glm::ivec2 CellGrid<CellData>::GetCoordinate(const unsigned index) const {
  return {index % resolution_.x, index / resolution_.x};
}

template <typename CellData>
CellData& CellGrid<CellData>::RefCell(const glm::vec2& position) {
  const auto coordinate =
      glm::ivec2(glm::clamp(static_cast<int>((position.x - min_bound_.x) / cell_size_), 0, resolution_.x - 1),
                 glm::clamp(static_cast<int>((position.y - min_bound_.y) / cell_size_), 0, resolution_.y - 1));
  const auto cell_index = coordinate.x + coordinate.y * resolution_.x;
  return cells_[cell_index];
}

template <typename CellData>
CellData& CellGrid<CellData>::RefCell(const glm::ivec2& coordinate) {
  const auto cell_index = coordinate.x + coordinate.y * resolution_.x;
  return cells_[cell_index];
}

template <typename CellData>
CellData& CellGrid<CellData>::RefCell(const unsigned index) {
  return cells_[index];
}

template <typename CellData>
const std::vector<CellData>& CellGrid<CellData>::PeekCells() const {
  return cells_;
}

template <typename CellData>
std::vector<CellData>& CellGrid<CellData>::RefCells() {
  return cells_;
}

template <typename CellData>
glm::vec2 CellGrid<CellData>::GetPosition(const glm::ivec2& coordinate) const {
  return min_bound_ + cell_size_ * glm::vec2(coordinate.x + 0.5f, coordinate.y + 0.5f);
}

template <typename CellData>
glm::vec2 CellGrid<CellData>::GetPosition(const unsigned index) const {
  const auto coordinate = GetCoordinate(index);
  return min_bound_ + cell_size_ * glm::vec2(coordinate.x + 0.5f, coordinate.y + 0.5f);
}

template <typename CellData>
void CellGrid<CellData>::ForEach(const glm::vec2& position, const float radius,
                                 const std::function<void(CellData& data)>& func) {
  const auto actual_center = position - min_bound_;
  const auto actual_min_bound = actual_center - glm::vec2(radius);
  const auto actual_max_bound = actual_center + glm::vec2(radius);
  const auto start = glm::ivec2(glm::floor(actual_min_bound / glm::vec2(cell_size_)));
  const auto end = glm::ivec2(glm::ceil(actual_max_bound / glm::vec2(cell_size_)));
  for (int i = start.x; i <= end.x; i++) {
    for (int j = start.y; j <= end.y; j++) {
      if (i < 0 || i >= resolution_.x || j < 0 || j >= resolution_.y)
        continue;
      func(RefCell(glm::ivec2(i, j)));
    }
  }
}
}  // namespace eco_sys_lab_plugin
