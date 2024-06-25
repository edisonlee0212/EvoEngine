#pragma once
#include "Jobs.hpp"
using namespace evo_engine;
namespace eco_sys_lab {
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

template <typename CellData>
class CellGrid {
  glm::vec2 min_bound_ = glm::vec2(0.0f);
  glm::vec2 max_bound_ = glm::vec2(0.0f);
  float cell_size_ = 1.0f;
  glm::ivec2 resolution_ = {0, 0};
  std::vector<CellData> cells_{};

 public:
  virtual ~CellGrid() = default;
  [[nodiscard]] glm::vec2 GetMinBound() const;
  [[nodiscard]] glm::vec2 GetMaxBound() const;
  [[nodiscard]] float GetCellSize() const;
  [[nodiscard]] glm::ivec2 GetResolution() const;
  CellGrid() = default;
  void Reset(float cell_size, const glm::vec2& min_bound, const glm::ivec2& resolution);
  void Reset(float cell_size, const glm::vec2& min_bound, const glm::vec2& max_bound);
  [[nodiscard]] glm::ivec2 GetCoordinate(const glm::vec2& position) const;
  [[nodiscard]] glm::ivec2 GetCoordinate(unsigned index) const;
  [[nodiscard]] CellData& RefCell(const glm::vec2& position);
  [[nodiscard]] CellData& RefCell(const glm::ivec2& coordinate);
  [[nodiscard]] CellData& RefCell(unsigned index);
  [[nodiscard]] const std::vector<CellData>& PeekCells() const;
  [[nodiscard]] std::vector<CellData>& RefCells();
  [[nodiscard]] glm::vec2 GetPosition(const glm::ivec2& coordinate) const;
  [[nodiscard]] glm::vec2 GetPosition(unsigned index) const;

  void ForEach(const glm::vec2& position, float radius, const std::function<void(CellData& data)>& func);
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
}  // namespace eco_sys_lab
