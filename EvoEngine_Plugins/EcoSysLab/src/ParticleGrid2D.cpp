#include "ParticleGrid2D.hpp"

#include "TreeVisualizer.hpp"

using namespace eco_sys_lab;

void ParticleCell::RegisterParticle(ParticleHandle handle) {
  atom_handles_[atom_count_] = handle;
  atom_count_ += atom_count_ < max_cell_index;
}

void ParticleCell::Clear() {
  atom_count_ = 0;
}

void ParticleCell::UnregisterParticle(ParticleHandle handle) {
  for (int i = 0; i < atom_count_; i++) {
    if (atom_handles_[i] == handle) {
      atom_handles_[i] = atom_handles_[atom_count_ - 1];
      atom_count_--;
      return;
    }
  }
}

void ParticleGrid2D::ApplyBoundaries(const ProfileConstraints& profile_boundaries) {
  auto& cells = cells_;
  if (profile_boundaries.boundaries.empty() && profile_boundaries.attractors.empty()) {
    for (int cell_index = 0; cell_index < cells_.size(); cell_index++) {
      auto& cell = cells[cell_index];
      cell.target = -GetPosition(cell_index);
    }
  } else {
    for (int cell_index = 0; cell_index < cells_.size(); cell_index++) {
      auto& cell = cells[cell_index];
      const auto cell_position = GetPosition(cell_index);
      cell.target = profile_boundaries.GetTarget(cell_position);
    }
  }
}

void ParticleGrid2D::Reset(const float cell_size, const glm::vec2& min_bound, const glm::ivec2& resolution) {
  resolution_ = resolution;
  cell_size_ = cell_size;
  min_bound_ = min_bound;
  max_bound_ = min_bound + cell_size * glm::vec2(resolution);
  cells_.resize(resolution.x * resolution.y);
  Clear();
}

void ParticleGrid2D::Reset(const float cell_size, const glm::vec2& min_bound, const glm::vec2& max_bound) {
  Reset(cell_size, min_bound,
        glm::ivec2(glm::ceil((max_bound.x - min_bound.x) / cell_size) + 1,
                   glm::ceil((max_bound.y - min_bound.y) / cell_size) + 1));
}

void ParticleGrid2D::RegisterParticle(const glm::vec2& position, ParticleHandle handle) {
  const auto coordinate =
      glm::ivec2(floor((position.x - min_bound_.x) / cell_size_), floor((position.y - min_bound_.y) / cell_size_));
  assert(coordinate.x < resolution_.x && coordinate.y < resolution_.y);
  const auto cell_index = coordinate.x + coordinate.y * resolution_.x;
  cells_[cell_index].RegisterParticle(handle);
}

glm::ivec2 ParticleGrid2D::GetCoordinate(const glm::vec2& position) const {
  const auto coordinate =
      glm::ivec2(floor((position.x - min_bound_.x) / cell_size_), floor((position.y - min_bound_.y) / cell_size_));
  assert(coordinate.x < resolution_.x && coordinate.y < resolution_.y);
  return coordinate;
}

glm::ivec2 ParticleGrid2D::GetCoordinate(const unsigned index) const {
  return {index % resolution_.x, index / resolution_.x};
}

ParticleCell& ParticleGrid2D::RefCell(const glm::vec2& position) {
  const auto coordinate =
      glm::ivec2(glm::clamp(static_cast<int>((position.x - min_bound_.x) / cell_size_), 0, resolution_.x - 1),
                 glm::clamp(static_cast<int>((position.y - min_bound_.y) / cell_size_), 0, resolution_.y - 1));
  const auto cell_index = coordinate.x + coordinate.y * resolution_.x;
  return cells_[cell_index];
}

ParticleCell& ParticleGrid2D::RefCell(const glm::ivec2& coordinate) {
  const auto cell_index = coordinate.x + coordinate.y * resolution_.x;
  return cells_[cell_index];
}

const std::vector<ParticleCell>& ParticleGrid2D::PeekCells() const {
  return cells_;
}

glm::vec2 ParticleGrid2D::GetPosition(const glm::ivec2& coordinate) const {
  return min_bound_ + cell_size_ * glm::vec2(coordinate.x + 0.5f, coordinate.y + 0.5f);
}

glm::vec2 ParticleGrid2D::GetPosition(const unsigned index) const {
  const auto coordinate = GetCoordinate(index);
  return min_bound_ + cell_size_ * glm::vec2(coordinate.x + 0.5f, coordinate.y + 0.5f);
}

void ParticleGrid2D::Clear() {
  for (auto& cell : cells_) {
    cell.atom_count_ = 0;
  }
}
