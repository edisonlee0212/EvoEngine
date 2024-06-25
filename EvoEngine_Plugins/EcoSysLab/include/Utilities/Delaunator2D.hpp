#pragma once

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <utility>
#include <vector>
// Source from https://github.com/delfrrr/delaunator-cpp

namespace Delaunator {
struct compare {
  std::vector<float> const& coords;
  float cx;
  float cy;

  bool operator()(std::size_t i, std::size_t j);
};

struct DelaunatorPoint {
  std::size_t i;
  float x;
  float y;
  std::size_t t;
  std::size_t prev;
  std::size_t next;
  bool removed;
};

class Delaunator2D {
 public:
  std::vector<float> const& coords;
  std::vector<std::size_t> triangles;
  std::vector<std::size_t> halfedges;
  std::vector<std::size_t> hull_prev;
  std::vector<std::size_t> hull_next;
  std::vector<std::size_t> hull_tri;
  std::size_t hull_start;

  Delaunator2D(std::vector<float> const& inCoords);

  float get_hull_area();

 private:
  std::vector<std::size_t> m_hash;
  float m_center_x;
  float m_center_y;
  std::size_t m_hash_size;
  std::vector<std::size_t> m_edge_stack;

  std::size_t legalize(std::size_t a);
  std::size_t hash_key(float x, float y) const;
  std::size_t add_triangle(std::size_t i0, std::size_t i1, std::size_t i2, std::size_t a, std::size_t b, std::size_t c);
  void link(std::size_t a, std::size_t b);
};
}  // namespace Delaunator
