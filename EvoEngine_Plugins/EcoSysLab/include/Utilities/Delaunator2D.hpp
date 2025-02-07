
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

/**
 * @struct compare
 * @brief Comparator for sorting points based on their coordinates.
 */
struct compare {
  std::vector<float> const& coords;  ///< Reference to the list of coordinates.
  float cx;                          ///< X-coordinate of the center.
  float cy;                          ///< Y-coordinate of the center.

  /**
   * @brief Comparison operator for sorting.
   * @param i Index of the first point.
   * @param j Index of the second point.
   * @return True if the first point is less than the second.
   */
  bool operator()(std::size_t i, std::size_t j);
};

/**
 * @struct DelaunatorPoint
 * @brief Represents a point in the Delaunay triangulation.
 */
struct DelaunatorPoint {
  std::size_t i;     ///< Index of the point.
  float x;           ///< X-coordinate of the point.
  float y;           ///< Y-coordinate of the point.
  std::size_t t;     ///< Triangle index.
  std::size_t prev;  ///< Previous point in the hull.
  std::size_t next;  ///< Next point in the hull.
  bool removed;      ///< Flag indicating if the point is removed.
};

/**
 * @class Delaunator2D
 * @brief Performs 2D Delaunay triangulation.
 */
class Delaunator2D {
 public:
  std::vector<float> const& coords;    ///< Reference to the list of coordinates.
  std::vector<std::size_t> triangles;  ///< List of triangle vertex indices.
  std::vector<std::size_t> halfedges;  ///< List of half-edges.
  std::vector<std::size_t> hull_prev;  ///< Previous hull connections.
  std::vector<std::size_t> hull_next;  ///< Next hull connections.
  std::vector<std::size_t> hull_tri;   ///< Hull triangle indices.
  std::size_t hull_start;              ///< Index of the starting hull vertex.

  /**
   * @brief Constructs a new Delaunator2D object.
   * @param inCoords Input coordinates for triangulation.
   */
  Delaunator2D(std::vector<float> const& inCoords);

  /**
   * @brief Computes the hull area.
   * @return The computed hull area.
   */
  float get_hull_area();

 private:
  std::vector<std::size_t> m_hash;        ///< Hash table for spatial partitioning.
  float m_center_x;                       ///< X-coordinate of the triangulation center.
  float m_center_y;                       ///< Y-coordinate of the triangulation center.
  std::size_t m_hash_size;                ///< Size of the hash table.
  std::vector<std::size_t> m_edge_stack;  ///< Stack for handling edges.

  /**
   * @brief Legalizes the triangulation by swapping edges.
   * @param a Index of the edge to be legalized.
   * @return The new edge index.
   */
  std::size_t legalize(std::size_t a);

  /**
   * @brief Computes a hash key based on the coordinates.
   * @param x X-coordinate.
   * @param y Y-coordinate.
   * @return The computed hash key.
   */
  std::size_t hash_key(float x, float y) const;

  /**
   * @brief Adds a triangle to the triangulation.
   * @param i0 First vertex index.
   * @param i1 Second vertex index.
   * @param i2 Third vertex index.
   * @param a Halfedge opposite to i0.
   * @param b Halfedge opposite to i1.
   * @param c Halfedge opposite to i2.
   * @return The index of the added triangle.
   */
  std::size_t add_triangle(std::size_t i0, std::size_t i1, std::size_t i2, std::size_t a, std::size_t b, std::size_t c);

  /**
   * @brief Links two half-edges.
   * @param a First half-edge index.
   * @param b Second half-edge index.
   */
  void link(std::size_t a, std::size_t b);
};

}  // namespace Delaunator
