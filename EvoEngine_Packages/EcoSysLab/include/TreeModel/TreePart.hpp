#pragma once

namespace eco_sys_lab_package {
using namespace evo_engine;

/**
 * @struct JunctionLine
 * @brief Represents a line that connects junction points in the skeletal graph.
 */
struct JunctionLine {
  int line_index = -1;       ///< Index of the line in the skeletal graph.
  glm::vec3 start_position;  ///< Start position of this line.
  glm::vec3 end_position;    ///< End position of this line.
  float start_radius;        ///< Radius at the start of the line.
  float end_radius;          ///< Radius at the end of the line.

  glm::vec3 start_direction;  ///< Direction at the start.
  glm::vec3 end_direction;    ///< Direction at the end.
};

/**
 * @struct TreePartData
 * @brief Holds data representing different parts of a tree.
 */
struct TreePartData {
  int tree_part_index;                           ///< Index representing this tree part.
  bool is_junction = false;                      ///< Flag indicating if this part is a junction.
  JunctionLine base_line;                        ///< The base line associated with this tree part.
  std::vector<JunctionLine> children_lines;      ///< List of child lines originating from this part.
  std::vector<SkeletonNodeHandle> node_handles;  ///< Node handles corresponding to the skeletal structure.
  std::vector<bool> is_end;                      ///< List indicating if a node is an endpoint.
  std::vector<int> line_index;                   ///< List of indices referring to corresponding graph lines.

  int num_of_leaves = 0;  ///< Number of leaves attached to this part.
};
}  // namespace eco_sys_lab_package