#pragma once

#include <fstream>
#include <limits>
#include <string>
#include "VoronoiMesh.hpp"

namespace kinDS {
class ObjExporter {
 private:
  static void writeFaces(std::ofstream& file, const Mesh& mesh, size_t lb, size_t ub) {
    const auto& indices = mesh.getVertexIndices();
    const auto& uv_indices = mesh.getUVIndices();

    for (size_t i = 3 * lb; i < 3 * ub; i += 3) {
      file << "f " << (indices[i] + 1)
           << ((uv_indices[i] != std::numeric_limits<size_t>::max()) ? ("/" + std::to_string(uv_indices[i] + 1)) : "")
           << " " << (indices[i + 1] + 1)
           << ((uv_indices[i] != std::numeric_limits<size_t>::max()) ? ("/" + std::to_string(uv_indices[i + 1] + 1))
                                                                     : "")
           << " " << (indices[i + 2] + 1)
           << ((uv_indices[i] != std::numeric_limits<size_t>::max()) ? ("/" + std::to_string(uv_indices[i + 2] + 1))
                                                                     : "")
           << "\n";
    }
  }

 public:
  static void writeMesh(const Mesh& mesh, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file for writing: " + filename);
    }

    // Write some metadata
    file << "# Exported by kinDS ObjExporter\n";

    // Write vertices
    file << "# Vertices\n";
    for (const auto& vertex : mesh.getVertices()) {
      file << "v " << vertex[0] << " " << vertex[1] << " " << vertex[2] << "\n";
    }

    // Write normals
    file << "# Normals\n";
    for (const auto& normal : mesh.getNormals()) {
      file << "vn " << normal[0] << " " << normal[1] << " " << normal[2] << "\n";
    }

    // Write UVs
    file << "# UVs\n";
    for (const auto& uv : mesh.getUVs()) {
      file << "vt " << uv[0] << " " << uv[1] << "\n";
    }

    // Write faces
    file << "# Faces\n";
    size_t group_count = mesh.getGroupOffsets().size();

    if (group_count > 0) {
      for (size_t group_index = 0; group_index < group_count - 1; group_index++) {
        file << "o group_" << group_index << "\n";
        size_t lb = mesh.getGroupOffsets()[group_index];
        size_t ub = mesh.getGroupOffsets()[group_index + 1];
        writeFaces(file, mesh, lb, ub);
      }

      // Write the last group

      file << "o group_" << (group_count - 1) << "\n";
      size_t lb = mesh.getGroupOffsets().back();
      size_t ub = mesh.getVertexIndices().size() / 3;
      writeFaces(file, mesh, lb, ub);
    } else {
      // No groups defined, write all faces
      writeFaces(file, mesh, 0, mesh.getVertexIndices().size() / 3);
    }

    file.close();
  }
};
}  // namespace kinDS
