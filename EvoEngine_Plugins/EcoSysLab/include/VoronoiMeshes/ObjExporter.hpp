#pragma once

#include <fstream>
#include <limits>
#include <string>
#include "VoronoiMesh.hpp"

namespace kinDS {
class ObjExporter {
 private:
  static void writeFaces(std::ofstream& file, const VoronoiMesh& mesh, size_t lb, size_t ub) {
    const auto& indices = mesh.getTriangles();
    const auto& uv_indices = mesh.getUVIndices();
    const auto& normals = mesh.getNormals();

    size_t material_id = -1;
    const auto& material_ids = mesh.getMaterialIDs();
    for (size_t i = 3 * lb; i < 3 * ub; i += 3) {
      // Check if we need to switch material
      if (!material_ids.empty()) {
        size_t current_material_id = material_ids[i / 3];
        if (current_material_id != material_id) {
          material_id = current_material_id;
          if (material_id < mesh.getMaterialNames().size()) {
            file << "usemtl " << mesh.getMaterialNames()[material_id] << "\n";
          }
        }
      }
      file << "f";

      for (size_t j = 0; j < 3; j++) {
        file << " " << (indices[i + j] + 1);  // 1-based index in OBJ

        size_t normal_index;
        if (mesh.getNormalMode() == NormalMode::PerTriangleCorner) {
          normal_index = i + j;
        } else {
          normal_index = indices[i + j];
        }

        bool has_uv = uv_indices[i + j] != std::numeric_limits<size_t>::max();
        bool has_normal = normals.size() > normal_index;

        if (has_uv || has_normal) {
          file << "/";
        }
        if (has_uv) {
          file << std::to_string(uv_indices[i + j] + 1);
        }
        if (has_normal) {
          file << "/" << std::to_string(normal_index + 1);
        }
      }
      file << "\n";
    }
  }

  static void writeMtl(const std::filesystem::path& mtl_path) {
    std::ofstream file(mtl_path);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open MTL file");
    }

    // TODO: Define proper materials or perhaps pass them as arguments

    // Bark material
    file << "newmtl bark\n";
    file << "Ka 0.2 0.1 0.05\n";
    file << "Kd 0.4 0.25 0.1\n";
    file << "Ks 0.0 0.0 0.0\n";
    file << "d 1.0\n\n";

    // Interior material
    file << "newmtl interior\n";
    file << "Ka 0.8 0.8 0.8\n";
    file << "Kd 0.8 0.8 0.8\n";
    file << "Ks 0.0 0.0 0.0\n";
    file << "d 1.0\n";

    file.close();
  }

 public:
  static void writeMesh(const VoronoiMesh& mesh, const std::filesystem::path& obj_path, double uv_height_factor = 1.0,
                        double uv_circum_factor = 1.0) {
    std::ofstream file(obj_path);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file for writing: " + obj_path.filename().string());
    }
    std::filesystem::path mtl_path = obj_path;
    mtl_path.replace_extension(".mtl");
    writeMtl(mtl_path);

    // Write some metadata
    file << "# Exported by kinDS ObjExporter\n";
    file << "mtllib " << mtl_path.filename() << "\n\n";

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
    for (size_t i = 0; i < mesh.getTriangleCount(); i++) {
      int material = mesh.getMaterialIDs()[i];

      for (size_t j = 0; j < 3; j++) {
        auto uv = mesh.getUV(3 * i + j);

        if (material == 0) {
          uv[0] *= uv_circum_factor;
          uv[1] *= uv_height_factor;
        } else {
          uv[2] *= uv_height_factor;
        }
        file << "vt " << uv[0] << " " << uv[1] << " " << uv[2] << "\n";
      }
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
      size_t ub = mesh.getTriangles().size() / 3;
      writeFaces(file, mesh, lb, ub);
    } else {
      // No groups defined, write all faces
      writeFaces(file, mesh, 0, mesh.getTriangles().size() / 3);
    }

    file.close();
  }
};
}  // namespace kinDS