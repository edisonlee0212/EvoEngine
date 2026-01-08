#include "BufferExporter.hpp"

using namespace eco_sys_lab_plugin;

void PlyExporter::ExportAscii(const std::filesystem::path& path,
                              const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                              const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                              double uv_height_factor, double uv_circum_factor) {
  std::ofstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open PLY file for writing");
  }

  WriteHeader(file, vertices.size(), triangles.size());
  WriteVertices(file, vertices);
  WriteFaces(file, triangles, uv_height_factor, uv_circum_factor);

  file.close();
}

void PlyExporter::WriteHeader(std::ofstream& file, size_t vertex_count, size_t face_count) {
  file << "ply\n";
  file << "format ascii 1.0\n";

  // Material convention
  file << "comment material 0 bark\n";
  file << "comment material 1 interior\n";

  // Vertices
  file << "element vertex " << vertex_count << "\n";
  file << "property float x\n";
  file << "property float y\n";
  file << "property float z\n";

  // Faces
  file << "element face " << face_count << "\n";
  file << "property list uchar int vertex_indices\n";
  file << "property list uchar float corner_normals\n";
  file << "property list uchar float corner_uvs\n";
  file << "property int material_id\n";

  file << "end_header\n";
}

void PlyExporter::WriteVertices(std::ofstream& file,
                                const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices) {
  for (const auto& v : vertices) {
    file << v.x.x << " " << v.x.y << " " << v.x.z << "\n";
  }
}

void PlyExporter::WriteFaces(std::ofstream& file,
                             const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                             double uv_height_factor, double uv_circum_factor) {
  for (const auto& t : triangles) {
    const int material_id = (t.neighbor_segment_index == -2) ? 0 : 1;

    // Vertex indices
    file << "3 " << t.vertex_index0 << " " << t.vertex_index1 << " " << t.vertex_index2 << " ";

    // Corner normals (3 * vec3)
    file << "9 ";
    for (int i = 0; i < 3; ++i) {
      file << t.normal[i].x << " " << t.normal[i].y << " " << t.normal[i].z << " ";
    }

    // Corner UVs (3 * vec2)
    file << "6 ";
    for (int i = 0; i < 3; ++i) {
      glm::vec2 uv = t.uv[i];

      if (material_id == 0) {
        uv.x *= uv_circum_factor;
        uv.y *= uv_height_factor;
      }

      file << uv.x << " " << uv.y << " ";
    }

    // Material
    file << material_id << "\n";
  }
}

void ObjExporter::ExportObj(const std::filesystem::path& obj_path,
                            const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                            const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                            const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor,
                            double uv_circum_factor) {
  std::filesystem::path mtl_path = obj_path;
  mtl_path.replace_extension(".mtl");
  std::filesystem::path json_path = obj_path;
  json_path.replace_extension(".json");

  WriteMtl(mtl_path);
  WriteObj(obj_path, mtl_path, vertices, triangles, uv_height_factor, uv_circum_factor);
  WriteJson(json_path, vertices, triangles, segments);
}

void ObjExporter::WriteMtl(const std::filesystem::path& mtl_path) {
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

void ObjExporter::WriteObj(const std::filesystem::path& obj_path, const std::filesystem::path& mtl_path,
                           const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                           const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                           double uv_height_factor, double uv_circum_factor) {
  std::ofstream file(obj_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open OBJ file");
  }

  file << "mtllib " << mtl_path.filename() << "\n\n";

  int index_offset = 1;  // OBJ indices are 1-based

  for (const auto& t : triangles) {
    const bool is_bark = (t.neighbor_segment_index == -2);
    file << "usemtl " << (is_bark ? "bark" : "interior") << "\n";

    const unsigned int v_idx[3] = {t.vertex_index0, t.vertex_index1, t.vertex_index2};

    // Write expanded vertices, normals, uvs
    for (int i = 0; i < 3; ++i) {
      const glm::vec3& p = vertices[v_idx[i]].x;
      const glm::vec4& n = t.normal[i];
      glm::vec2 uv = t.uv[i];

      if (is_bark) {
        uv.x *= uv_circum_factor;
        uv.y *= uv_height_factor;
      }

      file << "v  " << p.x << " " << p.y << " " << p.z << "\n";
      file << "vt " << uv.x << " " << uv.y << "\n";
      file << "vn " << n.x << " " << n.y << " " << n.z << "\n";
    }

    // Face (position/uv/normal)
    file << "f ";
    for (int i = 0; i < 3; ++i) {
      file << index_offset << "/" << index_offset << "/" << index_offset;
      if (i < 2)
        file << " ";
      ++index_offset;
    }
    file << "\n\n";
  }

  file.close();
}

void eco_sys_lab_plugin::ObjExporter::WriteJson(
    const std::filesystem::path& json_path,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
    const std::vector<DynamicStrands::GpuSegment>& segments) {
  std::ofstream file(json_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open JSON file");
  }

  auto write_values = [](std::ofstream& file, const std::string& key,
                         const std::function<std::string(size_t index)>& get_value, size_t size) {
    file << "  \"" << key << "\": [\n";
    for (size_t i = 0; i < size; ++i) {
      file << "    " << get_value(i);
      if (i < size - 1) {
        file << ",";
      }
      file << "\n";
    }
    file << "  ],\n";
  };

  file << "{\n";

  // write health values
  write_values(
      file, "health",
      [&](size_t index) {
        const auto& segment = segments[vertices[index].segment_index];
        return std::to_string(segment.HC);
      },
      vertices.size());

  // write moisture values
  write_values(
      file, "moisture",
      [&](size_t index) {
        const auto& segment = segments[vertices[index].segment_index];
        return std::to_string(segment.moisture);
      },
      vertices.size());

  // write stress values
  write_values(
      file, "stress",
      [&](size_t index) {
        const auto& segment = segments[vertices[index].segment_index];
        auto str = std::to_string(segment.shear_stretch_strain);
        return "[" + str + ", " + str + ", " + str + "]";
      },
      vertices.size());

  file << "}\n";
}
