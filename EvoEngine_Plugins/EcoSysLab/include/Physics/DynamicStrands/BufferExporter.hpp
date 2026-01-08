#pragma once

#include <fstream>
#include <string>
#include <vector>
#include "DsKineticVoronoiMeshing.hpp"
#include "DynamicStrands.hpp"

namespace eco_sys_lab_plugin {

class PlyExporter {
 public:
  static void ExportAscii(const std::filesystem::path& path,
                          const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                          const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                          double uv_height_factor = 1.0, double uv_circum_factor = 1.0);

 private:
  static void WriteHeader(std::ofstream& file, size_t vertex_count, size_t face_count);

  static void WriteVertices(std::ofstream& file,
                            const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices);

  static void WriteFaces(std::ofstream& file,
                         const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                         double uv_height_factor, double uv_circum_factor);
};

class ObjExporter {
 public:
  static void ExportObj(const std::filesystem::path& path,
                        const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                        const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                        const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor = 1.0,
                        double uv_circum_factor = 1.0);

 private:
  static void WriteMtl(const std::filesystem::path& mtl_path);

  static void WriteObj(const std::filesystem::path& obj_path, const std::filesystem::path& mtl_path,
                       const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                       const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                       double uv_height_factor, double uv_circum_factor);
  static void WriteJson(const std::filesystem::path& json_path,
                        const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                        const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                        const std::vector<DynamicStrands::GpuSegment>& segments);
};
}  // namespace eco_sys_lab_plugin
