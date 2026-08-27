#pragma once

#include <fstream>
#include <string>
#include <vector>
#include "DsKineticVoronoiMeshing.hpp"
#include "DynamicStrands.hpp"
#include "kinDS/kinDS/ObjExporter.hpp"
#include "kinDS/kinDS/VoronoiMesh.hpp"

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

/// Converts GPU meshlet buffers to VoronoiMesh and exports via kinDS::ObjExporter (framework mode).
class MeshletObjExport {
 public:
  /// Shared UI/export option: when true, @ref ApplySmoothing runs before writing the OBJ.
  static bool enable_smoothing;

  struct MeshGroup {
    std::string name;
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex> vertices;
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle> triangles;
  };

  static kinDS::VoronoiMesh ToVoronoiMesh(
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles, float fracture_distance = 0.0f);

  static kinDS::ObjExportGpuAttributes BuildGpuAttributes(
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
      const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor);

  static void AppendGpuAttributes(kinDS::ObjExportGpuAttributes& dst, const kinDS::ObjExportGpuAttributes& src);

  /// Merge currently co-located meshlet vertices that still share an intact segment-pair connection:
  /// group by exact @c x0, build a connection graph from active rod-element neighbors, then replace each
  /// connected component with its mean @c x.
  static void ApplySmoothing(std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                             std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                             const std::vector<DynamicStrands::GpuSegment>& segments,
                             const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs,
                             const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list);

  static void ExportObj(const std::filesystem::path& path,
                        const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                        const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                        const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor = 1.0,
                        double uv_circum_factor = 1.0, float fracture_distance = 0.0f,
                        const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs = {},
                        const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list = {});

  static void ExportObjCombined(const std::filesystem::path& path, const std::vector<MeshGroup>& groups,
                                const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor = 1.0,
                                double uv_circum_factor = 1.0, float fracture_distance = 0.0f,
                                const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs = {},
                                const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list = {});
};

}  // namespace eco_sys_lab_plugin
