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
  /// When true, OBJ export writes one `o` object per segment meshlet (grouped by @c segment_index).
  static bool per_meshlet_objects;

  /// Grouping for @ref ExportVisualizationObj (matches Visualization "Segment mode" Segment/Strand color).
  enum class VisualizationGrouping {
    Segments = 0,
    Strands = 1,
  };
  static VisualizationGrouping visualization_grouping;

  /// Object grouping for intersect-and-export visualization OBJ (Intersection Meshes group UI).
  enum class IntersectionVisualizationGrouping {
    IntersectionMeshes = 0,
    Strands = 1,
    Segments = 2,
  };
  static IntersectionVisualizationGrouping intersection_visualization_grouping;

  struct MeshGroup {
    std::string name;
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex> vertices;
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle> triangles;
  };

  static kinDS::VoronoiMesh ToVoronoiMesh(
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles, float fracture_distance = 0.0f,
      bool neighbor_connectivity_debug = false,
      const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs = {});

  static kinDS::ObjExportGpuAttributes BuildGpuAttributes(
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
      const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor);

  static void AppendGpuAttributes(kinDS::ObjExportGpuAttributes& dst, const kinDS::ObjExportGpuAttributes& src);

  /// Merge currently co-located meshlet vertices that still share an intact segment-pair connection:
  /// group by exact @c x0, build a connection graph from active rod-element neighbors, then replace each
  /// connected component with its mean @c x. Unmatched vertices on the same segment-pair triangle sheet
  /// For each owner segment, all triangles sharing a @c segment_pair_index are collected first; one similarity
  /// transform is fit from all matched control points on that sheet and applied to unmatched vertices there.
  /// Bark/cut/grey triangles (@c segment_pair_index < 0) are handled per owner segment.
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

  /// One OBJ object per segment or strand; solid materials match Visualization Segment/Strand color
  /// (unique RGB values only — shared materials across objects).
  static void ExportVisualizationObj(const std::filesystem::path& path,
                                     const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                                     const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                                     const std::vector<DynamicStrands::GpuSegment>& segments,
                                     VisualizationGrouping grouping, double uv_height_factor = 1.0,
                                     double uv_circum_factor = 1.0, float fracture_distance = 0.0f,
                                     const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs = {},
                                     const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list = {});

  /// Visualization-colored OBJ for multiple intersection results.
  /// @p grouping selects one object per input mesh, or subdivides each mesh into strand/segment objects
  /// (prefixed by the input mesh name). Materials are deduplicated by RGB.
  static void ExportVisualizationObjCombined(const std::filesystem::path& path, const std::vector<MeshGroup>& groups,
                                             const std::vector<DynamicStrands::GpuSegment>& segments,
                                             IntersectionVisualizationGrouping grouping,
                                             double uv_height_factor = 1.0, double uv_circum_factor = 1.0,
                                             float fracture_distance = 0.0f,
                                             const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs = {},
                                             const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list = {});
};

}  // namespace eco_sys_lab_plugin
