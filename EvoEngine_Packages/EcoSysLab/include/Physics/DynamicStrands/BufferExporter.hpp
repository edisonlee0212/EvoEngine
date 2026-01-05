#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>
#include "DsAlphaShapeMeshing.hpp"
#include "DsKineticVoronoiMeshing.hpp"
#include "DynamicStrands.hpp"
#include "kinDS/kinDS/ObjExporter.hpp"
#include "kinDS/kinDS/VoronoiMesh.hpp"

namespace eco_sys_lab_package {

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
  /// When true (default), combined OBJ export writes bark faces as object `bark` and the rest as `interior`.
  static bool separate_bark_obj_group;

  /// Color / highlight source for visualization OBJ materials (Visualization Segment/Strand color).
  enum class VisualizationColorMode {
    Segments = 0,
    Strands = 1,
  };
  /// Shared by regular and intersect visualization exports.
  static VisualizationColorMode visualization_color_mode;

  /// How faces are packed into OBJ objects for visualization export.
  enum class VisualizationObjectGrouping {
    /// One combined object (regular export) — faces still colored by @ref visualization_color_mode.
    Combined = 0,
    /// One object per intersection boundary result (intersect export only).
    IntersectionMeshes = 1,
    /// One object per strand or segment, matching @ref visualization_color_mode.
    ByHighlight = 2,
  };
  static VisualizationObjectGrouping visualization_object_grouping;
  static VisualizationObjectGrouping intersection_visualization_object_grouping;

  struct MeshGroup {
    std::string name;
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex> vertices;
    std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle> triangles;
  };

  static kinDS::VoronoiMesh ToVoronoiMesh(
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles, float fracture_distance = 0.0f,
      bool neighbor_connectivity_debug = false, const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs = {},
      const std::vector<std::string>& vertex_metadata = {}, const std::vector<std::string>& face_metadata = {});

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
                        const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list = {},
                        const std::vector<std::string>& vertex_metadata = {},
                        const std::vector<std::string>& face_metadata = {});

  static void ExportObjCombined(const std::filesystem::path& path, const std::vector<MeshGroup>& groups,
                                const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor = 1.0,
                                double uv_circum_factor = 1.0, float fracture_distance = 0.0f,
                                const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs = {},
                                const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list = {});

  /// Visualization-colored OBJ. @p color_mode selects Strand/Segment solid colors; @p object_grouping
  /// selects Combined (one object, per-face colors) or ByHighlight (one object per strand/segment).
  static void ExportVisualizationObj(const std::filesystem::path& path,
                                     const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                                     const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                                     const std::vector<DynamicStrands::GpuSegment>& segments,
                                     VisualizationColorMode color_mode, VisualizationObjectGrouping object_grouping,
                                     double uv_height_factor = 1.0, double uv_circum_factor = 1.0,
                                     float fracture_distance = 0.0f,
                                     const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs = {},
                                     const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list = {});

  /// Visualization-colored OBJ for multiple intersection results.
  /// @p object_grouping: IntersectionMeshes (one object per input) or ByHighlight (strand/segment objects).
  static void ExportVisualizationObjCombined(const std::filesystem::path& path, const std::vector<MeshGroup>& groups,
                                             const std::vector<DynamicStrands::GpuSegment>& segments,
                                             VisualizationColorMode color_mode,
                                             VisualizationObjectGrouping object_grouping, double uv_height_factor = 1.0,
                                             double uv_circum_factor = 1.0, float fracture_distance = 0.0f,
                                             const std::vector<DynamicStrands::GpuSegmentPair>& segment_pairs = {},
                                             const std::vector<DynamicStrands::GpuSegmentData>& segment_data_list = {});
};

/**
 * \brief Export Alpha Shape Delaunay tetrahedra to OBJ after a GPU download.
 * Alive tets only (@c inside == 1). Faces follow the GPU lookup table in AlphaShape.glsl.
 * Writes bark/interior materials + UVs via kinDS::ObjExporter, plus a GPU-attribute JSON sidecar.
 */
class AlphaShapeTetObjExport {
 public:
  /// When true (and @ref separate_bark_obj_group is false), write one @c o object per alive tetrahedron.
  static bool separate_tet_objects;
  /// When true (default), write OBJ groups @c bark then @c interior (Kinetic meshlet style).
  static bool separate_bark_obj_group;
  /// When true (default), only export surface triangles matching the mesh shader filter
  /// (@c render_neighbor == 1, else missing/dead neighbor). Distinct from bark (@c is_bark);
  /// includes fracture/cut surfaces that appear over time.
  static bool surface_only;
  /// When true, use particle @c position; otherwise @c initial_position.
  static bool use_current_position;

  static void ExportObj(
      const std::filesystem::path& path, const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles,
      const std::vector<DsAlphaShapeMeshing::GpuDelaunayTetrahedron>& tetrahedrons,
      const std::unordered_map<uint64_t, std::vector<glm::dvec2>>* profile_bundle_boundary_polygons = nullptr);
};

/**
 * \brief Volume-change heatmap OBJ: materials are named by % volume change (e.g. @c pct_+12.3)
 * and colored white at 0%, red for losses, blue for gains.
 * Single-mesh exports clamp display colors to @ref max_abs_percent (±).
 * Dual export uses @ref ColorScale so [min,0] and [0,max] are normalized independently (0% stays white).
 * Alpha heatmap mesh keeps live tetrahedra including those near-degenerate at initialization
 * (assigned 0% change). Cumulative / per-tet stats still exclude init-time near-degenerate tets.
 * When @ref AlphaShapeTetObjExport::surface_only is set (default), only surface faces are written
 * (same @c render_neighbor rule as Branches/Rendering.mesh).
 */
class VolumeChangeHeatmapExport {
 public:
  /// Display clamp for single-mesh material colors (± percent). Stored baseline comparisons are unclamped for logging.
  static double max_abs_percent;

  struct ChangeStats {
    double initial_cumulative = 0.0;
    double current_cumulative = 0.0;
    double cumulative_delta = 0.0;
    double cumulative_percent = 0.0;
    double max_percent = 0.0;  ///< Largest gain (or least negative).
    double min_percent = 0.0;  ///< Largest loss among elements with current volume > 0.
    bool has_max = false;
    bool has_min = false;
    unsigned int max_id = 0;
    unsigned int min_id = 0;
  };

  /// Asymmetric color extents: white at 0%, full red at @c min_percent, full blue at @c max_percent.
  struct ColorScale {
    double min_percent = -30.0;  ///< Most negative extent (≤ 0).
    double max_percent = 30.0;   ///< Most positive extent (≥ 0).

    static ColorScale Symmetric(double abs_percent);
    /// Merge element min/max from one or more @ref ChangeStats (0 is always kept as the white pivot).
    static ColorScale FromStats(const ChangeStats& a);
    static ColorScale FromStats(const ChangeStats& a, const ChangeStats& b);
  };

  static glm::dvec3 ColorFromPercent(double percent);
  static glm::dvec3 ColorFromPercent(double percent, const ColorScale& scale);

  static void LogStats(const std::string& label, const ChangeStats& stats);

  static ChangeStats ComputeKineticStats(
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
      const std::unordered_map<unsigned int, double>& initial_volumes_by_segment, double initial_cumulative);

  static ChangeStats ComputeAlphaStats(const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles,
                                       const std::vector<DsAlphaShapeMeshing::GpuDelaunayTetrahedron>& tetrahedrons,
                                       const std::vector<double>& initial_tet_volumes,
                                       const std::vector<uint8_t>& initial_near_degenerate_tets,
                                       double initial_cumulative, bool use_current_position = true);

  static void ExportKineticMeshlets(const std::filesystem::path& path,
                                    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                                    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                                    const std::unordered_map<unsigned int, double>& initial_volumes_by_segment,
                                    double initial_cumulative, ChangeStats* stats_out = nullptr,
                                    const ColorScale* color_scale = nullptr);

  static void ExportAlphaTetrahedra(const std::filesystem::path& path,
                                    const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& particles,
                                    const std::vector<DsAlphaShapeMeshing::GpuDelaunayTetrahedron>& tetrahedrons,
                                    const std::vector<double>& initial_tet_volumes,
                                    const std::vector<uint8_t>& initial_near_degenerate_tets, double initial_cumulative,
                                    bool use_current_position = true, ChangeStats* stats_out = nullptr,
                                    const ColorScale* color_scale = nullptr);

  /// Download-ready CPU buffers: write Kinetic + Alpha heatmaps with a shared ±@ref max_abs_percent color scale.
  static void ExportBothWithSharedScale(
      const std::filesystem::path& kinetic_path, const std::filesystem::path& alpha_path,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& kinetic_vertices,
      const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& kinetic_triangles,
      const std::unordered_map<unsigned int, double>& kinetic_initial_volumes, double kinetic_initial_cumulative,
      const std::vector<DsAlphaShapeMeshing::GpuUniformParticle>& alpha_particles,
      const std::vector<DsAlphaShapeMeshing::GpuDelaunayTetrahedron>& alpha_tetrahedrons,
      const std::vector<double>& alpha_initial_tet_volumes, const std::vector<uint8_t>& alpha_near_degenerate_tets,
      double alpha_initial_cumulative, bool alpha_use_current_position = true);
};

}  // namespace eco_sys_lab_package
