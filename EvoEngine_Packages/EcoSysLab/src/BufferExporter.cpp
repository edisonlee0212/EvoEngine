#include "BufferExporter.hpp"

using namespace eco_sys_lab_package;

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
      glm::vec4 uv = t.uv[i];

      if (material_id == 0) {
        uv.x *= uv_circum_factor;
        uv.y *= uv_height_factor;
      } else {
        uv.z *= uv_height_factor;
      }

      file << uv.x << " " << uv.y << " " << uv.z << " ";
    }

    // Material
    file << material_id << "\n";
  }
}

void ObjExporter::ExportObj(const std::filesystem::path& obj_path,
                            const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
                            const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
                            const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor,
                            double uv_circum_factor, float fracture_distance) {
  std::filesystem::path mtl_path = obj_path;
  mtl_path.replace_extension(".mtl");
  std::filesystem::path json_path = obj_path;
  json_path.replace_extension(".json");

  WriteMtl(mtl_path);
  WriteObj(obj_path, mtl_path, vertices, triangles, uv_height_factor, uv_circum_factor, fracture_distance);
  WriteJson(json_path, vertices, triangles, segments, uv_height_factor);
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
                           double uv_height_factor, double uv_circum_factor, float fracture_distance) {
  std::ofstream file(obj_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open OBJ file");
  }

  file << "# Exported from EcoSysLab\n";
  file << "mtllib " << mtl_path.filename() << "\n\n";

  const int index_offset = 1;  // OBJ indices are 1-based

  // first write all vertices
  file << "# Vertices\n";
  for (const auto& v : vertices) {
    const glm::vec3& p = v.x - fracture_distance * v.shift;
    file << "v  " << p.x << " " << p.y << " " << p.z << "\n";
  }

  // Sort triangles by material (bark vs interior)
  std::vector<size_t> bark_triangle_indices;
  std::vector<size_t> interior_triangle_indices;

  for (size_t i = 0; i < triangles.size(); i++) {
    if (triangles[i].neighbor_segment_index == -2) {
      bark_triangle_indices.push_back(i);
    } else {
      interior_triangle_indices.push_back(i);
    }
  }

  // First write all texture and normal coordinates
  file << "# Texture coordinates and normals\n";
  for (const auto& t : triangles) {
    // Write normals, uvs
    std::array<unsigned int, 3> v_idx = {t.vertex_index0, t.vertex_index1, t.vertex_index2};
    for (int i = 0; i < 3; ++i) {
      const glm::vec3& p = vertices[v_idx[i]].x;
      const glm::vec4& n = t.normal[i];
      glm::vec4 uv = t.uv[i];

      if (t.neighbor_segment_index == -2) {
        uv.x *= uv_circum_factor;
        uv.y *= uv_height_factor;
      } else {
        uv.z *= uv_height_factor;
      }

      file << "vt " << uv.x << " " << uv.y << " " << uv.z << "\n";
      // Somehow this keeps on being an issue that the x-coordinate has the incorrect sign
      file << "vn " << (-n.x) << " " << n.y << " " << n.z << "\n";
    }
  }

  // Finally write faces, grouped by material
  file << "# Faces grouped by material\n";
  file << "usemtl bark" << "\n";
  for (size_t i : bark_triangle_indices) {
    auto& t = triangles[i];
    std::array<unsigned int, 3> v_idx = {t.vertex_index0, t.vertex_index1, t.vertex_index2};
    file << "f";
    for (size_t j = 0; j < 3; j++) {
      file << " " << (v_idx[j] + index_offset) << "/" << (3 * i + j + index_offset) << "/"
           << (3 * i + j + index_offset);
    }
    file << "\n";
  }

  file << "usemtl interior" << "\n";
  for (size_t i : interior_triangle_indices) {
    auto& t = triangles[i];
    std::array<unsigned int, 3> v_idx = {t.vertex_index0, t.vertex_index1, t.vertex_index2};
    file << "f";
    for (size_t j = 0; j < 3; j++) {
      file << " " << (v_idx[j] + index_offset) << "/" << (3 * i + j + index_offset) << "/"
           << (3 * i + j + index_offset);
    }
    file << "\n";
  }

  file.close();
}

inline std::string json_vec2(const glm::vec2& v) {
  return "[" + std::to_string(v.x) + ", " + std::to_string(v.y) + "]";
}

inline std::string json_vec3(const glm::vec3& v) {
  return "[" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + "]";
}

inline std::string json_vec4(const glm::vec4& v) {
  return "[" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ", " +
         std::to_string(v.w) + "]";
}

inline std::string json_quat(const glm::quat& q) {
  return "[" + std::to_string(q.x) + ", " + std::to_string(q.y) + ", " + std::to_string(q.z) + ", " +
         std::to_string(q.w) + "]";
}

inline std::string json_mat4(const glm::mat4& m) {
  std::string s = "[";
  for (int c = 0; c < 4; ++c) {
    s += "[";
    for (int r = 0; r < 4; ++r) {
      s += std::to_string(m[c][r]);
      if (r < 3)
        s += ", ";
    }
    s += "]";
    if (c < 3)
      s += ", ";
  }
  s += "]";
  return s;
}

void eco_sys_lab_package::ObjExporter::WriteJson(
    const std::filesystem::path& json_path,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletVertex>& vertices,
    const std::vector<DsKineticVoronoiMeshing::GpuSegmentMeshletTriangle>& triangles,
    const std::vector<DynamicStrands::GpuSegment>& segments, double uv_height_factor) {
  std::ofstream file(json_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open JSON file");
  }

  auto write_values = [](std::ofstream& file, const std::string& key,
                         const std::function<std::string(size_t index)>& get_value, size_t size, bool last = false) {
    file << "  \"" << key << "\": [\n";
    for (size_t i = 0; i < size; ++i) {
      file << "    " << get_value(i);
      if (i < size - 1) {
        file << ",";
      }
      file << "\n";
    }

    if (!last) {
      file << "  ],\n";
    } else {
      file << "  ]\n";
    }
  };

  file << "{\n";

  // write all segment properties per vertex (uncomment to your needs)
#define SEG segments[vertices[index].segment_index]

  // write_values(
  //     file, "prev_handle",
  //     [&](size_t index) {
  //       return std::to_string(SEG.prev_handle);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "next_handle",
  //     [&](size_t index) {
  //       return std::to_string(SEG.next_handle);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "strand_handle",
  //     [&](size_t index) {
  //       return std::to_string(SEG.strand_handle);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "inv_mass",
  //     [&](size_t index) {
  //       return std::to_string(SEG.inv_mass);
  //     },
  //     vertices.size());
  write_values(
      file, "color",
      [&](size_t index) {
        return json_vec4(SEG.color);
      },
      vertices.size());
  // write_values(
  //     file, "q0",
  //     [&](size_t index) {
  //       return json_quat(SEG.q0);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "q",
  //     [&](size_t index) {
  //       return json_quat(SEG.q);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "last_q",
  //     [&](size_t index) {
  //       return json_quat(SEG.last_q);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "angular_v",
  //     [&](size_t index) {
  //       return json_vec3(SEG.angular_v);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "radius",
  //     [&](size_t index) {
  //       return std::to_string(SEG.radius);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "torque",
  //     [&](size_t index) {
  //       return json_vec3(SEG.torque);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "rest_length",
  //     [&](size_t index) {
  //       return std::to_string(SEG.rest_length);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "max_young_modulus",
  //     [&](size_t index) {
  //       return std::to_string(SEG.max_young_modulus);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "shear_stretch_alpha",
  //     [&](size_t index) {
  //       return std::to_string(SEG.shear_stretch_alpha);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "strength",
  //     [&](size_t index) {
  //       return std::to_string(SEG.strength);
  //     },
  //     vertices.size());
  write_values(
      file, "boundary_distance",
      [&](size_t index) {
        return std::to_string(SEG.boundary_distance);
      },
      vertices.size());
  write_values(
      file, "profile_position",
      [&](size_t index) {
        return json_vec2(SEG.profile_position);
      },
      vertices.size());
  write_values(
      file, "profile_polar_coordinate",
      [&](size_t index) {
        return json_vec2(SEG.profile_polar_coordinate);
      },
      vertices.size());
  // write_values(
  //     file, "inertia_tensor",
  //     [&](size_t index) {
  //       return json_vec3(SEG.inertia_tensor);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "max_shear_stretch_strain",
  //     [&](size_t index) {
  //       return std::to_string(SEG.max_shear_stretch_strain);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "inv_inertia_tensor",
  //     [&](size_t index) {
  //       return json_vec3(SEG.inv_inertia_tensor);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "shear_stretch_strain_limit",
  //     [&](size_t index) {
  //       return std::to_string(SEG.shear_stretch_strain_limit);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "inertia_w",
  //     [&](size_t index) {
  //       return json_mat4(SEG.inertia_w);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "inv_inertia_w",
  //     [&](size_t index) {
  //       return json_mat4(SEG.inv_inertia_w);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "shear_stretch_strain",
  //     [&](size_t index) {
  //       return std::to_string(SEG.shear_stretch_strain);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "node_handle",
  //     [&](size_t index) {
  //       return std::to_string(SEG.node_handle);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "original_mass",
  //     [&](size_t index) {
  //       return std::to_string(SEG.original_mass);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "group_index",
  //     [&](size_t index) {
  //       return std::to_string(SEG.group_index);
  //     },
  //     vertices.size());

  // write_values(
  //     file, "extra_mass",
  //     [&](size_t index) {
  //       return std::to_string(SEG.extra_mass);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "snow_amount",
  //     [&](size_t index) {
  //       return std::to_string(SEG.snow_amount);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "screen_depth",
  //     [&](size_t index) {
  //       return std::to_string(SEG.screen_depth);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "reach_ground",
  //     [&](size_t index) {
  //       return std::to_string(SEG.reach_ground);
  //     },
  //     vertices.size());

  // write_values(
  //     file, "C",
  //     [&](size_t index) {
  //       return std::to_string(SEG.C);
  //     },
  //     vertices.size());
  write_values(
      file, "HC",
      [&](size_t index) {
        return std::to_string(SEG.HC);
      },
      vertices.size());
  write_values(
      file, "HL",
      [&](size_t index) {
        return std::to_string(SEG.HL);
      },
      vertices.size());
  write_values(
      file, "RW",
      [&](size_t index) {
        return std::to_string(SEG.RW);
      },
      vertices.size());
  write_values(
      file, "RB",
      [&](size_t index) {
        return std::to_string(SEG.RB);
      },
      vertices.size());

  // write_values(
  //     file, "C_pre",
  //     [&](size_t index) {
  //       return std::to_string(SEG.C_pre);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "HC_pre",
  //     [&](size_t index) {
  //       return std::to_string(SEG.HC_pre);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "HL_pre",
  //     [&](size_t index) {
  //       return std::to_string(SEG.HL_pre);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "RW_pre",
  //     [&](size_t index) {
  //       return std::to_string(SEG.RW_pre);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "RB_pre",
  //     [&](size_t index) {
  //       return std::to_string(SEG.RB_pre);
  //     },
  //     vertices.size());

  // write_values(
  //     file, "K",
  //     [&](size_t index) {
  //       return std::to_string(SEG.K);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "diffusion_c",
  //     [&](size_t index) {
  //       return std::to_string(SEG.diffusion_c);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "diffusion_w",
  //     [&](size_t index) {
  //       return std::to_string(SEG.diffusion_w);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "diffusion_b",
  //     [&](size_t index) {
  //       return std::to_string(SEG.diffusion_b);
  //     },
  //     vertices.size());

  // write_values(
  //     file, "pairs_count",
  //     [&](size_t index) {
  //       return std::to_string(SEG.pairs_count);
  //     },
  //     vertices.size());
  write_values(
      file, "moisture",
      [&](size_t index) {
        return std::to_string(SEG.moisture);
      },
      vertices.size());
  // write_values(
  //     file, "moisture_pre",
  //     [&](size_t index) {
  //       return std::to_string(SEG.moisture_pre);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "diffusion_m",
  //     [&](size_t index) {
  //       return std::to_string(SEG.diffusion_m);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "internal_pattern",
  //     [&](size_t index) {
  //       return std::to_string(SEG.internal_pattern);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "cube_pattern",
  //     [&](size_t index) {
  //       return std::to_string(SEG.cube_pattern);
  //     },
  //     vertices.size());

  // write_values(
  //     file, "Obstruction_w",
  //     [&](size_t index) {
  //       return json_vec3(SEG.Obstruction_w);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "unlink_constraint",
  //     [&](size_t index) {
  //       return std::to_string(SEG.unlink_constraint);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "Obstruction_b",
  //     [&](size_t index) {
  //       return json_vec3(SEG.Obstruction_b);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "ground_damping",
  //     [&](size_t index) {
  //       return std::to_string(SEG.ground_damping);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "Obstruction_c",
  //     [&](size_t index) {
  //       return json_vec3(SEG.Obstruction_c);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "quasi_stable",
  //     [&](size_t index) {
  //       return std::to_string(SEG.quasi_stable);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "Obstruction_m",
  //     [&](size_t index) {
  //       return json_vec3(SEG.Obstruction_m);
  //     },
  //     vertices.size());
  // write_values(
  //     file, "quasi_damping",
  //     [&](size_t index) {
  //       return std::to_string(SEG.quasi_damping);
  //     },
  //     vertices.size());

  // also write average position of the two particles forming the segment
  write_values(
      file, "position0",
      [&](size_t index) {
        glm::vec3 p0 = SEG.particle0.x0;
        glm::vec3 p1 = SEG.particle1.x0;
        glm::vec3 avg = 0.5f * (p0 + p1);
        return json_vec3(avg);
      },
      vertices.size());

  write_values(
      file, "direction0",
      [&](size_t index) {
        glm::vec3 p0 = SEG.particle0.x0;
        glm::vec3 p1 = SEG.particle1.x0;
        glm::vec3 direction = glm::normalize(p1 - p0);
        return json_vec3(direction);
      },
      vertices.size());

  write_values(
      file, "root_distance",
      [&](size_t index) {
        float d1 = SEG.particle0.root_distance;
        float d2 = SEG.particle1.root_distance;
        float average = 0.5f * (d1 + d2);
        return std::to_string(average);
      },
      vertices.size(), false);

  // obtain third UV coordinate
  std::vector<float> uv_3(vertices.size());

  // iterate over triangles and fill array
  for (auto& tri : triangles) {
    bool is_bark = (tri.neighbor_segment_index == -2);

    std::array<size_t, 3> v_idxs = {tri.vertex_index0, tri.vertex_index1, tri.vertex_index2};
    for (size_t i = 0; i < 3; i++) {
      glm::vec4 uv = tri.uv[i];
      float value;
      if (is_bark) {
        value = uv.y * uv_height_factor;
      } else {
        value = uv.z * uv_height_factor;
      }
      uv_3[v_idxs[i]] = value;
    }
  }

  write_values(
      file, "uv_3",
      [&](size_t index) {
        return std::to_string(uv_3[index]);
      },
      vertices.size(), true);

  // write properties per face
  write_values(
      file, "has_neighbor",
      [&](size_t index) {
        return (triangles[index].neighbor_segment_index >= 0) ? "true" : "false";
      },
      triangles.size(), true);

#undef SEG

  file << "}\n";
}
