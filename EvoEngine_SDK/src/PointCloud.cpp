//
// Created by lllll on 8/22/2021.
//
#include "PointCloud.hpp"
#include "EditorLayer.hpp"
#include "Entity.hpp"
#include "Material.hpp"
#include "Particles.hpp"
#include "ProjectManager.hpp"
#include "Resources.hpp"
#include "Scene.hpp"
#include "Tinyply.hpp"
using namespace evo_engine;
using namespace tinyply;

bool PointCloud::LoadInternal(const std::filesystem::path& path) {
  if (path.extension() == ".ply") {
    return Load({}, path);
  }

  return IAsset::LoadInternal(path);
}

bool PointCloud::SaveInternal(const std::filesystem::path& path) const {
  if (path.extension() == ".ply") {
    return Save({}, path);
  }
  return IAsset::SaveInternal(path);
}

bool PointCloud::Load(const PointCloudLoadSettings& settings, const std::filesystem::path& path) {
  std::unique_ptr<std::istream> file_stream;
  std::vector<uint8_t> byte_buffer;
  try {
    file_stream.reset(new std::ifstream(path.string(), std::ios::binary));

    if (!file_stream || file_stream->fail())
      throw std::runtime_error("file_stream failed to open " + path.string());

    file_stream->seekg(0, std::ios::end);
    const float size_mb = file_stream->tellg() * float(1e-6);
    file_stream->seekg(0, std::ios::beg);

    PlyFile file;
    file.parse_header(*file_stream);

    std::cout << "\t[ply_header] Type: " << (file.is_binary_file() ? "binary" : "ascii") << std::endl;
    for (const auto& c : file.get_comments())
      std::cout << "\t[ply_header] Comment: " << c << std::endl;
    for (const auto& c : file.get_info())
      std::cout << "\t[ply_header] Info: " << c << std::endl;

    for (const auto& e : file.get_elements()) {
      std::cout << "\t[ply_header] element: " << e.name << " (" << e.size << ")" << std::endl;
      for (const auto& p : e.properties) {
        std::cout << "\t[ply_header] \tproperty: " << p.name << " (type=" << tinyply::PropertyTable[p.propertyType].str
                  << ")";
        if (p.isList)
          std::cout << " (list_type=" << tinyply::PropertyTable[p.listType].str << ")";
        std::cout << std::endl;
      }
    }

    // Because most people have their own mesh types, tinyply treats parsed data as structured/typed byte buffers.
    // See examples below on how to marry your own application-specific data structures with this one.
    std::shared_ptr<PlyData> vertices, normals, colors, texcoords, faces, tripstrip;

    // The header information can be used to programmatically extract properties on elements
    // known to exist in the header prior to reading the data. For brevity of this sample, properties
    // like vertex position are hard-coded:
    try {
      vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
    } catch (const std::exception& e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      normals = file.request_properties_from_element("vertex", {"nx", "ny", "nz"});
    } catch (const std::exception& e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      colors = file.request_properties_from_element("vertex", {"red", "green", "blue"});
    } catch (const std::exception& e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      colors = file.request_properties_from_element("vertex", {"r", "g", "b", "a"});
    } catch (const std::exception& e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    try {
      texcoords = file.request_properties_from_element("vertex", {"u", "v"});
    } catch (const std::exception& e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    // Providing a list size hint (the last argument) is a 2x performance improvement. If you have
    // arbitrary ply files, it is best to leave this 0.
    try {
      faces = file.request_properties_from_element("face", {"vertex_indices"}, 3);
    } catch (const std::exception& e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    // Tristrips must always be read with a 0 list size hint (unless you know exactly how many elements
    // are specifically in the file, which is unlikely);
    try {
      tripstrip = file.request_properties_from_element("tristrips", {"vertex_indices"}, 0);
    } catch (const std::exception& e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }
    file.read(*file_stream);
    if (vertices) {
      has_positions = true;
      std::cout << "\tRead " << vertices->count << " total vertices " << std::endl;
    } else {
      has_positions = false;
    }
    if (normals) {
      std::cout << "\tRead " << normals->count << " total vertex normals " << std::endl;
      has_normals = true;
    } else
      has_normals = false;
    if (colors) {
      std::cout << "\tRead " << colors->count << " total vertex colors " << std::endl;
      has_colors = true;
    } else {
      has_colors = false;
    }
    if (has_positions) {
      // Example One: converting to your own application types
      const size_t num_vertices_bytes = vertices->buffer.size_bytes();
      if (vertices->t == tinyply::Type::FLOAT64) {
        std::vector<glm::dvec3> tmp_points;
        tmp_points.resize(vertices->count);
        this->positions.resize(vertices->count);
        std::memcpy(tmp_points.data(), vertices->buffer.get(), num_vertices_bytes);
        for (int i = 0; i < vertices->count; i++) {
          this->positions[i].x = tmp_points[i].x;
          this->positions[i].y = tmp_points[i].y;
          this->positions[i].z = tmp_points[i].z;
        }
      } else if (vertices->t == tinyply::Type::FLOAT32) {
        std::vector<glm::vec3> tmp_points;
        tmp_points.resize(vertices->count);
        this->positions.resize(vertices->count);
        std::memcpy(tmp_points.data(), vertices->buffer.get(), num_vertices_bytes);

        for (int i = 0; i < vertices->count; i++) {
          this->positions[i].x = tmp_points[i].x;
          this->positions[i].y = tmp_points[i].y;
          this->positions[i].z = tmp_points[i].z;
        }
      }
    }
    if (has_colors) {
      const size_t num_vertices_bytes = colors->buffer.size_bytes();
      if (colors->t == tinyply::Type::UINT8) {
        std::vector<unsigned char> tmp_points;
        tmp_points.resize(colors->count * 3);
        this->colors.resize(colors->count);
        std::memcpy(tmp_points.data(), colors->buffer.get(), num_vertices_bytes);
        for (int i = 0; i < colors->count; i++) {
          this->colors[i].x = tmp_points[3 * i] / 255.0f;
          this->colors[i].y = tmp_points[3 * i + 1] / 255.0f;
          this->colors[i].z = tmp_points[3 * i + 2] / 255.0f;
          this->colors[i].w = 1.0f;
        }
      } else if (colors->t == tinyply::Type::FLOAT32) {
        std::vector<glm::vec3> tmp_points;
        tmp_points.resize(colors->count);
        this->colors.resize(colors->count);
        std::memcpy(tmp_points.data(), colors->buffer.get(), num_vertices_bytes);

        for (int i = 0; i < colors->count; i++) {
          this->colors[i].x = tmp_points[i].x;
          this->colors[i].y = tmp_points[i].y;
          this->colors[i].z = tmp_points[i].z;
          this->colors[i].w = 1.0f;
        }
      }
    }
    RecalculateBoundingBox();
  } catch (const std::exception& e) {
    std::cerr << "Caught tinyply exception: " << e.what() << std::endl;

    return false;
  }
  return true;
}

void PointCloud::OnCreate() {
}

bool PointCloud::OnInspect(const std::shared_ptr<EditorLayer>& editor_layer) {
  bool changed = false;
  ImGui::Text("Has Colors: %s", (has_colors ? "True" : "False"));
  ImGui::Text("Has Positions: %s", (has_positions ? "True" : "False"));
  ImGui::Text("Has Normals: %s", (has_normals ? "True" : "False"));
  if (ImGui::DragScalarN("Offset", ImGuiDataType_Double, &offset.x, 3))
    changed = true;
  ImGui::Text(("Original amount: " + std::to_string(positions.size())).c_str());
  if (ImGui::DragFloat("Point size", &point_size, 0.01f, 0.01f, 100.0f))
    changed = true;
  if (ImGui::DragFloat("Compress factor", &compress_factor, 0.001f, 0.0001f, 10.0f))
    changed = true;

  if (ImGui::Button("Apply compressed")) {
    ApplyCompressed();
  }

  if (ImGui::Button("Apply original")) {
    ApplyOriginal();
  }

  FileUtils::OpenFile(("Load PLY file##Particles"), "PointCloud", {".ply"},
                      [&](const std::filesystem::path& file_path) {
                        try {
                          Load({}, file_path);
                          EVOENGINE_LOG("Loaded from " + file_path.string());
                        } catch (std::exception& e) {
                          EVOENGINE_ERROR("Failed to load from " + file_path.string());
                        }
                      },
                      false);
  FileUtils::SaveFile(("Save Compressed to PLY##Particles"), "PointCloud", {".ply"},
                      [&](const std::filesystem::path& file_path) {
                        try {
                          Save({}, file_path);
                          EVOENGINE_LOG("Saved to " + file_path.string());
                        } catch (std::exception& e) {
                          EVOENGINE_ERROR("Failed to save to " + file_path.string());
                        }
                      },
                      false);
  if (ImGui::Button("Clear all positions")) {
    positions.clear();
    changed = true;
  }

  return changed;
}
void PointCloud::ApplyCompressed() {
  const auto scene = Application::GetActiveScene();
  const auto owner = scene->CreateEntity("Compressed Point Cloud");
  const auto particles = scene->GetOrSetPrivateComponent<Particles>(owner).lock();
  particles->material = ProjectManager::CreateTemporaryAsset<Material>();
  particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CUBE");
  particles->particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  auto compressed = std::vector<glm::dvec3>();
  Compress(compressed);
  const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  std::vector<ParticleInfo> particle_infos;

  particle_infos.resize(compressed.size());
  for (int i = 0; i < particle_infos.size(); i++) {
    particle_infos[i].instance_matrix.value =
        glm::translate(static_cast<glm::vec3>(compressed[i] + offset)) * glm::scale(glm::vec3(point_size));
    particle_infos[i].instance_color = colors[i];
  }
  particle_info_list->SetParticleInfos(particle_infos);
}
void PointCloud::Compress(std::vector<glm::dvec3>& points) {
  RecalculateBoundingBox();
  if (compress_factor == 0) {
    EVOENGINE_ERROR("Resolution invalid!");
    return;
  }
  points.clear();

  const double x_min = min_.x - std::fmod(min_.x, compress_factor) - (min_.x < 0 ? compress_factor : 0);
  const double y_min = min_.y - std::fmod(min_.y, compress_factor) - (min_.y < 0 ? compress_factor : 0);
  const double z_min = min_.z - std::fmod(min_.z, compress_factor) - (min_.z < 0 ? compress_factor : 0);

  const double x_max = max_.x - std::fmod(max_.x, compress_factor) + (max_.x > 0 ? compress_factor : 0);
  const double y_max = max_.y - std::fmod(max_.y, compress_factor) + (max_.y > 0 ? compress_factor : 0);
  const double z_max = max_.z - std::fmod(max_.z, compress_factor) + (max_.z > 0 ? compress_factor : 0);

  EVOENGINE_LOG("X, Y, Z MIN: [" + std::to_string(x_min) + ", " + std::to_string(y_min) + ", " + std::to_string(z_min) +
                "]");
  EVOENGINE_LOG("X, Y, Z MAX: [" + std::to_string(x_max) + ", " + std::to_string(y_max) + ", " + std::to_string(z_max) +
                "]");

  std::vector<int> voxels;
  const int range_x = static_cast<int>(((x_max - x_min) / static_cast<double>(compress_factor)));
  const int range_y = static_cast<int>(((y_max - y_min) / static_cast<double>(compress_factor)));
  const int range_z = static_cast<int>(((z_max - z_min) / static_cast<double>(compress_factor)));
  const int voxel_size = (range_x + 1) * (range_y + 1) * (range_z + 1);

  if (voxel_size > 1000000000 || voxel_size < 0) {
    EVOENGINE_ERROR("Resolution too small: " + std::to_string(voxel_size));
    return;
  } else {
    EVOENGINE_LOG("Voxel size: " + std::to_string(voxel_size));
  }

  voxels.resize(voxel_size);
  memset(voxels.data(), 0, voxel_size * sizeof(int));

  for (const auto& i : points) {
    int pos_x = static_cast<int>(((i.x - min_.x) / static_cast<double>(compress_factor)));
    int pos_y = static_cast<int>(((i.y - min_.y) / static_cast<double>(compress_factor)));
    int pos_z = static_cast<int>(((i.z - min_.z) / static_cast<double>(compress_factor)));
    auto index = pos_x * (range_y + 1) * (range_z + 1) + pos_y * (range_z + 1) + pos_z;
    if (index >= voxel_size) {
      EVOENGINE_ERROR("Out of range!");
      continue;
    }
    voxels[index]++;
  }

  for (int x = 0; x <= range_x; x++) {
    for (int y = 0; y <= range_y; y++) {
      for (int z = 0; z <= range_z; z++) {
        auto index = x * (range_y + 1) * (range_z + 1) + y * (range_z + 1) + z;
        if (voxels[index] != 0) {
          points.push_back(((glm::dvec3(x, y, z)) * static_cast<double>(compress_factor)) + min_);
        }
      }
    }
  }
}
void PointCloud::RecalculateBoundingBox() {
  if (positions.empty()) {
    min_ = glm::vec3(0.0f);
    max_ = glm::vec3(0.0f);
    return;
  }
  auto min_bound = glm::dvec3(INT_MAX);
  auto max_bound = glm::dvec3(-INT_MAX);
  for (const auto& i : positions) {
    min_bound = glm::vec3((glm::min)(min_bound.x, i.x), (glm::min)(min_bound.y, i.y), (glm::min)(min_bound.z, i.z));
    max_bound = glm::vec3((glm::max)(max_bound.x, i.x), (glm::max)(max_bound.y, i.y), (glm::max)(max_bound.z, i.z));
  }
  max_ = max_bound;
  min_ = min_bound;

  auto avg = glm::dvec3(0);
  for (const auto& i : positions) {
    avg += i / static_cast<double>(positions.size());
  }
  offset = -min_;
}
void PointCloud::Serialize(YAML::Emitter& out) const {
  out << YAML::Key << "offset" << offset;
  out << YAML::Key << "point_size" << point_size;
  out << YAML::Key << "compress_factor" << compress_factor;
  out << YAML::Key << "min_" << min_;
  out << YAML::Key << "max_" << max_;
  if (!positions.empty()) {
    out << YAML::Key << "positions" << YAML::Value
        << YAML::Binary((const unsigned char*)positions.data(), positions.size() * sizeof(glm::dvec3));
  }
  if (!normals.empty()) {
    out << YAML::Key << "normals" << YAML::Value
        << YAML::Binary((const unsigned char*)normals.data(), normals.size() * sizeof(glm::dvec3));
  }
  if (!colors.empty()) {
    out << YAML::Key << "colors" << YAML::Value
        << YAML::Binary((const unsigned char*)colors.data(), colors.size() * sizeof(glm::vec3));
  }
}
void PointCloud::Deserialize(const YAML::Node& in) {
  if (in["offset"])
    offset = in["offset"].as<glm::dvec3>();
  if (in["point_size"])
    point_size = in["point_size"].as<float>();
  if (in["compress_factor"])
    compress_factor = in["compress_factor"].as<float>();
  if (in["min_"])
    min_ = in["min_"].as<glm::dvec3>();
  if (in["max_"])
    max_ = in["max_"].as<glm::dvec3>();
  if (in["positions"]) {
    has_positions = true;
    auto vertex_data = in["positions"].as<YAML::Binary>();
    positions.resize(vertex_data.size() / sizeof(glm::dvec3));
    std::memcpy(positions.data(), vertex_data.data(), vertex_data.size());
  } else {
    has_positions = false;
  }

  if (in["colors"]) {
    has_colors = true;
    const auto vertex_data = in["colors"].as<YAML::Binary>();
    colors.resize(vertex_data.size() / sizeof(glm::vec3));
    std::memcpy(colors.data(), vertex_data.data(), vertex_data.size());
  } else {
    has_colors = false;
  }

  if (in["normals"]) {
    has_normals = true;
    const auto vertex_data = in["normals"].as<YAML::Binary>();
    normals.resize(vertex_data.size() / sizeof(glm::vec3));
    std::memcpy(normals.data(), vertex_data.data(), vertex_data.size());
  } else {
    has_normals = false;
  }
}
void PointCloud::ApplyOriginal() const {
  const auto scene = Application::GetActiveScene();
  const auto owner = scene->CreateEntity("Original Point Cloud");
  const auto particles = scene->GetOrSetPrivateComponent<Particles>(owner).lock();
  particles->material = ProjectManager::CreateTemporaryAsset<Material>();
  particles->mesh = Resources::GetResource<Mesh>("PRIMITIVE_CUBE");
  particles->particle_info_list = ProjectManager::CreateTemporaryAsset<ParticleInfoList>();
  const auto particle_info_list = particles->particle_info_list.Get<ParticleInfoList>();
  std::vector<ParticleInfo> particle_infos;
  particle_infos.resize(positions.size());
  for (int i = 0; i < particle_infos.size(); i++) {
    particle_infos[i].instance_matrix.value =
        glm::translate((glm::vec3)(positions[i] + offset)) * glm::scale(glm::vec3(point_size));
    particle_infos[i].instance_color = colors[i];
  }
  particle_info_list->SetParticleInfos(particle_infos);
}

bool PointCloud::Save(const PointCloudSaveSettings& settings, const std::filesystem::path& path) const {
  try {
    PlyFile cube_file;
    if (has_positions) {
      if (settings.double_precision) {
        cube_file.add_properties_to_element("vertex", {"x", "y", "z"}, Type::FLOAT64, positions.size(),
                                            (uint8_t*)positions.data(), Type::INVALID, 0);
      } else {
        std::vector<glm::vec3> points;
        points.resize(points.size());
        Jobs::RunParallelFor(points.size(), [&](const unsigned index) {
          points[index] = glm::vec3(points[index]);
        });
        cube_file.add_properties_to_element("vertex", {"x", "y", "z"}, Type::FLOAT32, points.size(),
                                            (uint8_t*)points.data(), Type::INVALID, 0);
      }
    }
    if (has_colors) {
      cube_file.add_properties_to_element("vertex", {"red", "green", "blue"}, Type::FLOAT32, colors.size(),
                                          (uint8_t*)(colors.data()), Type::INVALID, 0);
    }
    if (settings.binary) {
      // Write a binary file
      std::filebuf fb_binary;
      fb_binary.open(path.string(), std::ios::out | std::ios::binary);
      std::ostream outstream_binary(&fb_binary);
      if (outstream_binary.fail())
        throw std::runtime_error("failed to open " + path.string());
      cube_file.write(outstream_binary, true);
    } else {
      std::filebuf fb_ascii;
      fb_ascii.open(path.string(), std::ios::out);
      std::ostream outstream_ascii(&fb_ascii);
      if (outstream_ascii.fail())
        throw std::runtime_error("failed to open " + path.string());
      // Write an ASCII file
      cube_file.write(outstream_ascii, false);
    }
  } catch (const std::exception& e) {
    EVOENGINE_ERROR(e.what());
    return false;
  }
  return true;
}
void PointCloud::Crop(std::vector<glm::dvec3>& points, const glm::dvec3& min, const glm::dvec3& max) {
  for (const auto& i : points) {
    if (i.x >= min.x && i.y >= min.y && i.z >= min.z && i.x <= max.x && i.y <= max.y && i.z <= max.z) {
      points.push_back(i);
    }
  }
}
