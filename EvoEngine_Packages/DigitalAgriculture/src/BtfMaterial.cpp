//
// Created by lllll on 10/13/2022.
//

#include "DigitalAgricultureSerializationAdapters.hpp"

using namespace evo_engine;

#include "Console.hpp"
#include "Utilities.hpp"

bool ParseFloatData(const std::string &file_name, int &num_of_rows, int &num_of_cols, float &min_value,
                    float &max_value, std::vector<float> &data) {
  FILE *fp;
  if ((fp = fopen(file_name.c_str(), "r")) == nullptr) {
    EVOENGINE_ERROR("Error")
    return false;
  }
  int v = fscanf(fp, "%d %d %f %f\n", &num_of_rows, &num_of_cols, &min_value, &max_value);
  assert(v == 4);
  data.resize(num_of_cols * num_of_rows);
  for (int row = 0; row < num_of_rows; row++) {
    for (int col = 0; col < num_of_cols; col++) {
      v = fscanf(fp, "%f ", &data[row * num_of_cols + col]);
      assert(v == 1);
    }
    fscanf(fp, "\n");
  }
  fclose(fp);
  return true;
}

auto ParseIntData(const std::string &file_name, int &num_of_rows, int &num_of_cols, int &min_value, int &max_value,
                  std::vector<int> &data) -> bool {
  FILE *fp;
  if ((fp = fopen(file_name.c_str(), "r")) == nullptr) {
    EVOENGINE_ERROR("Error");
    return false;
  }
  int v = fscanf(fp, "%d %d %d %d\n", &num_of_rows, &num_of_cols, &min_value, &max_value);
  assert(v == 4);
  data.resize(num_of_cols * num_of_rows);
  for (int row = 0; row < num_of_rows; row++) {
    for (int col = 0; col < num_of_cols; col++) {
      v = fscanf(fp, "%d ", &data[row * num_of_cols + col]);
      assert(v == 1);
    }
    fscanf(fp, "\n");
  }
  fclose(fp);
  return true;
}

std::string LoadFileAsString(const std::string &path) {
  std::ifstream file;
  file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  try {
    // open files
    file.open(path);
    std::stringstream stream;
    // read file's buffer contents into streams
    stream << file.rdbuf();
    // close file handlers
    file.close();
    // convert stream into string
    return stream.str();
  } catch (const std::ifstream::failure &e) {
    EVOENGINE_ERROR("Load file failed: " + std::string(e.what()))
    throw;
  }
}

using namespace evo_engine;

bool BtfMaterial::ImportFromFolder(const std::filesystem::path &path) {
  auto material_directory_path = path.string();
#pragma region Path check
  std::string all_material_info_path = material_directory_path + "/all_materialInfo.txt";
  try {
    LoadFileAsString(all_material_info_path);
  } catch (const std::ifstream::failure &e) {
    EVOENGINE_ERROR("Load file failed: " + std::string(e.what()))
    return false;
  }
#pragma endregion
#pragma region Line 82 from ibtfbase.cpp
  btf_base.material_order = 0;
  btf_base.color_channel = 0;
  // initial size of arrays
  // How the beta is discretized, either uniformly in degrees
  // or uniformly in cosinus of angle
  btf_base.use_cos_beta = true;
#pragma endregion
#pragma region Tilemap
  // Since tilemap is not used, the code here is not implemented.
#pragma endregion
#pragma region Scale info
  btf_base.post_scale = 1.0f;
  // Since no material contains the scale.txt is not used, the code here is not
  // implemented.
#pragma endregion
#pragma region material info
  FILE *fp;
  if ((fp = fopen(all_material_info_path.c_str(), "r")) == NULL) {
    EVOENGINE_ERROR("Failed to load BTF material");
    return false;
  }
  // First save the info about BTFbase: name, materials saved, and how saved
  char line[1000];
  int load_materials;
  int max_materials;
  int flag_all_materials;
  int flag_use_34dview_rep;
  int flag_use_pdf2_compact_rep;

  // First save the info about BTFbase: name, materials saved, and how saved
  if (fscanf(fp, "%s\n%d\n%d\n%d\n%d\n%d\n", &line[0], &load_materials, &max_materials, &flag_all_materials,
             &flag_use_34dview_rep, &flag_use_pdf2_compact_rep) != 6) {
    fclose(fp);
    EVOENGINE_ERROR("File is corrupted for reading basic parameters");
    return false;
  }
  // Here we need to read this information about original data
  int color_size, view_size, illumination_size, tile_size;
  if (fscanf(fp, "%d\n%d\n%d\n%d\n", &color_size, &view_size, &illumination_size, &tile_size) != 4) {
    fclose(fp);
    EVOENGINE_ERROR("File is corrupted for reading basic parameters about orig database")
    return false;
  }

  // Here we load how parameterization is done
  // It is meant: beta/stepPerBeta, alpha/stepsPerAlpha, theta/stepsPerTheta,
  // phi/stepPerPhi, reserve/reserv, reserve/reserve
  int use_cos_beta_flag, tmp3, tmp5, tmp7, tmp9, tmp10, tmp11, tmp12;
  if (fscanf(fp, "%d %d %d %d %d %d %d %d %d %d %d %d\n", &use_cos_beta_flag, &btf_base.beta_size, &tmp3,
             &btf_base.alpha_size, &tmp5, &btf_base.theta_size, &tmp7, &btf_base.phi_size, &tmp9, &tmp10, &tmp11,
             &tmp12) != 12) {
    fclose(fp);
    EVOENGINE_ERROR("File is corrupted for reading angle parameterization settings\n")
    return false;
  }
  btf_base.use_cos_beta = use_cos_beta_flag ? true : false;
  assert(btf_base.beta_size % 2 == 1);
  assert(btf_base.alpha_size % 2 == 1);
  assert(btf_base.theta_size >= 2);
  assert(btf_base.phi_size >= 1);
#pragma endregion
#pragma region Create shared variables
  std::vector<float> beta_angles;
  // we always must have odd number of quantization steps per 180 degrees
  if (btf_base.use_cos_beta) {
    beta_angles.resize(btf_base.beta_size);
    for (int i = 0; i < btf_base.beta_size; i++) {
      float sin_beta = -1.0f + 2.0f * i / (btf_base.beta_size - 1);
      if (sin_beta > 1.0f)
        sin_beta = 1.0f;
      // in degrees
      beta_angles[i] = glm::degrees(glm::asin(sin_beta));
    }
    beta_angles[0] = -90.f;
    beta_angles[(btf_base.beta_size - 1) / 2] = 0.f;
    beta_angles[btf_base.beta_size - 1] = 90.f;
  } else {
    float step_beta = 0.f;
    // uniform quantization in angle
    step_beta = 180.f / static_cast<float>(btf_base.beta_size - 1);
    beta_angles.resize(btf_base.beta_size);
    for (int i = 0; i < btf_base.beta_size; i++) {
      beta_angles[i] = i * step_beta - 90.f;
    }
    beta_angles[(btf_base.beta_size - 1) / 2] = 0.f;
    beta_angles[btf_base.beta_size - 1] = 90.0f;
  }
  // Here we set alpha
  btf_base.alpha_step = 180.f / static_cast<float>(btf_base.alpha_size - 1);
  btf_base.theta_step = 90.0f / static_cast<float>(btf_base.theta_size - 1);
  btf_base.phi_step = 360.0f / static_cast<float>(btf_base.phi_size);
  btf_base.shared_coordinates.Set(tmp12, btf_base.use_cos_beta, btf_base.beta_size, btf_base.alpha_size,
                                  btf_base.alpha_step, btf_base.theta_size, btf_base.theta_step, btf_base.phi_size,
                                  btf_base.phi_step);
  shared_coordinates_beta_angles = beta_angles;
#pragma endregion
#pragma region Current settings
  // Here we need to read this information about current material setting
  // where are the starting points for the next search, possibly
  int fPDF1, fAB, fIAB, fPDF2, fPDF2L, fPDF2AB, fPDF3, fPDF34, fPDF4, fRESERVE;
  if (fscanf(fp, "%d %d %d %d %d %d %d %d %d %d\n", &fPDF1, &fAB, &fIAB, &fPDF2, &fPDF2L, &fPDF2AB, &fPDF3, &fPDF34,
             &fPDF4, &fRESERVE) != 10) {
    fclose(fp);
    EVOENGINE_ERROR("File is corrupted for reading starting search settings\n");
    return false;
  }
  // Here we need to save this information about current material setting
  int lsPDF1, lsAB, lsIAB, lsPDF2, lsPDF2L, lsPDF2AB, lsPDF3, lsPDF34, lsPDF4, lsRESERVE;
  if (fscanf(fp, "%d %d %d %d %d %d %d %d %d %d\n", &lsPDF1, &lsAB, &lsIAB, &lsPDF2, &lsPDF2L, &lsPDF2AB, &lsPDF3,
             &lsPDF34, &lsPDF4, &lsRESERVE) != 10) {
    fclose(fp);
    EVOENGINE_ERROR("File is corrupted for reading starting search points\n");
    return false;
  }

  int metric;
  float baseEps, rPDF1, epsAB, epsIAB, rPDF2, rPDF2L, epsPDF2AB, rPDF3, rPDF34, rPDF4, rPDF4b;
  if (fscanf(fp, "%d %f %f %f %f %f %f %f %f %f %f %f\n", &metric, &baseEps, &rPDF1, &epsAB, &epsIAB, &rPDF2, &rPDF2L,
             &epsPDF2AB, &rPDF3, &rPDF34, &rPDF4, &rPDF4b) != 12) {
    fclose(fp);
    EVOENGINE_ERROR("File is corrupted for reading epsilon search settings\n")
    return false;
  }
#pragma endregion
#pragma region Load sizes
  // !!!!!! If we have only one database for all materials or
  // we share some databases except PDF6 for all materials
  btf_base.use34_view_representation = flag_use_34dview_rep;
  btf_base.use_pdf2_compact_rep = flag_use_pdf2_compact_rep;

  if (load_materials > max_materials)
    load_materials = max_materials;
  btf_base.material_count = max_materials;
  if (flag_all_materials) {
    btf_base.all_materials_in_one_database = true;
  } else {
    btf_base.all_materials_in_one_database = false;
  }

#pragma endregion
#pragma region Allocate arrays
  if (!btf_base.all_materials_in_one_database && load_materials != 1) {
    EVOENGINE_ERROR("Database for multiple materials are not supported!")
    return false;
  }
  // Here we only allow single material, so the array representations in
  // original BtfMaterial lib are not implemented.
#pragma endregion
#pragma region HDR
  std::string material_name;
  float hdr_value = 1.0f;
  int ro, co, pr, pc;
  char l1[1000], l2[1000], l3[1000], l4[1000];
  int hdr_flag = 0;
  if (fscanf(fp, "%s %s %s %s %d %d %d %d %f\n", l1, l2, l3, l4, &ro, &co, &pr, &pc, &hdr_value) == 9) {
    // Here we need to allocate the arrays for names
    material_name = std::string(l1);

    if (fabs(hdr_value - 1.0f) < 1e-6 || fabs(hdr_value) < 1e-6) {
      hdr_flag = 0;
      hdr_value = 1.0f;
    } else {
      hdr_flag = 1;
    }
    btf_base.shared_coordinates.hdr_flag = hdr_flag;
    btf_base.hdr = hdr_flag;
    btf_base.hdr_value = hdr_value;
  }
  fclose(fp);
#pragma endregion
#pragma region Load material
  // Note that nrows and ncols are not set during loading !
  std::string file_name = material_directory_path + "/" + material_name + "_materialInfo.txt";
  // Now creating PDF6 for each material using common database
  if ((fp = fopen(file_name.c_str(), "r")) == nullptr) {
    EVOENGINE_ERROR("Cannot open file" + file_name)
    return false;
  }
  char name_m[200];
  if (fscanf(fp, "%s %s %s %s %d %d %d %d %f\n", &name_m[0], l1, l2, l3, &ro, &co, &pr, &pc, &hdr_value) != 9) {
    EVOENGINE_ERROR("Reading the information about material failed\n")
    fclose(fp);
    return false;
  }

  fclose(fp);
  if (glm::abs(hdr_value - 1.0f) < 1e-6 || glm::abs(hdr_value) < 1e-6) {
    hdr_flag = 0;
    hdr_value = 1.0f;
  } else {
    hdr_flag = 1;
  }

  if (strcmp(name_m, material_name.c_str()) != 0) {
    EVOENGINE_ERROR("Some problem material name in file=" + std::string(name_m) + " other name=" + material_name +
                    "\n");
    return false;
  }
  // Now we can create the database, PDF6 is allocated
  // with right values
  btf_base.shared_coordinates.hdr_flag = hdr_flag;
  btf_base.hdr = hdr_flag;
  btf_base.hdr_value = hdr_value;

  auto &ab = btf_base.pdf6d.pdf4d.pdf3d.pdf2d.index_ab.vector_color;
  auto &iab = btf_base.pdf6d.pdf4d.pdf3d.pdf2d.index_ab;
  auto &pdf1 = btf_base.pdf6d.pdf4d.pdf3d.pdf2d.pdf1d;
  auto &pdf2 = btf_base.pdf6d.pdf4d.pdf3d.pdf2d;
  auto &pdf3 = btf_base.pdf6d.pdf4d.pdf3d;
  auto &pdf4 = btf_base.pdf6d.pdf4d;
  pdf1.Init(btf_base.beta_size);
  ab.Init();
  iab.Init(btf_base.beta_size);
  pdf2.Init();
  pdf3.Init(btf_base.theta_size);
  pdf4.Init(btf_base.phi_size);
  btf_base.pdf6d.Init(pr, pc, ro, co, btf_base.color_channel);

#pragma region Load Data
  std::string prefix = material_directory_path + "/" + material_name;
  int min_int_val, max_int_val;
  float min_float_val, max_float_val;

  ParseIntData(prefix + "_PDF6Dslices.txt", btf_base.pdf6d.row_size, btf_base.pdf6d.column_size, min_int_val,
               max_int_val, pdf6d);
  ParseFloatData(prefix + "_PDF6Dscale.txt", btf_base.pdf6d.row_size, btf_base.pdf6d.column_size, min_float_val,
                 max_float_val, pdf6d_scales);

  prefix = material_directory_path + "/" + "all";

  ParseFloatData(prefix + "_PDF1Dslice.txt", pdf1.pdf1d_size, pdf1.beta_size, min_float_val, max_float_val, pdf1d);

  ParseFloatData(prefix + "_colors.txt", ab.color_size, ab.channel_size, min_float_val, max_float_val, vector_color);

  ParseIntData(prefix + "_indexAB.txt", iab.index_size, iab.beta_size, min_int_val, max_int_val, index_ab);

  ParseIntData(prefix + "_PDF2Dcolours.txt", pdf2.color.pdf2d_size, pdf2.color.alpha_size, min_int_val, max_int_val,
               pdf2d_colors);

  ParseIntData(prefix + "_PDF2Dslices.txt", pdf2.luminance.pdf2d_size, pdf2.luminance.alpha_size, min_int_val,
               max_int_val, pdf2d);
  ParseFloatData(prefix + "_PDF2Dscale.txt", pdf2.luminance.pdf2d_size, pdf2.luminance.alpha_size, min_float_val,
                 max_float_val, pdf2d_scales);

  ParseIntData(prefix + "_PDF2Dindices.txt", pdf2.pdf2d_size, pdf2.slice_length, min_int_val, max_int_val,
               luminance_color_indices);

  ParseFloatData(prefix + "_PDF3Dscale.txt", pdf3.pdf3d_size, pdf3.theta_size, min_float_val, max_float_val,
                 pdf3d_scales);

  ParseIntData(prefix + "_PDF3Dslices.txt", pdf3.pdf3d_size, pdf3.theta_size, min_int_val, max_int_val, pdf3d);

  ParseFloatData(prefix + "_PDF4Dscale.txt", pdf4.pdf4d_size, pdf4.phi_size, min_float_val, max_float_val,
                 pdf4d_scales);

  ParseIntData(prefix + "_PDF4Dslices.txt", pdf4.pdf4d_size, pdf4.phi_size, min_int_val, max_int_val, pdf4d);

#pragma endregion
  btf_base.has_data = true;
  return true;  // OK - database loaded, or at least partially
#pragma endregion
}

bool BtfMaterial::DrawGui(const std::shared_ptr<EditorLayer> &editor_layer) {
  bool changed = false;
  FileUtils::OpenFolder(
      "Import Database",
      [&](const std::filesystem::path &path) {
        try {
          const bool succeed = ImportFromFolder(path);
          if (succeed)
            changed = true;
          EVOENGINE_LOG((std::string("BTF Material import ") + (succeed ? "succeed" : "failed")))
        } catch (const std::exception &e) {
          EVOENGINE_ERROR(std::string(e.what()))
        }
      },
      false);

  if (btf_base.has_data) {
    if (ImGui::DragFloat("TexCoord Multiplier", &btf_base.tex_coord_multiplier, 0.1f)) {
      changed = true;
    }

    if (ImGui::Checkbox("HDR", &btf_base.hdr)) {
      changed = true;
    }
    if (btf_base.hdr) {
      if (ImGui::DragFloat("HDR Value", &btf_base.hdr_value, 0.01f)) {
        changed = true;
      }
    }
    if (ImGui::DragFloat("Gamma Value", &btf_base.gamma, 0.01f)) {
      changed = true;
    }
  }
  return changed;
}

void SerializeSharedCoordinates(const SharedCoordinates &shared_coordinates, YAML::Emitter &out) {
  out << YAML::Key << "use_cos_beta" << YAML::Value << shared_coordinates.use_cos_beta;

  out << YAML::Key << "beta_size" << YAML::Value << shared_coordinates.beta_size;
  out << YAML::Key << "alpha_step" << YAML::Value << shared_coordinates.alpha_step;
  out << YAML::Key << "alpha_size" << YAML::Value << shared_coordinates.alpha_size;
  out << YAML::Key << "theta_step" << YAML::Value << shared_coordinates.theta_step;
  out << YAML::Key << "theta_size" << YAML::Value << shared_coordinates.theta_size;
  out << YAML::Key << "phi_step" << YAML::Value << shared_coordinates.phi_step;
  out << YAML::Key << "phi_size" << YAML::Value << shared_coordinates.phi_size;

  out << YAML::Key << "beta" << YAML::Value << shared_coordinates.beta;
  out << YAML::Key << "alpha" << YAML::Value << shared_coordinates.alpha;
  out << YAML::Key << "theta" << YAML::Value << shared_coordinates.theta;
  out << YAML::Key << "phi" << YAML::Value << shared_coordinates.phi;

  out << YAML::Key << "current_beta_low_bound" << YAML::Value << shared_coordinates.current_beta_low_bound;
  out << YAML::Key << "beta_weight" << YAML::Value << shared_coordinates.beta_weight;
  out << YAML::Key << "beta_min2" << YAML::Value << shared_coordinates.beta_min2;

  out << YAML::Key << "current_alpha_low_bound" << YAML::Value << shared_coordinates.current_alpha_low_bound;
  out << YAML::Key << "alpha_weight" << YAML::Value << shared_coordinates.alpha_weight;
  out << YAML::Key << "alpha_min2" << YAML::Value << shared_coordinates.alpha_min2;

  out << YAML::Key << "current_theta_low_bound" << YAML::Value << shared_coordinates.current_theta_low_bound;
  out << YAML::Key << "theta_weight" << YAML::Value << shared_coordinates.theta_weight;
  out << YAML::Key << "theta_min2" << YAML::Value << shared_coordinates.theta_min2;

  out << YAML::Key << "current_phi_low_bound" << YAML::Value << shared_coordinates.current_phi_low_bound;
  out << YAML::Key << "phi_weight" << YAML::Value << shared_coordinates.phi_weight;

  out << YAML::Key << "scale" << YAML::Value << shared_coordinates.scale;

  out << YAML::Key << "hdr_flag" << YAML::Value << shared_coordinates.hdr_flag;
  out << YAML::Key << "use_btf_flag" << YAML::Value << shared_coordinates.use_btf_flag;
}

void DeserializeSharedCoordinates(SharedCoordinates &target, const YAML::Node &in) {
  if (in["use_cos_beta"])
    target.use_cos_beta = in["use_cos_beta"].as<bool>();

  if (in["beta_size"])
    target.beta_size = in["beta_size"].as<int>();
  if (in["alpha_step"])
    target.alpha_step = in["alpha_step"].as<float>();
  if (in["alpha_size"])
    target.alpha_size = in["alpha_size"].as<int>();
  if (in["theta_step"])
    target.theta_step = in["theta_step"].as<float>();
  if (in["theta_size"])
    target.theta_size = in["theta_size"].as<int>();
  if (in["phi_step"])
    target.phi_step = in["phi_step"].as<float>();
  if (in["phi_size"])
    target.phi_size = in["phi_size"].as<int>();

  if (in["beta"])
    target.beta = in["beta"].as<float>();
  if (in["alpha"])
    target.alpha = in["alpha"].as<float>();
  if (in["theta"])
    target.theta = in["theta"].as<float>();
  if (in["phi"])
    target.phi = in["phi"].as<float>();

  if (in["current_beta_low_bound"])
    target.current_beta_low_bound = in["current_beta_low_bound"].as<int>();
  if (in["beta_weight"])
    target.beta_weight = in["beta_weight"].as<float>();
  if (in["beta_min2"])
    target.beta_min2 = in["beta_min2"].as<float>();

  if (in["current_alpha_low_bound"])
    target.current_alpha_low_bound = in["current_alpha_low_bound"].as<int>();
  if (in["alpha_weight"])
    target.alpha_weight = in["alpha_weight"].as<float>();
  if (in["alpha_min2"])
    target.alpha_min2 = in["alpha_min2"].as<float>();

  if (in["current_theta_low_bound"])
    target.current_theta_low_bound = in["current_theta_low_bound"].as<int>();
  if (in["theta_weight"])
    target.theta_weight = in["theta_weight"].as<float>();
  if (in["theta_min2"])
    target.theta_min2 = in["theta_min2"].as<float>();

  if (in["current_phi_low_bound"])
    target.current_phi_low_bound = in["current_phi_low_bound"].as<int>();
  if (in["phi_weight"])
    target.phi_weight = in["phi_weight"].as<float>();

  if (in["scale"])
    target.scale = in["scale"].as<float>();

  if (in["hdr_flag"])
    target.hdr_flag = in["hdr_flag"].as<bool>();
  if (in["use_btf_flag"])
    target.use_btf_flag = in["use_btf_flag"].as<bool>();
}

void SerializeVectorColor(const VectorColor &target, YAML::Emitter &out) {
  out << YAML::Key << "start_index" << YAML::Value << target.start_index;
  out << YAML::Key << "channel_size" << YAML::Value << target.channel_size;
  out << YAML::Key << "color_size" << YAML::Value << target.color_size;
}

void DeserializeVectorColor(VectorColor &target, const YAML::Node &in) {
  if (in["start_index"])
    target.start_index = in["start_index"].as<int>();
  if (in["channel_size"])
    target.channel_size = in["channel_size"].as<int>();
  if (in["color_size"])
    target.color_size = in["color_size"].as<int>();
}

void SerializeIndexAb(const IndexAB &target, YAML::Emitter &out) {
  out << YAML::Key << "index_size" << YAML::Value << target.index_size;
  out << YAML::Key << "beta_size" << YAML::Value << target.beta_size;
  // vector_color
  out << YAML::Key << "vector_color" << YAML::Value << YAML::BeginMap;
  SerializeVectorColor(target.vector_color, out);
  out << YAML::EndMap;
}

void DeserializeIndexAb(IndexAB &target, const YAML::Node &in) {
  if (in["index_size"])
    target.index_size = in["index_size"].as<int>();
  if (in["beta_size"])
    target.beta_size = in["beta_size"].as<int>();
  if (in["vector_color"])
    DeserializeVectorColor(target.vector_color, in["vector_color"]);
}

void SerializePdf1D(const Pdf1D &target, YAML::Emitter &out) {
  out << YAML::Key << "beta_size" << YAML::Value << target.beta_size;
  out << YAML::Key << "pdf1d_size" << YAML::Value << target.pdf1d_size;
}

void DeserializePdf1D(Pdf1D &target, const YAML::Node &in) {
  if (in["beta_size"])
    target.beta_size = in["beta_size"].as<int>();
  if (in["pdf1d_size"])
    target.pdf1d_size = in["pdf1d_size"].as<int>();
}

void SerializePdf2D(const Pdf2D &target, YAML::Emitter &out) {
  out << YAML::Key << "pdf2d_size" << YAML::Value << target.pdf2d_size;
  out << YAML::Key << "slice_length" << YAML::Value << target.slice_length;

  out << YAML::Key << "color.pdf2d_size" << YAML::Value << target.color.pdf2d_size;
  out << YAML::Key << "color.alpha_size" << YAML::Value << target.color.alpha_size;

  out << YAML::Key << "luminance.pdf2d_size" << YAML::Value << target.luminance.pdf2d_size;
  out << YAML::Key << "luminance.alpha_size" << YAML::Value << target.luminance.alpha_size;

  // index_ab
  out << YAML::Key << "index_ab" << YAML::Value << YAML::BeginMap;
  SerializeIndexAb(target.index_ab, out);
  out << YAML::EndMap;
  // pdf1d
  out << YAML::Key << "pdf1d" << YAML::Value << YAML::BeginMap;
  SerializePdf1D(target.pdf1d, out);
  out << YAML::EndMap;
}

void DeserializePdf2D(Pdf2D &target, const YAML::Node &in) {
  if (in["pdf2d_size"])
    target.pdf2d_size = in["pdf2d_size"].as<int>();
  if (in["slice_length"])
    target.slice_length = in["slice_length"].as<int>();

  if (in["color.pdf2d_size"])
    target.color.pdf2d_size = in["color.pdf2d_size"].as<int>();
  if (in["color.alpha_size"])
    target.color.alpha_size = in["color.alpha_size"].as<int>();
  if (in["luminance.pdf2d_size"])
    target.luminance.pdf2d_size = in["luminance.pdf2d_size"].as<int>();
  if (in["luminance.alpha_size"])
    target.luminance.alpha_size = in["luminance.alpha_size"].as<int>();

  if (in["index_ab"])
    DeserializeIndexAb(target.index_ab, in["index_ab"]);
  if (in["pdf1d"])
    DeserializePdf1D(target.pdf1d, in["pdf1d"]);
}

void SerializePdf3D(const BtfPdf3D &target, YAML::Emitter &out) {
  out << YAML::Key << "pdf3d_size" << YAML::Value << target.pdf3d_size;
  out << YAML::Key << "theta_size" << YAML::Value << target.theta_size;

  // m_pdf2
  out << YAML::Key << "pdf2d" << YAML::Value << YAML::BeginMap;
  SerializePdf2D(target.pdf2d, out);
  out << YAML::EndMap;
}

void DeserializePdf3D(BtfPdf3D &target, const YAML::Node &in) {
  if (in["pdf3d_size"])
    target.pdf3d_size = in["pdf3d_size"].as<int>();
  if (in["theta_size"])
    target.theta_size = in["theta_size"].as<int>();

  if (in["pdf2d"])
    DeserializePdf2D(target.pdf2d, in["pdf2d"]);
}

void SerializePdf4D(const BtfPdf4D &target, YAML::Emitter &out) {
  out << YAML::Key << "pdf4d_size" << YAML::Value << target.pdf4d_size;
  out << YAML::Key << "phi_size" << YAML::Value << target.phi_size;
  out << YAML::Key << "phi_step" << YAML::Value << target.phi_step;

  // m_pdf3
  out << YAML::Key << "pdf3d" << YAML::Value << YAML::BeginMap;
  SerializePdf3D(target.pdf3d, out);
  out << YAML::EndMap;
}

void DeserializePdf4D(BtfPdf4D &target, const YAML::Node &in) {
  if (in["pdf4d_size"])
    target.pdf4d_size = in["pdf4d_size"].as<int>();
  if (in["phi_size"])
    target.phi_size = in["phi_size"].as<int>();
  if (in["phi_step"])
    target.phi_step = in["phi_step"].as<float>();

  if (in["pdf3d"])
    DeserializePdf3D(target.pdf3d, in["pdf3d"]);
}

void SerializePdf6D(const BtfPdf6D &target, YAML::Emitter &out) {
  out << YAML::Key << "row_size" << YAML::Value << target.row_size;
  out << YAML::Key << "column_size" << YAML::Value << target.column_size;
  out << YAML::Key << "row_offset" << YAML::Value << target.row_offset;
  out << YAML::Key << "column_offset" << YAML::Value << target.column_offset;
  out << YAML::Key << "color_size" << YAML::Value << target.color_size;

  // m_pdf4
  out << YAML::Key << "pdf4d" << YAML::Value << YAML::BeginMap;
  SerializePdf4D(target.pdf4d, out);
  out << YAML::EndMap;
}

void DeserializePdf6D(BtfPdf6D &target, const YAML::Node &in) {
  if (in["row_size"])
    target.row_size = in["row_size"].as<int>();
  if (in["column_size"])
    target.column_size = in["column_size"].as<int>();
  if (in["row_offset"])
    target.row_offset = in["row_offset"].as<int>();

  if (in["column_offset"])
    target.column_offset = in["column_offset"].as<int>();

  if (in["color_size"])
    target.color_size = in["color_size"].as<int>();

  if (in["pdf4d"])
    DeserializePdf4D(target.pdf4d, in["pdf4d"]);
}

void SerializeBtfBase(const BtfBase &target, YAML::Emitter &out) {
  out << YAML::Key << "shared_coordinates" << YAML::Value << YAML::BeginMap;
  SerializeSharedCoordinates(target.shared_coordinates, out);
  out << YAML::EndMap;

  out << YAML::Key << "pdf6d" << YAML::Value << YAML::BeginMap;
  SerializePdf6D(target.pdf6d, out);
  out << YAML::EndMap;

  out << YAML::Key << "hdr" << YAML::Value << target.hdr;
  out << YAML::Key << "hdr_value" << YAML::Value << target.hdr_value;

  out << YAML::Key << "tex_coord_multiplier" << YAML::Value << target.tex_coord_multiplier;
  out << YAML::Key << "gamma" << YAML::Value << target.gamma;

  out << YAML::Key << "material_order" << YAML::Value << target.material_order;
  out << YAML::Key << "color_channel" << YAML::Value << target.color_channel;

  out << YAML::Key << "use_cos_beta" << YAML::Value << target.use_cos_beta;
  out << YAML::Key << "post_scale" << YAML::Value << target.post_scale;

  out << YAML::Key << "beta_size" << YAML::Value << target.beta_size;
  out << YAML::Key << "alpha_size" << YAML::Value << target.alpha_size;
  out << YAML::Key << "theta_size" << YAML::Value << target.theta_size;
  out << YAML::Key << "phi_size" << YAML::Value << target.phi_size;

  out << YAML::Key << "alpha_step" << YAML::Value << target.alpha_step;
  out << YAML::Key << "theta_step" << YAML::Value << target.theta_step;
  out << YAML::Key << "phi_step" << YAML::Value << target.phi_step;

  out << YAML::Key << "all_materials_in_one_database" << YAML::Value << target.all_materials_in_one_database;
  out << YAML::Key << "use34_view_representation" << YAML::Value << target.use34_view_representation;
  out << YAML::Key << "use_pdf2_compact_rep" << YAML::Value << target.use_pdf2_compact_rep;

  out << YAML::Key << "material_count" << YAML::Value << target.material_count;
}

void DeserializeBtfBase(BtfBase &target, const YAML::Node &in) {
  if (in["shared_coordinates"])
    DeserializeSharedCoordinates(target.shared_coordinates, in["shared_coordinates"]);
  if (in["pdf6d"])
    DeserializePdf6D(target.pdf6d, in["pdf6d"]);
  if (in["hdr"])
    target.hdr = in["hdr"].as<bool>();
  if (in["hdr_value"])
    target.hdr_value = in["hdr_value"].as<float>();
  if (in["tex_coord_multiplier"])
    target.tex_coord_multiplier = in["tex_coord_multiplier"].as<float>();
  if (in["gamma"])
    target.gamma = in["gamma"].as<float>();
  if (in["material_order"])
    target.material_order = in["material_order"].as<int>();
  if (in["color_channel"])
    target.color_channel = in["color_channel"].as<int>();
  if (in["use_cos_beta"])
    target.use_cos_beta = in["use_cos_beta"].as<bool>();
  if (in["post_scale"])
    target.post_scale = in["post_scale"].as<float>();
  if (in["beta_size"])
    target.beta_size = in["beta_size"].as<int>();
  if (in["alpha_size"])
    target.alpha_size = in["alpha_size"].as<int>();
  if (in["theta_size"])
    target.theta_size = in["theta_size"].as<int>();
  if (in["phi_size"])
    target.phi_size = in["phi_size"].as<int>();

  if (in["alpha_step"])
    target.alpha_step = in["alpha_step"].as<float>();
  if (in["theta_step"])
    target.theta_step = in["theta_step"].as<float>();
  if (in["phi_step"])
    target.phi_step = in["phi_step"].as<float>();

  if (in["all_materials_in_one_database"])
    target.all_materials_in_one_database = in["all_materials_in_one_database"].as<bool>();
  if (in["use34_view_representation"])
    target.use34_view_representation = in["use34_view_representation"].as<bool>();
  if (in["use_pdf2_compact_rep"])
    target.use_pdf2_compact_rep = in["use_pdf2_compact_rep"].as<bool>();

  if (in["material_count"])
    target.material_count = in["material_count"].as<int>();
}

template <typename T>
void LoadBinaryList(const std::string &name, const YAML::Node &in, std::vector<T> &target) {
  if (in[name]) {
    const auto &data = in[name].as<YAML::Binary>();
    target.resize(data.size() / sizeof(T));
    std::memcpy(target.data(), data.data(), data.size());
  }
}

template <typename T>
void SaveBinaryList(const std::string &name, YAML::Emitter &out, const std::vector<T> &target) {
  if (!target.empty()) {
    out << YAML::Key << name << YAML::Value
        << YAML::Binary((const unsigned char *)target.data(), target.size() * sizeof(T));
  }
}

void evo_engine::SerializeBtfMaterial(YAML::Emitter &out, const BtfMaterial &target) {
  if (target.btf_base.has_data) {
    out << YAML::Key << "btf_base" << YAML::Value << YAML::BeginMap;
    SerializeBtfBase(target.btf_base, out);
    out << YAML::EndMap;

    SaveBinaryList("shared_coordinates_beta_angles", out, target.shared_coordinates_beta_angles);

    SaveBinaryList("pdf6d", out, target.pdf6d);
    SaveBinaryList("pdf6d_scales", out, target.pdf6d_scales);

    SaveBinaryList("pdf4d", out, target.pdf4d);
    SaveBinaryList("pdf4d_scales", out, target.pdf4d_scales);

    SaveBinaryList("pdf3d", out, target.pdf3d);
    SaveBinaryList("pdf3d_scales", out, target.pdf3d_scales);

    SaveBinaryList("luminance_color_indices", out, target.luminance_color_indices);
    SaveBinaryList("pdf2d_colors", out, target.pdf2d_colors);
    SaveBinaryList("pdf2d_scales", out, target.pdf2d_scales);
    SaveBinaryList("pdf2d", out, target.pdf2d);

    SaveBinaryList("index_ab", out, target.index_ab);

    SaveBinaryList("pdf1d", out, target.pdf1d);
    SaveBinaryList("vector_color", out, target.vector_color);
  }
}

void evo_engine::DeserializeBtfMaterial(const YAML::Node &in, BtfMaterial &target) {
  target.btf_base.has_data = false;
  if (in["btf_base"]) {
    DeserializeBtfBase(target.btf_base, in["btf_base"]);

    LoadBinaryList("shared_coordinates_beta_angles", in, target.shared_coordinates_beta_angles);

    LoadBinaryList("pdf6d", in, target.pdf6d);
    LoadBinaryList("pdf6d_scales", in, target.pdf6d_scales);

    LoadBinaryList("pdf4d", in, target.pdf4d);
    LoadBinaryList("pdf4d_scales", in, target.pdf4d_scales);

    LoadBinaryList("pdf3d", in, target.pdf3d);
    LoadBinaryList("pdf3d_scales", in, target.pdf3d_scales);

    LoadBinaryList("luminance_color_indices", in, target.luminance_color_indices);
    LoadBinaryList("pdf2d_colors", in, target.pdf2d_colors);
    LoadBinaryList("pdf2d_scales", in, target.pdf2d_scales);
    LoadBinaryList("pdf2d", in, target.pdf2d);

    LoadBinaryList("index_ab", in, target.index_ab);

    LoadBinaryList("pdf1d", in, target.pdf1d);
    LoadBinaryList("vector_color", in, target.vector_color);

    target.btf_base.has_data = true;
  }
  target.saved_ = true;
}
