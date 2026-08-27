#pragma once

namespace evo_engine {
struct BtfSharedCoordinates {
  bool use_cos_beta = false;
  int beta_size = 0;
  float alpha_step = 0.0f;
  int alpha_size = 0;
  float theta_step = 0.0f;
  int theta_size = 0;
  float phi_step = 0.0f;
  int phi_size = 0;
  float beta = 0.0f;
  float alpha = 0.0f;
  float theta = 0.0f;
  float phi = 0.0f;
  int current_beta_low_bound = 0;
  float beta_weight = 0.0f;
  float beta_min2 = 0.0f;
  int current_alpha_low_bound = 0;
  float alpha_weight = 0.0f;
  float alpha_min2 = 0.0f;
  int current_theta_low_bound = 0;
  float theta_weight = 0.0f;
  float theta_min2 = 0.0f;
  int current_phi_low_bound = 0;
  float phi_weight = 0.0f;
  float scale = 0.0f;
  bool hdr_flag = false;
  bool use_btf_flag = false;

  void Set(bool use_btf, bool use_cos, int beta_count, int alpha_count, float alpha_increment, int theta_count,
           float theta_increment, int phi_count, float phi_increment) {
    use_btf_flag = use_btf;
    use_cos_beta = use_cos;
    beta_size = beta_count;
    alpha_size = alpha_count;
    alpha_step = alpha_increment;
    theta_size = theta_count;
    theta_step = theta_increment;
    phi_size = phi_count;
    phi_step = phi_increment;
  }
};

struct BtfVectorColor {
  int start_index = 0;
  int channel_size = 2;
  int color_size = 0;
  void Init() {
    start_index = 0;
    channel_size = 2;
    color_size = 0;
  }
};

struct BtfIndexAb {
  int index_size = 0;
  int beta_size = 0;
  BtfVectorColor vector_color;
  void Init(int beta_count) {
    beta_size = beta_count;
    index_size = 0;
  }
};

struct BtfPdf1D {
  int beta_size = 0;
  int pdf1d_size = 0;
  void Init(int beta_count) {
    beta_size = beta_count;
    pdf1d_size = 0;
  }
};

struct BtfPdf2D {
  struct Channel {
    int pdf2d_size = 0;
    int alpha_size = 0;
  };

  int pdf2d_size = 0;
  int slice_length = 0;
  BtfIndexAb index_ab;
  BtfPdf1D pdf1d;
  Channel color;
  Channel luminance;

  void Init() {
    color.alpha_size = pdf1d.beta_size;
    color.pdf2d_size = 0;
    luminance.alpha_size = pdf1d.beta_size;
    luminance.pdf2d_size = 0;
  }
};

struct BtfPdf3D {
  int pdf3d_size = 0;
  int theta_size = 0;
  BtfPdf2D pdf2d;
  void Init(int theta_count) {
    theta_size = theta_count;
    pdf3d_size = 0;
  }
};

struct BtfPdf4D {
  int pdf4d_size = 0;
  int phi_size = 0;
  float phi_step = 0.0f;
  BtfPdf3D pdf3d;
  void Init(int phi_count) {
    phi_size = phi_count;
    phi_step = 360.0f / static_cast<float>(phi_count);
    pdf4d_size = 0;
  }
};

struct BtfPdf6D {
  int row_size = 0;
  int column_size = 0;
  int row_offset = 0;
  int column_offset = 0;
  int color_size = 0;
  BtfPdf4D pdf4d;
  void Init(int rows, int columns, int row_start, int column_start, int colors) {
    row_size = rows;
    column_size = columns;
    row_offset = row_start;
    column_offset = column_start;
    color_size = colors;
  }
};

struct BtfBase {
  bool has_data = false;
  BtfSharedCoordinates shared_coordinates;
  BtfPdf6D pdf6d;
  bool hdr = false;
  float hdr_value = 1.0f;
  float tex_coord_multiplier = 1.0f;
  float gamma = 1.0f;
  int material_order = 0;
  int color_channel = 0;
  bool use_cos_beta = false;
  float post_scale = 1.0f;
  int beta_size = 0;
  int alpha_size = 0;
  int theta_size = 0;
  int phi_size = 0;
  float alpha_step = 0.0f;
  float theta_step = 0.0f;
  float phi_step = 0.0f;
  bool all_materials_in_one_database = false;
  bool use34_view_representation = false;
  bool use_pdf2_compact_rep = false;
  int material_count = 0;
};

using SharedCoordinates = BtfSharedCoordinates;
using VectorColor = BtfVectorColor;
using IndexAB = BtfIndexAb;
using Pdf1D = BtfPdf1D;
using Pdf2D = BtfPdf2D;
template <typename T>
using Pdf3D = BtfPdf3D;
template <typename T>
using Pdf4D = BtfPdf4D;
template <typename T>
using Pdf6D = BtfPdf6D;
}  // namespace evo_engine
