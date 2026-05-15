#pragma once

#include <Optix7.hpp>

#include <Pdf6D.cuh>

#include <SharedCoordinates.cuh>

#include <glm/glm.hpp>

namespace evo_engine {
struct BtfBase {
  bool has_data = false;

  SharedCoordinates shared_coordinates;
  Pdf6D<glm::vec3> pdf6d;

  bool hdr = false;
  float hdr_value = 1.0f;

  float tex_coord_multiplier = 1.0f;
  float gamma = 1.f;

  int material_order;  //! order of the material processed
  int color_channel;   //! number of spectral channels in BTF data

  bool use_cos_beta;  //! use cos angles
  float post_scale;

  int beta_size;
  int alpha_size;
  int theta_size;
  int phi_size;

  float alpha_step;
  float theta_step;
  float phi_step;

  bool all_materials_in_one_database;  //! if to compress all materials into one
  //! database
  //! if view direction represented directly by UBO measurement quantization
  bool use34_view_representation;
  bool use_pdf2_compact_rep;  //! If we do not separate colors and luminance for
  //! 2D functions

  int material_count;  //! how many materials are stored in the database
#pragma region CUDA
  __device__ void GetValueDeg(const glm::vec2 &tex_coord, const float illumination_theta, const float illumination_phi,
                              const float view_theta, const float view_phi, glm::vec3 &out) const {
    if (!has_data) {
      out = glm::vec3(255, 0, 255);
      return;
    }

    if (illumination_theta > 90.0f || view_theta > 90.0f) {
      out = glm::vec3(0.0f);
      return;
    }
    SharedCoordinates temp_coordinate = shared_coordinates;
    // fast version, pre-computation of interpolation values only once
    pdf6d.GetValDeg2(tex_coord * tex_coord_multiplier, illumination_theta, illumination_phi, view_theta, view_phi, out,
                     temp_coordinate);
    if (hdr) {
      // we encode the values multiplied by a user coefficient
      // before it is converted to User Color Model
      // Now we have to multiply it back.
      const float multi = 1.0f / hdr_value;
      out *= multi;
    }
    out = glm::pow(out, glm::vec3(gamma));
  }
#pragma endregion
};
}  // namespace evo_engine
