#pragma once

#include "CIELab.cuh"
#include "IndexAB.cuh"
#include "Pdf1D.cuh"

/**
 * \brief Since CUDA have poor support for virtual class functions, I used this
 * instead.
 */
namespace evo_engine {
struct Pdf2D {
  /**
   * \brief The used number of 2D functions
   */
  int pdf2d_size;
  /**
   * \brief The length of index slice, should be 2 here.
   */
  int slice_length;
  IndexAB index_ab;
  Pdf1D pdf1d;

  struct Color {
    /**
     * \brief The used number of 2D functions
     */
    int pdf2d_size;
    /**
     * \brief The length of index slice.
     */
    int alpha_size;
    void Init(const int num_of_alpha) {
      alpha_size = num_of_alpha;
      pdf2d_size = 0;
    }

#pragma region CUDA
    // Here are the indices to CIndexAB class
    CudaBuffer pdf2d_buffer;
    int *pdf2d_device_ptr;
    __device__ void ToAbColor(const int slice, glm::vec3 &out, const SharedCoordinates &tc,
                              const IndexAB &target_index_ab) const {
      const int i = tc.current_alpha_low_bound;
      assert(i >= 0 && i < alpha_size - 1);
      assert(slice >= 0 && slice < pdf2d_size);
      const float w = tc.alpha_weight;
      glm::vec3 ab1, ab2;
      // colors
      target_index_ab.ToAbColor(pdf2d_device_ptr[slice * alpha_size + i], ab1, tc);
      target_index_ab.ToAbColor(pdf2d_device_ptr[slice * alpha_size + i + 1], ab2, tc);
      out[1] = ab1[0] * (1.0f - w) + ab2[0] * w;
      out[2] = ab1[1] * (1.0f - w) + ab2[1] * w;
    }

#pragma endregion
  };

  struct Luminance {
    /**
     * \brief The used number of 2D functions
     */
    int pdf2d_size;
    /**
     * \brief The length of index slice.
     */
    int alpha_size;

    void Init(const int num_of_slice) {
      alpha_size = num_of_slice;
      pdf2d_size = 0;
    }

#pragma region CUDA
    // the database of 1D functions over luminance
    // Here are the indices to Pdf1D class
    CudaBuffer pdf1d_buffer;
    int *pdf1d_device_ptr;
    // Here are the scale to Pdf1D class, PDF1 functions are multiplied by that
    CudaBuffer pdf1d_scale_buffer;
    float *pdf1d_scale_device_ptr;
    // This is optional, not required for rendering, except importance sampling
    // float* m_pdf2DNorm;
    __device__ void ToLuminance(const int slice, glm::vec3 &out, const SharedCoordinates &tc, const Pdf1D &pdf1) const {
      assert(slice >= 0 && slice < pdf2d_size);
      const int i = tc.current_alpha_low_bound;
      const float w = tc.alpha_weight;
      assert(i >= 0 && i < alpha_size - 1);
      // This is different to compact representation! we interpolate in luminance
      const float l1 =
          pdf1d_scale_device_ptr[slice * alpha_size + i] * pdf1.GetVal(pdf1d_device_ptr[slice * alpha_size + i], tc);
      const float l2 = pdf1d_scale_device_ptr[slice * alpha_size + i + 1] *
                       pdf1.GetVal(pdf1d_device_ptr[slice * alpha_size + i + 1], tc);
      out[0] = (1.f - w) * l1 + w * l2;
    }

#pragma endregion
  };

  /**
   * \brief Here are the instances of color 2D function database.
   */
  Color color;
  /**
   * \brief Here are the instances of luminance 2D function database.
   */
  Luminance luminance;

  void Init() {
    color.Init(pdf1d.beta_size);
    luminance.Init(pdf1d.beta_size);
  }

#pragma region CUDA
  /**
   * \brief Here are the indices of luminance + color 2D functions. Index [][0] is luminance, index [][1] is color.
   */
  CudaBuffer luminance_color_index_buffer;
  int *luminance_color_indices_device_ptr;
  __device__ void GetVal(const int slice, glm::vec3 &out, const SharedCoordinates &tc) const {
    assert(slice >= 0 && slice < pdf2d_size);

    glm::vec3 user_color_model_data;
    // First, get only luminance
    out = glm::vec3(1.0f);
    color.ToAbColor(luminance_color_indices_device_ptr[slice * slice_length + 1], user_color_model_data, tc, index_ab);
    luminance.ToLuminance(luminance_color_indices_device_ptr[slice * slice_length + 0], user_color_model_data, tc,
                          pdf1d);

    // Convert to RGB
    UserCmToRgb(user_color_model_data, out, tc);
  }
#pragma endregion
};

}  // namespace evo_engine
