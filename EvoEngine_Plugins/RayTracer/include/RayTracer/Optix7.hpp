#pragma once
#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "iostream"
#include "optix.h"
#include "optix_stubs.h"

#include "sstream"
#include "string"
#define CUDA_CHECK(call)                                                       \
  {                                                                            \
    cudaError_t rc = cuda##call;                                               \
    if (rc != cudaSuccess) {                                                   \
      std::stringstream txt;                                                   \
      cudaError_t err = rc; /*cudaGetLastError();*/                            \
      txt << std::string("CUDA Error ") << cudaGetErrorName(err) << " ("                    \
          << cudaGetErrorString(err) << ")";                                   \
      fprintf(stderr, "CUDA Error (%s: line %d): %s\n", __FILE__, __LINE__,    \
              cudaGetErrorString(rc));                                         \
      throw std::runtime_error(txt.str());                                     \
    }                                                                          \
  }

#define CUDA_CHECK_NOEXCEPT(call)                                              \
  { cuda##call; }

#define OPTIX_CHECK(call)                                                      \
  {                                                                            \
    OptixResult res = call;                                                    \
    if (res != OPTIX_SUCCESS) {                                                \
      fprintf(stderr, "Optix call (%s) failed with code %d (line %d)\n",       \
              #call, res, __LINE__);                                           \
      exit(2);                                                                 \
    }                                                                          \
  }

#define CUDA_ERROR_CHECK()                                                     \
  {                                                                            \
    cudaError_t error = cudaGetLastError();                                    \
    if (error != cudaSuccess) {                                                \
      fprintf(stderr, "CUDA Error (%s: line %d): %s\n", __FILE__, __LINE__,    \
              cudaGetErrorString(error));                                      \
      exit(2);                                                                 \
    }                                                                          \
  }

#define CUDA_SYNC_CHECK()                                                      \
  {                                                                            \
    cudaDeviceSynchronize();                                                   \
    cudaError_t error = cudaGetLastError();                                    \
    if (error != cudaSuccess) {                                                \
      fprintf(stderr, "CUDA Error (%s: line %d): %s\n", __FILE__, __LINE__,    \
              cudaGetErrorString(error));                                      \
      exit(2);                                                                 \
    }                                                                          \
  }
