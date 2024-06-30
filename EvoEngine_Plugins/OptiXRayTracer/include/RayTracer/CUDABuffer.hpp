#pragma once

#include "cuda.h"
#include "vector"

struct cudaGraphicsResource;
namespace evo_engine {

/*! simple wrapper for creating, and managing a device-side CUDA
        buffer */
class CudaBuffer {

public:
  void *d_ptr = nullptr;
  size_t size_in_bytes = 0;
  /**
   * \brief Get CUDA Device pointer.
   * \return CUDA Device memory pointer.
   */
  [[nodiscard]] CUdeviceptr DevicePointer() const;
  /**
   * \brief Resize device memory by size.
   * \param size How many bytes for resize.
   */
  void Resize(const size_t &size);
  /**
   * \brief Free memory from device.
   */
  void Free();
  void Upload(void *t, const size_t &size, const size_t &count);
  void Download(void *t, const size_t &size, const size_t &count) const;
  template <typename T> void Upload(std::vector<T> &data);
  template <typename T> void Upload(T *t, const size_t &count);
  template <typename T> void Download(T *t, const size_t &count);
};
template <typename T> void CudaBuffer::Upload(std::vector<T> &data) {
  Upload(data.data(), data.size());
}

template <typename T> void CudaBuffer::Upload(T *t, const size_t &count) {
  Upload(t, sizeof(T), count);
}

template <typename T> void CudaBuffer::Download(T *t, const size_t &count) {
  Download(static_cast<void *>(t), sizeof(T), count);
}
} // namespace EvoEngine
