
#include <CUDABuffer.hpp>
#include <Optix7.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>

#include <glm/glm.hpp>

using namespace evo_engine;

CUdeviceptr CudaBuffer::DevicePointer() const {
  return reinterpret_cast<CUdeviceptr>(d_ptr);
}

void CudaBuffer::Resize(const size_t &size) {
  if (this->size_in_bytes == size)
    return;
  Free();
  this->size_in_bytes = size;
  CUDA_CHECK(Malloc(&d_ptr, size_in_bytes));
}

void CudaBuffer::Free() {
  if (d_ptr == nullptr)
    return;
  CUDA_CHECK(Free(d_ptr));
  d_ptr = nullptr;
  size_in_bytes = 0;
}

void CudaBuffer::Upload(void *t, const size_t &size, const size_t &count) {
  Resize(count * size);
  CUDA_CHECK(Memcpy(d_ptr, t, count * size, cudaMemcpyHostToDevice));
}

void CudaBuffer::Download(void *t, const size_t &size, const size_t &count) const {
  CUDA_CHECK(Memcpy(t, d_ptr, count * size, cudaMemcpyDeviceToHost));
}
