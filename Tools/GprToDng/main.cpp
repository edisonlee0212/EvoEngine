#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>
#include "gpr.h"

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: GprToDng input.GPR output.DNG\n";
    return 1;
  }
  std::ifstream input(argv[1], std::ios::binary);
  if (!input)
    return 1;
  std::vector<char> bytes((std::istreambuf_iterator<char>(input)), {});
  gpr_allocator allocator{malloc, free};
  gpr_buffer source{bytes.data(), bytes.size()}, destination{nullptr, 0};
  gpr_parameters parameters;
  gpr_parameters_set_defaults(&parameters);
  if (!gpr_parse_metadata(&allocator, &source, &parameters))
    return 2;
  // Decompress the RAW Bayer data, preserving camera metadata for LibRaw demosaicing.
  if (!gpr_convert_gpr_to_dng(&allocator, &parameters, &source, &destination)) {
    gpr_parameters_destroy(&parameters, allocator.Free);
    std::cerr << "GoPro SDK conversion failed\n";
    return 2;
  }
  std::ofstream output(argv[2], std::ios::binary);
  output.write(static_cast<const char*>(destination.buffer), destination.size);
  allocator.Free(destination.buffer);
  gpr_parameters_destroy(&parameters, allocator.Free);
  return output ? 0 : 3;
}
