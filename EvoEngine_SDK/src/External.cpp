#define VOLK_IMPLEMENTATION
#include <volk.h>
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#else

#endif
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include <stb_image_resize2.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <cstdio>
#include <cstdlib>

#ifdef _DEBUG
#  define VMA_LEAK_LOG_FORMAT(format, ...)                                                     \
    do {                                                                                       \
      if (const char* path = std::getenv("EVOENGINE_VMA_LEAK_LOG"); path && path[0] != '\0') { \
        if (FILE* file = std::fopen(path, "a")) {                                              \
          std::fprintf(file, format "\n", __VA_ARGS__);                                        \
          std::fclose(file);                                                                   \
        }                                                                                      \
      } else {                                                                                 \
        std::fprintf(stderr, format "\n", __VA_ARGS__);                                        \
      }                                                                                        \
    } while (false)
#endif
#define VMA_IMPLEMENTATION
#include "vk_mem_alloc.h"
