#define VOLK_IMPLEMENTATION
#include <volk.h>
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#else

#endif

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include <stb_image_resize.h>
#include <stb_image_write.h>

#define VMA_IMPLEMENTATION
#include "vk_mem_alloc.h"