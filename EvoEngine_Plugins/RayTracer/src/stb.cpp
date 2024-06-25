#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
#else
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#endif
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_RESIZE_IMPLEMENTATION

#include <stb_image_resize.h>
#include <stb_image_write.h>