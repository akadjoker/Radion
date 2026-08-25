#include "PCH.h"

// Single translation unit for both stb single-header implementations - see
// each header's own comment for why this define-then-include pattern exists.
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
