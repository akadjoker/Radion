#include "PCH.h"

// The single translation unit that carries the glad implementation. Every
// other file only sees the declarations.

#include <cstdlib>

#define GLAD_MALLOC(size) std::malloc(size)
#define GLAD_FREE(pointer) std::free(pointer)
#define GLAD_GL_IMPLEMENTATION
#include <glad.h>
