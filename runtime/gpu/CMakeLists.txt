add_library(radion_gpu
    src/GPU.cpp
    backends/gl/GLLoader.cpp
    backends/gl/GLCaps.cpp
    backends/gl/GLContext.cpp
    backends/gl/GLDevice.cpp
    backends/gl/GLProfiler.cpp
)

target_precompile_headers(radion_gpu PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_include_directories(radion_gpu PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(radion_gpu PUBLIC radion_core SDL2::SDL2 OpenGL::GL)
