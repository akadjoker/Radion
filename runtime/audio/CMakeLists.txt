# Audio library: mixer, sound loading and voices, on top of miniaudio.
# Depends only on radion_core, for FileSystem, ByteArray and Log - it does
# not touch render/gpu/scene/engine.

set(MINIAUDIO_ROOT ${CMAKE_SOURCE_DIR}/vendor/miniaudio)

# Built as C on purpose: compiling the implementation as C++ mangles the
# names miniaudio's own backends resolve at runtime.
add_library(radion_miniaudio STATIC
    ${MINIAUDIO_ROOT}/miniaudio_impl.c
)
target_include_directories(radion_miniaudio SYSTEM PUBLIC ${CMAKE_SOURCE_DIR}/vendor)

if(UNIX AND NOT APPLE)
    find_package(Threads REQUIRED)
    target_link_libraries(radion_miniaudio PUBLIC Threads::Threads ${CMAKE_DL_LIBS} m)
endif()

add_library(radion_audio
    src/AudioEngine.cpp
)

target_precompile_headers(radion_audio PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)

target_include_directories(radion_audio PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(radion_audio PUBLIC radion_core PRIVATE radion_miniaudio)
