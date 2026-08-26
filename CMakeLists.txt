cmake_minimum_required(VERSION 3.21)
project(Radion VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)

# Refuse to configure in the source tree. Not pedantry: it scatters
# CMakeCache.txt, CMakeFiles/ and every object file among the sources, and the
# only way back out is deleting them by hand from among files you want to keep.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR
        "Radion does not configure in the source tree.\n"
        "  cmake -S . -B build\n"
        "  cmake --build build")
endif()

# Programs land in bin/, next to nothing else. The build tree keeps every
# intermediate - objects, static libraries, CMake's own bookkeeping - so bin/
# holds only things you can actually run, and deleting the build tree costs a
# reconfigure and nothing more.
#
# Static libraries deliberately stay behind: they are not runnable, and mixing
# them in is what made it unclear which files in here mattered.
set(RADION_BINARY_DIR "${PROJECT_SOURCE_DIR}/bin")

# The check above misses `cmake -B bin`: bin/ is a subdirectory, not the
# project root, so it slips through and mixes CMake's own files in with the
# executables. Refuse that case too.
if(CMAKE_BINARY_DIR STREQUAL RADION_BINARY_DIR)
    message(FATAL_ERROR
        "bin/ is where programs land, not where Radion configures.\n"
        "  cmake -S . -B build\n"
        "  cmake --build build")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${RADION_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${RADION_BINARY_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Multi-config generators (Visual Studio, Ninja Multi-Config) otherwise append
# another Debug/Release directory and break the same layout used on Linux.
foreach(config Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${config}" config_upper)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${config_upper} "${RADION_BINARY_DIR}")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${config_upper} "${RADION_BINARY_DIR}")
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${config_upper} "${CMAKE_BINARY_DIR}/lib")
endforeach()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_SHARED_LIBS "Build Radion libraries as shared libraries" OFF)
option(RADION_ENABLE_SANITIZERS "Enable sanitizers in Debug builds" ON)
option(RADION_BUILD_EDITOR "Build the Radion editor" ON)
option(RADION_BUILD_RUNNER "Build the standalone Radion game runner" ON)
option(RADION_BUILD_BLENDER "Build the Radion Blender tool" ON)
option(RADION_BUILD_VOXEL_DEMO "Build the voxel terrain demo" ON)
 

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(RadionOptions)
include(RadionFormat)

# Pinned Git submodules.  Keep their own test/benchmark CMake projects out of
# Radion's build; the engine exposes just the dependency targets it consumes.
add_library(radion_containers INTERFACE)
target_include_directories(radion_containers SYSTEM INTERFACE
    ${PROJECT_SOURCE_DIR}/vendor/containers/include
)

add_library(radion_math STATIC
    ${PROJECT_SOURCE_DIR}/vendor/mathc/mathc.cpp
)
target_include_directories(radion_math SYSTEM PUBLIC
    ${PROJECT_SOURCE_DIR}/vendor/mathc
)
target_link_libraries(radion_math PUBLIC radion_options)

find_package(SDL2 REQUIRED)
find_package(OpenGL REQUIRED)

if(WIN32 AND BUILD_SHARED_LIBS)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

if(BUILD_SHARED_LIBS)
    message(STATUS "Radion libraries: shared")
else()
    message(STATUS "Radion libraries: static")
endif()

add_subdirectory(runtime/core)

# Before the modules, because runtime/engine runs it at build time to produce
# the default pack it compiles in.
add_subdirectory(tools/packer)

add_subdirectory(runtime/gpu)
add_subdirectory(runtime/render)
add_subdirectory(runtime/physics)
add_subdirectory(runtime/geometry)
add_subdirectory(runtime/ai)
add_subdirectory(runtime/voxel)

# Vendored Zen scripting language (core VM + simple builtin modules only),
# and the Radion-side bindings on top of it - before runtime/scene, which
# links radion_script for its ZenBehaviour component.
add_subdirectory(vendor/zen)
add_subdirectory(runtime/script)

add_subdirectory(runtime/scene)
add_subdirectory(runtime/engine)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

if(RADION_BUILD_EDITOR)
    add_subdirectory(editor)
endif()

if(RADION_BUILD_RUNNER)
    add_subdirectory(runner)
endif()

if(RADION_BUILD_BLENDER)
    add_subdirectory(blend)
endif()

if(RADION_BUILD_VOXEL_DEMO)
    add_subdirectory(examples/voxel_demo)
endif()

add_subdirectory(tools/lightmapbake)

# A release is a self-contained application tree: the tools and editors in
# bin/, plus the assets they browse and use as fallback content.  Keep this at
# the top level so optional applications simply drop out with their build
# options instead of making installation configuration-specific.
include(GNUInstallDirs)

set(RADION_INSTALL_TARGETS radion_pack radion_lightmapbake)
if(RADION_BUILD_EDITOR)
    list(APPEND RADION_INSTALL_TARGETS radion_editor)
endif()
if(RADION_BUILD_RUNNER)
    list(APPEND RADION_INSTALL_TARGETS radion_runner)
endif()
if(RADION_BUILD_BLENDER)
    list(APPEND RADION_INSTALL_TARGETS radion_blender)
endif()

install(TARGETS ${RADION_INSTALL_TARGETS}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY assets DESTINATION .)
install(FILES README.md DESTINATION .)
