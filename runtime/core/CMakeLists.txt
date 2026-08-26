add_library(radion_core
    src/Archive.cpp
    src/ByteArray.cpp
    src/ChaCha20.cpp
    src/FilePack.cpp
    src/Color.cpp
    src/Pixmap.cpp
    src/Profiler.cpp
    src/RadionFormat.cpp
    src/Window.cpp
    src/BoundsTree.cpp
    src/Thread.cpp
    src/FileSystem.cpp
    src/Input.cpp
    src/Log.cpp
    src/Math.cpp
    src/Noise.cpp
    src/Timer.cpp
    src/ZipArchive.cpp
    src/stb_impl.cpp
    ${PROJECT_SOURCE_DIR}/vendor/miniz/miniz.c
)

target_precompile_headers(radion_core PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${CMAKE_CURRENT_SOURCE_DIR}/include/PCH.h>"
)

target_include_directories(radion_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(radion_core SYSTEM PUBLIC
    ${PROJECT_SOURCE_DIR}/vendor
    ${PROJECT_SOURCE_DIR}/vendor/glm
    ${PROJECT_SOURCE_DIR}/vendor/miniz
)
target_link_libraries(radion_core PUBLIC radion_options SDL2::SDL2)
