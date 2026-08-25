add_library(radion_engine
    src/DefaultPack.cpp
    src/Engine.cpp
    src/EngineSettings.cpp
    src/SceneManager.cpp
)

# The files a build cannot start without, packed at build time and compiled
# in. Off when the assets are not beside the source (a checkout carrying only
# code), so the build still configures - it just has nothing to fall back on.
set(RADION_DEFAULT_PACK_SHADERS "${PROJECT_SOURCE_DIR}/assets/shaders")
set(RADION_DEFAULT_PACK_FLARES "${PROJECT_SOURCE_DIR}/assets/textures/lensflare")

option(RADION_EMBED_DEFAULT_PACK "Compile the default shaders and lens flares into the binary" ON)
if(RADION_EMBED_DEFAULT_PACK AND IS_DIRECTORY "${RADION_DEFAULT_PACK_SHADERS}")
    set(default_pack "${CMAKE_CURRENT_BINARY_DIR}/default.rpak")
    set(default_pack_source "${CMAKE_CURRENT_BINARY_DIR}/DefaultPackData.cpp")

    file(GLOB_RECURSE default_pack_inputs CONFIGURE_DEPENDS
        "${RADION_DEFAULT_PACK_SHADERS}/*"
        "${RADION_DEFAULT_PACK_FLARES}/*"
    )

    add_custom_command(
        OUTPUT "${default_pack_source}"
        COMMAND ${CMAKE_COMMAND} -E env ASAN_OPTIONS=detect_leaks=0
                $<TARGET_FILE:radion_pack>
                -o "${default_pack}"
                -c "${default_pack_source}"
                "${RADION_DEFAULT_PACK_SHADERS}"
                -p shaders "${RADION_DEFAULT_PACK_SHADERS}"
                -p lensflare "${RADION_DEFAULT_PACK_FLARES}"
        DEPENDS radion_pack ${default_pack_inputs}
        COMMENT "Packing the default shaders and lens flares"
        VERBATIM
    )

    target_sources(radion_engine PRIVATE "${default_pack_source}")
    target_compile_definitions(radion_engine PRIVATE RADION_HAS_DEFAULT_PACK=1)
endif()
target_precompile_headers(radion_engine PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_include_directories(radion_engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(radion_engine PRIVATE ${PROJECT_SOURCE_DIR}/vendor)
target_link_libraries(radion_engine PUBLIC radion_render radion_scene)
