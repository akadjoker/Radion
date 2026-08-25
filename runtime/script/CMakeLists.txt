# Scripting bindings: wraps the vendored Zen VM (vendor/zen) and exposes the
# "radion" native module to scripts. Depends only on radion_core and the Zen
# static library - no render/scene/engine.
add_library(radion_script
    src/ScriptVM.cpp
    src/ScriptModule.cpp
)

target_precompile_headers(radion_script PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)

target_include_directories(radion_script PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(radion_script PUBLIC radion_core PRIVATE zen_static)
