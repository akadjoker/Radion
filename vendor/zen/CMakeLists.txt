# Vendored libzen (the Zen scripting language). builtin_numpy is the one
# module intentionally not part of this list; see runtime/script for the
# Radion-side bindings.
add_library(zen_static STATIC
    src/builtin_base.cpp
    src/builtin_math.cpp
    src/builtin_json.cpp
    src/builtin_time.cpp
    src/builtin_io.cpp
    src/builtin_path.cpp
    src/builtin_os.cpp
    src/builtin_struct.cpp
    src/builtin_net.cpp
    src/builtin_http.cpp
    src/bytecode.cpp
    src/compiler.cpp
    src/compiler_expressions.cpp
    src/compiler_statements.cpp
    src/debug.cpp
    src/emitter.cpp
    src/lexer.cpp
    src/memory.cpp
    src/vm.cpp
    src/vm_dispatch.cpp
    src/zen_arena.cpp
)

target_include_directories(zen_static PUBLIC include include/zen PRIVATE src)
target_link_libraries(zen_static PUBLIC ${CMAKE_DL_LIBS})
# builtin_net/http speak sockets directly; on Windows that is ws2_32, which
# every other platform gets from libc.
if(WIN32)
    target_link_libraries(zen_static PUBLIC ws2_32)
endif()
set_target_properties(zen_static PROPERTIES
    OUTPUT_NAME zen
    POSITION_INDEPENDENT_CODE ON
)
target_compile_options(zen_static PRIVATE $<$<CONFIG:Release>:-O3>)
target_compile_definitions(zen_static PRIVATE $<$<CONFIG:Release>:NDEBUG>)
