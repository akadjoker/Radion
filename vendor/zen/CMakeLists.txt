# How Radion builds libzen out of the zenpy submodule. The sources live in
# vendor/zenpy/libzen and are not Radion's to edit; this file is, which is why
# it stays outside the submodule rather than using the upstream CMakeLists
# (that one globs src/*.cpp and would pull in modules Radion does not want).
#
# builtin_numpy.cpp is the one module deliberately left off this list. The
# submodule delivers the file - it just never gets compiled or registered, and
# module imports resolve through a runtime registry, so one that was never
# registered simply is not there.
set(ZEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../zenpy/libzen)

if(NOT EXISTS ${ZEN_DIR}/src/vm.cpp)
    message(FATAL_ERROR
        "vendor/zenpy is empty - the zen submodule was not checked out. "
        "Run: git submodule update --init --recursive")
endif()

add_library(zen_static STATIC
    ${ZEN_DIR}/src/builtin_base.cpp
    ${ZEN_DIR}/src/builtin_math.cpp
    ${ZEN_DIR}/src/builtin_json.cpp
    ${ZEN_DIR}/src/builtin_time.cpp
    ${ZEN_DIR}/src/builtin_io.cpp
    ${ZEN_DIR}/src/builtin_path.cpp
    ${ZEN_DIR}/src/builtin_os.cpp
    ${ZEN_DIR}/src/builtin_struct.cpp
    ${ZEN_DIR}/src/builtin_net.cpp
    ${ZEN_DIR}/src/builtin_http.cpp
    ${ZEN_DIR}/src/bytecode.cpp
    ${ZEN_DIR}/src/compiler.cpp
    ${ZEN_DIR}/src/compiler_expressions.cpp
    ${ZEN_DIR}/src/compiler_statements.cpp
    ${ZEN_DIR}/src/debug.cpp
    ${ZEN_DIR}/src/emitter.cpp
    ${ZEN_DIR}/src/lexer.cpp
    ${ZEN_DIR}/src/memory.cpp
    ${ZEN_DIR}/src/vm.cpp
    ${ZEN_DIR}/src/vm_dispatch.cpp
    ${ZEN_DIR}/src/zen_arena.cpp
)

target_include_directories(zen_static
    PUBLIC ${ZEN_DIR}/include ${ZEN_DIR}/include/zen
    PRIVATE ${ZEN_DIR}/src
)
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
if(MSVC)
    target_compile_options(zen_static PRIVATE $<$<CONFIG:Release>:/O2>)
else()
    target_compile_options(zen_static PRIVATE $<$<CONFIG:Release>:-O3>)
endif()
target_compile_definitions(zen_static PRIVATE $<$<CONFIG:Release>:NDEBUG>)
