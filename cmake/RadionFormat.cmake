find_program(RADION_CLANG_FORMAT_EXECUTABLE NAMES clang-format)

if(RADION_CLANG_FORMAT_EXECUTABLE)
    file(GLOB_RECURSE RADION_FORMAT_FILES CONFIGURE_DEPENDS
        "${PROJECT_SOURCE_DIR}/runtime/*.h"
        "${PROJECT_SOURCE_DIR}/runtime/*.hpp"
        "${PROJECT_SOURCE_DIR}/runtime/*.cpp"
        "${PROJECT_SOURCE_DIR}/examples/*.h"
        "${PROJECT_SOURCE_DIR}/examples/*.hpp"
        "${PROJECT_SOURCE_DIR}/examples/*.cpp"
        "${PROJECT_SOURCE_DIR}/tests/*.h"
        "${PROJECT_SOURCE_DIR}/tests/*.hpp"
        "${PROJECT_SOURCE_DIR}/tests/*.cpp"
    )
    list(FILTER RADION_FORMAT_FILES EXCLUDE REGEX "/stb_[^/]*\\.h$")
    list(FILTER RADION_FORMAT_FILES EXCLUDE REGEX "/font_data\\.h$")

    add_custom_target(format
        COMMAND ${RADION_CLANG_FORMAT_EXECUTABLE} -i ${RADION_FORMAT_FILES}
        COMMENT "Formatting Radion sources"
        VERBATIM
    )

    add_custom_target(format-check
        COMMAND ${RADION_CLANG_FORMAT_EXECUTABLE} --dry-run --Werror ${RADION_FORMAT_FILES}
        COMMENT "Checking Radion source formatting"
        VERBATIM
    )
else()
    message(STATUS "clang-format not found; format targets are unavailable")
endif()
