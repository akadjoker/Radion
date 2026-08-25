cmake_minimum_required(VERSION 3.16)
project(radion_blender)

add_subdirectory(minirender)

# ImAnim (animation widgets)
set(IMANIM_SOURCES
    vendor/ImAnim/im_anim.cpp
    vendor/ImAnim/im_anim.h
)

# Source files
set(BLENDER_SOURCES
    src/main.cpp
    src/BlenderApplication.cpp
    src/BlenderApplication.h
    src/BlenderPanel.h
    src/BlenderTheme.h
    src/BlenderSelection.cpp
    src/BlenderSelection.h
    src/BlenderSettings.cpp
    src/BlenderSettings.h
    src/MaterialEditor.cpp
    src/MaterialEditor.h
    src/ObjExporter.cpp
    src/ObjExporter.h

    # Panels
    src/panels/ViewportPanel.cpp
    src/panels/ViewportPanel.h
    src/panels/TimelinePanel.cpp
    src/panels/TimelinePanel.h
    src/panels/HierarchyPanel.cpp
    src/panels/HierarchyPanel.h
    src/panels/ConsolePanel.cpp
    src/panels/ConsolePanel.h
    src/panels/PropertiesPanel.cpp
    src/panels/PropertiesPanel.h
    src/panels/MeshHealthPanel.cpp
    src/panels/MeshHealthPanel.h
    src/panels/MaterialsPanel.cpp
    src/panels/MaterialsPanel.h

    # ImAnim
    ${IMANIM_SOURCES}

    ${CMAKE_SOURCE_DIR}/vendor/imgui/widgets/ImGuiFileDialog.cpp
    ${CMAKE_SOURCE_DIR}/vendor/imgui/widgets/ImGuizmo.cpp
    ${CMAKE_SOURCE_DIR}/vendor/imgui/imgui_stdlib.cpp
)

add_executable(radion_blender ${BLENDER_SOURCES})

# Link against Radion engine (pulls render/gpu/scene/core transitively,
# same as radion_editor does) and the standalone MiniRenderer lib
target_link_libraries(radion_blender PRIVATE radion_engine radion_minirender radion_geometry)

target_compile_definitions(radion_blender PRIVATE
    RADION_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets"
    IMGUI_DEFINE_MATH_OPERATORS
)

# Include directories
target_include_directories(radion_blender PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/ImAnim
    ${CMAKE_SOURCE_DIR}/vendor/imgui/widgets
)

# C++17 standard
set_target_properties(radion_blender PROPERTIES CXX_STANDARD 17)

# Set output directory
set_target_properties(radion_blender PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

if(TARGET SDL2::SDL2main)
    target_link_libraries(radion_blender PRIVATE SDL2::SDL2main)
endif()

if(WIN32)
    add_custom_command(TARGET radion_blender POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:radion_blender>
            $<TARGET_FILE_DIR:radion_blender>
        COMMAND_EXPAND_LISTS
        COMMENT "Copying runtime DLLs"
    )
endif()
