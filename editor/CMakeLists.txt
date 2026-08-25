add_executable(radion_editor
    ${CMAKE_SOURCE_DIR}/vendor/imgui/editor/TextEditor.cpp
    src/main.cpp
    src/EditorApplication.cpp
    src/EditorSelection.cpp
    src/EditorToasts.cpp
    src/LightmapUnwrapJob.cpp
    src/EditorSettings.cpp
    src/panels/HierarchyPanel.cpp
    src/panels/ViewportPanel.cpp
    src/panels/InspectorPanel.cpp
    src/panels/SettingsPanel.cpp
    src/panels/ConsolePanel.cpp
    src/panels/DebugPanel.cpp
    src/panels/ProfilerPanel.cpp
    src/panels/AnimationPanel.cpp
    src/panels/GamePanel.cpp
    src/panels/AssetsPanel.cpp
    src/panels/ScriptEditorPanel.cpp
    src/panels/LightmapPanel.cpp
    src/panels/MeshToolsPanel.cpp
    src/panels/VolumePanel.cpp
    ${CMAKE_SOURCE_DIR}/vendor/imgui/widgets/ImGuiFileDialog.cpp
    ${CMAKE_SOURCE_DIR}/vendor/imgui/widgets/ImGuizmo.cpp
    ${CMAKE_SOURCE_DIR}/vendor/imgui/imgui_stdlib.cpp
)
target_link_libraries(radion_editor PRIVATE radion_engine radion_scene radion_geometry)
target_compile_definitions(radion_editor PRIVATE
    RADION_ASSET_DIR="${CMAKE_SOURCE_DIR}/assets"

    IMGUI_DEFINE_MATH_OPERATORS
)
target_include_directories(radion_editor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/vendor/icons
    ${CMAKE_SOURCE_DIR}/vendor/imgui/editor
    ${CMAKE_SOURCE_DIR}/vendor/imgui/widgets
)

if(TARGET SDL2::SDL2main)
    target_link_libraries(radion_editor PRIVATE SDL2::SDL2main)
endif()

if(WIN32)
    add_custom_command(TARGET radion_editor POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:radion_editor>
            $<TARGET_FILE_DIR:radion_editor>
        COMMAND_EXPAND_LISTS
        COMMENT "Copying runtime DLLs"
    )
endif()
