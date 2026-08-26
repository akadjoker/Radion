add_executable(radion_scene_tests SceneTests.cpp)
target_precompile_headers(radion_scene_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_scene_tests PRIVATE radion_scene)
target_include_directories(radion_scene_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/runtime/render/src
)
target_compile_definitions(radion_scene_tests PRIVATE
    RADION_TEST_ASSET_DIR="${PROJECT_SOURCE_DIR}/assets"
)
add_test(NAME scene_logic COMMAND radion_scene_tests)
set_tests_properties(scene_logic PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_ai_tests AITests.cpp)
target_precompile_headers(radion_ai_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_ai_tests PRIVATE radion_ai)
add_test(NAME ai_logic COMMAND radion_ai_tests)
set_tests_properties(ai_logic PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_voxel_tests VoxelTests.cpp)
target_precompile_headers(radion_voxel_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_voxel_tests PRIVATE radion_voxel)
add_test(NAME voxel_logic COMMAND radion_voxel_tests)
set_tests_properties(voxel_logic PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_mesh_clipper_tests MeshClipperTests.cpp)
target_precompile_headers(radion_mesh_clipper_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_mesh_clipper_tests PRIVATE radion_render)
add_test(NAME mesh_clipper_logic COMMAND radion_mesh_clipper_tests)

add_executable(radion_mesh_optimize_tests MeshOptimizeTests.cpp)
target_precompile_headers(radion_mesh_optimize_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_mesh_optimize_tests PRIVATE radion_render)
add_test(NAME mesh_optimize_logic COMMAND radion_mesh_optimize_tests)
set_tests_properties(mesh_optimize_logic PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")
set_tests_properties(mesh_clipper_logic PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_volume_tests VolumeTests.cpp)
target_precompile_headers(radion_volume_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_volume_tests PRIVATE radion_geometry)
add_test(NAME volume_logic COMMAND radion_volume_tests)
set_tests_properties(volume_logic PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_geometry_hull_tests ConvexHullTests.cpp)
target_precompile_headers(radion_geometry_hull_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_geometry_hull_tests PRIVATE radion_geometry)
add_test(NAME geometry_hull_logic COMMAND radion_geometry_hull_tests)
set_tests_properties(geometry_hull_logic PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_mesh_import_tests MeshImportTests.cpp)
target_precompile_headers(radion_mesh_import_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_mesh_import_tests PRIVATE radion_render)
target_include_directories(radion_mesh_import_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/runtime/render/src
)
target_compile_definitions(radion_mesh_import_tests PRIVATE
    RADION_TEST_ASSET_DIR="${PROJECT_SOURCE_DIR}/assets"
)
add_test(NAME mesh_import COMMAND radion_mesh_import_tests)
set_tests_properties(mesh_import PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_dynamics_tests DynamicsTests.cpp)
target_precompile_headers(radion_dynamics_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_dynamics_tests PRIVATE radion_physics)
add_test(NAME dynamics COMMAND radion_dynamics_tests)
set_tests_properties(dynamics PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_joint_tests JointTests.cpp)
target_precompile_headers(radion_joint_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_joint_tests PRIVATE radion_physics)
add_test(NAME joints COMMAND radion_joint_tests)
set_tests_properties(joints PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_vehicle_tests VehicleTests.cpp)
target_precompile_headers(radion_vehicle_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_vehicle_tests PRIVATE radion_physics)
add_test(NAME vehicle COMMAND radion_vehicle_tests)
set_tests_properties(vehicle PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_soft_body_tests SoftBodyTests.cpp)
target_precompile_headers(radion_soft_body_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_soft_body_tests PRIVATE radion_physics)
add_test(NAME soft_body COMMAND radion_soft_body_tests)
set_tests_properties(soft_body PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_collision_tests CollisionTests.cpp)
target_precompile_headers(radion_collision_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_collision_tests PRIVATE radion_physics)
add_test(NAME collision COMMAND radion_collision_tests)
set_tests_properties(collision PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_bounds_tree_tests BoundsTreeTests.cpp)
target_precompile_headers(radion_bounds_tree_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_bounds_tree_tests PRIVATE radion_core)
add_test(NAME bounds_tree COMMAND radion_bounds_tree_tests)
set_tests_properties(bounds_tree PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_physics_bench PhysicsBench.cpp)
target_precompile_headers(radion_physics_bench PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_physics_bench PRIVATE radion_physics)

# Kept outside CTest: this reports timings and validates the generated scene,
# while CTest remains the deterministic correctness suite.
add_executable(radion_scene_bench SceneBench.cpp)
target_precompile_headers(radion_scene_bench PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_scene_bench PRIVATE radion_scene)

add_executable(radion_math_utils_tests MathUtilsTests.cpp)
target_precompile_headers(radion_math_utils_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_math_utils_tests PRIVATE radion_core)
add_test(NAME math_utils COMMAND radion_math_utils_tests)
set_tests_properties(math_utils PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_hull_mesh_tests HullMeshTests.cpp)
target_precompile_headers(radion_hull_mesh_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_hull_mesh_tests PRIVATE radion_geometry)
add_test(NAME hull_mesh COMMAND radion_hull_mesh_tests)
set_tests_properties(hull_mesh PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_volume_mesh_source_tests VolumeMeshSourceTests.cpp)
target_precompile_headers(radion_volume_mesh_source_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_volume_mesh_source_tests PRIVATE radion_geometry)
add_test(NAME volume_mesh_source COMMAND radion_volume_mesh_source_tests)
set_tests_properties(volume_mesh_source PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_mesh_uv_tests MeshUVTests.cpp)
target_precompile_headers(radion_mesh_uv_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_mesh_uv_tests PRIVATE radion_render)
add_test(NAME mesh_uv COMMAND radion_mesh_uv_tests)
set_tests_properties(mesh_uv PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_mesh_diagnostics_tests MeshDiagnosticsTests.cpp)
target_precompile_headers(radion_mesh_diagnostics_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_mesh_diagnostics_tests PRIVATE radion_render)
add_test(NAME mesh_diagnostics COMMAND radion_mesh_diagnostics_tests)
set_tests_properties(mesh_diagnostics PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_mesh_selection_tests MeshSelectionTests.cpp)
target_precompile_headers(radion_mesh_selection_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_mesh_selection_tests PRIVATE radion_render)
add_test(NAME mesh_selection COMMAND radion_mesh_selection_tests)
set_tests_properties(mesh_selection PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_mesh_extrude_tests MeshExtrudeTests.cpp)
target_precompile_headers(radion_mesh_extrude_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_mesh_extrude_tests PRIVATE radion_render)
add_test(NAME mesh_extrude COMMAND radion_mesh_extrude_tests)
set_tests_properties(mesh_extrude PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_mesh_transform_tests MeshTransformTests.cpp)
target_precompile_headers(radion_mesh_transform_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_mesh_transform_tests PRIVATE radion_render)
add_test(NAME mesh_transform COMMAND radion_mesh_transform_tests)
set_tests_properties(mesh_transform PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_blender_selection_tests
    BlenderSelectionTests.cpp
    ${PROJECT_SOURCE_DIR}/blend/src/BlenderSelection.cpp
)
target_precompile_headers(radion_blender_selection_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_blender_selection_tests PRIVATE radion_core)
target_include_directories(radion_blender_selection_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/blend/src
)
add_test(NAME blender_selection COMMAND radion_blender_selection_tests)
set_tests_properties(blender_selection PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_file_pack_tests FilePackTests.cpp)
target_precompile_headers(radion_file_pack_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_file_pack_tests PRIVATE radion_core)
add_test(NAME file_pack COMMAND radion_file_pack_tests)
set_tests_properties(file_pack PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_thread_tests ThreadTests.cpp)
target_precompile_headers(radion_thread_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_thread_tests PRIVATE radion_core)
add_test(NAME thread COMMAND radion_thread_tests)
set_tests_properties(thread PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_script_tests ScriptTests.cpp)
target_precompile_headers(radion_script_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_script_tests PRIVATE radion_script)
add_test(NAME script COMMAND radion_script_tests)
set_tests_properties(script PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_zen_behaviour_tests ZenBehaviourTests.cpp)
target_precompile_headers(radion_zen_behaviour_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_zen_behaviour_tests PRIVATE radion_scene)
target_compile_definitions(radion_zen_behaviour_tests PRIVATE
    RADION_TEST_ASSET_DIR="${PROJECT_SOURCE_DIR}/assets"
)
add_test(NAME zen_behaviour COMMAND radion_zen_behaviour_tests)
set_tests_properties(zen_behaviour PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")

add_executable(radion_scene_collision_tests SceneCollisionTests.cpp)
target_precompile_headers(radion_scene_collision_tests PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:${PROJECT_SOURCE_DIR}/runtime/core/include/PCH.h>"
)
target_link_libraries(radion_scene_collision_tests PRIVATE radion_scene)
add_test(NAME scene_collision COMMAND radion_scene_collision_tests)
set_tests_properties(scene_collision PROPERTIES ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0")
