add_executable(radion_pack main.cpp)
target_link_libraries(radion_pack PRIVATE radion_core)

if(TARGET SDL2::SDL2main)
    target_link_libraries(radion_pack PRIVATE SDL2::SDL2main)
endif()
