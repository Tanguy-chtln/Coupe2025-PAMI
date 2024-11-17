cmake_minimum_required(VERSION 3.10)
project(pami)

option(ENABLE_DISPLAY_DEBUGGER "Compile display debugger" OFF)
option(ENABLE_DISPLAY "Enable display" ON)

set(CMAKE_C_STANDARD 99)
set(RESSOURCES_DIR ${CMAKE_SOURCE_DIR}/ressources)
set(DISPLAY_DIR ${CMAKE_SOURCE_DIR}/src/display)
set(PAMI_DIR ${CMAKE_SOURCE_DIR}/src/PAMI)

include(FindPkgConfig)

add_executable(${PROJECT_NAME} ${PAMI_DIR}/control.c ${PAMI_DIR}/pathing.c)
target_link_libraries(${PROJECT_NAME} PRIVATE m)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Ofast -march=native)
add_compile_definitions(RESSOURCES_DIR="${RESSOURCES_DIR}")

if (ENABLE_DISPLAY_DEBUGGER)
    PKG_SEARCH_MODULE(SDL2IMAGE REQUIRED SDL2_image>=2.0.0)
    PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)


    add_executable(visual_debugger ${DISPLAY_DIR}/visual.c)
    target_compile_definitions(visual_debugger PRIVATE ENABLE_DISPLAY_DEBUG)
    target_include_directories(visual_debugger PRIVATE ${DISPLAY_DIR} ${SDL2_INCLUDE_DIRS} ${SDL2IMAGE_INCLUDE_DIRS})
    target_link_libraries(visual_debugger PRIVATE m pthread ${SDL2_LIBRARIES} ${SDL2IMAGE_LIBRARIES})
    target_compile_options(visual_debugger PRIVATE -Wall -Wextra -Ofast -march=native)
endif()

if (ENABLE_DISPLAY)
    PKG_SEARCH_MODULE(SDL2IMAGE REQUIRED SDL2_image>=2.0.0)
    PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)

    add_library(displayer SHARED ${DISPLAY_DIR}/visual.c)
    target_include_directories(displayer PRIVATE ${SDL2_INCLUDE_DIRS} ${SDL2IMAGE_INCLUDE_DIRS} ${DISPLAY_DIR})
    target_link_libraries(displayer PRIVATE m pthread ${SDL2_LIBRARIES} ${SDL2IMAGE_LIBRARIES})
    target_compile_options(displayer PRIVATE -Wall -Wextra -Ofast -march=native)

    target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_DISPLAY)
    target_include_directories(${PROJECT_NAME} PRIVATE ${DISPLAY_DIR})
    target_link_libraries(${PROJECT_NAME} PRIVATE displayer)
endif()
