message(STATUS "=== CMAKE FILE RUNNING ===")
message(STATUS "File: ${CMAKE_CURRENT_LIST_FILE}")
message(STATUS "Source Dir: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "Imgui Dir: ${CMAKE_CURRENT_SOURCE_DIR}/imgui")

cmake_minimum_required(VERSION 3.15)
project(nprn_biped)

set(CMAKE_CXX_STANDARD 17)

add_executable(nprn_biped main.cpp)

file(GLOB IMGUI_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/backends/imgui_impl_glfw.cpp ${CMAKE_CURRENT_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)

message(STATUS "Imgui Sources: ${IMGUI_SOURCES}")

add_compile_definitions(IMGUI_ENABLE_DOCKING IMGUI_ENABLE_VIEWPORTS)

add_library(imgui STATIC ${IMGUI_SOURCES})

target_include_directories(imgui PUBLIC imgui backends)

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

target_include_directories(nprn_biped PRIVATE . imgui backends)

target_link_libraries(nprn_biped PRIVATE imgui glfw OpenGL::GL)
