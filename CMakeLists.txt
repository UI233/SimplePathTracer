cmake_minimum_required(VERSION 3.16)

project(subdivision
    LANGUAGES CXX
)
set(CMAKE_CXX_STANDARD 17)

include(./cmake/CPM.cmake)

CPMAddPackage("gh:libigl/libigl@2.3.0")
CPMAddPackage("gh:kongaskristjan/fire-hpp@0.2")
CPMAddPackage("gh:tinyobjloader/tinyobjloader@2.0.0rc9")
find_package(OpenMP REQUIRED)


# project
# -- excutable --
add_executable(pt src/camera.cpp src/light.cpp  src/scene.cpp src/shape.cpp src/helper.cpp src/main.cpp src/material.cpp src/integrator.cpp)
target_include_directories(pt PUBLIC include stb)
target_link_libraries(pt igl::core igl::xml fire-hpp tinyobjloader OpenMP::OpenMP_CXX)