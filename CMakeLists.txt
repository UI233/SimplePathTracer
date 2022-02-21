cmake_minimum_required(VERSION 3.16)

project(subdivision
    LANGUAGES CXX
)

include(./cmake/CPM.cmake)

CPMAddPackage("gh:libigl/libigl@2.3.0")
CPMAddPackage("gh:kongaskristjan/fire-hpp@0.2")
CPMAddPackage("gh:tinyobjloader/tinyobjloader@2.0.0rc9")


# project
# -- excutable --
add_executable(pt src/main.cpp)
target_include_directories(pt PUBLIC include)
target_link_libraries(pt igl::opengl igl::opengl_glfw igl::opengl_glfw_imgui igl fire-hpp tinyobjloader)