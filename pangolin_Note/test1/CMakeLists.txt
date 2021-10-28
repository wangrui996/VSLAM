cmake_minimum_required(VERSION 2.8)

project(Pandolin_Test)

SET(CMAKE_BUILD_TYPE Release)


find_package(Pangolin REQUIRED)


include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${Pangolin_INCLUDE_DIRS}
)


add_library(Viewer SHARED
    src/Viewer.cc
)


add_executable(pangolin_Test src/main.cpp)

target_link_libraries(pangolin_Test Viewer ${Pangolin_LIBRARIES})