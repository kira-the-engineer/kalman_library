cmake_minimum_required(VERSION 3.13)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_C_COMPILER "/usr/bin/clang-17")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++-17")

#project info
project(UKF)

#add libraries
add_library(eigen INTERFACE src)
add_library(ukflib ukf.cpp)

#create executable
add_executable(tukf test-ukf.cpp)

target_link_libraries(tukf
    PUBLICc
        eigen
        ukflib
)
