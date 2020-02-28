cmake_minimum_required(VERSION 3.10)
project(src)

set(CMAKE_CXX_STANDARD 11)

include_directories(.)

add_executable(src
        helper_functions.h
        json.hpp
        main.cpp
        map.h
        particle_filter.cpp
        particle_filter.h)
