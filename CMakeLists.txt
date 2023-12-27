cmake_minimum_required(VERSION 3.0)
project(mav)

set(CMAKE_BUILD_TYPE DEBUG)
include_directories(/home/ubuntu/wjd/c_library_v2-master/)
set(SRC src/main.cpp)

add_executable(a ${SRC})


