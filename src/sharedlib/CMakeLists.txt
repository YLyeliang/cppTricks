cmake_minimum_required(VERSION 3.21)
project(sharedlib)
# C++11 编译
set(CMAKE_CXX_STANDARD 11)

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
#创建共享库
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

add_library(sharedlib SHARED ${SOURCE_FILES} src/library.cpp include/library.h)

#install(FILES ${CMAKE_CURRENT_LIST_DIR}/include/library.h
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/library)