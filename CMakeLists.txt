cmake_minimum_required(VERSION 3.21)
project(cppTricks)

set(CMAKE_CXX_STANDARD 17)

# settings
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# sub module
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/thirdparty/argparse)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/sharedlib)

include_directories(${CMAKE_CURRENT_LIST_DIR}/src/sharedlib/include)

add_executable(cppTricks main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
        argparse
        sharedlib
        )

#add_library(tricks SHARED main.cpp)
#set_target_properties(tricks PROPERTIES outputname "tricks")

