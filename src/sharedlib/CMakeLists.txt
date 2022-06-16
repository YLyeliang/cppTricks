project(sharedlib)
# C++11 编译
set(CMAKE_CXX_STANDARD 11)

# print message
message("cmake source dir:" ${CMAKE_SOURCE_DIR})
message("cmake current dir:" ${CMAKE_CURRENT_LIST_DIR})
message("cmake binary dir:" ${CMAKE_BINARY_DIR})
message("project binary dir" ${PROJECT_BINARY_DIR})

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
#创建共享库
#include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

add_library(sharedlib SHARED ${SOURCE_FILES} src/library.cpp include/library.h)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

#install(FILES ${CMAKE_CURRENT_LIST_DIR}/include/library.h
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/library)