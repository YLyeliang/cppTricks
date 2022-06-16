project(postprocess)
# C++11 编译
set(CMAKE_CXX_STANDARD 11)

# print message
message("Project: postprocess")
message("cmake source dir:" ${CMAKE_SOURCE_DIR})
message("cmake current dir:" ${CMAKE_CURRENT_LIST_DIR})
message("cmake binary dir:" ${CMAKE_BINARY_DIR})
message("project binary dir" ${PROJECT_BINARY_DIR})

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
#创建共享库
add_library(postprocess SHARED ${SOURCE_FILES} src/leastsquare.cpp include/leastsquare.h)
# 包含头文件
target_include_directories(${PROJECT_NAME} PUBLIC include)
# 链接三方库
target_link_libraries(${PROJECT_NAME} PUBLIC eigen)


# install settings
install(TARGETS postprocess DESTINATION bin)
install(FILES include/leastsquare.h DESTINATION include/postprocess)