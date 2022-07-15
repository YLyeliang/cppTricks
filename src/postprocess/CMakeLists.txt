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
add_library(postprocess SHARED ${SOURCE_FILES} src/leastsquare.cpp include/leastsquare.h src/matrix_demo.cpp include/matrix_demo.h include/ransac.h src/ransac.cpp)
# 包含头文件
target_include_directories(${PROJECT_NAME} PUBLIC include ${OpenMP_CXX_INCLUDE_DIRS})

find_package(OpenMP REQUIRED)

# 链接三方库
target_link_libraries(${PROJECT_NAME} PUBLIC eigen yaml-cpp ${OpenMP_CXX_LIBRARIES} )


# install settings
install(TARGETS postprocess DESTINATION bin)
install(FILES include/leastsquare.h include/matrix_demo.h DESTINATION include/postprocess)