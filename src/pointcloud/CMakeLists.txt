project(pointcloud)


file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
# 生成动态库
add_library(pointcloud SHARED ${SOURCE_FILES} include/demo.h)

# pcl
find_package(PCL 1.10 REQUIRED)
target_include_directories(${PROJECT_NAME} PUBLIC include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
message(pcl libraries: ${PCL_LIBRARIES})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# 指定MathFunctions库的安装路径
install(TARGETS pointcloud DESTINATION bin)
install(FILES include/demo.h DESTINATION include)