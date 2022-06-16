cmake_minimum_required(VERSION 3.21)
project(cppTricks)

set(CMAKE_CXX_STANDARD 17)
# setting
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加编译选项
# 加入配置文件，用于处理CMake对源码的设置
# 可以将 MathFunctions 库设为一个可选的库，如果该选项为 ON ，就使用该库定义的数学函数来进行运算。否则就调用标准库中的数学函数库。
# 注意config.h，我们并不直接编写这个文件，而是通过config.h.in来导入
configure_file(
        "${PROJECT_SOURCE_DIR}/config.h.in"
        "${PROJECT_BINARY_DIR}/config.h"
)

# cmake时，使用-DUSE_MYMATH来控制
option(USE_MYMATH
        "Use my math function" ON)
# 是否加入 MATHFunctions库
if (USE_MYMATH)
    add_subdirectory(src/math)
    include_directories(src/math/include)
    set(EXTRA_LIBS ${EXTRA_LIBS} MathFunctions)
endif (USE_MYMATH)
message("extra libs: " ${EXTRA_LIBS})

# 使得 CMake 可以找到生成的 config.h
include_directories("${PROJECT_BINARY_DIR}")

# thirdparty
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/thirdparty/argparse)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/thirdparty/json)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/thirdparty/eigen)
#include_directories(${CMAKE_CURRENT_LIST_DIR}/src/sharedlib/include)

# sub-function
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/sharedlib)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/postprocess)
include_directories(${CMAKE_CURRENT_LIST_DIR}/src/postprocess/include)

add_executable(cppTricks main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC
        argparse
        sharedlib
        postprocess
        nlohmann_json
        eigen
        ${EXTRA_LIBS}
        ${OpenCV_LIBS}
        )

#add_library(tricks SHARED main.cpp)
#set_target_properties(tricks PROPERTIES outputname "tricks")

# 指定安装路径
install(TARGETS cppTricks DESTINATION ${PROJECT_BINARY_DIR}/install/bin)
install(FILES "${PROJECT_BINARY_DIR}/config.h" DESTINATION ${PROJECT_BINARY_DIR}/install/include)


