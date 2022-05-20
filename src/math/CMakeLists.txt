project(mathfunc)

aux_source_directory(./src DIR_LIB_SRCS)
file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
# 生成动态库
add_library(MathFunctions SHARED ${SOURCE_FILES} include/MathFunctions.h)

# 指定MathFunctions库的安装路径
install(TARGETS MathFunctions DESTINATION bin)
install(FILES include/MathFunctions.h DESTINATION include)