cmake_minimum_required(VERSION 3.0.0)
project(BASESTADECT VERSION 0.1.0)

# 规定.h头文件路径
include_directories(src)

# 生成静态链接库
# add_library(method STATIC
#     src/method.cpp
# )

# 生成动态链接库
# add_library(basedock SHARED
#     src/linextra.cpp
#     src/method.cpp
# )


# 将.cpp/.c/.cc生成可执行文件 第一个参数指定可执行文件名称
# add_executable(BASESTADECT src/linextra.cpp)

# 对add_library和add_executable生成的文件进行链接
# 库文件名称通常为lib***.so 这里只需写***即可
# target_link_libraries(BASESTADECT)

add_executable(WRITEFILE src/writefile.cpp)
target_link_libraries(WRITEFILE)

# find_package(PCL 1.12.0 REQUIRED)
# include_directories(
#     include 
#     &{catkin_INCLUDE_DIRS} 
#     /usr/local/include/pcl-1.12
#     /usr/local/include/eigen3
#     /usr/local/include/vtk-9.2
# )
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

# add_executable(RABBIT src/rabbit.cpp)
# target_link_libraries(RABBIT ${PCL_LIBRARIES})

# 设置编译为debug版本并显示警告信息
# 相当于 g++ -g -Wall -c main.c -o test
add_definitions("-Wall -g")
