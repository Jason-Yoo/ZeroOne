# * @Copyright (c) 2020 Zero_One NUAA
# *
# * @author         : Yang Jiduocai
# * @createDate     : 2020/09/04
# * @lastEditors    :
# * @lastEditTime   : 2020/09/16
# * @brief          : 
cmake_minimum_required(VERSION 3.5)
project(ZeroOne)

set(USER_INC  "${CMAKE_CURRENT_SOURCE_DIR}/inc")
set(USER_SRC  "${CMAKE_CURRENT_SOURCE_DIR}/src")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")

#设置库文件文件输出目录
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##递归获取目录下所有的C文件
file(GLOB_RECURSE c_files ${PROJECT_SOURCE_DIR}/src/*.cpp)

##生成动态库
add_library(ZeroOneVision  SHARED  ${c_files})
INSTALL (TARGETS ZeroOneVision ZeroOneVision LIBRARY DESTINATION lib)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${PROJECT_NAME} /usr/include/eigen3/Eigen)
include_directories(${USER_INC})


#设置opencv的位置
find_package(OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS}  )

aux_source_directory(${USER_SRC} USER_SOURCES)
add_executable(${PROJECT_NAME}  ${USER_SOURCES})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    libpthread.so
    libgxiapi.so
    )

