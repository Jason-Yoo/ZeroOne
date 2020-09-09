cmake_minimum_required(VERSION 3.5)

project(ZeroOne LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#设置库文件文件输出目录
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#递归获取目录下所有的C文件
file(GLOB_RECURSE c_files ${PROJECT_SOURCE_DIR}/src/*.cpp)

#生成动态库
add_library(ZeroOneVision  SHARED  ${c_files})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${PROJECT_NAME}
      inc
#    /usr/include/Eigen3
     /usr/include/eigen3/Eigen
    )

add_executable(${PROJECT_NAME}
    src/DHCamera.cpp
    src/BoxProcess.cpp
    src/SRPnP.cpp
    src/math/arun.cpp
    src/math/roots.cpp
    src/SolveSRPnP.cpp
    src/ModulesDetect.cpp
    )



#设置opencv的位置
#set(OpenCV_DIR /home/nvidia/opencv/new_build)
#find_package(OpenCV 3.4.3  REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS}  )
find_package(OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS}  )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBRARIES}
    libpthread.so
    libgxiapi.so
    )

