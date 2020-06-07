cmake_minimum_required(VERSION 3.5)

project(camera_driver LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${PROJECT_NAME}
    inc
    usr/include/Eigen3
    )

add_executable(${PROJECT_NAME}
    src/DHCamera.cpp
    src/BoxProcess.cpp
    src/TemplateMatch.cpp
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

