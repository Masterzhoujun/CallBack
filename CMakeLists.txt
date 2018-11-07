cmake_minimum_required(VERSION 2.8.12)
project(vio_ekf)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

#modify cmake module path if new. cmake files are cmake_minimum_required
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    image_transport
    cv_bridge
    message_filters
)

# system dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

catkin_package()

add_executable(vio_ekf
    app/vio_ekf_node.cpp
    src/common/parameter.cpp 
    src/common/utils.cpp
    src/frontend/image_processor.cpp 
)

target_link_libraries(vio_ekf ${catkin_LIBRARIES} ${OpenCV_LIBS})
