cmake_minimum_required(VERSION 3.0.2)
project(zed_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  sensor_msgs
  cv_bridge
)

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES zed_driver
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

add_executable(zed_driver_node
   ./src/zed_driver_node.cpp
   ./src/sl_tools.cpp)
target_include_directories(zed_driver_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(zed_driver_node 
                      ${catkin_LIBRARIES} 
                      ${ZED_LIBRARIES} 
                      ${Boost_LIBRARIES}
                      ${CUDA_CUDA_LIBRARY} 
                      ${CUDA_LIBRARY})
