cmake_minimum_required(VERSION 3.0.2)
project(rospersp2equi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(VISP REQUIRED)

# Añadir la ruta de instalación de libper SIEMPREEEEEEEEEEEEEEEEEE
set(CMAKE_PREFIX_PATH "/root/libPeR_base/cmake" ${CMAKE_PREFIX_PATH})
set(CMAKE_MODULE_PATH "/root/libPeR_base/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

find_package(PER REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${PER_INCLUDE_DIRS}
)

add_executable(perps2 src/perps2.cpp)
target_link_libraries(perps2 ${catkin_LIBRARIES})

target_link_libraries(perps2
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${PER_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/monitor_rosbag.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
