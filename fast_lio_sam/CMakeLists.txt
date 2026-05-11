cmake_minimum_required(VERSION 3.10)
project(fast_lio_sam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra")

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)

if(PCL_FOUND)
  message(WARNING "PCL_VER: ${PCL_VERSION}, and PCL_DIR: ${PCL_INCLUDE_DIRS}")
endif()
if(Eigen3_FOUND)
  message(WARNING "EIGEN_VER: ${EIGEN3_VERSION_STRING}, and Eigen DIR: ${EIGEN3_INCLUDE_DIR}")
endif()
if(GTSAM_FOUND)
  message(WARNING "GTSAM_VER: ${GTSAM_VERSION}, and GTSAM DIR: ${GTSAM_INCLUDE_DIR}")
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

set(ROS_DEPENDENCIES
  ament_index_cpp
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_eigen
  tf2_ros
  pcl_conversions
  message_filters
)

add_library(${PROJECT_NAME}_modules src/loop_closure.cpp src/fast_lio_sam.cpp)
ament_target_dependencies(${PROJECT_NAME}_modules ${ROS_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_modules ${PCL_LIBRARIES} gtsam)

add_executable(${PROJECT_NAME}_node src/main.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${ROS_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_modules ${PCL_LIBRARIES} gtsam)

install(TARGETS ${PROJECT_NAME}_modules ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
