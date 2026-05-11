cmake_minimum_required(VERSION 3.10)
project(fast_lio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-O3)
add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processor number: ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN -DMP_PROC_NUM=3)
  elseif(N GREATER 3)
    add_definitions(-DMP_EN -DMP_PROC_NUM=2)
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenMP QUIET)
find_package(PythonLibs REQUIRED)

if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
)

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
ament_target_dependencies(fastlio_mapping
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  pcl_conversions
  livox_ros_driver2
)
target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${cpp_typesupport_target})
add_dependencies(fastlio_mapping ${PROJECT_NAME}__rosidl_generator_cpp)
target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

install(TARGETS fastlio_mapping
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

