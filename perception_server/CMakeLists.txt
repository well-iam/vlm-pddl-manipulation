cmake_minimum_required(VERSION 3.8)
project(perception_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# find PCL (Point Cloud Library)
find_package(PCL REQUIRED COMPONENTS common features filters io kdtree search segmentation sample_consensus surface)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetSceneObjects.srv"
  DEPENDENCIES sensor_msgs moveit_msgs
)

# Configure PCL Includes
include_directories(include ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Compile library of algorithms (Cluster, Segm, etc.)
add_library(perception_algo
  src/cluster_extraction.cpp
  src/normals_curvature_and_rsd_estimation.cpp
  src/object_segmentation.cpp
  src/plane_segmentation.cpp
)
target_include_directories(perception_algo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies to library
ament_target_dependencies(perception_algo
  rclcpp sensor_msgs moveit_msgs shape_msgs geometry_msgs pcl_conversions tf2_eigen
)
target_link_libraries(perception_algo ${PCL_LIBRARIES})

# Compile Server Node
add_executable(get_scene_objects_server src/get_scene_objects_server.cpp)
target_include_directories(get_scene_objects_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link everything: Algorithm library + ROS Dependencies + Generated Service
target_link_libraries(get_scene_objects_server perception_algo ${PCL_LIBRARIES})
ament_target_dependencies(get_scene_objects_server
  rclcpp sensor_msgs moveit_msgs tf2_ros tf2_eigen pcl_conversions
)

# FONDAMENTALE: Assicura che il server trovi l'header del servizio generato
rosidl_target_interfaces(get_scene_objects_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Installation:
install(TARGETS perception_algo get_scene_objects_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY srv launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
