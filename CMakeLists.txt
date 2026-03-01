cmake_minimum_required(VERSION 3.8)
project(safe_bayesian_optimization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard for CGAL library
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trusses_custom_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(Qhull REQUIRED)
find_package(turtlesim REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CURL REQUIRED libcurl)

# enable fetch content

include(FetchContent)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolygonArray.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_include_directories(
  include
)

include_directories(
  include
  ${CURL_INCLUDE_DIRS}
)


add_library(reactive_planner
    src/libraries/polygeom_lib.cpp
    src/libraries/reactive_planner_lib.cpp
    src/libraries/reactive_planner_lib_ext.cpp
)

# Add executables
add_executable(safe_bayesian_optimization_node src/safe_bayesian_optimization_node.cpp)
add_executable(goal_point_publisher src/goal_point_publisher.cpp)
add_executable(reactive_navigation_node src/reactive_navigation_node.cpp)

# Add dependencies
ament_target_dependencies(
    safe_bayesian_optimization_node rclcpp trusses_custom_interfaces Eigen3 geometry_msgs sensor_msgs cv_bridge Boost OpenCV CGAL visualization_msgs
)

target_link_libraries(reactive_planner
  ${Boost_LIBRARIES}
  CGAL::CGAL
  CGAL::CGAL_Core
)

target_link_libraries(
    safe_bayesian_optimization_node
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    Qhull::qhullcpp
    Qhull::qhull_r
    CGAL::CGAL
    CGAL::CGAL_Core
    reactive_planner
    ${CURL_LIBRARIES}
)

target_link_libraries(
    reactive_navigation_node
    ${Boost_LIBRARIES}
    reactive_planner
    Qt5::Core
    Qt5::Widgets
    CGAL::CGAL
    CGAL::CGAL_Core
)

rosidl_target_interfaces(safe_bayesian_optimization_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(reactive_navigation_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(
    goal_point_publisher rclcpp geometry_msgs visualization_msgs std_srvs
)

ament_target_dependencies(
    reactive_navigation_node rclcpp geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs Boost turtlesim visualization_msgs
)

ament_target_dependencies(
    reactive_planner rclcpp geometry_msgs CGAL Boost
)

# Install Python scripts
install(PROGRAMS
  src/turtlesim_spatial_publisher.py
  src/evaluation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  safe_bayesian_optimization_node
  goal_point_publisher
  reactive_navigation_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install library
install(TARGETS
  reactive_planner
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)


# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install data files
install(DIRECTORY
  data
  DESTINATION share/${PROJECT_NAME}/
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
