cmake_minimum_required(VERSION 3.5)
project(boustrophedon_server)

set(CMAKE_CXX_STANDARD 14)

# For CGAL performance reason
set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(boustrophedon_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}/boustrophedon_planner_node.cpp
  src/${PROJECT_NAME}/boustrophedon_planner_server.cpp
  src/${PROJECT_NAME}/striping_planner.cpp
  src/${PROJECT_NAME}/outline_planner.cpp
  src/${PROJECT_NAME}/cgal_utils.cpp
  src/${PROJECT_NAME}/boustrophedon_types.cpp
  src/${PROJECT_NAME}/cellular_decomposition/polygon_decomposer.cpp
  src/${PROJECT_NAME}/cellular_decomposition/cell.cpp
)

target_link_libraries(${PROJECT_NAME}
  CGAL::CGAL
  CGAL::CGAL_Core
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  CGAL
  nav_msgs
  geometry_msgs
  boustrophedon_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(
  TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()