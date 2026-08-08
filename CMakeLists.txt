cmake_minimum_required(VERSION 3.8)
project(jie_ware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

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

include_directories(include ${OpenCV_INCLUDE_DIRS})
# ---------- lidar_loc ----------
add_executable(lidar_loc src/lidar_loc.cpp)
ament_target_dependencies(lidar_loc
  rclcpp sensor_msgs nav_msgs geometry_msgs std_srvs
  tf2 tf2_ros tf2_geometry_msgs cv_bridge)
target_link_libraries(lidar_loc ${OpenCV_LIBS})

# ---------- costmap_cleaner ----------
add_executable(costmap_cleaner src/costmap_cleaner.cpp)
ament_target_dependencies(costmap_cleaner
  rclcpp geometry_msgs std_srvs)

# ---------- lidar_filter_node ----------
add_executable(lidar_filter_node src/lidar_filter_node.cpp)
ament_target_dependencies(lidar_filter_node
  rclcpp sensor_msgs)

install(TARGETS
  lidar_loc
  costmap_cleaner
  lidar_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
