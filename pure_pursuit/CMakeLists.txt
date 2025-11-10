cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# ★ ADDED: markers/text topics
find_package(std_msgs REQUIRED)                 # for /pp_debug/* Float32/Bool
find_package(visualization_msgs REQUIRED)       # for MarkerArray

# pure_pursuit 노드
add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit
  rclcpp
  control_msgs
  ackermann_msgs
  nav_msgs
  tf2
  sensor_msgs
  geometry_msgs
  # ★ ADDED:
  std_msgs
  visualization_msgs
)
install(TARGETS pure_pursuit DESTINATION lib/${PROJECT_NAME})

# dwa_local_planner 노드 (사용하지 않아도 유지)
add_executable(dwa_local_planner src/dwa_local_planner.cpp)
ament_target_dependencies(dwa_local_planner
  rclcpp
  sensor_msgs
  nav_msgs
  ackermann_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
install(TARGETS dwa_local_planner DESTINATION lib/${PROJECT_NAME})

# scan_crop_node (A→B 전방 슬라이스 퍼블리셔)
add_executable(scan_crop_node src/scan_crop_node.cpp)
ament_target_dependencies(scan_crop_node
  rclcpp
  sensor_msgs
)
install(TARGETS scan_crop_node DESTINATION lib/${PROJECT_NAME})

# ★ 런치/파라미터 파일 설치 (launch/params 폴더가 있을 때)
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

ament_package()

