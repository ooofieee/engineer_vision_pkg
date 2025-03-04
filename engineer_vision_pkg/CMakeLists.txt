cmake_minimum_required(VERSION 3.10)
project(engineer_vision_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/resource)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(video_capturer_node src/video_capturer_node.cpp)
add_executable(redeem_box_node src/redeem_box_node.cpp)
ament_target_dependencies(video_capturer_node rclcpp sensor_msgs cv_bridge std_msgs)
ament_target_dependencies(redeem_box_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(video_capturer_node ${OpenCV_LIBRARIES})
target_link_libraries(redeem_box_node ${OpenCV_LIBRARIES})


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

install(
  TARGETS video_capturer_node redeem_box_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
