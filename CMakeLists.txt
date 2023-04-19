cmake_minimum_required(VERSION 3.8)
project(picobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



add_executable(picobot_keyboard src/picobot_keyboard.cpp)
add_executable(picobot_controller src/picobot_controller.cpp)
add_executable(picobot_listener src/picobot_listener.cpp)
add_executable(picobot_line_follower src/picobot_line_follower.cpp)

ament_target_dependencies(picobot_keyboard rclcpp std_msgs)
ament_target_dependencies(picobot_controller rclcpp std_msgs)
ament_target_dependencies(picobot_listener rclcpp std_msgs)
ament_target_dependencies(picobot_line_follower rclcpp std_msgs)

install(TARGETS
  picobot_keyboard
  picobot_controller
  picobot_listener
  picobot_line_follower
  DESTINATION lib/${PROJECT_NAME})

ament_package()
