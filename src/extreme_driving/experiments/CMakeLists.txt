cmake_minimum_required(VERSION 3.5)
project(experiments)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -c -MMD -pipe -std=gnu++17 -g -flto -fPIC -fno-math-errno -O3 -march=native)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ackermann_mux REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(slam_toolbox REQUIRED)
find_package(robot_localization REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
include_directories(/usr/local/include/eigen-3.4.0/)

# add_executable(experiment src/manual_test.cpp)
# ament_target_dependencies(experiment rclcpp std_msgs sensor_msgs ackermann_msgs nav_msgs)

add_executable(joystick_teleop src/joystick_teleop.cpp)
ament_target_dependencies(joystick_teleop rclcpp std_msgs sensor_msgs ackermann_msgs nav_msgs)

add_executable(tf_publish src/tf_publisher.cpp)
ament_target_dependencies(tf_publish rclcpp nav_msgs tf2_ros geometry_msgs)

# install(TARGETS
#   experiment
#   tf_publish
#   DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  joystick_teleop
  tf_publish
  DESTINATION lib/${PROJECT_NAME})
  
 
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  description
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
