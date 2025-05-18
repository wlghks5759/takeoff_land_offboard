cmake_minimum_required(VERSION 3.8)
project(takeoff_land_offboard)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(geometry_msgs REQUIRED)

find_package(MAVSDK REQUIRED)

add_executable(takeoff_land_offboard
    src/takeoff_land_offboard.cpp
)

ament_target_dependencies(takeoff_land_offboard
    rclcpp
    std_msgs
    std_srvs
    geometry_msgs
)

# Link MAVSDK
target_link_libraries(takeoff_land_offboard
    MAVSDK::mavsdk
)

install(TARGETS
    takeoff_land_offboard
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
    DIRECTORY launch
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
