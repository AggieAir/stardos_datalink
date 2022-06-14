cmake_minimum_required(VERSION 3.5)
project(stardos_datalink)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(stardos_interfaces REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(jsoncpp REQUIRED)

add_executable(datalink_ground src/ground.cpp)
ament_target_dependencies(datalink_ground rclcpp stardos_interfaces MAVSDK)

add_executable(datalink_copilot src/copilot.cpp)
ament_target_dependencies(datalink_copilot rclcpp stardos_interfaces MAVSDK)

add_library(floattelem src/floattelem.cpp)
ament_target_dependencies(floattelem stardos_interfaces)

add_library(datalink src/datalink.cpp)
ament_target_dependencies(datalink rclcpp stardos_interfaces MAVSDK)

include_directories(include/stardos_datalink)

target_link_libraries(datalink_ground
	datalink
        floattelem
	jsoncpp_lib
	MAVSDK::mavsdk)

target_link_libraries(datalink_copilot
	datalink
        floattelem
	jsoncpp_lib
	MAVSDK::mavsdk)

install(TARGETS
	datalink_ground
	datalink_copilot
	DESTINATION lib/${PROJECT_NAME})

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
