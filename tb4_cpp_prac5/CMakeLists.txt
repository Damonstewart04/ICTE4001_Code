cmake_minimum_required(VERSION 3.8)
project(tb4_cpp_prac5)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_amcl REQUIRED)


include_directories(
  include
)

set(executable_name iar_amcl)
add_executable(${executable_name} src/main.cpp src/amcl_node.cpp src/likelihood_field_model.cpp)


include(CheckSymbolExists)
check_symbol_exists(drand48 stdlib.h HAVE_DRAND48)

if(HAVE_DRAND48)
  target_compile_definitions(${executable_name} PRIVATE "HAVE_DRAND48")
endif()

target_include_directories(${executable_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${executable_name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

set(dependencies
  "nav2_common"
  "rclcpp"
  "rclcpp_lifecycle"
  "message_filters"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_srvs"
  "tf2_ros"
  "tf2"
  "nav2_util"
  "nav2_msgs"
  "nav2_amcl"
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  map
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
