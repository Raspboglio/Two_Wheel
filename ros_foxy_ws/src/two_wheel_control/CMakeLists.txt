cmake_minimum_required(VERSION 3.5)
project(two_wheel_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

add_compile_options(-g)

# add_definitions(-DHPIPM)

set(CMAKE_CXX_FLAGS -pthread)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros2_control REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED) 
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(two_wheel_control_msgs REQUIRED)
find_package(ct_core REQUIRED)
find_package(ct_optcon)
find_package(tf2_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(nav_msgs REQUIRED)

# For balsfeo and HPIPM since they are in /opt
# list(APPEND CMAKE_PREFIX_PATH "/opt")
find_package(blasfeo REQUIRED)
find_package(hpipm REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# message(WARNING ${hpipm_INCLUDE_DIRS})

add_library(diff_drive_mpc include/two_wheel_control/diff_drive_MPC.hpp)
set_target_properties(diff_drive_mpc PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(diff_drive_mpc blasfeo hpipm ct_core ct_optcon)
ament_target_dependencies(diff_drive_mpc
  ct_core
  ct_optcon
  blasfeo
  hpipm
)

add_executable(two_wheel_sliding_balance
  src/two_wheel_sliding_balance.cpp
)
target_include_directories(two_wheel_sliding_balance
  PRIVATE
  include
)
ament_target_dependencies(two_wheel_sliding_balance
  rclcpp
  two_wheel_control_msgs
)

add_executable(diff_drive_mpc_node
  src/diff_drive_MPC_node.cpp
)
target_include_directories(diff_drive_mpc_node
  PRIVATE
  include
)
ament_target_dependencies(diff_drive_mpc_node
  rclcpp
  two_wheel_control_msgs
)

add_executable(two_wheel_control_testing test/test.cpp)
target_link_libraries(two_wheel_control_testing
  diff_drive_mpc
)
add_dependencies(two_wheel_control_testing 
  diff_drive_mpc
)
target_include_directories(two_wheel_control_testing
  PRIVATE
  include
)


add_library(two_wheel_control
  SHARED
  src/two_wheel_ID_controller.cpp
  src/two_wheel_MPC_controller.cpp
  src/two_wheel_EKF.cpp
  src/two_wheel_input_forward.cpp
)
target_include_directories(two_wheel_control
  PRIVATE
  include
)
ament_target_dependencies(two_wheel_control
  controller_interface
  hardware_interface
  control_toolbox
  pluginlib
  rclcpp
  rclcpp_lifecycle
  two_wheel_control_msgs
  ct_core
  ct_optcon
  nav_msgs
  tf2_msgs
  blasfeo
  hpipm
)
target_compile_definitions(two_wheel_control PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface two_wheel_ID_controller.xml)
pluginlib_export_plugin_description_file(controller_interface two_wheel_MPC_controller.xml)
pluginlib_export_plugin_description_file(controller_interface two_wheel_EKF.xml)
pluginlib_export_plugin_description_file(controller_interface two_wheel_input_forward.xml)

install(DIRECTORY 
  include/  
  DESTINATION include
)
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS
  two_wheel_sliding_balance
  diff_drive_mpc_node
  two_wheel_control_testing
  diff_drive_mpc
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS
  two_wheel_control
  two_wheel_sliding_balance
  diff_drive_mpc_node
  diff_drive_mpc
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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
ament_export_include_directories(
  include
)
ament_export_libraries(
  two_wheel_control
  diff_drive_mpc
)

ament_package()
