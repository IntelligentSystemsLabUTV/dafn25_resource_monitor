cmake_minimum_required(VERSION 3.8)
project(resource_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Pub component
add_library(pub SHARED src/pub.cpp)
target_compile_definitions(pub PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pub
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs)
rclcpp_components_register_nodes(pub "resource_monitor::ResourceMonitorPublisher")

add_executable(pub_app src/pub_main.cpp)
target_include_directories(pub_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(pub_app PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(pub_app pub)
ament_target_dependencies(
  pub_app
  rclcpp)

# Sub component
add_library(sub SHARED src/sub.cpp)
target_compile_definitions(sub PRIVATE COMPOSITION_BUILDING_DLL)
target_include_directories(sub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  sub
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs)
rclcpp_components_register_nodes(sub "resource_monitor::ResourceMonitorSubscriber")

add_executable(sub_app src/sub_main.cpp)
target_include_directories(sub_app PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(sub_app PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(sub_app sub)
ament_target_dependencies(
  sub_app
  rclcpp)

install(TARGETS pub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS sub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS pub_app
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS sub_app
  DESTINATION lib/${PROJECT_NAME})

# Launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

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
