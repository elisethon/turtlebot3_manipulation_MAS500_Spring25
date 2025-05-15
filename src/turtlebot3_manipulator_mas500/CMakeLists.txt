cmake_minimum_required(VERSION 3.8)
project(turtlebot3_manipulator_mas500)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# find dependencies
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)


#[===[
add_executable(Qtip_Test2 src/Qtip_Test2.cpp)
target_include_directories(Qtip_Test2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Qtip_Test2 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(Qtip_Test2 "moveit_ros_planning_interface" "rclcpp")

install(TARGETS Qtip_Test2
  DESTINATION lib/${PROJECT_NAME})



add_executable(sensor_Test src/sensor_Test.cpp)
target_include_directories(sensor_Test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sensor_Test PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(sensor_Test "moveit_ros_planning_interface" "rclcpp")

install(TARGETS sensor_Test
  DESTINATION lib/${PROJECT_NAME})
  
  
add_executable(Qtip_Test_P3 src/Qtip_Test_P3.cpp)
target_include_directories(Qtip_Test_P3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Qtip_Test_P3 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(Qtip_Test_P3 "moveit_ros_planning_interface" "rclcpp")

install(TARGETS Qtip_Test_P3
  DESTINATION lib/${PROJECT_NAME})



add_executable(sensor_Test_v2 src/sensor_Test_v2.cpp)
target_include_directories(sensor_Test_v2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sensor_Test_v2 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(sensor_Test_v2 "moveit_ros_planning_interface" "rclcpp")

install(TARGETS sensor_Test_v2
  DESTINATION lib/${PROJECT_NAME})

  
add_executable(Qtip_Test_P3_v2 src/Qtip_Test_P3_v2.cpp)
target_include_directories(Qtip_Test_P3_v2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Qtip_Test_P3_v2 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(Qtip_Test_P3_v2 "moveit_ros_planning_interface" "rclcpp")

install(TARGETS Qtip_Test_P3_v2
  DESTINATION lib/${PROJECT_NAME})


add_executable(TestSimTildeBill src/TestSimTildeBill.cpp)
target_include_directories(TestSimTildeBill PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(TestSimTildeBill PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(TestSimTildeBill "moveit_ros_planning_interface" "rclcpp")

install(TARGETS TestSimTildeBill
  DESTINATION lib/${PROJECT_NAME})
  
  
  
  
add_executable(Qtip_Test_P3_v3 src/Qtip_Test_P3_v3.cpp)
target_include_directories(Qtip_Test_P3_v3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Qtip_Test_P3_v3 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(Qtip_Test_P3_v3 "moveit_ros_planning_interface" "rclcpp")

install(TARGETS Qtip_Test_P3_v3
  DESTINATION lib/${PROJECT_NAME})
  
  
  
add_executable(Qtip_Test_P3_v4 src/Qtip_Test_P3_v34.cpp)
target_include_directories(Qtip_Test_P3_v4 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Qtip_Test_P3_v4 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(Qtip_Test_P3_v4 "moveit_ros_planning_interface" "rclcpp")

install(TARGETS Qtip_Test_P3_v4
  DESTINATION lib/${PROJECT_NAME})
  
  add_executable(sensor_Test_v3 src/sensor_Test_v3.cpp)
target_include_directories(sensor_Test_v3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sensor_Test_v3 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(sensor_Test_v3 "moveit_ros_planning_interface" "rclcpp")

install(TARGETS sensor_Test_v3
  DESTINATION lib/${PROJECT_NAME})
  
  
  add_executable(sensor_Test_v3 src/sensor_Test_v3.cpp)
target_include_directories(sensor_Test_v3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sensor_Test_v3 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(sensor_Test_v3 "moveit_ros_planning_interface" "rclcpp")

install(TARGETS sensor_Test_v3
  DESTINATION lib/${PROJECT_NAME})
  
  
 
 
add_executable(Qtip_Test_P3_v5 src/Qtip_Test_P3_v5.cpp)
target_include_directories(Qtip_Test_P3_v5 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Qtip_Test_P3_v5 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(Qtip_Test_P3_v5 "moveit_ros_planning_interface" "rclcpp")

install(TARGETS Qtip_Test_P3_v5
  DESTINATION lib/${PROJECT_NAME})
  
]===]


add_executable(Sensor_detect_liquid src/Sensor_detect_liquid.cpp)
target_include_directories(Sensor_detect_liquid PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Sensor_detect_liquid PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(Sensor_detect_liquid "moveit_ros_planning_interface" "rclcpp")

install(TARGETS Sensor_detect_liquid
  DESTINATION lib/${PROJECT_NAME})


  
  
add_executable(ReadJointState src/ReadJointState.cpp)
target_include_directories(ReadJointState PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(ReadJointState PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(ReadJointState "moveit_ros_planning_interface" "rclcpp")

install(TARGETS ReadJointState
  DESTINATION lib/${PROJECT_NAME})
  


add_executable(Sensor_liquid_clasification src/Sensor_liquid_clasification.cpp)
target_include_directories(Sensor_liquid_clasification PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Sensor_liquid_clasification PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(Sensor_liquid_clasification "moveit_ros_planning_interface" "rclcpp")

install(TARGETS Sensor_liquid_clasification
  DESTINATION lib/${PROJECT_NAME})
  
  
  
add_executable(Qtip_liquid_sample src/Qtip_liquid_sample.cpp)
target_include_directories(Qtip_liquid_sample PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Qtip_liquid_sample PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(Qtip_liquid_sample "moveit_ros_planning_interface" "rclcpp")

install(TARGETS Qtip_liquid_sample
  DESTINATION lib/${PROJECT_NAME})
  
  
add_executable(Qtip_liquid_sample_coo_v3 src/Qtip_liquid_sample_coo_v3.cpp)
target_include_directories(Qtip_liquid_sample_coo_v3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(Qtip_liquid_sample_coo_v3 PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(Qtip_liquid_sample_coo_v3 "moveit_ros_planning_interface" "rclcpp")

install(TARGETS Qtip_liquid_sample_coo_v3
  DESTINATION lib/${PROJECT_NAME})


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
