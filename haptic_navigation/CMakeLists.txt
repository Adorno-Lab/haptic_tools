cmake_minimum_required(VERSION 3.22...3.26)
project(haptic_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_msgs REQUIRED)
find_package(haptic_tools_msgs REQUIRED)


##### CPP LIBRARY #####
# The navigation strategy and its safety layer. They do not use ROS.
add_library(${PROJECT_NAME} SHARED
    src/haptic_navigation_strategy.cpp
    src/haptic_navigation_safety.cpp
    src/haptic_pose_mapping.cpp
    src/dq_validation.cpp
    )
target_link_libraries(${PROJECT_NAME}
    PUBLIC
    Eigen3::Eigen
    dqrobotics
    )
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

install(
    DIRECTORY include/
    DESTINATION include
    )

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )
##END## CPP LIBRARY #####


##### CPP BINARY #####
add_executable(haptic_navigation_node
    src/haptic_navigation_node.cpp
    src/haptic_navigation.cpp
    )
target_link_libraries(haptic_navigation_node
    ${PROJECT_NAME}
    )
ament_target_dependencies(haptic_navigation_node
    rclcpp
    geometry_msgs
    sas_core
    sas_common
    sas_msgs
    haptic_tools_msgs
    )
install(TARGETS haptic_navigation_node
    DESTINATION lib/${PROJECT_NAME})
##END## CPP BINARY #####


install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}/
    )

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_haptic_navigation_strategy test/test_haptic_navigation_strategy.cpp)
  target_link_libraries(test_haptic_navigation_strategy ${PROJECT_NAME})
  ament_add_gtest(test_haptic_navigation_safety test/test_haptic_navigation_safety.cpp)
  target_link_libraries(test_haptic_navigation_safety ${PROJECT_NAME})
  ament_add_gtest(test_haptic_pose_mapping test/test_haptic_pose_mapping.cpp)
  target_link_libraries(test_haptic_pose_mapping ${PROJECT_NAME})
  ament_add_gtest(test_dq_validation test/test_dq_validation.cpp)
  target_link_libraries(test_dq_validation ${PROJECT_NAME})
endif()

ament_package()
