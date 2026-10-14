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
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sas_core REQUIRED)
find_package(sas_common REQUIRED)
find_package(sas_msgs REQUIRED)
find_package(haptic_tools_msgs REQUIRED)
find_package(haptic_tools_core REQUIRED)


##### CPP BINARY #####
add_executable(haptic_navigation_node
    src/haptic_navigation_node.cpp
    src/haptic_navigation.cpp
    )
target_link_libraries(haptic_navigation_node
    haptic_tools_core::haptic_tools_core
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


ament_package()
