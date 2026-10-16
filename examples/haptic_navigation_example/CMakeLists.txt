cmake_minimum_required(VERSION 3.22...3.26)
project(haptic_navigation_example)

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
find_package(haptic_tools_core REQUIRED)
find_package(haptic_tools_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)


##### CPP BINARY #####
add_executable(haptic_navigation_example_node
    src/haptic_navigation_example_node.cpp
    src/haptic_navigation_example.cpp
    )
target_link_libraries(haptic_navigation_example_node
    haptic_tools_core::haptic_tools_core
    )
ament_target_dependencies(haptic_navigation_example_node
    rclcpp
    geometry_msgs
    sas_core
    sas_common
    sas_msgs
    haptic_tools_msgs
    )
install(TARGETS haptic_navigation_example_node
    DESTINATION lib/${PROJECT_NAME})

# A basic GUI: the dead zone, the stylus, and its angle (the status of haptic_navigation_example_node).
add_executable(haptic_navigation_example_gui
    src/haptic_navigation_example_gui.cpp
    src/navigation_view.cpp
    )
target_link_libraries(haptic_navigation_example_gui
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    )
ament_target_dependencies(haptic_navigation_example_gui
    rclcpp
    haptic_tools_msgs
    )
install(TARGETS haptic_navigation_example_gui
    DESTINATION lib/${PROJECT_NAME})
##END## CPP BINARY #####


install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}/
    )

ament_package()
