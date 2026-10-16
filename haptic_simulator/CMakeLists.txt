cmake_minimum_required(VERSION 3.22...3.26)
project(haptic_simulator LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sas_msgs REQUIRED)


##### CPP BINARY #####
add_executable(haptic_simulator
    src/main.cpp
    src/haptic_simulator_window.hpp
    src/haptic_simulator_window.cpp
    )
target_link_libraries(haptic_simulator
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Eigen3::Eigen
    dqrobotics
    )
ament_target_dependencies(haptic_simulator
    rclcpp
    sas_msgs
    )
install(TARGETS haptic_simulator
    DESTINATION lib/${PROJECT_NAME})
##END## CPP BINARY #####


install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
    )

ament_package()
