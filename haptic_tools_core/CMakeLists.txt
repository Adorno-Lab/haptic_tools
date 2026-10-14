cmake_minimum_required(VERSION 3.22...3.26)
project(haptic_tools_core)

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


##### CPP LIBRARY #####
# The haptic tools without ROS: the navigation strategy of Aldhafeeri et al. (TAROS 2025) and its safety layer
# (navigation/), the pose mapping (pose_mapping/, not from the paper), and the checks of their inputs.
add_library(${PROJECT_NAME} SHARED
    src/dq_validation.cpp
    src/navigation/haptic_navigation_strategy.cpp
    src/navigation/haptic_navigation_safety.cpp
    src/pose_mapping/haptic_pose_mapping.cpp
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


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dq_validation test/test_dq_validation.cpp)
  target_link_libraries(test_dq_validation ${PROJECT_NAME})
  ament_add_gtest(test_haptic_navigation_strategy test/test_haptic_navigation_strategy.cpp)
  target_link_libraries(test_haptic_navigation_strategy ${PROJECT_NAME})
  ament_add_gtest(test_haptic_navigation_safety test/test_haptic_navigation_safety.cpp)
  target_link_libraries(test_haptic_navigation_safety ${PROJECT_NAME})
  ament_add_gtest(test_haptic_pose_mapping test/test_haptic_pose_mapping.cpp)
  target_link_libraries(test_haptic_pose_mapping ${PROJECT_NAME})
endif()

ament_package()
