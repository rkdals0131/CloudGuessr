cmake_minimum_required(VERSION 3.8)
project(cloudguessr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 required for PCL and modern features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# =============================================================================
# Backend Library (cloudguessr_backend)
# =============================================================================
add_library(cloudguessr_backend SHARED
  src/backend/io.cpp
  src/backend/preprocess.cpp
  src/backend/roi.cpp
  src/backend/icp.cpp
  src/backend/scoring.cpp
  src/backend/round_dataset.cpp
)
target_include_directories(cloudguessr_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cloudguessr_backend
  ${PCL_LIBRARIES}
  yaml-cpp
)
ament_target_dependencies(cloudguessr_backend
  rclcpp
  std_msgs
  sensor_msgs
)

# =============================================================================
# Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit Tests
  ament_add_gtest(test_io test/unit/test_io.cpp)
  target_link_libraries(test_io cloudguessr_backend)

  ament_add_gtest(test_preprocess test/unit/test_preprocess.cpp)
  target_link_libraries(test_preprocess cloudguessr_backend)

  ament_add_gtest(test_roi test/unit/test_roi.cpp)
  target_link_libraries(test_roi cloudguessr_backend)

  ament_add_gtest(test_icp test/unit/test_icp.cpp)
  target_link_libraries(test_icp cloudguessr_backend)

  ament_add_gtest(test_scoring test/unit/test_scoring.cpp)
  target_link_libraries(test_scoring cloudguessr_backend)

  ament_add_gtest(test_round_dataset test/unit/test_round_dataset.cpp)
  target_link_libraries(test_round_dataset cloudguessr_backend)

  # Integration Tests
  ament_add_gtest(test_pipeline_e2e test/integration/test_pipeline_e2e.cpp)
  target_link_libraries(test_pipeline_e2e cloudguessr_backend)
endif()

# =============================================================================
# Install
# =============================================================================
install(TARGETS cloudguessr_backend
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
