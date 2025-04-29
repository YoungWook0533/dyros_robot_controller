cmake_minimum_required(VERSION 3.5)
project(dyros_robot_controller)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-O3 -march=native)

# 1) Package configuration
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(eigenpy REQUIRED)
find_package(pinocchio REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${eigenpy_INCLUDE_DIRS}
)

# 2) Subdirectories (e.g., fr3_controller, husky_controller, etc.)
add_subdirectory(fr3_controller)
# add_subdirectory(husky_controller)

# 3) (Optional) Install Python package
# ament_python_install_package(${PROJECT_NAME})
# Uncomment the line above if there is a top-level Python module to install.

# 4) Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
