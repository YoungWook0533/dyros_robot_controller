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

find_package(OsqpEigen REQUIRED)

#-----------------------------------------
# Include directories
#-----------------------------------------
include_directories(
  include
)

#-----------------------------------------
# 1) C++ library (robot logic)
#-----------------------------------------
add_library(${PROJECT_NAME} SHARED
  # fr3_controller
  src/fr3_controller/controller.cpp
  src/fr3_controller/robot_data.cpp

  # husky_fr3_controller
  src/husky_fr3_controller/controller.cpp
  src/husky_fr3_controller/robot_data.cpp
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  rclcpp::rclcpp
  OsqpEigen::OsqpEigen
  # and other necessary libraries
)

#-----------------------------------------
# 2) Python wrapper library (Boost.Python)
#-----------------------------------------
add_library(${PROJECT_NAME}_wrapper_cpp SHARED
  src/bindings.cpp
)

set_target_properties(${PROJECT_NAME}_wrapper_cpp PROPERTIES
  OUTPUT_NAME "${PROJECT_NAME}_wrapper_cpp"  # actual .so file name
  PREFIX ""                                  # (remove 'lib' on Linux)
)

target_link_libraries(${PROJECT_NAME}_wrapper_cpp
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  Eigen3::Eigen
  Python3::Python
  eigenpy::eigenpy
  pinocchio::pinocchio
  rclcpp::rclcpp
)

#-----------------------------------------
# 3) Optional header installation
#-----------------------------------------
# install(DIRECTORY include/
#   DESTINATION include/
# )

#-----------------------------------------
# 4) Python module installation (if the folder contains __init__.py, etc.)
#-----------------------------------------
# set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
set(PYTHON_INSTALL_DIR "lib")
ament_python_install_package(${PROJECT_NAME})

#-----------------------------------------
# 5) Boost.Python wrapper library installation
#    => the final "dyros_robot_controller_wrapper_cpp.so" will be located under python3.x/site-packages/
#-----------------------------------------
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_wrapper_cpp
    LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_wrapper_cpp
  LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
)

install(DIRECTORY robot/ DESTINATION share/${PROJECT_NAME}/robot)
#-----------------------------------------
# NOTE: ament_package() is not called here.
#       It is only called in the root CMakeLists.
#-----------------------------------------
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  PROGRAMS
    dyros_robot_controller/fr3_controller/ros2_qt_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME FR3ControllerQT
)

install(
  PROGRAMS
    dyros_robot_controller/husky_fr3_controller/ros2_qt_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME HuskyFR3ControllerQT
)

install(
  PROGRAMS
    dyros_robot_controller/husky_controller/ros2_qt_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME HuskyControllerQT
)

install(
  PROGRAMS
    dyros_robot_controller/xls_controller/ros2_qt_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME XLSControllerQT
)

install(
  PROGRAMS
    dyros_robot_controller/pcv_controller/ros2_qt_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME PCVControllerQT
)

install(
  PROGRAMS
    dyros_robot_controller/ur5e_controller/ros2_qt_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME UR5eControllerQT
)

ament_package()
