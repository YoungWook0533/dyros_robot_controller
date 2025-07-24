cmake_minimum_required(VERSION 3.14)
project(dyros_robot_controller LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release )
endif()

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(Eigen3             REQUIRED)
find_package(pinocchio          REQUIRED)
find_package(OsqpEigen          REQUIRED)


include_directories(include)

add_library(RobotData SHARED
  src/robot_data/mobile/base.cpp
  src/robot_data/manipulator/base.cpp
  src/robot_data/mobile_manipulator/base.cpp
)

target_link_libraries(RobotData PUBLIC
  pinocchio::pinocchio
  rclcpp::rclcpp
)

add_library(QPController SHARED
  src/QP/base.cpp
  src/QP/manipulator_IK.cpp
  src/QP/manipulator_ID.cpp
  src/QP/mobile_manipulator_IK.cpp
  src/QP/mobile_manipulator_ID.cpp
)

target_link_libraries(QPController PUBLIC
  RobotData
  OsqpEigen::OsqpEigen
)

add_library(RobotController SHARED
  src/robot_controller/mobile/base.cpp
  src/robot_controller/manipulator/base.cpp
  src/robot_controller/mobile_manipulator/base.cpp
)

target_link_libraries(RobotController PUBLIC
  RobotData
  QPController
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS RobotData QPController RobotController
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(RobotData RobotController)
ament_export_dependencies(
  rclcpp
  Eigen3
  pinocchio
  OsqpEigen
)

add_library(dyros_robot_controller::RobotData       ALIAS RobotData)
add_library(dyros_robot_controller::RobotController ALIAS RobotController)
ament_export_targets(export_${PROJECT_NAME})
            
ament_python_install_package(${PROJECT_NAME})
ament_package()
