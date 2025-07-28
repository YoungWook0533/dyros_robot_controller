cmake_minimum_required(VERSION 3.14)
project(dyros_robot_controller LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release )
endif()

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module Development)
# find_package(Boost              REQUIRED COMPONENTS python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR})
find_package(eigenpy            REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(Eigen3             REQUIRED)
find_package(pinocchio          REQUIRED)
find_package(OsqpEigen          REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${eigenpy_INCLUDE_DIRS}
)

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

add_library(${PROJECT_NAME}_cpp_wrapper MODULE
  src/bindings.cpp
)

target_link_libraries(${PROJECT_NAME}_cpp_wrapper PUBLIC
  RobotData
  RobotController
  Boost::python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}
  Python3::Module
  eigenpy::eigenpy
)

target_include_directories(${PROJECT_NAME}_cpp_wrapper PUBLIC
  ${Python3_INCLUDE_DIRS})

set_target_properties(${PROJECT_NAME}_cpp_wrapper PROPERTIES
  PREFIX ""
  SUFFIX ".so")

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

install(TARGETS ${PROJECT_NAME}_cpp_wrapper
  LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
)

install(TARGETS ${PROJECT_NAME}_cpp_wrapper
  LIBRARY DESTINATION lib
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
