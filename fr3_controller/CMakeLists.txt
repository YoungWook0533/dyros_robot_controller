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
add_library(fr3_controller SHARED
  src/controller.cpp
  src/robot_data.cpp
)

target_link_libraries(fr3_controller
  Eigen3::Eigen
  rclcpp::rclcpp
  OsqpEigen::OsqpEigen
  # and other necessary libraries
)

#-----------------------------------------
# 2) Python wrapper library (Boost.Python)
#-----------------------------------------
add_library(fr3_controller_wrapper_cpp SHARED
  src/bindings.cpp
)

set_target_properties(fr3_controller_wrapper_cpp PROPERTIES
  OUTPUT_NAME "fr3_controller_wrapper_cpp"  # actual .so file name
  PREFIX ""                                 # (remove 'lib' on Linux)
)

target_link_libraries(fr3_controller_wrapper_cpp
  fr3_controller
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
set(PYTHON_INSTALL_DIR "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
ament_python_install_package(fr3_controller)
# => installs the "fr3_controller/" directory as a Python module (setup.py can be integrated in the root or placed here)

#-----------------------------------------
# 5) Boost.Python wrapper library installation
#    => the final "fr3_controller_wrapper_cpp.so" will be located under python3.x/site-packages/
#-----------------------------------------
install(TARGETS fr3_controller fr3_controller_wrapper_cpp
    LIBRARY DESTINATION lib
)

install(TARGETS fr3_controller_wrapper_cpp
  LIBRARY DESTINATION lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages
)

install(DIRECTORY robot/ DESTINATION share/${PROJECT_NAME}/fr3_controller/robot)
#-----------------------------------------
# NOTE: ament_package() is not called here.
#       It is only called in the root CMakeLists.
#-----------------------------------------
