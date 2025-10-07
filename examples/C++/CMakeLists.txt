cmake_minimum_required(VERSION 3.16)
project(dyros_robot_controller_example)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# --- output to bin/ ---
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin)

add_definitions(-DROBOTS_DIRECTORY="${CMAKE_SOURCE_DIR}/../robots")
message(STATUS "ROBOTS_DIRECTORY = ${CMAKE_SOURCE_DIR}/../robots")

find_package(dyros_robot_controller REQUIRED)
find_package(Eigen3 REQUIRED)

# =====================================================================
# download dependencies
include(FetchContent)

set(MUJOCO_DEP_VERSION_lodepng
    b4ed2cd7ecf61d29076169b49199371456d4f90b
    CACHE STRING "Version of `lodepng` to be fetched."
)

# Fetch lodepng dependency.
if (NOT TARGET lodepng)
  FetchContent_Declare(
      lodepng
      GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
      GIT_TAG ${MUJOCO_DEP_VERSION_lodepng}
  )
  FetchContent_GetProperties(lodepng)
  if (NOT lodepng_POPULATED)
    FetchContent_Populate(lodepng)
    # This is not a CMake project.
    set(LODEPNG_SRCS ${lodepng_SOURCE_DIR}/lodepng.cpp)
    set(LODEPNG_HEADERS ${lodepng_SOURCE_DIR}/lodepng.h)
    add_library(lodepng STATIC ${LODEPNG_HEADERS} ${LODEPNG_SRCS})
    target_compile_options(lodepng PRIVATE ${MUJOCO_MACOS_COMPILE_OPTIONS})
    target_link_options(lodepng PRIVATE ${MUJOCO_MACOS_LINK_OPTIONS})
    target_include_directories(lodepng PUBLIC ${lodepng_SOURCE_DIR})
  endif ()
endif ()

# find dependencies
set(OpenGL_GL_PREFERENCE GLVND)
find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)
# =====================================================================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  src/mujoco/glfw_adapter.cc
  src/mujoco/glfw_dispatch.cc
  src/mujoco/platform_ui_adapter.cc
  src/mujoco/simulate.cc
  src/fr3_controller.cpp
  src/xls_controller.cpp
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
  mujoco::mujoco 
  glfw 
  lodepng 
  Threads::Threads 
  dyros_robot_controller::dyros_robot_controller
)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/mujoco>" PRIVATE src)
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT ${PROJECT_NAME}
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT ${PROJECT_NAME}
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT ${PROJECT_NAME}
    BUNDLE DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT ${PROJECT_NAME}
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" COMPONENT ${PROJECT_NAME}
)