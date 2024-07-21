cmake_minimum_required(VERSION 3.8.0)
project(argos3_examples)

# Set the compilation type
if(CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel ..." FORCE)
endif(CMAKE_BUILD_TYPE STREQUAL "")

# Deactivate RPATH for MacOSX
set(CMAKE_MACOSX_RPATH 0)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH $ENV{CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(ARGoS REQUIRED)
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS})
link_directories(${ARGOS_LIBRARY_DIR})
link_libraries(${ARGOS_LDFLAGS})

# Look for the (optional) galib library
find_package(GALIB)
if(GALIB_FOUND)
  include_directories(${GALIB_INCLUDE_DIRS})
endif(GALIB_FOUND)

include_directories(${CMAKE_SOURCE_DIR}/include)

# Controllers
add_library(controllers SHARED ./src/controllers/footbot_foraging.cc)
target_link_libraries(controllers
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)

# Loop Functions
link_directories(${CMAKE_BINARY_DIR}/controllers)
set(loop_functions_SOURCES ./src/loop_functions/foraging_loop_functions.cc)

if(ARGOS_QTOPENGL_FOUND)
  include_directories(${ARGOS_QTOPENGL_INCLUDE_DIRS})
  set(loop_functions_SOURCES ${loop_functions_SOURCES}
      ./src/loop_functions/foraging_qt_user_functions.cc)
endif(ARGOS_QTOPENGL_FOUND)

add_library(loop_functions MODULE ${loop_functions_SOURCES})
target_link_libraries(loop_functions
  controllers
  argos3core_simulator
  argos3plugin_simulator_dynamics2d
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  argos3plugin_simulator_media)

if(ARGOS_QTOPENGL_FOUND)
  target_link_libraries(loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_QTOPENGL_FOUND)
