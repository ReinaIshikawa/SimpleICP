# Create Project
project( ICPalgorithm )
cmake_minimum_required(VERSION 3.13)
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )
add_executable(ICPalgorithm icp.cpp)

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "ICPalgorithm" )

set(CMAKE_CXX_STANDARD 14)

# Find Package
find_package( PCL 1.8 REQUIRED )
find_package( OpenCV REQUIRED)

if(OpenCV_FOUND)
  include_directories( ${OpenCV_INCLUDE_DIRS})
  target_link_libraries( ICPalgorithm ${OpenCV_LIBS})
endif(OpenCV_FOUND)

if(PCL_FOUND)
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( ICPalgorithm ${PCL_LIBRARIES} )
endif()
