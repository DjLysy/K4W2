# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

option(USE_OPENCL_REGISTRATION "Use OpenCL implementation of the depth registration" ON)
option(USE_OPENCV_OPENCL       "Use OpenCV OpenCL functions for rectification"       OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -std=c++11 -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

# Find required packages

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(freenect2 REQUIRED)

IF(OPENMP_FOUND)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
ENDIF()

# Display additional files in qtcreator
execute_process(COMMAND find ${PROJECT_SOURCE_DIR} -type f OUTPUT_VARIABLE FILES_STRING)
string(REPLACE "\n" ";" FILES_LIST ${FILES_STRING})
add_custom_target(additional_files_${PROJECT_NAME}
  SOURCES
  ${FILES_LIST}
  ${PROJECT_SOURCE_DIR}/README.md
)

INCLUDE_DIRECTORIES(include
    ${freenect2_INCLUDE_DIR}
)

IF(USE_OPENCL_REGISTRATION)
  ADD_DEFINITIONS(-DUSE_OPENCL_REGISTRATION)
ENDIF()
IF(USE_OPENCV_OPENCL)
  ADD_DEFINITIONS(-USE_OPENCV_OPENCL)
ENDIF()

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
ADD_LIBRARY(Kinect2Device SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(Kinect2Device ${DisCODe_LIBRARIES} ${OpenCV_LIBS} ${freenect2_LIBRARIES} ${GLEWmx_LIBRARY})

INSTALL_COMPONENT(Kinect2Device)
