# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -std=c++11 -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

# Find required packages

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(freenect2 REQUIRED)
FIND_PACKAGE(OpenMP)
FIND_PACKAGE(Eigen)
FIND_PACKAGE(OpenCL)


if(OPENMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#CHOOSE CPU OR OPENCL
if(EIGEN_FOUND)
  message(STATUS "CPU based depth registration enabled")
  set(DEPTH_REG_CPU ON)
  add_definitions(-DDEPTH_REG_CPU)
else()
  message(STATUS "CPU based depth registration disabled")
  set(DEPTH_REG_CPU OFF)
endif()

if(OPENCL_FOUND)
  message(STATUS "OpenCL based depth registration enabled")
  set(DEPTH_REG_OPENCL ON)
  add_definitions(-DDEPTH_REG_OPENCL)
else()
  message(STATUS "OpenCL based depth registration disabled")
  set(DEPTH_REG_OPENCL OFF)
endif()

if(NOT DEPTH_REG_CPU AND NOT DEPTH_REG_OPENCL)
  message(FATAL_ERROR "No registration method available!")
endif()

#BUILD CONFIG
if(DEPTH_REG_CPU)
  include_directories(${Eigen_INCLUDE_DIRS})
  set(MODULES ${MODULES} ./depth_registration_cpu.cpp)
  set(MODULE_LIBS ${MODULE_LIBS} ${Eigen_LIBRARIES})
endif()

if(DEPTH_REG_OPENCL)
  include_directories(${OPENCL_INCLUDE_DIRS})
  add_definitions(-DREG_OPENCL_FILE="${PROJECT_SOURCE_DIR}/srcComponents/Kinect2Bridge/depth_registration.cl")
  set(MODULES ${MODULES} ./depth_registration_opencl.cpp)
  set(MODULE_LIBS ${MODULE_LIBS} ${OPENCL_LIBRARIES})
endif()

# Create a variable containing all .cpp files:
#FILE(GLOB files *.cpp)
SET(files ${files} ./Kinect2Bridge.cpp)
SET(files ${files} ./depth_registration.cpp)

# Create an executable file from sources:
ADD_LIBRARY(Kinect2Bridge SHARED ${files} ${MODULES})

# Link external libraries
TARGET_LINK_LIBRARIES(Kinect2Bridge ${DisCODe_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${freenect2_LIBRARIES} ${MODULE_LIBS})

INSTALL_COMPONENT(Kinect2Bridge)
