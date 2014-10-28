# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -DBT_USE_DOUBLE_PRECISION -Wall")
# Unused warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wuninitialized -Winit-self -Wunused-function -Wunused-label -Wunused-variable -Wunused-but-set-variable -Wunused-but-set-parameter")
# Additional warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Warray-bounds -Wtype-limits -Wreturn-type -Wsequence-point -Wparentheses -Wmissing-braces -Wchar-subscripts -Wswitch -Wwrite-strings -Wenum-compare -Wempty-body -Wlogical-op")

# Find required packages
FIND_PACKAGE( OpenCV REQUIRED )
FIND_PACKAGE( freenect2 REQUIRED )
INCLUDE_DIRECTORIES( ${freenect2_INCLUDE_DIR} )

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
ADD_LIBRARY(Kinect2Device SHARED ${files})

# GLEW
#INCLUDE_DIRECTORIES("/home/dkaczmar/lib/include/")
#LINK_DIRECTORIES("/home/dkaczmar/lib/lib64")
#ADD_DEFINITIONS(-DGLEW_MX -DGLEW_STATIC)

# Link external libraries

TARGET_LINK_LIBRARIES(Kinect2Device ${DisCODe_LIBRARIES} ${OpenCV_LIBS} ${freenect2_LIBRARIES} )


INSTALL_COMPONENT(Kinect2Device)
