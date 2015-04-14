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


INCLUDE_DIRECTORIES(include
    ${freenect2_INCLUDE_DIR}
)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Create an executable file from sources:
ADD_LIBRARY(Kinect2Device SHARED ${files})

# Link external libraries
TARGET_LINK_LIBRARIES(Kinect2Device ${DisCODe_LIBRARIES} ${OpenCV_LIBS} ${freenect2_LIBRARIES} ${GLEWmx_LIBRARY})

INSTALL_COMPONENT(Kinect2Device)
