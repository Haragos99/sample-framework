cmake_minimum_required(VERSION 3.12)
project(sample-framework)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
message(STATUS "Configuring ${CMAKE_MODULE_PATH}")

# Include directories
# Set the path to libQGLViewer if it's not in the default location
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include_directories(
        ${PROJECT_SOURCE_DIR}/include
        /usr/include/eigen3
        /usr/local/include/
)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-Wall -fPIC -Wno-reorder")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DDEBUG -fsanitize=address")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -DNDEBUG")
endif(CMAKE_COMPILER_IS_GNUCXX)
add_definitions(${Qt6Widgets_DEFINITIONS} -D_USE_MATH_DEFINES)
# Find required packages

find_package(Qt6 REQUIRED COMPONENTS Core OpenGL OpenGLWidgets Widgets Xml)
find_package(OpenGL REQUIRED)
find_package(OpenMesh REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(QGLViewer REQUIRED)
#find_package(TBB REQUIRED)

# Source files
file(GLOB_RECURSE SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})


message("ah6: ${QGLVIEWER_LIBRARY}")
# Link libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
        OpenGL::GL
        OpenMeshCore
        Qt6::Core
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        Qt6::Widgets
        Qt6::Xml
        ${QGLVIEWER_LIBRARY}
)
