cmake_minimum_required(VERSION 2.4)

#
# Project name.
#

project(robotic-arm-usb)

#
# Non-target-specific project settings. 
#

# Select 'Release' or 'Debug' build.
set(CMAKE_BUILD_TYPE Release)

# Add CMake modules.
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})

# libusb 1.0 is required.
find_package(LibUSB REQUIRED)

# Set compiler options.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
include_directories(
	${LibUSB_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/library/include
)


#
# Interface to the Velleman/OWI robotics arm's USB interface as a shared library.
#

add_library(roboticarmusb SHARED
       	library/src/robotic-arm-usb.cc
)
target_link_libraries(roboticarmusb
       	${LibUSB_LIBRARIES}
)


#
# Simple example to show and test connecting to and disconnecting from the robotic arm and
# switching the LED in the gripper on and off.
#

add_executable(test-library
	examples/test-library/test-library.cc
)
target_link_libraries(test-library
	roboticarmusb
)
