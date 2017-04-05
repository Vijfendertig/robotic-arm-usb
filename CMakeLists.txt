cmake_minimum_required(VERSION 2.6)

#
# Project name.
#

project(robotic-arm-usb)

#
# Non-target-specific project settings. 
#

# Select 'Release' or 'Debug' build.
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_VERBOSE_MAKEFILE OFF)

# Add CMake modules.
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})

# libusb 1.0 is required for the library.
find_package(LibUSB REQUIRED)

# Qt4 is required for the qt-control-unit example.
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})

# Set compiler options.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED 11)
include_directories(
	${LibUSB_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/library/include
	${CMAKE_BINARY_DIR}
)


#
# Interface to the Velleman/OWI robotic arm's USB interface as a shared library.
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

#
# Qt control unit (resembling the physical control unit).
#

qt4_wrap_cpp(QT_CONTROL_UNIT_HPP
       	examples/qt-control-unit/include/qt-control-unit-window.h
)
qt4_wrap_ui(QT_CONTROL_UNIT_UI
       	examples/qt-control-unit/ui/qt-control-unit.ui
)
qt4_add_resources(QT_CONTROL_UNIT_RESOURCES
	examples/qt-control-unit/resources/robotic-arm.qrc
)
add_executable(qt-control-unit
	examples/qt-control-unit/src/main.cc
	examples/qt-control-unit/src/qt-control-unit-window.cc
	${QT_CONTROL_UNIT_HPP}
	${QT_CONTROL_UNIT_UI}
	${QT_CONTROL_UNIT_RESOURCES}
)
target_link_libraries(qt-control-unit
	${QT_LIBRARIES}
	roboticarmusb
)

