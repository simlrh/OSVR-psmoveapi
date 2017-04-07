cmake_minimum_required(VERSION 3.1)
project(psmoveapiPlugin) # Change this line.

set (CMAKE_CXX_STANDARD 11)

set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH} )
find_package(osvr REQUIRED)
find_package(Eigen3 REQUIRED)

osvr_convert_json(je_nourish_psmovecontroller_json
    je_nourish_psmovecontroller.json
    "${CMAKE_CURRENT_BINARY_DIR}/je_nourish_psmovecontroller_json.h")

osvr_convert_json(je_nourish_psmovecamera_json
    je_nourish_psmovecamera.json
    "${CMAKE_CURRENT_BINARY_DIR}/je_nourish_psmovecamera_json.h")

include_directories(
	"${CMAKE_CURRENT_BINARY_DIR}"
	"${EIGEN3_INCLUDE_DIR}"
	"vendor/psmoveapi/include"
	"vendor/psmoveapi/src/math"
)

osvr_add_plugin(NAME je_nourish_psmoveapi
    CPP # indicates we'd like to use the C++ wrapper
    SOURCES
    je_nourish_psmoveapi.cpp
    "${CMAKE_CURRENT_BINARY_DIR}/je_nourish_psmovecontroller_json.h"
    "${CMAKE_CURRENT_BINARY_DIR}/je_nourish_psmovecamera_json.h"
	)

add_subdirectory("vendor/psmoveapi/src")

target_link_libraries(je_nourish_psmoveapi psmoveapi_static psmoveapi_tracker_static)
