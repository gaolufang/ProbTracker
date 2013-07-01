project(ProbTracker)

cmake_minimum_required(VERSION 2.6)

find_package(OpenCV REQUIRED)

#-------------------------------------------------------------------------------
#version
set(PROBTRACKER_VERSION_MAJOR 1)
set(PROBTRACKER_VERSION_MINOR 3)
set(PROBTRACKER_VERSION_PATCH 0)
set(PROBTRACKER_VERSION ${PROBTRACKER_VERSION_MAJOR}.${PROBTRACKER_VERSION_MINOR}.${PROBTRACKER_VERSION_PATCH})

include_directories( )

add_library(
	)

target_link_libraries()

add_executable(ProbTracker ProbTracker.cxx)
