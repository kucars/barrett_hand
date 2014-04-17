cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

add_library(openmath STATIC Inertia.cc SE3.cc SO3.cc)

include(../detect_cpu.cmake)

if (DEFINED ENV{ROS_MARCH_FLAGS})
    message(STATUS "Using mtune flags set in environment: $ENV{ROS_MARCH_FLAGS}")
    add_definitions( "$ENV{ROS_MARCH_FLAGS}" )
elseif(VENDOR_ID STREQUAL "GenuineIntel" AND CPU_FAMILY EQUAL 6 AND MODEL EQUAL 28)
    message(STATUS "Building for Intel Atom")
    add_definitions("-march=core2 -mtune=native -mmmx -msse2 -msse3 -mfpmath=sse")
elseif(VENDOR_ID STREQUAL "CentaurHauls")
    message(STATUS "Building for VIA - Original Barrett WAM PC")
    add_definitions("-march=c3-2")
endif  (DEFINED ENV{ROS_MARCH_FLAGS})

add_definitions("-ggdb3")

