cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(find_xenomai.cmake)

#set (CANBUS_TYPE "ESD")
set (CANBUS_TYPE "PEAK")
set (CMAKE_VERBOSE_MAKEFILE on)

set(ROS_BUILD_STATIC_LIBS true)
set(ROS_BUILD_SHARED_LIBS false)


add_subdirectory(openwam)
add_subdirectory(openmath)

include_directories (openwam openmath)
add_definitions("-O0 -ggdb3 -DWRIST -DBH8")

if (CANBUS_TYPE STREQUAL "ESD")
message(STATUS "using ESD CANbus driver")
set (CANBUS_DEFS "-DESD_CAN -I../esdcan-pci200/lib32")
set (CANBUS_LIBS "ntcan")
set (CANBUS_LDFLAGS "-L../esdcan-pci200/lib32")
elseif (CANBUS_TYPE  STREQUAL "PEAK")
message(STATUS "using PEAK CANbus driver")
set (CANBUS_DEFS "-DPEAK_CAN")
set (CANBUS_LIBS "pcan")
else (CANBUS_TYPE STREQUAL "ESD")
message(STATUS "No CANbus type recognized; only building owdsim.")
set (CANBUS_DEFS "")
set (CANBUS_LIBS "")
endif (CANBUS_TYPE STREQUAL "ESD")

if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
#add_executable(owd owd.cpp openwamdriver.cpp bhd280.cc
#                            ft.cc tactile.cc)
#target_link_libraries(owd wamcan openwam openmath lapack blas
			  #gfortran ${CANBUS_LIBS}   ${catkin_LIBRARIES})
#set(owd -DBH280_ONLY "${CANBUS_DEFS}")
#set(owd "${CANBUS_LDFLAGS}")

#add_executable(canbhd owd.cpp openwamdriver.cpp bhd280.cc
#                               ft.cc tactile.cc)
#target_link_libraries(canbhd bhdcan openwam openmath lapack blas
#			     gfortran ${CANBUS_LIBS}   ${catkin_LIBRARIES})
#set(canbhd "-DBH280 -DBH280_ONLY ${CANBUS_DEFS}")
#set(canbhd "${CANBUS_LDFLAGS}" )
endif (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")


#add_executable(owd_traj_timer owd_traj_timer.cpp)
#if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
#  target_link_libraries(owd_traj_timer openwam)
#else (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
#  target_link_libraries(owd_traj_timer openwamsim)
#endif (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")

if (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")
if (RT_BUILD)
  #add_executable(owdrt owd.cpp openwamdriver.cpp bhd280.cc
  #                              ft.cc tactile.cc)
  #target_link_libraries(owdrt wamcanrt openwam openmath lapack blas
  #                            gfortran ${CANBUS_LIBS} ${RT_LIBS})
  #set(owdrt "${CANBUS_DEFS} ${RT_DEFS}")
  #set(owdrt "${CANBUS_LDFLAGS}")

  #add_executable(canbhdrt owd.cpp openwamdriver.cpp bhd280.cc
  #                                 ft.cc tactile.cc)
  #target_link_libraries(canbhdrt bhdcanrt openwam openmath lapack blas
  #    	                         gfortran ${CANBUS_LIBS} ${RT_LIBS})
  #set(canbhdrt "-DBH280 -DBH280_ONLY ${CANBUS_DEFS} ${RT_DEFS}")
  #set(canbhdrt "${CANBUS_LDFLAGS}")
endif (RT_BUILD)
endif (CANBUS_TYPE STREQUAL "ESD" OR CANBUS_TYPE STREQUAL "PEAK")

include(detect_cpu.cmake)

if (DEFINED ENV{OWD_MARCH_FLAGS})
    message(STATUS "Using mtune flags set in environment: $ENV{OWD_MARCH_FLAGS}")
    add_definitions( "$ENV{OWD_MARCH_FLAGS}" )
elseif (VENDOR_ID STREQUAL "GenuineIntel" AND CPU_FAMILY EQUAL 6 AND MODEL EQUAL 28)
    message(STATUS "Building for Intel Atom")
    add_definitions("-march=core2 -mtune=native -mmmx -msse2 -msse3 -mfpmath=sse")
elseif (VENDOR_ID STREQUAL "CentaurHauls")
    message(STATUS "Building for VIA - Original Barrett WAM PC")
    add_definitions("-march=c3-2")
endif  (DEFINED ENV{OWD_MARCH_FLAGS})

#add_custom_target(SUID ALL sudo chown root ../owd COMMAND sudo chmod u+s ../owd DEPENDS owd)
