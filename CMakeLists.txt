cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(trajectoryService)

find_package(gazebo 7.0 REQUIRED)
if(GAZEBO_VERSION)
  if(${GAZEBO_VERSION} GREATER 6)
      add_definitions(-DGAZEBO_GREATER_6)
  endif()
endif()
message(STATUS "Found GAZEBO, version: ${GAZEBO_VERSION}")
#message(STATUS "GAZEBO_INCLUDE_DIRS: ${GAZEBO_INCLUDE_DIRS}")
#message(STATUS "GAZEBO_LIBRARY_DIRS: ${GAZEBO_LIBRARY_DIRS}")
#message(STATUS "GAZEBO_LIBRARIES: ${GAZEBO_LIBRARIES}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


find_package(ignition-transport2 QUIET)
if(IGNITION-TRANSPORT_FOUND OR ignition-transport2_FOUND)
	message(STATUS "Found IGNITION-TRANSPORT, version: ${IGNITION-TRANSPORT_VERSION}, ${ignition-transport2_VERSION}")
else()
	find_package(ignition-transport3 REQUIRED)
	if(ignition-transport3_FOUND)
		message(STATUS "Found IGNITION-TRANSPORT, version: ${ignition-transport3_VERSION}")
	endif()
endif()
#message(STATUS "IGNITION-TRANSPORT_INCLUDE_DIRS: ${IGNITION-TRANSPORT_INCLUDE_DIRS}")
#message(STATUS "IGNITION-TRANSPORT_LIBRARY_DIRS: ${IGNITION-TRANSPORT_LIBRARY_DIRS}")
#message(STATUS "IGNITION-TRANSPORT_LIBRARIES: ${IGNITION-TRANSPORT_LIBRARIES}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

# add_library(trajectory_wp SHARED src/gazebo/trajectory_wp.cc)
# target_link_libraries(trajectory_wp ${GAZEBO_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})

add_library(virtual_element_manager SHARED src/gazebo/virtual_element_manager.cc)
target_link_libraries(virtual_element_manager ${GAZEBO_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})

add_library(cosima_gazebo_virtual_spring SHARED src/gazebo/virtual_spring.cc)
target_link_libraries(cosima_gazebo_virtual_spring ${GAZEBO_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})

# add_library(virtual_spring_visual_sp SHARED src/gazebo/virtual_spring_visual_sp.cc)
# target_link_libraries(virtual_spring_visual_sp ${GAZEBO_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})

# add_executable(commander example/gazebo/commander.cc)
# target_link_libraries(commander ${GAZEBO_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})


find_package(OROCOS-RTT ${RTT_HINTS})

add_compile_options(-std=c++11)
if(OROCOS-RTT_FOUND)
	include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
	# orocos_service(orocosTrajectoryService src/orocos/orocosTrajectoryService.cpp)
	# target_link_libraries(orocosTrajectoryService ${IGNITION-TRANSPORT_LIBRARIES})
	orocos_generate_package()
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")