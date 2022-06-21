cmake_minimum_required(VERSION 3.2)
project(customGoals VERSION 1.0.1 DESCRIPTION "some custom goals")

set(CMAKE_CXX_STANDARD 11)

find_package(OpenSim 4.3 REQUIRED HINTS
    "${CMAKE_SOURCE_DIR}/../../../../../../")
include("${OpenSim_USE_FILE}")
		
add_library(osimMocoCoordinateAccelerationGoal SHARED
        ./MocoCoordinateAccelerationGoal/MocoCoordinateAccelerationGoal.h
        ./MocoCoordinateAccelerationGoal/MocoCoordinateAccelerationGoal.cpp
        ./MocoCoordinateAccelerationGoal/osimMocoCoordinateAccelerationGoalDLL.h
        ./MocoCoordinateAccelerationGoal/RegisterTypes_osimMocoCoordinateAccelerationGoal.h
        ./MocoCoordinateAccelerationGoal/RegisterTypes_osimMocoCoordinateAccelerationGoal.cpp
        )
add_library(osimMocoMarkerAccelerationGoal SHARED
        ./MocoMarkerAccelerationGoal/MocoMarkerAccelerationGoal.h
        ./MocoMarkerAccelerationGoal/MocoMarkerAccelerationGoal.cpp
        ./MocoMarkerAccelerationGoal/osimMocoMarkerAccelerationGoalDLL.h
        ./MocoMarkerAccelerationGoal/RegisterTypes_osimMocoMarkerAccelerationGoal.h
        ./MocoMarkerAccelerationGoal/RegisterTypes_osimMocoMarkerAccelerationGoal.cpp
        )
		
target_link_libraries(osimMocoCoordinateAccelerationGoal
        osimTools osimExampleComponents osimMoco)
target_link_libraries(osimMocoMarkerAccelerationGoal
        osimTools osimExampleComponents osimMoco)
		
set_target_properties(osimMocoCoordinateAccelerationGoal PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(osimMocoCoordinateAccelerationGoal PROPERTIES SOVERSION 1)
set_target_properties(osimMocoCoordinateAccelerationGoal PROPERTIES PUBLIC_HEADER osimMocoCoordinateAccelerationGoal.h)

string(TOUPPER MocoCoordinateAccelerationGoal walk_name_upper)
set_target_properties(osimMocoCoordinateAccelerationGoal PROPERTIES
        DEFINE_SYMBOL OSIM${walk_name_upper}_EXPORTS
        )

set_target_properties(osimMocoMarkerAccelerationGoal PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(osimMocoMarkerAccelerationGoal PROPERTIES SOVERSION 1)
set_target_properties(osimMocoMarkerAccelerationGoal PROPERTIES PUBLIC_HEADER osimMocoCoordinateAccelerationGoal.h)

string(TOUPPER MocoMarkerAccelerationGoal walk_name_upper)
set_target_properties(osimMocoMarkerAccelerationGoal PROPERTIES
        DEFINE_SYMBOL OSIM${walk_name_upper}_EXPORTS
        )

install(TARGETS osimMocoCoordinateAccelerationGoal
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(TARGETS osimMocoMarkerAccelerationGoal
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# For Windows: make sure DLLs for dependencies are available.
OpenSimCopyDependencyDLLsForWin(DEP_NAME OpenSim DEP_BIN_DIR "${OpenSim_BIN_DIR}")
	
if(WIN32)
	add_dependencies(osimMocoCoordinateAccelerationGoal osimMocoMarkerAccelerationGoal Copy_OpenSim_DLLs)
endif()

file(COPY  DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")