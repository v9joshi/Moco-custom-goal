cmake_minimum_required(VERSION 3.2)
project(WalkSim)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenSim REQUIRED HINTS
    "${CMAKE_SOURCE_DIR}/../../../../../../")
include("${OpenSim_USE_FILE}")

add_library(osimMocoActivationSquaredGoal SHARED
        MocoActivationSquaredGoal.h
        MocoActivationSquaredGoal.cpp
        osimMocoActivationSquaredGoalDLL.h
        RegisterTypes_osimMocoActivationSquaredGoal.h
        RegisterTypes_osimMocoActivationSquaredGoal.cpp
        )
target_link_libraries(osimMocoActivationSquaredGoal
        osimTools osimExampleComponents osimMoco)

string(TOUPPER MocoActivationSquaredGoal example_name_upper)
set_target_properties(osimMocoActivationSquaredGoal PROPERTIES
        DEFINE_SYMBOL OSIM${example_name_upper}_EXPORTS
        )

add_executable(walksim_all WalkSim_All.cpp)
target_link_libraries(walksim_all osimMocoActivationSquaredGoal)

# For Windows: make sure DLLs for dependencies are available.
OpenSimCopyDependencyDLLsForWin(DEP_NAME OpenSim
        DEP_BIN_DIR "${OpenSim_BIN_DIR}")
if(WIN32)
    add_dependencies(osimMocoActivationSquaredGoal Copy_OpenSim_DLLs)
endif()

file(COPY  DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
