cmake_minimum_required(VERSION 3.2)
project(OpenSim_exampleSlidingMassAdvanced)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenSim REQUIRED HINTS
    "${CMAKE_SOURCE_DIR}/../../../../../")
include("${OpenSim_USE_FILE}")

foreach(exe exampleSlidingMassAdvanced)
    add_executable(${exe} ${exe}.cpp)
    target_link_libraries(${exe}
            osimTools osimExampleComponents osimMoco)

    # For Windows: make sure DLLs for dependencies are available.
    OpenSimCopyDependencyDLLsForWin(DEP_NAME OpenSim
            DEP_BIN_DIR "${OpenSim_BIN_DIR}")
    if(WIN32)
        add_dependencies(${exe} Copy_OpenSim_DLLs)
    endif()
endforeach()

file(COPY  DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
