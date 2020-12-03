cmake_minimum_required(VERSION 3.14)

# The project name
set(PROJ_NAME PyLoops)

message(STATUS "Cxx : ${CMAKE_CXX_COMPILER_ID}")

option(PYLOOPS_WITH_INTERFACE "PyLoops with interface" OFF)

find_package(pybind11 CONFIG REQUIRED)
# Setup the project
set(PYLOOPS_MODULE_SRC 
PyLoops.cpp)
set(SRC 
src/DS/PyField.cpp
src/DS/PyGraph.cpp
src/DS/PyTrajectory.cpp
src/Frechet/PyStrongFrechetGraphData.cpp
src/Loops/FieldGen.cpp
src/Loops/PyDecompositionAlgs.cpp
src/MapMatching/FastMapMatching.cpp
src/Math/PyDynamicVector.cpp
src/Models/PyDecompositionResult.cpp
src/Models/PyProblemInstance.cpp
src/Processing/fmm.cpp
src/Processing/PySpeedBoundedFmm.cpp
src/Processing/PyTrajectoryChecker.cpp
src/Processing/TrajectoryProcessing.cpp
src/Processing/PyComputeCoverage.cpp
)

set(HEADERS 
include/PyLoops/PyLoops.h
include/PyLoops/Processing/PyComputeCoverage.h
include/PyLoops/DS/PyField.h
include/PyLoops/DS/PyGraph.h
include/PyLoops/DS/PyTrajectory.h
include/PyLoops/Frechet/PyStrongFrechetGraphData.h
include/PyLoops/IO/WkbCsvTrajectoryReader.h
include/PyLoops/Loops/FieldGen.h
include/PyLoops/Loops/GraphGen.h
include/PyLoops/Loops/PyDecompositionAlgs.h
include/PyLoops/MapMatching/FastMapMatching.h
include/PyLoops/Math/PyDynamicVector.h
include/PyLoops/Models/PyDecompositionResult.h
include/PyLoops/Models/PyLongDouble.h
include/PyLoops/Models/PyProblemInstance.h
include/PyLoops/Movetk/GeometryBindings.h
include/PyLoops/Processing/FastMapMatchJob.h
include/PyLoops/Processing/fmm.h
include/PyLoops/Processing/parallel/RangJobPool.h
include/PyLoops/Processing/PySpeedBoundedFmm.h
include/PyLoops/Processing/PyTrajectoryChecker.h
include/PyLoops/Processing/TrajectoryProcessing.h
include/PyLoops/PyLoops.h
include/PyLoops/PyLoops.inc.h
)
set(GUI_IPC_HEADERS
include/PyLoops/Gui/GuiIpc.h
)
set(GUI_IPC_SRC
src/Gui/GuiIpc.cpp
)
# Create pybinf module
pybind11_add_module(${PROJ_NAME} ${SRC} ${HEADERS} ${PYLOOPS_MODULE_SRC} ${GUI_IPC_HEADERS} ${GUI_IPC_SRC})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

if(MSVC)
message(STATUS "${CMAKE_MODULE_PATH}")
	include(CopyDependentDlls)
	CopyDependentDlls(${PROJ_NAME} "${PROJ_NAME}.cp37-win_amd64.pyd")
endif()

target_include_directories(${PROJ_NAME} PUBLIC include)

#target_include_directories(PyLoopsAlgs PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/PyLoopsAlgs)

message(STATUS "Py include"  ${PYTHON_INCLUDE_DIRS})

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SRC} ${HEADERS})

# Link to Loops lib.
find_package(Boost REQUIRED COMPONENTS graph serialization)

#Auto moc and uic files for Qt UI.
set(CMAKE_INCLUDE_CURRENT_DIR ON)

target_link_libraries(${PROJ_NAME} PUBLIC Guimessages)

if(UNIX)
    message(STATUS "PyLoops Unix shit")
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    # No static boost
    set(Boost_USE_STATIC_LIBS OFF)
    # Add threads library, default included in Windows apparently
    find_package(Threads REQUIRED)
    target_link_libraries(${PROJ_NAME} PUBLIC LoopsLib fmm Boost::serialization Boost::graph LoopsIO LoopsAlgs stdc++fs Threads::Threads)
    #target_link_libraries(PyLoopsAlgs PUBLIC LoopsLib fmm Boost::serialization Boost::graph LoopsIO LoopsAlgs stdc++fs Threads::Threads)
else()
    target_link_libraries(${PROJ_NAME} PUBLIC LoopsLib fmm Boost::serialization Boost::graph LoopsIO LoopsAlgs)
    #target_link_libraries(PyLoopsAlgs PUBLIC LoopsLib fmm Boost::serialization Boost::graph LoopsIO LoopsAlgs)
endif()
if(PYLOOPS_WITH_INTERFACE)
	# Create separate library for inclusion in embedded version
	add_library(PyLoops_interface STATIC ${SRC} ${HEADERS})

	target_include_directories(PyLoops_interface PUBLIC include)
	if(UNIX)
		message(STATUS "PyLoops Unix shit")
		set(CMAKE_POSITION_INDEPENDENT_CODE ON)
		# No static boost
		set(Boost_USE_STATIC_LIBS OFF)
		# Add threads library, default included in Windows apparently
		find_package(Threads REQUIRED)
		target_link_libraries(PyLoops_interface PUBLIC LoopsLib fmm Boost::serialization Boost::graph LoopsIO LoopsAlgs stdc++fs Threads::Threads)
		#target_link_libraries(PyLoopsAlgs PUBLIC LoopsLib fmm Boost::serialization Boost::graph LoopsIO LoopsAlgs stdc++fs Threads::Threads)
	else()
		target_link_libraries(PyLoops_interface PUBLIC LoopsLib fmm Boost::serialization Boost::graph LoopsIO LoopsAlgs)
		#target_link_libraries(PyLoopsAlgs PUBLIC LoopsLib fmm Boost::serialization Boost::graph LoopsIO LoopsAlgs)
	endif()

	target_link_libraries(PyLoops_interface PUBLIC pybind11::module Guimessages)
	message(STATUS "Boost static: ${Boost_USE_STATIC_LIBS}")
	add_library(PyLoops::interface ALIAS PyLoops_interface)
endif()
#target_include_directores(${PROJ_NAME} )