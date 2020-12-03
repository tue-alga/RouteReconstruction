cmake_minimum_required(VERSION 3.14)

# The project name
set(PROJ_NAME LoopsLib)

# Policies
cmake_policy(SET CMP0074 NEW)

if(${UNIX})
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
endif()

message( "Mod path: " ${CMAKE_MODULE_PATH})

# Input variables
set(FIND_CPLEX_PATH "" CACHE PATH "Find CPlex cmake script path")

if(UNIX)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fPIC")
#set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} "-static")
#set(CMAKE_LIBRARY_SUFFIXES ".a;.so")
endif()

# Sources
set(SRC
#src/Algs/LongestPath/HeuristicApproach.cpp
src/Algs/LongestPath/LongestPathILP.cpp
src/Algs/LongestPath/k_SGL.cpp
src/Algs/MaxFlow/MaxFlowCplex.cpp
src/Algs/NNLS.cpp
src/Algs/Processing/TrajectorySetFrechet.cpp
src/DS/BaseGraph.cpp
src/DS/CompactGraph.cpp
src/DS/EmbeddedGraph.cpp
src/DS/Graph.cpp
src/DS/GraphHelpers/DagView.cpp
src/DS/GraphHelpers/GraphModificationGuard.cpp
src/DS/GraphView.cpp
src/DS/OwnGraph.cpp
src/Geometry/Index/EdgeIndex.cpp
src/Geometry/Index/PointIndex.cpp
src/Geometry/Interval.cpp
src/GraphAlgs/BFS.cpp
src/GraphAlgs/DFS.cpp
src/Helpers/DecompositionObject.cpp
src/Helpers/FieldGenerator.cpp
src/Helpers/GraphGenerator.cpp
src/Helpers/Logger.cpp
src/Models/DecompositionResult.cpp
src/Models/FlowField.cpp
src/Models/ProblemInstance.cpp
src/Models/TrajectoryTypes.cpp
src/Models/ReconstructedTrajectory.cpp
)
set(HEADERS 
include/LoopsLib/Models/ReconstructedTrajectory.h
include/LoopsLib/Algs/EdgeHittingPath.h
include/LoopsLib/DS/BoostModificationGraph.h
#include/LoopsLib/Algs/ExpandedGraph.h
include/LoopsLib/Algs/Flow/FlowInfo.h
include/LoopsLib/Algs/LongestPath/DAGLongestPath.h
#include/LoopsLib/Algs/LongestPath/HeuristicApproach.h
include/LoopsLib/Algs/LongestPath/ILongestPathAlg.h
include/LoopsLib/Algs/LongestPath/LongestPathILP.h
include/LoopsLib/DS/BoostInterface.h
include/LoopsLib/Algs/LongestPath/RandomVertexOrder.h
include/LoopsLib/Algs/LongestPath/k_SGL.h
include/LoopsLib/Algs/LongestPath.h
include/LoopsLib/Algs/MaxFlow/MaxFlowCplex.h
include/LoopsLib/Algs/NNLS.h
include/LoopsLib/Algs/Processing/BFS.h
include/LoopsLib/Algs/Processing/ConnectedComponents.h
include/LoopsLib/Algs/Processing/DFS.h
include/LoopsLib/Algs/Processing/GramSchmidt.h
include/LoopsLib/Algs/Processing/IsReachable.h
include/LoopsLib/Algs/Processing/NormalVector.h
include/LoopsLib/Algs/Processing/ReachabilityLabeling.h
include/LoopsLib/Algs/Processing/RemoveDeadends.h
include/LoopsLib/Algs/Processing/TrajectorySetFrechet.h
include/LoopsLib/Algs/ShortestPath/BellmanFord.h
include/LoopsLib/Algs/Types.h
include/LoopsLib/Concepts.h
include/LoopsLib/DS/AbstractGraph.h
include/LoopsLib/DS/BaseGraph.h
include/LoopsLib/DS/BiMap.h
include/LoopsLib/DS/CompactGraph.h
include/LoopsLib/DS/EmbeddedGraph.h
include/LoopsLib/DS/Graph.h
include/LoopsLib/DS/GraphConcepts.h
include/LoopsLib/DS/GraphEmbedding.h
include/LoopsLib/DS/GraphHelpers/DagView.h
include/LoopsLib/DS/GraphHelpers/GraphModificationGuard.h
include/LoopsLib/DS/GraphInterface.h
include/LoopsLib/DS/GraphView.h
include/LoopsLib/DS/Heap.h
include/LoopsLib/DS/OwnGraph.h
include/LoopsLib/DS/VertexOrder.h
include/LoopsLib/Geometry/BoostInterface.h
include/LoopsLib/Geometry/Index/EdgeIndex.h
include/LoopsLib/Geometry/Index/PointIndex.h
include/LoopsLib/Geometry/Interval.h
include/LoopsLib/GraphAlgs/AStar.h
include/LoopsLib/GraphAlgs/BFS.h
include/LoopsLib/GraphAlgs/CycleFinder.h
include/LoopsLib/GraphAlgs/DFS.h
include/LoopsLib/GraphAlgs/GraphPredicates.h
include/LoopsLib/GraphAlgs/ShortestPath.h
include/LoopsLib/GraphAlgs/WidestPath.h
include/LoopsLib/Helpers/CplexHelpers.h
include/LoopsLib/Helpers/DecompositionObject.h
include/LoopsLib/Helpers/FieldGenerator.h
include/LoopsLib/Helpers/GraphConcepts.h
include/LoopsLib/Helpers/GraphGenerator.h
include/LoopsLib/Helpers/IFieldGenerator.h
include/LoopsLib/Helpers/IGraphGenerator.h
include/LoopsLib/Helpers/Iterators.h
include/LoopsLib/Helpers/Logger.h
include/LoopsLib/Helpers/RandomDAGFieldGenerator.h
include/LoopsLib/Helpers/RandomHelpers.h
include/LoopsLib/Helpers/StdHelpers.h
include/LoopsLib/Helpers/Timer.h
include/LoopsLib/Helpers/TypeTraits.h
include/LoopsLib/Helpers/reduce.h
include/LoopsLib/Math/Vector.h
include/LoopsLib/Models/DecompositionResult.h
include/LoopsLib/Models/FlowField.h
include/LoopsLib/Models/ProblemInstance.h
include/LoopsLib/Models/TrajectoryTypes.h
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SRC} ${HEADERS})


# Get the tulib dir (manual assignment of directory in CMake GUI or via commandline)
set(MOVETK_DIR "" CACHE PATH "Directory of MoveTk CMakeLists.txt file")
set(LONGESTPATH_DIR "" CACHE PATH "Directory to Longest path code")

#set(CMAKE_INCLUDE_CURRENT_DIR ON) #Was previously needed for moc, now automatically handled.
#Add the executable project
add_library(${PROJ_NAME} SHARED ${SRC} ${HEADERS} ${RESOURCES} ${UI})
if(WIN32)
set_target_properties(${PROJ_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()

#Setup path CMake variable to algorithms project directory
#set(ALGORITHMS_DIR "" CACHE PATH "Directory with the algorithms")

target_compile_features(${PROJ_NAME} PUBLIC cxx_std_17)

#
# Includes CPLEX
#
if(NOT ${FIND_CPLEX_PATH} STREQUAL "")
	message(STATUS "Including CPLEX")
	list(APPEND CMAKE_MODULE_PATH "${FIND_CPLEX_PATH}")
	find_package(Cplex)
	# include_directories(SYSTEM ${CPLEX_INCLUDE_DIRS})
	# target_link_libraries(${PROJ_NAME} PRIVATE ${CPLEX_LIBRARIES})
	target_link_libraries(${PROJ_NAME} PUBLIC Cplex)
	target_compile_definitions(${PROJ_NAME} PUBLIC IL_STD)
endif()

# find_package(GDAL REQUIRED)
# target_link_libraries(${PROJ_NAME} PRIVATE GDAL)
# find_package(PROJ4 REQUIRED)
# target_link_libraries(${PROJ_NAME} PUBLIC PROJ4::proj4)

#Include appropriate directories
target_include_directories(${PROJ_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# Include algorithms folder
#target_include_directories(${PROJ_NAME} PRIVATE ${ALGORITHMS_DIR}) 

#Include movetk
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/tulib) #Create binary dir for movetk, is required when doing add_subdirectory outside of project root.

add_subdirectory(${MOVETK_DIR} ${CMAKE_BINARY_DIR}/movetk)

# Link to our project
target_link_libraries(${PROJ_NAME} PUBLIC movetk)

find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED COMPONENTS Core)
target_link_libraries(${PROJ_NAME} PUBLIC CGAL::CGAL CGAL::CGAL_Core Eigen3::Eigen)
message( "CGAL VERSION: ${CGAL_VERSION}")

if(UNIX)
find_package(Threads REQUIRED)
set(Boost_USE_STATIC_LIBS OFF)
target_link_libraries(${PROJ_NAME} PUBLIC Threads::Threads)
endif()

#Include boost
find_package(Boost REQUIRED COMPONENTS date_time timer graph)
target_link_libraries(${PROJ_NAME} PUBLIC Boost::date_time Boost::timer Boost::graph)