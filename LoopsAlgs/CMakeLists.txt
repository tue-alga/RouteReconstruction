cmake_minimum_required(VERSION 3.11)

# The project name
set(PROJ_NAME LoopsAlgs)

# Setup the project
project(${PROJ_NAME})

# Sources
set(SRC
src/FlowDecomposition/FrechetDecomposition.cpp
src/FlowDecomposition/FrechetHittingDecomposition.cpp
src/FlowDecomposition/HittingPathsDecomposition.cpp
src/FlowDecomposition/ILPFlowDecomposition.cpp
src/FlowDecomposition/TrivialDecomposition.cpp
src/FlowDecomposition/WeightedFrechetPathsDecomposition.cpp
src/FlowDecomposition/FrechetBandDecomposition.cpp
src/IFlowDecomposition.cpp
src/Frechet/FrechetHelpers.cpp
src/Frechet/FrechetOnGraph.cpp
src/Frechet/PathHittingStrongFrechet.cpp
src/Frechet/PathHittingStrongFrechet.cpp
src/Frechet/FrechetSetFinding.cpp
# src/Frechet/WeakFrechetOnGraph.cpp
src/Frechet/WeightedStrongFrechet.cpp
src/Frechet/StrongFrechetGraphBuilder.cpp
src/Memory/GetStats.cpp
src/Frechet/LowMemoryFrechetData.cpp
src/FlowDecomposition/ShortestPathsDecomposition.cpp
src/Flow/UncapturableFlow.cpp
src/MapMatching/StrongFrechetMapMatching.cpp
src/Flow/MultiCommodityMinCostFlow.cpp
src/FlowDecomposition/GlobalMinCostDecomposition.cpp
src/FlowDecomposition/MultiCommodityDecomposition.cpp
src/MapMatching/StrongFrechetMapMatching.cpp
src/MapMatching/FastMapMatching.cpp
src/Trajectories/Coverage.cpp
)
set(HEADERS
include/LoopsAlgs/Trajectories/Coverage.h
include/LoopsAlgs/MapMatching/Parts/TransitionGraph.h
include/LoopsAlgs/MapMatching/FastMapMatching.h
include/LoopsAlgs/FlowDecomposition/MultiCommodityDecomposition.h
include/LoopsAlgs/FlowDecomposition/GlobalMinCostDecomposition.h
include/LoopsAlgs/FlowDecomposition/FrechetBandDecomposition.h
include/LoopsAlgs/Flow/DecomposeFlow.h
include/LoopsAlgs/Flow/MultiCommodityMinCostFlow.h
include/LoopsAlgs/AlgsInterface.h
include/LoopsAlgs/Flow/UncapturableFlow.h
include/LoopsAlgs/Memory/GetStats.h
include/LoopsAlgs/Flow/MinCostFlow.h
include/LoopsAlgs/Frechet/FrechetSetFinding.h
include/LoopsAlgs/Frechet/NewSweepline.h
include/LoopsAlgs/Frechet/LowMemoryFrechetData.h
include/LoopsAlgs/Multithreading/Helpers.h
include/LoopsAlgs/Frechet/StrongFrechetGraphBuilder.h
include/LoopsAlgs/MapMatching/StrongFrechetMapMatching.h
include/LoopsAlgs/FlowDecomposition/FrechetDecomposition.h
include/LoopsAlgs/Frechet/CleanAltSweepline.h
include/LoopsAlgs/Frechet/FrechetIO.h
include/LoopsAlgs/FlowDecomposition/FrechetHittingDecomposition.h
include/LoopsAlgs/FlowDecomposition/HittingPathsDecomposition.h
include/LoopsAlgs/IFlowDecomposition.h
include/LoopsAlgs/FlowDecomposition/ILPFlowDecomposition.h
include/LoopsAlgs/FlowDecomposition/SimpleDecomposition.h
include/LoopsAlgs/FlowDecomposition/StateSpaceDecomposition.h
include/LoopsAlgs/FlowDecomposition/TrivialDecomposition.h
include/LoopsAlgs/FlowDecomposition/WeightedFrechetPathsDecomposition.h
include/LoopsAlgs/FlowDecomposition.h
include/LoopsAlgs/Frechet/FrechetHelpers.h
include/LoopsAlgs/Frechet/FrechetOnGraph.h
include/LoopsAlgs/Frechet/PathHittingStrongFrechet.h
include/LoopsAlgs/Frechet/PathHittingStrongFrechet.h
include/LoopsAlgs/FlowDecomposition/ShortestPathsDecomposition.h
# include/LoopsAlgs/Frechet/WeakFrechetOnGraph.h
include/LoopsAlgs/Frechet/WeightedStrongFrechet.h
include/LoopsAlgs/Frechet/WenkSweepline.h
)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SRC} ${HEADERS})

#Add the executable project
add_library(${PROJ_NAME} SHARED ${SRC} ${HEADERS} ${RESOURCES} ${UI})
if(WIN32)
set_target_properties(${PROJ_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()

target_compile_features(${PROJ_NAME} PUBLIC cxx_std_17)

#Include appropriate directories
target_include_directories(${PROJ_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link to our project
target_link_libraries(${PROJ_NAME} PRIVATE LoopsLib)

find_package(Boost REQUIRED COMPONENTS serialization)

# Link to our project
target_link_libraries(${PROJ_NAME} PUBLIC Boost::serialization)