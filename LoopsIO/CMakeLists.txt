cmake_minimum_required(VERSION 3.11)

# The project name
set(PROJ_NAME LoopsIO)
       
# Setup the project
project(${PROJ_NAME})

# Make Qt UIC search in the current folder.
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCEDIR})

# Sources
set(SRC
       src/DecompositionProviders/JsonDecompositionProvider.cpp
       src/DecompositionProviders/TxtDecompositionProvider.cpp
       src/FieldProviders/IFieldProvider.cpp
       src/FieldProviders/JsonFieldProvider.cpp
       src/FieldProviders/TxtFieldProvider.cpp
       src/MapProviders/BoostBinaryProvider.cpp
       src/MapProviders/HereMapProvider.cpp
       src/MapProviders/IpeMapProvider.cpp
       src/MapProviders/JsonMapProvider.cpp
       src/MapProviders/MapTxtMapProvider.cpp
       src/MapProviders/OsmXMLMapProvider.cpp
       src/DecompositionIO.cpp
       src/FieldIO.cpp
       src/GraphIO.cpp
       src/IOHelpers.cpp
       src/WkbCsvTrajectoryReader.cpp
       src/IFieldProvider.cpp
       src/MapProviders/Hdf5MapProvider.cpp
       src/FieldProviders/IpeFieldProvider.cpp
       src/ProblemInstanceProviders/IpePIProvider.cpp
       src/TrajectorySetProviders/WkbCsvTrajectorySetProvider.cpp
       src/TrajectorySetProviders/WktCsvTrajectorySetProvider.cpp
       src/ProblemInstanceProviders/BoostProviders.cpp
       src/TrajectorySetProviders/BoostProviders.cpp
       src/Helpers/InStreamWrapper.cpp
       src/Helpers/JsonInStream.cpp
       src/Helpers/JsonStream.cpp
       src/Helpers/OutStreamWrapper.cpp
       src/Helpers/ReadBuffer.cpp
       src/Helpers/XmlReader.cpp
)
set(HEADERS
include/LoopsIO/TrajectorySetProviders/BoostProviders.h
include/LoopsIO/DecompositionIO.h
include/LoopsIO/DecompositionProviders.h
include/LoopsIO/DecompositionProviders/JsonDecompositionProvider.h
include/LoopsIO/DecompositionProviders/TxtDecompositionProvider.h
include/LoopsIO/DecompositionResultProviders/BoostProviders.h
include/LoopsIO/DecompositionResultSerializer.h
include/LoopsIO/FieldIO.h
include/LoopsIO/Helpers/InStreamWrapper.h
include/LoopsIO/Helpers/JsonInStream.h
include/LoopsIO/Helpers/JsonStream.h
include/LoopsIO/Helpers/OutStreamWrapper.h
include/LoopsIO/Helpers/ReadBuffer.h
include/LoopsIO/Helpers/XmlReader.h
include/LoopsIO/FieldProviders.h
include/LoopsIO/FieldProviders/IpeFieldProvider.h
include/LoopsIO/FieldProviders/JsonFieldProvider.h
include/LoopsIO/FieldProviders/TxtFieldProvider.h
include/LoopsIO/GraphIO.h
include/LoopsIO/FieldSerializer.h
include/LoopsIO/MapSerializer.h
include/LoopsIO/IDecompositionProvider.h
include/LoopsIO/IDecompositionResultProvider.h
include/LoopsIO/IFieldProvider.h
include/LoopsIO/IMapProvider.h
include/LoopsIO/IOHelpers.h
include/LoopsIO/IProblemInstanceProvider.h
include/LoopsIO/ISerializer.h
include/LoopsIO/MapProviders.h
include/LoopsIO/MapProviders/BoostBinaryProvider.h
include/LoopsIO/MapProviders/Hdf5MapProvider.h
include/LoopsIO/MapProviders/HereMapProvider.h
include/LoopsIO/MapProviders/IpeMapProvider.h
include/LoopsIO/MapProviders/JsonMapProvider.h
include/LoopsIO/MapProviders/MapTxtMapProvider.h
include/LoopsIO/MapProviders/OsmXMLMapProvider.h
include/LoopsIO/ProblemInstanceProviders/BoostProviders.h
include/LoopsIO/ProblemInstanceProviders/IpePIProvider.h
include/LoopsIO/ProblemInstanceSerializer.h
include/LoopsIO/Serializers.h
include/LoopsIO/Serializers/BaseTypes.h
include/LoopsIO/Serializers/DecompositionResultSerializer.h
include/LoopsIO/Serializers/FieldSerializer.h
include/LoopsIO/Serializers/GraphSerializer.h
include/LoopsIO/Serializers/ProblemInstanceSerializer.h
include/LoopsIO/TrajectorySetProviders/WkbCsvTrajectorySetProvider.h
include/LoopsIO/TrajectorySetProviders/WktCsvTrajectorySetProvider.h
include/LoopsIO/TrajectorySetSerializer.h
include/LoopsIO/WkbCsvTrajectoryReader.h
src/TrajectorySetProviders/OGRInterface.h
)

if(UNIX)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(Boost_USE_STATIC_LIBS OFF)
endif()

# set(RESOURCES TrajectoryOutliers.qrc)
# set(UI TrajectoryOutliers.ui SpeedBoundedGenerationWidget.ui SettingsPanel.ui SettingsDialog.ui PhysicsModelParametersWidget.ui OutlierDetectionDialog.ui)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SRC} ${HEADERS})
#set(CMAKE_INCLUDE_CURRENT_DIR ON) #Was previously needed for moc, now automatically handled.
#Add the executable project
add_library(${PROJ_NAME} SHARED ${SRC} ${HEADERS} ${RESOURCES} ${UI})
if(WIN32)
set_target_properties(${PROJ_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
endif()

#Setup path CMake variable to algorithms project directory
#set(ALGORITHMS_DIR "" CACHE PATH "Directory with the algorithms")
target_compile_features(${PROJ_NAME} PUBLIC cxx_std_17)

# find_package(GDAL REQUIRED)
# target_link_libraries(${PROJ_NAME} PRIVATE GDAL)
# find_package(PROJ4 REQUIRED)
# target_link_libraries(${PROJ_NAME} PUBLIC PROJ4::proj4)

#Include appropriate directories
target_include_directories(${PROJ_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# Include algorithms folder
#target_include_directories(${PROJ_NAME} PRIVATE ${ALGORITHMS_DIR}) 
if(${MSVC})
find_package(PROJ4 CONFIG REQUIRED)
endif()

if(${UNIX})
find_package(HDF5 REQUIRED COMPONENTS C CXX)
target_include_directories(${PROJ_NAME} PUBLIC ${HDF5_INCLUDE_DIRS})
target_link_libraries(${PROJ_NAME} PUBLIC ${HDF5_LIBRARIES})
else()
find_package(HDF5 CONFIG REQUIRED COMPONENTS C CXX)
target_link_libraries(${PROJ_NAME} PUBLIC hdf5::hdf5-shared hdf5::hdf5_hl-shared hdf5::hdf5_cpp-shared hdf5::hdf5_hl_cpp-shared)
endif()

find_package(pybind11 CONFIG REQUIRED)
target_link_libraries(${PROJ_NAME}  PRIVATE pybind11::embed)

find_package(Qt5 COMPONENTS Core Xml REQUIRED)
target_link_libraries(${PROJ_NAME} PUBLIC Qt5::Core Qt5::Xml fmm)

find_package(Boost REQUIRED COMPONENTS serialization)

# Link to our project
target_link_libraries(${PROJ_NAME} PRIVATE LoopsLib)
target_link_libraries(${PROJ_NAME} PUBLIC proj Boost::serialization)