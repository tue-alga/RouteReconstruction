cmake_minimum_required(VERSION 3.14)

# The project name
set(PROJ_NAME Loops)

# Policies
cmake_policy(SET CMP0074 NEW)

# Option to build tests for the library
option(LOOPS_BUILD_TESTS "Build tests" ON)
option(LOOPS_WITH_UI "Build UI" ON)
option(WITH_BERRY_LIB "Build Berry lib" OFF)

# Setup the project
project(Loops)

# Make Qt UIC search in the current folder.
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR})

if(${UNIX})
	list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
endif()

message( "Mod path: " ${CMAKE_MODULE_PATH})

# Input variables
set(FIND_CPLEX_PATH "" CACHE PATH "Find CPlex cmake script path")

if(UNIX)
	set(CMAKE_POSITION_INDEPENDENT_CODE ON)
	#list (APPEND CMAKE_CXX_FLAGS  "-fPIC")
#set(CMAKE_EXE_LINKER_FLAGS ${CMAKE_EXE_LINKER_FLAGS} "-static")
#set(CMAKE_LIBRARY_SUFFIXES ".a;.so")
endif()

# Get the tulib dir (manual assignment of directory in CMake GUI or via commandline)
set(MOVETK_DIR "" CACHE PATH "Directory of MoveTk CMakeLists.txt file")
set(LONGESTPATH_DIR "" CACHE PATH "Directory to Longest path code")

# Maybe wrap in if statement
if(${LOOPS_BUILD_TESTS})
	find_package(Catch2 REQUIRED)
	enable_testing()

	add_executable(Heap_test tests/heap_tests.cpp tests/main_tests.cpp)
	add_test(HeapTest Heap_test)
	target_link_libraries(Heap_test PRIVATE Catch2::Catch2)
	target_include_directories(Heap_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

	add_executable(FrechetHitting_test tests/frechethitting_tests.cpp tests/main_tests.cpp)
	add_test(FrechetHittingTest FrechetHitting_test)
	target_link_libraries(FrechetHitting_test PRIVATE Catch2::Catch2)
	target_include_directories(FrechetHitting_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
endif()

add_subdirectory(FMM)
add_subdirectory(LoopsLib)
add_subdirectory(LoopsIO)
add_subdirectory(LoopsAlgs)
add_subdirectory(GuiMessages)
add_subdirectory(PyLoops)

if(${LOOPS_WITH_UI}) #Conditionally build UI
	add_subdirectory(LoopsGui)
endif()
#add_subdirectory(src/third_party/fmm)
if(${WITH_BERRY_LIB})
	add_subdirectory(BerryLib)
endif()