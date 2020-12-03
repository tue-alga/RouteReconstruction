cmake_minimum_required(VERSION 3.14)

# The project name
set(PROJ_NAME Guimessages)

# Setup the project
set(SRC 
src/BaseMessage.cpp
src/Messages.cpp
src/BytesBuffer.cpp
)

set(HEADERS 
include/GuiMessages/BytesBuffer.h
include/GuiMessages/BaseMessage.h
include/GuiMessages/GenericMessage.h
include/GuiMessages/Messages.h
)

# Create pybinf module
add_library(${PROJ_NAME} STATIC ${SRC} ${HEADERS})

target_include_directories(${PROJ_NAME} PUBLIC include)

target_link_libraries(${PROJ_NAME} PUBLIC LoopsLib)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SRC} ${HEADERS})