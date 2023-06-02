cmake_minimum_required(VERSION 2.8)

project(nec_plugin)

set(PLUGIN_SOURCE nec.c)

include(cmake/FindScorep.cmake)

if(SCOREP_FOUND)
	include_directories(${SCOREP_INCLUDE_DIRS})
	link_directories(${SCOREP_LIBRARY_DIRS})
else()
	message(SEND_ERROR "Score-P not found!")
endif()

include_directories("/opt/nec/ve/share/veoffload-veda/include/")
link_directories("/opt/nec/ve/veos/lib64")

add_library(${PROJECT_NAME} SHARED ${PLUGIN_SOURCE})
target_link_libraries(${PROJECT_NAME} pthread veda)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
