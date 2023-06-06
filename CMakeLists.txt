cmake_minimum_required(VERSION 2.8)

project(nec_plugin)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(PLUGIN_SOURCE src/nec.cpp)

add_subdirectory(lib/scorep_plugin_cxx_wrapper)
add_subdirectory(lib/fmt)

include(cmake/FindScorep.cmake)

if(SCOREP_FOUND)
	include_directories(${SCOREP_INCLUDE_DIRS})
	link_directories(${SCOREP_LIBRARY_DIRS})
else()
	message(SEND_ERROR "Score-P not found!")
endif()

include_directories(nec_plugin PRIVATE
    include
    "/opt/nec/ve/share/veoffload-veda/include/")
link_directories("/opt/nec/ve/veos/lib64")

add_library(${PROJECT_NAME} SHARED ${PLUGIN_SOURCE})
target_link_libraries(${PROJECT_NAME} PRIVATE  pthread veda Scorep::scorep-plugin-cxx fmt::fmt)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
