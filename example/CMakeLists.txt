cmake_minimum_required (VERSION 2.6)
INCLUDE(CheckLibraryExists)
INCLUDE(CheckFunctionExists)

ADD_DEFINITIONS(-Os -Wall  --std=gnu99 -g3 -Wmissing-declarations)


include_directories(${PROJECT_SOURCE_DIR})
add_executable(timer timer.c)
add_executable(echosrv echosrv.c)
target_link_libraries(timer eloop)
target_link_libraries(echosrv eloop)

INSTALL(TARGETS timer RUNTIME DESTINATION bin)
INSTALL(TARGETS echosrv RUNTIME DESTINATION bin)
