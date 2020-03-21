cmake_minimum_required (VERSION 2.6)
INCLUDE(CheckLibraryExists)
INCLUDE(CheckFunctionExists)

PROJECT(eloop C)
ADD_DEFINITIONS(-Os -Wall  --std=gnu99 -g3 -Wmissing-declarations)

add_library(eloop SHARED ${PROJECT_SOURCE_DIR}/eloop.c)
install(TARGETS eloop LIBRARY DESTINATION lib)
install(FILES ${PROJECT_SOURCE_DIR}/eloop.h ${PROJECT_SOURCE_DIR}/list.h  DESTINATION include)

ADD_SUBDIRECTORY(example)
