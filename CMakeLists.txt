cmake_minimum_required(VERSION 3.0.0)
project(losp VERSION 0.1.0)

include(CTest)
enable_testing()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

add_executable(losp main.c utils.c MAENS/MAENS.c heuristic.c heuristic.h simulator.c simulator.h arrayoperations.c 
MAENS/MAENS.h MAENS/operators.c lsoperator.c ls.c lsoperator.h MASDC/MASDC.c MASDC/MASDC.h MASDC/MASDCop.c others.c)

# The followed line is for <math.h>
target_link_libraries(losp PRIVATE m)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})


include(CPack)
