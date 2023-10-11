cmake_minimum_required (VERSION 3.13)

set(PHYSICS_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" PARENT_SCOPE)

file(GLOB PHYSICS_HFILES "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h" 
                         "${CMAKE_CURRENT_SOURCE_DIR}/include/PD/*.h"
                         "${CMAKE_CURRENT_SOURCE_DIR}/include/XPBD/*.h")
file(GLOB PHYSICS_SRCFILES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
                           "${CMAKE_CURRENT_SOURCE_DIR}/src/PD/*.cpp"
                           "${CMAKE_CURRENT_SOURCE_DIR}/src/XPBD/*.cpp")

set(PHYSICS_H "${PHYSICS_HFILES}" PARENT_SCOPE)
set(PHYSICS_SRC "${PHYSICS_SRCFILES}" PARENT_SCOPE)


