cmake_minimum_required (VERSION 3.13)
project(__cpp-interface)

set(EXTERNAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern")
file(GLOB UNITY_PLUGIN_API_FILES "${EXTERNAL_DIR}/Unity/PluginAPI/*.c*" "${EXTERNAL_DIR}/Unity/PluginAPI/*.h*")

add_subdirectory("Native")
add_subdirectory("Physics")

add_library(${PROJECT_NAME} SHARED ${UNITY_PLUGIN_API_FILES}
			${NATIVE_H} ${NATIVE_SRC}
			${PHYSICS_H} ${PHYSICS_SRC})

set_target_properties(${PROJECT_NAME} PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY_DEBUG "${UNITY_DLL_OUTPUT_DIRECTORY_ABS}"
		RUNTIME_OUTPUT_DIRECTORY_RELEASE "${UNITY_DLL_OUTPUT_DIRECTORY_ABS}")
message(STATUS "Saving ${PROJECT_NAME}.dll to: ${UNITY_DLL_OUTPUT_DIRECTORY_ABS}")

find_package(OpenMP REQUIRED)
find_package(libigl REQUIRED)

find_path(LIBIGL_INCLUDE_DIR NAMES igl/readMESH.h)
message(STATUS "libigl include dir: ${LIBIGL_INCLUDE_DIR}")

#message(STATUS "physics h files: ${PHYSICS_H}")
#message(STATUS "physics src files: ${PHYSICS_SRC}")

target_include_directories(${PROJECT_NAME} PRIVATE ${NATIVE_INCLUDE_DIR} ${PHYSICS_INCLUDE_DIR} "${EXTERNAL_DIR}/Unity" ${LIBIGL_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX igl::igl_core)
#target_compile_definitions(${PROJECT_NAME} PRIVATE NODEBUG)

message(STATUS "Include directories: ${NATIVE_INCLUDE_DIR} ${PHYSICS_INCLUDE_DIR} ${EXTERNAL_DIR}/Unity ${LIBIGL_INCLUDE_DIR}") 
