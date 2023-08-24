cmake_minimum_required (VERSION 3.13)
project(__cpp-interface)

set(HEADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(EXTERNAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/extern")

file(GLOB UNITY_PLUGIN_API_FILES "${EXTERNAL_DIR}/Unity/PluginAPI/*.c*" "${EXTERNAL_DIR}/Unity/PluginAPI/*.h*")
file(GLOB HFILES "${HEADER_DIR}/*.h" "${HEADER_DIR}/*/*.h")
file(GLOB SRCFILES "${SOURCE_DIR}/*.cpp" "${SOURCE_DIR}/*/*.cpp")

# message(STATUS "Unity Plugin API Files ${PROJECT_NAME}: ${UNITY_PLUGIN_API_FILES}")
message(STATUS "Header Files ${PROJECT_NAME}: ${HFILES}")
message(STATUS "Source Files ${PROJECT_NAME}: ${SRCFILES}")

add_library(${PROJECT_NAME} SHARED ${HFILES} ${SRCFILES}
		${UNITY_PLUGIN_API_FILES})

set_target_properties(${PROJECT_NAME} PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY_DEBUG "${UNITY_DLL_OUTPUT_DIRECTORY_ABS}"
		RUNTIME_OUTPUT_DIRECTORY_RELEASE "${UNITY_DLL_OUTPUT_DIRECTORY_ABS}")
message(STATUS "Saving ${PROJECT_NAME}.dll to: ${UNITY_DLL_OUTPUT_DIRECTORY_ABS}")

find_package(OpenMP REQUIRED)
find_package(libigl REQUIRED)

find_path(LIBIGL_INCLUDE_DIR NAMES igl/readMESH.h)
message(STATUS "libigl include dir: ${LIBIGL_INCLUDE_DIR}")

target_include_directories(${PROJECT_NAME} PRIVATE "${HEADER_DIR}" "${EXTERNAL_DIR}/Unity" "${LIBIGL_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME} PRIVATE OpenMP::OpenMP_CXX igl::igl_core)

message(STATUS "Include directories: ${HEADER_DIR} ${EXTERNAL_DIR}/Unity ${LIBIGL_INCLUDE_DIR}") 
