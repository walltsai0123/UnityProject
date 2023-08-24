cmake_minimum_required (VERSION 3.13)

project(CppInterface)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(UNITY_PROJECT_ROOT "${PROJECT_SOURCE_DIR}" CACHE STRING
	"Path to Unity project root where Assets and ProjectSettings resides")

set(UNITY_DLL_OUTPUT_DIRECTORY "Assets/Plugins" CACHE STRING 
	"Where the DLL will be after building, this should be inside the Assets/Plugins folder. 
	Note: this should be and empty directory as everything is cleared on build")

set(UNITY_DLL_OUTPUT_DIRECTORY_ABS "${UNITY_PROJECT_ROOT}/${UNITY_DLL_OUTPUT_DIRECTORY}")

add_subdirectory("Interface")