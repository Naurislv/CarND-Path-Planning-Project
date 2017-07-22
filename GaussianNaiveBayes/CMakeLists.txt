# CMake entry point
cmake_minimum_required (VERSION 3.5)
project (gaussiannb)

set (CMAKE_CXX_STANDARD 11)

add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
)

add_executable(gnb
	src/main.cpp
	src/classifier.cpp
)

target_link_libraries(gnb)