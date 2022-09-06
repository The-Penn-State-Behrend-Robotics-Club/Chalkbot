cmake_minimum_required(VERSION 2.6)

project(gpr)

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
	SET(EXTRA_CXX_COMPILE_FLAGS "-std=c++11 -I./src -I./test -Werror -Wall")

	SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_COMPILE_FLAGS}")
endif(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))

SET(SOURCES 
    src/draw.cpp)

SET(HEADERS
    include/draw.h)

add_executable(drawTest test/drawTest.cpp ${SOURCES} ${HEADERS})
target_link_directories(drawTest PUBLIC lib)
target_include_directories(drawTest PUBLIC include)
target_link_libraries(drawTest gpr)

