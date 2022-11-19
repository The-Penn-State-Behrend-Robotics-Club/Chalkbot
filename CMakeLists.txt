cmake_minimum_required(VERSION 2.6)

project (ChalkBot-OpenCV)

set (CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED) 

add_executable(findFiducial tests/findFiducial.cpp)
#add_executable(drawMarker tests/drawMarker.cpp)
add_executable(libraryTest tests/libraryTest.cpp)
add_executable(pipelineTest tests/pipelineTest.cpp)


add_library(markerTracking SHARED src/markerTracking.cpp)
target_include_directories(markerTracking PUBLIC include/)
target_link_libraries(markerTracking ${OpenCV_LIBS})

target_link_libraries(findFiducial ${OpenCV_LIBS})
#target_link_libraries(drawMarker ${OpenCV_LIBS})
target_link_libraries(libraryTest ${OpenCV_LIBS} markerTracking)
target_link_libraries(pipelineTest ${OpenCV_LIBS} markerTracking)

