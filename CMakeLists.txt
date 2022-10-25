cmake_minimum_required(VERSION 2.6)

project (ChalkBot-OpenCV)

set (CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED) 

add_executable(findFiducial tests/findFiducial.cpp)
#add_executable(drawMarker tests/drawMarker.cpp)

target_link_libraries(findFiducial ${OpenCV_LIBS})
#target_link_libraries(drawMarker ${OpenCV_LIBS})

