cmake_minimum_required(VERSION 2.8)
project( VideoFlow )
find_package( OpenCV REQUIRED )
add_executable( OptFlow optflow.cpp )
target_link_libraries( OptFlow ${OpenCV_LIBS} )
