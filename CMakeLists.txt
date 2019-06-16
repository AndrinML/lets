cmake_minimum_required(VERSION 2.8)

project( VideoProcessing )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3" )
#set(BIN_DIR "${WORKSPACE}/bin" )

find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( VideoProcessing
    VideoProcessing.cpp
    FeatureTracking.cpp
    VideoStabilizing.cpp
    VideoFrame.cpp
    FFeature.cpp
    Drawing.cpp
    )

target_link_libraries( VideoProcessing ${OpenCV_LIBS} )
