cmake_minimum_required(VERSION 3.4)

project( VideoProcessing )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3" )
set(SRC_DIR "src/")
#set(BIN_DIR "${WORKSPACE}/bin" )

# OpenCV
find_package( OpenCV REQUIRED )

include_directories( ${OpenCV_INCLUDE_DIRS} )

# Gflags
# find_package(gflags REQUIRED)
# if(NOT gflags)
#   message("gflags not found")
# endif()

add_executable( VideoProcessing
  ${SRC_DIR}VideoProcessing.cpp
  ${SRC_DIR}FeatureTracking.cpp
  ${SRC_DIR}VideoStabilizing.cpp
  ${SRC_DIR}VideoFrame.cpp
  ${SRC_DIR}FFeature.cpp
  ${SRC_DIR}Drawing.cpp
)

target_link_libraries( VideoProcessing
  ${OpenCV_LIBS}
  # gflags::gflags
)
