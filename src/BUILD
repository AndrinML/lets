cc_binary(
  name = "vision_utils_main",
  srcs = ["vision_utils_main.cc"],
  includes = ["."],
  copts = [""],
  visibility = ["//visibility:public"],
  deps = [
    ":vision_utils",
  ],
)

cc_binary(
  name = "video_processing_main",
  srcs = ["video_processing_main.cc"],
  includes = ["."],
  copts = [""],
  visibility = ["//visibility:public"],
  deps = [
    ":video_processing",
  ],
)

# "//external:gflags"
# "//third_party/eigen3:eigen3",

cc_library(
  name = "vision_utils",
  srcs = [
    "vision_utils.cc",
  ],
  hdrs = [
    "vision_utils.h",
  ],
  includes = ["."],
  copts = [],
  visibility = ["//visibility:public"],
  deps = [
    "@opencv//:opencv"
  ],
)

# LETS library
cc_library(
  name = "video_processing",
  srcs = [
    "VideoProcessing.cpp",
    "FeatureTracking.cpp",
    "VideoStabilizing.cpp",
    "VideoFrame.cpp",
    "FFeature.cpp",
    "Drawing.cpp",
  ],
  hdrs = [
    "VideoProcessing.hpp",
    "FeatureTracking.hpp",
    "FeatureTrackingParams.hpp",
    "VideoStabilizing.hpp",
    "VideoFrame.hpp",
    "FFeature.cpp",
    "Drawing.hpp",
    "Timer.hpp",
  ],
  includes = ["."],
  copts = [],
  visibility = ["//visibility:public"],
  deps = [
    "@opencv//:opencv"
  ],
)