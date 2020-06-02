cc_library(
    name = "opencv",
    srcs = glob(["lib/libopencv*.so.4*"]),
    hdrs = glob(["include/opencv4/opencv2/**/*.hpp"])
           + glob(["include/opencv4/opencv2/**/*.h"]),
    includes = ["include/opencv4"],
    visibility = ["//visibility:public"],
    linkopts = [],
    linkstatic = 1,
)
