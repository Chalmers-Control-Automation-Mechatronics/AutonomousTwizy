load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_lossy_map",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    linkopts = [
        "-lopencv_core",
        "-lopencv_highgui",
        "-lopencv_imgproc",
    ],
    deps = [
        "//modules/localization/msf/common/util:localization_msf_common_util",
        "//modules/localization/msf/local_map/base_map:localization_msf_base_map",
        "@eigen//:eigen",
    ],
)

cpplint()
