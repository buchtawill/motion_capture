// test_pattern.hpp: the --test DRM sanity path.
#pragma once

#include <string>

// DRM sanity path (--test): drive the exact NV12/overlay/CSC scanout pipeline
// with a locally generated luma buffer of vertical grayscale bars -- no camera,
// no V4L2, no dmabuf export. If this shows bars but the live path is black, the
// fault is isolated to the V4L2->DRM buffer import, not the DP display path.
int run_test_pattern(const std::string &drm_dev, const std::string &conn_name,
                      unsigned w, unsigned h);
