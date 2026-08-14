#include "screen_capture.hpp"
#include "signals.hpp"

#include <cstdio>
#include <fstream>
#include <iostream>

#include <poll.h>
#include <unistd.h>

std::atomic<bool> g_save_req{false};

void stdin_monitor_thread() {
    char buf[256];
    while (!g_quit) {
        pollfd pfd{STDIN_FILENO, POLLIN, 0};
        int r = poll(&pfd, 1, 200); // 200 ms: wake to re-check g_quit
        if (r <= 0)
            continue; // timeout (r==0) or EINTR/error -> loop and re-check
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
            return; // stdin is gone
        ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
        if (n <= 0)
            return; // EOF or read error: no further input possible
        for (ssize_t i = 0; i < n; ++i)
            if (buf[i] == '\n')
                g_save_req.store(true);
    }
}

std::string save_screen_raw(const uint8_t *luma, unsigned w, unsigned h,
                            std::size_t stride) {
    static unsigned counter = 0;
    char name[64];
    std::snprintf(name, sizeof(name), "screen_%04u_%ux%u.gray", counter, w, h);

    std::ofstream f(name, std::ios::binary);
    if (!f) {
        std::cerr << "save_screen_raw: cannot open '" << name << "' for writing\n";
        return {};
    }
    // Pack tightly to w bytes/row (drop the hardware stride padding) so the file
    // is exactly w*h and opens as `-size WxH -depth 8 gray:`.
    for (unsigned y = 0; y < h; ++y)
        f.write(reinterpret_cast<const char *>(luma + static_cast<std::size_t>(y) * stride), w);
    if (!f) {
        std::cerr << "save_screen_raw: write failed for '" << name << "'\n";
        return {};
    }
    ++counter;
    return name;
}
