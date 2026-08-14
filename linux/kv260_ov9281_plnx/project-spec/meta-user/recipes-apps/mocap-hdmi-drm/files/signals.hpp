// signals.hpp: process-wide Ctrl-C/SIGTERM handling shared by the live
// render loop (main.cpp) and the --test static-pattern path (test_pattern.cpp).
#pragma once

#include <atomic>

extern std::atomic<bool> g_quit;

void on_signal(int);
