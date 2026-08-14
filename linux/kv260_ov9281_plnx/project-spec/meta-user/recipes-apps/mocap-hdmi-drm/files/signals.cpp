#include "signals.hpp"

std::atomic<bool> g_quit{false};

void on_signal(int) { g_quit = true; }
