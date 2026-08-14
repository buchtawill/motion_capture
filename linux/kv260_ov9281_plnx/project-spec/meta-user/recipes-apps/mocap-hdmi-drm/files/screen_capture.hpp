// screen_capture.hpp: press-ENTER-to-save-frame support.
//
// A dedicated thread watches stdin; each newline (Enter) sets g_save_req. The
// render loop -- the ONLY thread that owns the on-screen capture buffer
// (slot[fb_screen]) and controls when it is re-queued to V4L2 -- polls that flag
// and writes the current 8-bit luma frame to a raw file. Doing the write on the
// render thread (not the stdin thread) means it can never race the capture DMA
// re-queuing/overwriting the buffer, the same ownership discipline the watchdog
// uses for g_recover_req.
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

// Set by the stdin monitor thread on each Enter; cleared (exchange) by the
// render loop when it performs the save.
extern std::atomic<bool> g_save_req;

// Blocks on stdin via poll (200 ms timeout so it periodically re-checks g_quit
// and stays joinable) and sets g_save_req on every newline. Returns when g_quit
// is set or stdin reaches EOF.
void stdin_monitor_thread();

// Writes w*h bytes of 8-bit luma -- rows 0..h-1 read at `stride`, packed tightly
// to w bytes/row (stride padding dropped) -- to a new raw file in the cwd named
// screen_<NNNN>_<w>x<h>.gray. Returns the path written, or an empty string on
// error (message already printed). The caller must have made the DMA buffer
// coherent (dmabuf_sync_read) around this call.
std::string save_screen_raw(const uint8_t *luma, unsigned w, unsigned h,
                            std::size_t stride);
