# Plan — Blob-detector software: UIO drivers + DRM red-box overlay

Goal: give the KV260 Linux stack userspace access to the `mocap_wrapper` block
(fused ISP histogram + RLE blob detector, AXI-Lite @ 0xA0011000, one IRQ), and
make `mocap-hdmi-drm` draw **red boxes** around detected blobs on the live HDMI/DP
output. Keep the *pipeline* driver and the *blob* driver modular. Consolidate to
one source of truth (no copied headers).

Background: see `AAR_blob_detection.md` (register map §7, ownership/IRQ contract),
`PLAN_isp_uio_driver.md` (the pattern this mirrors), and `CLAUDE.md` (Linux/DT
architecture, UIO gotchas).

---

## 1. Architecture decisions

### 1a. One UIO device, two modular C++ drivers
The hardware is a single AXI-Lite slave (one `/dev/uioN`, one IRQ). The register
map interleaves pipeline and blob concerns in one 0x6C region, so we cannot split
it into two UIO devices. Instead:

- **`MocapPipeline`** (owns the resource) — opens/mmaps the UIO device, discovers
  it by name, and manages *pipeline-level* control: HRES/VRES, ENABLE, THRESHOLD,
  RESET, the histogram readback, the cycle counter, and the **frame lifecycle**:
  block on the IRQ (UIO `read()`), then `RESULTS_ACK` via the CMD doorbell. It
  exposes the mapped `volatile mocap_regs_t*` to collaborators.
- **`BlobDetector`** (a view, owns nothing) — constructed from a `MocapPipeline&`
  (borrows its register pointer). Reads *blob* results only: `BLOB_COUNT`, and per
  blob the bounding box + centroid via the `BLOB_ADDR` indirect-read window. Does
  not touch the UIO fd, IRQ, or pipeline control.

This keeps the two drivers as separate, independently-usable classes (modular)
while respecting the single-UIO hardware. The IRQ/ACK ownership protocol lives in
`MocapPipeline` because it governs *both* banks (hist + blobs) as one coherent
published buffer — `BlobDetector` just reads whatever bank is currently published.

Typical app loop:
```
pipe.arm(hres, vres, threshold);        // HRES/VRES, ENABLE, autoinc, threshold
for each frame:
    pipe.wait_frame(timeout);           // blocks on UIO IRQ (frame_done)
    n = blobs.read_all(out_vector);     // BLOB_ADDR-indirect readback (published bank)
    pipe.ack();                         // CMD.RESULTS_ACK: clear IRQ + release buffer
```

### 1b. Single source of truth (no copied headers)
- **`mocap_regs.h`** (PeakRDL-generated `mocap_regs_t` struct + `MOCAP_REGS__*`
  masks) is generated at `vivado/ip_repo/mocap/rdl/rdl_out/sw/mocap_regs.h` by
  `make -C vivado/ip_repo/mocap/rdl`. Do **not** copy it into the recipe. Instead
  place a **relative symlink** in the recipe files dir:
  `recipes-libs/mocap-common/files/mocap_regs.h -> ../../../../../../../vivado/ip_repo/mocap/rdl/rdl_out/sw/mocap_regs.h`
  Bitbake's `file://` follows the symlink when staging. (Prereq: run the RDL
  Makefile once so the target exists; it's a build product, hence gitignored.)
  This is the same generated header used by RTL (`.svh`) and sim — the RDL is the
  one source of truth for the register map.
  NOTE: the existing `isp_regs.h` is a *copied* header (the old pattern). We do
  not repeat that for mocap; leave isp as-is (out of scope) but prefer symlinks
  going forward.
- **Driver headers** `mocap_pipeline.hpp` and `blob_detect.hpp` are new,
  hand-written, and live in exactly one place: `mocap-common/files/`. The app
  consumes them via the staged sysroot include `<mocap/...>` — no second copy.

### 1c. Red boxes = graphics-plane ARGB8888 overlay (chosen)
The video is monochrome (Y8) presented as NV12 on the DP **overlay** plane; the
RGB **primary/graphics** plane is composited *on top* and today is forced
invisible (`alpha = 0`). We repurpose that primary plane as the box overlay:
- Allocate a 1920×1080 **ARGB8888** dumb buffer on the primary plane.
- Each frame: clear to fully transparent (A=0), then draw red (A=255, R=255)
  1–3 px rectangle outlines at each blob's bounding box, scaled from sensor space
  to the video plane's on-screen rect (same letterbox transform the video uses).
- Set the primary plane's global `alpha` to opaque (255) and rely on **per-pixel
  alpha** so only the box pixels occlude the video underneath.

Rationale: non-invasive to the zero-copy NV12 capture path, uses the plane the app
already manages, standard KMS compositing. **Fallback if this DP ignores per-pixel
alpha** (documented in code): draw the boxes into per-slot NV12 chroma (set Y high
+ red UV at outline pixels) instead — deterministic but touches the video buffers.

### 1d. Frame correlation (accepted latency)
The displayed frame (V4L2 capture path) and the blob results (UIO/AXI-Lite path)
are separate. v1 correlates loosely: draw the most-recent blob snapshot over the
most-recent displayed frame (≤1–2 frame skew, invisible for slow marker motion).
`FRAME_ID` is available if tighter correlation is needed later.

---

## 2. Work items

### 2a. `recipes-libs/mocap-common/files/mocap_regs.h` (symlink)
Relative symlink to the generated header (see §1b). Add to `mocap-common.bb`
`SRC_URI` + `do_install` so it stages as `<mocap/mocap_regs.h>`.

### 2b. `recipes-libs/mocap-common/files/mocap_pipeline.hpp` (new)
Header-only `MocapPipeline` (mirror `isp_stats.hpp` style: RAII, `open()` /
`discover(name_substr)`, `volatile mocap_regs_t*` over UIO mmap). API:
- `arm(hres, vres, threshold)` — write HRES/VRES, then CTRL = ENABLE | autoincs |
  THRESHOLD (keep a CTRL shadow; never write pulse bits here).
- `disable()`, `reset()` (CMD.RESET pulse), `set_threshold(t)`.
- `wait_frame(timeout_ms)` — UIO IRQ: `write(fd, &1, 4)` to re-arm, `poll(POLLIN)`,
  `read()` the count. Returns true on frame-done.
- `ack()` — `regs->CMD = RESULTS_ACK` (clear IRQ + release buffer).
- `frame_id()`, `dropped_frames()`, `pixel_sum()`, `read_bank()`, status bits.
- Histogram readback (HIST_ADDR/HIST_DATA autoinc) — port from `isp_stats.hpp`.
- Cycle-counter snapshot (CMD.CYCLE_SNAPSHOT then read LO/HI).
- Register offsets/masks come from `<mocap/mocap_regs.h>` — no magic numbers.

### 2c. `recipes-libs/mocap-common/files/blob_detect.hpp` (new)
Header-only `BlobDetector`, a view over `MocapPipeline`'s registers:
- `struct Blob { uint32_t count, sum_x, sum_y; uint16_t xmin, xmax, ymin, ymax;
   float cx, cy; };` (cx=sum_x/count, cy=sum_y/count).
- `int read_all(std::vector<Blob>&)` — reads `STATUS.BLOB_COUNT`, then for each
  blob sets `BLOB_ADDR` (autoinc off for deterministic field reads, per the TB's
  read_blobs discipline) and reads the 7 descriptor fields. Skips degenerate
  blobs (count==0). Returns count; sets an `overflow()` flag from
  `STATUS.BLOB_OVERFLOW`.
- Must be called between `wait_frame()` and `ack()` (buffer held by HW for SW).

### 2d. `mocap-common.bb`
Add `mocap_regs.h` (symlink), `mocap_pipeline.hpp`, `blob_detect.hpp` to `SRC_URI`
and `do_install`. (isp_* entries stay.)

### 2e. Device tree — `mocap-pipeline-overlay.dts`
The BD now instantiates `mocap_wrapper_0` (replacing `isp_math_wrapper_0`). After
the XSA/`pl.dtsi` regen it carries `mocap_wrapper_0`'s `reg` (0xA0011000) and,
because its `frame_done_irq` is wired to the PS in the BD, an `interrupts`
property. Replace the ISP UIO stanza with:
```dts
&mocap_wrapper_0 {
    compatible = "generic-uio";
    status = "okay";
};
```
This gives an interrupt-capable `/dev/uioN` (blocking `read()` returns on
frame-done). Remove the old `&isp_math_wrapper_0` node. Requires
`CONFIG_UIO_PDRV_GENIRQ=m` (already set). The `&mipi_csirx_out...` re-point to
`&vcap_in` is unchanged (mocap, like the ISP wrapper, is a transparent snooper on
the media graph — CSI still feeds VDMA directly).

### 2f. App — `mocap-hdmi-drm.cpp`
- `#include <mocap/mocap_pipeline.hpp>`, `<mocap/blob_detect.hpp>`.
- Startup: `MocapPipeline::discover("mocap")`; `pipe.arm(capture_w, capture_h,
  threshold)` (threshold from a new `--threshold` arg, default 128).
- Repurpose the primary/graphics plane: allocate a 1920×1080 ARGB8888 dumb FB;
  add prop plumbing to commit it (FB_ID/CRTC_* /SRC_*), set its `alpha` to 255.
- Per displayed frame: `blobs.read_all()` (non-blocking, current published bank)
  then `pipe.ack()`; clear the ARGB buffer; draw red outlines at each blob bbox
  mapped through the video plane's on-screen rect; include the box FB in the
  atomic commit. (Decouple from `wait_frame()` for v1 — poll the latest snapshot
  each display frame; a later rev can align via IRQ/FRAME_ID.)
- Helpers: `draw_rect_outline(argb, W, H, x, y, w, h, thickness, color)` and the
  sensor→screen coordinate transform.

### 2g. `mocap-hdmi-drm.bb`
No new deps (`mocap-common`, `libdrm` already). Confirm the new headers stage via
`mocap-common`.

---

## 3. Test / bring-up

- Build: `petalinux-build -c mocap-common && petalinux-build -c mocap-hdmi-drm`
  (regen `mocap_regs.h` first via the RDL Makefile so the symlink resolves).
- DT: rebuild `-c device-tree` after the new XSA; load the `.dtbo`, confirm
  `/dev/uioN` appears with `cat /sys/class/uio/uio*/name` = mocap_wrapper.
- Run: `mocap-hdmi-drm --threshold N`; verify red boxes track bright markers.
- Correlation/latency and per-pixel-alpha behavior are the two things to eyeball
  on hardware; §1c and §1d list the fallbacks.

## 4. Files touched / created

Created: `mocap-common/files/{mocap_regs.h(symlink), mocap_pipeline.hpp,
blob_detect.hpp}`.
Edited: `mocap-common.bb`, `mocap-pipeline-overlay.dts`, `mocap-hdmi-drm.cpp`,
(maybe) `mocap-hdmi-drm.bb`.
One source of truth: register map = the RDL (`mocap_regs.h` symlinked, not
copied); driver code lives once in `mocap-common`.

---

## 5. Status (implemented) + key findings

Implemented and cross-compiled clean (aarch64, `-O2 -Wall`) against the target
sysroot: `mocap_regs.h` symlink, `mocap_pipeline.hpp`, `blob_detect.hpp`,
`mocap-common.bb`, `mocap-pipeline-overlay.dts`, and the `mocap-hdmi-drm.cpp`
red-box overlay (primary/graphics plane, ARGB8888, per-pixel alpha).

**Critical HW finding — the mocap wrapper gates video on ENABLE.** Unlike the old
`isp_math` snoop, `mocap_wrapper` is INLINE and its passthrough only drains while
the blob core is enabled and framed: `run_extractor.s_ready = (state==RE_ACTIVE)
&& out_empty && enable`, feeding `in_fifo_m_ready`. So **the app must always
`arm()` the block (at the correct HRES/VRES) for any video to reach VDMA** —
`--no-blobs` only suppresses the overlay, it does NOT skip arming. When the mocap
UIO is absent the app assumes a transparent/older bitstream and lets video flow.
The ownership ACK is independent of passthrough (never ack'ing just advances
DROPPED_FRAMES via keep-latest), so video keeps flowing regardless.

**Two things to verify on hardware** (documented fallbacks in code/plan §1c):
per-pixel alpha on the DP graphics plane (else switch to NV12-UV tint), and
blob/display frame correlation (≤1–2 frame skew accepted for v1). Box overlay is
redrawn in place each mocap frame (thin lines → tearing imperceptible); a later
rev can double-buffer + commit the box FB in the video page-flip for exact
alignment.

**CLI added:** `--no-blobs`, `--threshold N` (default 128), `--box-thickness N`
(default 3). The mocap UIO IRQ fd is multiplexed into the existing render
`poll()` set (3rd fd), so blob readback runs at the mocap frame rate without
blocking display.
