#!/usr/bin/env python3
"""gfi_model.py -- bit-exact functional golden for the GFI (Gaussian For Image)
block.

GFI is a streaming 3x3 spatial pre-filter inserted in the mocap blob *snoop*
path only (u_in_fifo -> GFI -> run_extractor); the video passthrough to DRAM and
the histogram/AE snoop stay on RAW luma. Its job is to reject isolated
single-pixel threshold noise before connected-component labeling, without an
area filter (small real markers must survive).

CONTROL (by wire from mocap_top's register block -- GFI has no registers):
  enable  : 1 = filter, 0 = bypass (identity passthrough; pipeline stays active
            with IDENTICAL latency so frame timing never changes).
  strength: selects one of 3 fixed presets (below). 2-bit wire; values 0..2.

The 3 presets are all 3x3, normalized by 16 (>>4), integer weights only
(shift-add in HW -- no multiplier, no divide), ordered by increasing blur:

  0 LIGHT  : center 0.75, gentle -- best small-marker preservation
        [ 0  1  0 ]
        [ 1 12  1 ] / 16
        [ 0  1  0 ]
  1 MEDIUM : center 0.25, the standard 3x3 Gaussian
        [ 1  2  1 ]
        [ 2  4  2 ] / 16
        [ 1  2  1 ]
  2 STRONG : center 0.00, 8-neighbour mean -- most aggressive spike removal
        [ 2  2  2 ]           (the pixel's own value is ignored; an isolated
        [ 2  0  2 ] / 16       spike is fully replaced by its neighbours)
        [ 2  2  2 ]

ROUNDING: round-to-nearest: out = (weighted_sum + 8) >> 4. The RTL must add the
same +8 before the >>4.

BORDER: replicate (edge clamp) -- the top/bottom rows and left/right columns are
mirrored outward, which is what a HW line-buffer edge-replicate does at frame
and row boundaries. Each frame's filtering is independent (line buffers reset on
SOF), so no data bleeds between frames.

The output is a filtered *pixel* image (0..255); run_extractor thresholds it
downstream exactly as before (fg = pixel >= threshold), so GFI does not change
the detector's interface -- only what it sees.
"""
import numpy as np

# Preset kernels. All sum to 16 -> normalize by >>4. Index == `strength` wire.
PRESETS = {
    0: np.array([[0, 1, 0], [1, 12, 1], [0, 1, 0]], dtype=np.int32),  # light
    1: np.array([[1, 2, 1], [2, 4, 2], [1, 2, 1]], dtype=np.int32),   # medium
    2: np.array([[2, 2, 2], [2, 0, 2], [2, 2, 2]], dtype=np.int32),   # strong
}
NORM_SHIFT = 4                       # every preset sums to 16
ROUND_BIAS = 1 << (NORM_SHIFT - 1)   # +8, round-to-nearest


def gfi_filter(img, strength, enable=True):
    """Bit-exact GFI model.

    img      : (H, W) uint8 luma.
    strength : 0..2 preset index (ignored when enable is False).
    enable   : False -> identity passthrough (bypass).
    Returns  : (H, W) uint8, same convention as the RTL output.
    """
    img = np.asarray(img)
    if not enable:
        return img.astype(np.uint8, copy=True)
    if strength not in PRESETS:
        raise ValueError(f"strength must be 0..2, got {strength}")

    k = PRESETS[strength]
    H, W = img.shape
    # Replicate-padded so every output pixel has a full 3x3 window.
    p = np.pad(img.astype(np.int32), 1, mode="edge")

    acc = np.zeros((H, W), dtype=np.int32)
    for dy in range(3):
        for dx in range(3):
            w = int(k[dy, dx])
            if w:
                acc += w * p[dy:dy + H, dx:dx + W]

    out = (acc + ROUND_BIAS) >> NORM_SHIFT
    return out.astype(np.uint8)


def preset_name(strength):
    return {0: "light", 1: "medium", 2: "strong"}.get(strength, f"strength{strength}")


if __name__ == "__main__":
    # Tiny self-check: a lone spike is removed; a solid region is preserved.
    im = np.full((5, 5), 150, np.uint8)
    im[2, 2] = 202                      # isolated weak spike (like the real noise)
    for s in (0, 1, 2):
        print(f"strength {s} ({preset_name(s)}): center {150} spike 202 -> "
              f"{gfi_filter(im, s)[2, 2]}")
    solid = np.full((5, 5), 255, np.uint8)
    for s in (0, 1, 2):
        assert gfi_filter(solid, s)[2, 2] == 255, "solid region must stay 255"
    print("solid-region preservation: OK")
