# Helper Scripts

Host-side tools for the motion capture FPGA project. Run from the development machine (not on the KV260).

## Prerequisites

```bash
source mocap_env/bin/activate    # Python venv with numpy, matplotlib, opencv-python
```

## Scripts

| Script | Description | Usage |
|--------|-------------|-------|
| `blob_detect_rle_verify.py` | End-to-end RLE blob detection verify + visualize. Generates stimulus, runs Python model, optionally runs xsim RTL, compares outputs, shows 3-panel matplotlib figure (input / model / RTL). | `python blob_detect_rle_verify.py [--width 1280] [--height 800] [--num-blobs 8] [--seed 42] [--no-xsim]` |
| `blob_detect_verify.py` | Same workflow as above but for the grid-based blob detection IP. | `python blob_detect_verify.py [--width 1280] [--height 800] [--num-blobs 5] [--seed 99] [--no-xsim]` |
| `stream_client.py` | TCP client for `mocap-server` on the KV260. Receives frames over MCAP/FRAM binary protocol, displays with OpenCV, reports FPS. | `python stream_client.py [HOST] [PORT]` (default: 10.0.0.100:5001) |
| `camera.py` | OpenCV multi-camera capture with threaded frame grabbing. Measures per-camera and collective FPS. | `python camera.py` (hardcoded indices [1,2,3,4]) |
| `display_image.py` | Displays a raw 1280x800 grayscale image captured from the KV260 using matplotlib. | `python display_image.py` (hardcoded path) |
| `socket_server.py` | Simple TCP echo/logging server on port 5000. | `python socket_server.py [PORT]` |
| `test.py` | Minimal camera connectivity check (open + release). | `python test.py` |

## Blob detection visualization

The `blob_detect_*_verify.py` scripts produce a figure with:
- **Top panel**: input frame (grayscale)
- **Bottom-left**: Python model results (centroids as red X's, bounding boxes in green)
- **Bottom-right**: RTL/xsim results (same markers)

Use `--no-xsim` to skip RTL simulation and only run the Python model (much faster, no Vivado required).
