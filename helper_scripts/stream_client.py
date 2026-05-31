#!/usr/bin/env python3
"""
stream_client.py — receive and display frames from mocap-server.

Usage:
    python stream_client.py [HOST] [PORT]
    python stream_client.py 10.0.0.100        # default port 5001
    python stream_client.py 10.0.0.100 5001

Displays frames with OpenCV if available, otherwise prints per-frame stats.
Press 'q' to quit (OpenCV mode) or Ctrl-C.
"""

import socket
import struct
import sys
import time

STREAM_HDR = struct.Struct("<IIIIIIII")  # 32 bytes
FRAME_HDR = struct.Struct("<IIIIII")     # 24 bytes

STREAM_MAGIC = ord("M") | (ord("C") << 8) | (ord("A") << 16) | (ord("P") << 24)
FRAME_MAGIC = ord("F") | (ord("R") << 8) | (ord("A") << 16) | (ord("M") << 24)


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("connection closed")
        buf.extend(chunk)
    return bytes(buf)


def fourcc_str(v: int) -> str:
    return "".join(chr((v >> (8 * i)) & 0xFF) for i in range(4))


def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "10.0.0.100"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 5001

    try:
        import cv2
        import numpy as np
        have_cv2 = True
    except ImportError:
        have_cv2 = False
        print("OpenCV not found — printing stats only")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print(f"Connected to {host}:{port}")

    # --- stream header -------------------------------------------------------
    raw = recv_exact(sock, STREAM_HDR.size)
    magic, ver, w, h, pixfmt, bpl, frame_sz, _ = STREAM_HDR.unpack(raw)
    if magic != STREAM_MAGIC:
        print(f"Bad stream magic: 0x{magic:08X}")
        return 1
    print(f"Stream: {w}x{h} '{fourcc_str(pixfmt)}' v{ver}, "
          f"{frame_sz} bytes/frame, bytesperline={bpl}")

    # --- frame loop ----------------------------------------------------------
    count = 0
    t_start = time.monotonic()
    t_last_report = t_start
    frames_since_report = 0

    try:
        while True:
            raw = recv_exact(sock, FRAME_HDR.size)
            fmagic, seq, sz, ts_s, ts_us, _ = FRAME_HDR.unpack(raw)
            if fmagic != FRAME_MAGIC:
                print(f"Bad frame magic: 0x{fmagic:08X}")
                break
            frame_data = recv_exact(sock, sz)

            count += 1
            frames_since_report += 1
            now = time.monotonic()

            if now - t_last_report >= 1.0:
                fps = frames_since_report / (now - t_last_report)
                print(f"  seq={seq}  {fps:.1f} fps  "
                      f"ts={ts_s}.{ts_us:06d}  {sz} bytes")
                t_last_report = now
                frames_since_report = 0

            if have_cv2:
                img = np.frombuffer(frame_data, dtype=np.uint8)
                img = img.reshape((h, bpl))[:, :w]
                cv2.imshow("mocap-server", img)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    except (ConnectionError, KeyboardInterrupt):
        pass

    elapsed = time.monotonic() - t_start
    if count and elapsed > 0:
        print(f"\n{count} frames in {elapsed:.1f}s = {count / elapsed:.1f} fps")

    sock.close()
    if have_cv2:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
