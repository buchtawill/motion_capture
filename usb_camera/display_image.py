#!/home/will/Desktop/motion_capture/mocap_env/bin/python3

import sys
import numpy as np
from PIL import Image

image_path = "/home/will/Desktop/motion_capture/vitis/kv260_ov9281/mem_dump.bin"

WIDTH  = 1280
HEIGHT = 800

# if len(sys.argv) < 2:
#     print(f"Usage: {sys.argv[0]} <image_file>")
#     sys.exit(1)

with open(image_path, 'rb') as f:
    data = f.read(WIDTH * HEIGHT)

frame = np.frombuffer(data, dtype=np.uint8).reshape((HEIGHT, WIDTH))
Image.fromarray(frame, mode='L').show()
