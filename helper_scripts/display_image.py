#!/home/will/Desktop/motion_capture/mocap_env/bin/python3

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

image_path = "/home/will/Desktop/motion_capture/linux/kv260_ov9281_plnx/image_capture.raw"

# os.system('cd ../vitis/kv260_ov9281 && make dump_image')

WIDTH  = 1280
HEIGHT = 800

# if len(sys.argv) < 2:
#     print(f"Usage: {sys.argv[0]} <image_file>")
#     sys.exit(1)

with open(image_path, 'rb') as f:
    data = f.read(WIDTH * HEIGHT)

frame = np.frombuffer(data, dtype=np.uint8).reshape((HEIGHT, WIDTH))
plt.imshow(frame, cmap='gray', vmin=0, vmax=255)
plt.axis('off')
plt.show()
