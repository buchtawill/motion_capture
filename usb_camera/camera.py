import cv2
import threading
import queue
import time
from typing import Optional
import numpy as np
import os


class Camera:
    def __init__(
        self,
        index: int,
        backend=cv2.CAP_DSHOW,
        width:int=640,
        height:int=480,
        fps:int=120,
        exposure:int=-5,
        gain:int=0,
        autofocus:bool=False,
    ):
        self.index = index
        self.backend = backend
        self.cap = cv2.VideoCapture(index, backend)

        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera {index}")

        # Camera configuration
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # manual
        self.cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
        self.cap.set(cv2.CAP_PROP_GAIN, gain)
        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 1 if autofocus else 0)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        self.last_frame: Optional[cv2.Mat] = None
        
        self.queue = queue.Queue(maxsize=1)
        self.running = False
        self.thread = None

        print(
            f"Camera {index}: "
            f"{self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x"
            f"{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} @ "
            f"{self.cap.get(cv2.CAP_PROP_FPS)} FPS"
        )

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _capture_loop(self):
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.last_frame = frame
                # Only keep the latest frame in the queue
                if self.queue.full():
                    try:
                        self.queue.get_nowait()
                    except queue.Empty:
                        pass
                self.queue.put(frame)

    def read(self, wait_for_frame=False):
        """Get the latest frame."""
        if wait_for_frame:
            frame = self.queue.get()  # blocks until a frame is available
            self.last_frame = frame
            return True, frame
        try:
            frame = self.queue.get_nowait()
            return True, frame
        except queue.Empty:
            return False, self.last_frame

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join()
        self.cap.release()

    def measure_fps(self, num_frames=300):
        t0 = time.perf_counter()
        count = 0

        while count < num_frames:
            ret, _ = self.cap.read()
            if not ret:
                break
            count += 1

        elapsed = time.perf_counter() - t0
        fps = count / elapsed if elapsed > 0 else 0.0
        print(f"INFO [Camera {self.index}] Measured FPS: {fps:.2f}")
        return fps

    def release(self):
        self.cap.release()

def list_available_cameras(max_devices=10, backend=cv2.CAP_DSHOW)->list:
    available = []
    for i in range(max_devices):
        cap = cv2.VideoCapture(i, backend)
        if cap.isOpened():
            available.append(i)
            cap.release()
    return available

def main():
    camera_indices = [1, 2, 3, 4]
    cameras = [Camera(idx, fps=120) for idx in camera_indices]

    for cam in cameras:
        cam.measure_fps()

    # Start all camera threads
    for cam in cameras:
        cam.start()

    print("INFO [camera.py] Measuring collective frame rate")

    frame_count = 0
    last_frames = {}  # index -> last valid frame

    t_start = time.perf_counter()
    try:
        while frame_count < 2000:
            for cam in cameras:
                ret, frame = cam.read(wait_for_frame=True)
                if not ret:
                    print(f"Camera {cam.index} has no frame yet")
                    continue

                last_frames[cam.index] = frame

            frame_count += 1

    except KeyboardInterrupt:
        pass
    finally:
        t_end = time.perf_counter()
        elapsed = t_end - t_start
        system_fps = frame_count / elapsed if elapsed > 0 else 0.0

        print(f"Captured {frame_count} frames in {elapsed:.3f}s -> {system_fps:.2f} FPS")

        # Ensure output directory exists
        os.makedirs("pics", exist_ok=True)

        # Save last frame from each camera
        for cam_idx, frame in last_frames.items():
            out_path = f"pics/last_frame_{cam_idx}.png"
            cv2.imwrite(out_path, frame)
            print(f"Saved {out_path}")

        for cam in cameras:
            cam.stop()

if __name__ == "__main__":
    main()