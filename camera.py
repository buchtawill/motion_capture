import cv2
import time
from typing import Optional


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

        print(
            f"Camera {index}: "
            f"{self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}x"
            f"{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)} @ "
            f"{self.cap.get(cv2.CAP_PROP_FPS)} FPS"
        )

    def read(self):
        ret, frame = self.cap.read()
        if ret:
            self.last_frame = frame
        return ret, frame

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
    # print(list_available_cameras())
    # exit()
    
    camera_indices = [1, 2, 3, 4]

    # Open all cameras
    cameras = []
    for idx in camera_indices:
        cam = Camera(idx, fps=100)
        cameras.append(cam)
        # time.sleep(2)

    print("\n--- Measuring FPS individually ---")
    for cam in cameras:
        cam.measure_fps()

    print("\n--- Reading frames from all cameras (ESC to stop) ---")

    frame_count = 0
    t_start = time.perf_counter()
    try:
        while True:
            for cam in cameras:
                ret, frame = cam.read()
                if not ret:
                    print(f"Camera {cam.index} failed to read")
                    return

            frame_count += 1

            if frame_count >= 1000:
                break

    except KeyboardInterrupt:
        pass

    finally:
        t_end = time.perf_counter()
        elapsed = t_end - t_start

        system_fps = frame_count / elapsed if elapsed > 0 else 0.0

        print("\n--- Multi-camera FPS measurement ---")
        print(f"Total frames (per camera): {frame_count}")
        print(f"Elapsed time: {elapsed:.3f} s")
        print(f"Effective system FPS: {system_fps:.2f} Hz")

        for cam in cameras:
            print(f"Camera {cam.index} effective FPS: {system_fps:.2f} Hz")

        for cam in cameras:
            cam.release()



if __name__ == "__main__":
    main()

