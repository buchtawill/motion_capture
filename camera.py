import cv2
import time

def list_cameras(max_devices=10, backend=cv2.CAP_DSHOW):
    available = []
    for i in range(max_devices):
        cap = cv2.VideoCapture(i, backend)
        if cap.isOpened():
            available.append(i)
            cap.release()
    return available

def measure_fps(cap: cv2.VideoCapture):
    t1 = time.perf_counter()
    count = 0
    while count < 300:
        ret, frame = cap.read()
        if not ret:
            break
        count += 1
    elapsed = time.perf_counter() - t1
    print(f"INFO [measure_fps] Camera frame rate: {count / elapsed}")

if __name__ == '__main__':
    cams = list_cameras()
    print("Available camera indices:", cams)

    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)  # Windows: DirectShow backend

    if not cap.isOpened():
        raise RuntimeError("Could not open webcam")
    
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual mode (backend-dependent)
    cap.set(cv2.CAP_PROP_EXPOSURE, -5)         # Short exposure (log scale)
    cap.set(cv2.CAP_PROP_GAIN, 0)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 120)
    
    print("Width:", cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    print("Height:", cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("FPS:", cap.get(cv2.CAP_PROP_FPS))
    
    measure_fps(cap)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        cv2.imshow("Webcam", frame)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
            break

    


    cap.release()
    cv2.destroyAllWindows()

