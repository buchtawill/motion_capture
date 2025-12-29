import cv2
import time

cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
print("Opened:", cap.isOpened())

time.sleep(1)

cap.release()