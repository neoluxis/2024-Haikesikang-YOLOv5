# Scan QR Code

import cv2 as cv
import time
import pyzbar.pyzbar as pyzbar
from tqdm import tqdm

cap = cv.VideoCapture(2)

cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 400)
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"MJPG"))
cap.set(cv.CAP_PROP_FPS, 240)

frames = []
try:
    while True:
        t0 = time.time()
        ret, frame = cap.read()
        if not ret:
            print("Error: failed to capture image")
            break
        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        frame = cv.resize(frame, (0, 0), fx=0.4, fy=0.4)
        t1 = time.time()
        decoded_objects = pyzbar.decode(frame)
        for obj in decoded_objects:
            cv.rectangle(frame, obj.rect, (0, 255, 0), 2)
            text = obj.data.decode("utf-8")
            cv.putText(frame, text, (obj.rect[0], obj.rect[1]), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            print(f"Detected: {text}")
        print(f"Decoding time: {time.time() - t1:.2f}")
        frames.append(frame)
        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            break
        print(f"FPS: {1 / (time.time() - t0):.2f}")
except KeyboardInterrupt:
    print("Interrupted by user")
cap.release()

# out_fourcc = cv.VideoWriter_fourcc(*"MJPG")
# out = cv.VideoWriter("qrc.avi", out_fourcc, 30, (frame.shape[1], frame.shape[0]))
# for frame in tqdm(frames):
#     frame = cv.cvtColor(frame, cv.COLOR_GRAY2BGR)
#     out.write(frame)
# out.release()
