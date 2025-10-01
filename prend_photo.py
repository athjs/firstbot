# Script pour prendre des photos et voir un peu ce que voit le webcam


import cv2 as cv

def open_usb_cam(dev="/dev/video0", w=640, h=480, fps=45):
    cap = cv.VideoCapture(dev, cv.CAP_V4L2)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv.CAP_PROP_FPS, fps)
    return cap

cap = open_usb_cam()
if not cap.isOpened():
    print("Cannot open camera")
    raise SystemExit(1)

ok, frame = cap.read()
if not ok:
    print("Can't receive frame (stream end?). Exiting ...")
    raise SystemExit(1)

gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
cv.imwrite("snapshot.jpg", frame)
print("Saved snapshot.jpg")

