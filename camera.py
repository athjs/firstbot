import cv2 as cv
import numpy as np
import math
from enum import Enum


class Color(Enum):
    Yellow = 1
    Blue = 2
    Red = 3
    Brown = 4


def open_usb_cam(dev="/dev/video0", w=640, h=480, fps=45):
    cap = cv.VideoCapture(dev, cv.CAP_V4L2)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv.CAP_PROP_FPS, fps)
    return cap

def detect_lines_by_color(img, color, last_angle):
    # Conversion de l'image en espace de couleur HSV
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Plage élargie pour le jaune (en HSV)
    lower_yellow = np.array([15, 40, 80])  # H, S, V min
    upper_yellow = np.array([40, 200, 255])  # H, S, V max
    yellow_mask = cv.inRange(hsv, lower_yellow, upper_yellow)

    # Plage de couleur pour le bleu (en HSV)
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([140, 255, 255])
    blue_mask = cv.inRange(hsv, lower_blue, upper_blue)

    # Plage élargie pour le rouge / rose foncé (#bb5f6e)
    lower_red = np.array([160, 50, 50])
    upper_red = np.array([180, 255, 255])
    red_mask = cv.inRange(hsv, lower_red, upper_red)

    lower_marron = np.array([115, 5, 70])  # H, S, V min
    upper_marron = np.array([150, 40, 150])  # H, S, V max
    marron_mask = cv.inRange(hsv, lower_marron, upper_marron)

    match color:
        case Color.Yellow:
            mask = yellow_mask
        case Color.Blue:
            mask = blue_mask
        case Color.Red:
            mask = red_mask
        case Color.Brown:
            mask = marron_mask
        case _:
            mask = yellow_mask

    # Appliquer le masque à l'image originale
    result = cv.bitwise_and(img, img, mask=mask)

    # Calcul du centroïde
    M = cv.moments(mask)  # moments géométriques
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        print(f"Centroïde : ({cX}, {cY})")
        height, width = img.shape[:2]
        centre = width / 2
        angle = math.atan2(cX - centre, height - cY)
        dX = cX - centre
        dY = height - cY

    else:
        print("Aucun pixel détecté pour calculer le centroïde.")
        angle, dY, dX = last_angle, None, None

    # print(angle*180/math.pi)
    return [dX, dY, angle]
