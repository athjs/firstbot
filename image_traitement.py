import cv2
import numpy as np
import math
from enum import Enum


class Color(Enum):
    Yellow = 1
    Blue = 2
    Red = 3
    Brown = 4


def detect_lines_by_color(img, color):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([15, 40, 80])  # H, S, V min
    upper_yellow = np.array([40, 200, 255])  # H, S, V max
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([140, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    lower_red = np.array([160, 50, 50])
    upper_red = np.array([180, 255, 255])
    red_mask = cv2.inRange(hsv, lower_red, upper_red)
    lower_marron = np.array([115, 5, 70])  # H, S, V min
    upper_marron = np.array([150, 40, 150])  # H, S, V max
    marron_mask = cv2.inRange(hsv, lower_marron, upper_marron)
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
    M = cv2.moments(mask)  # moments géométriques
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
        angle, dY, dX = None, None, None
    return [dX, dY, angle]
