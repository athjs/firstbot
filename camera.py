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

def detect_red_path(hsv): 
    # Conversion de l'image en espace de couleur HSV
    

    # Plage élargie pour le rouge / rose foncé (#bb5f6e)
    lower_red1 = np.array([  0, 80, 50], dtype=np.uint8)
    upper_red1 = np.array([ 10,255,255], dtype=np.uint8)
    lower_red2 = np.array([160, 80, 50], dtype=np.uint8)
    upper_red2 = np.array([179,255,255], dtype=np.uint8)

    red_mask = cv.bitwise_or(cv.inRange(hsv, lower_red1, upper_red1),
                             cv.inRange(hsv, lower_red2, upper_red2))
    
    contours, _ = cv.findContours(red_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cX = cY = None

    if contours:
        # choisir le contour avec le point (Y) le plus bas
        lowest_y = -1
        chosen = None
        for ctr in contours:
            max_y = ctr[:, 0, 1].max()
            if max_y > lowest_y:
                lowest_y = max_y
                chosen = ctr

        # dessiner le contour choisi + centroïde
        #cv.drawContours(result_img, [chosen], -1, (0, 0, 255), 2)

        M = cv.moments(chosen)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            
    return cX, cY 

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
    
    cX, cY = None, None

    match color:
        case Color.Yellow:
            mask = yellow_mask
        case Color.Blue:
            mask = blue_mask
        case _:
            mask = yellow_mask

    if color == Color.Red: 
        cX, cY = detect_red_path(hsv)
    else : 
        # Appliquer le masque à l'image originale
        result = cv.bitwise_and(img, img, mask=mask)

        # Calcul du centroïde
        M = cv.moments(mask)  # moments géométriques
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            #print(f"Centroïde : ({cX}, {cY})")
        
    if (cX, cY) != (None, None): 
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


# --- Plages MARRON (tolérantes) en HSV OpenCV ---
lower_brown_1 = np.array([  0,   8,  56], dtype=np.uint8)
upper_brown_1 = np.array([  4, 140, 140], dtype=np.uint8)
lower_brown_2 = np.array([147,   8,  56], dtype=np.uint8)
upper_brown_2 = np.array([178, 140, 140], dtype=np.uint8)

def detect_lines_brown(img, seuil=0.35):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    marron_mask = cv.bitwise_or(cv.inRange(hsv, lower_brown_1, upper_brown_1),
                                cv.inRange(hsv, lower_brown_2, upper_brown_2))
    result = cv.bitwise_and(img, img, mask=marron_mask)
    ratio = np.count_nonzero(marron_mask) / marron_mask.size
    if ratio >= seuil:
        print( "Pourcentage de MarronMask : ", ratio)
        return True
    else: 
        return False
    
