import cv2
import numpy as np
import math

def detect_lines_by_color(img):
    # Conversion de l'image en espace de couleur HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Plage élargie pour le jaune (en HSV)
    lower_yellow = np.array([15, 40, 80])    # H, S, V min
    upper_yellow = np.array([40, 200, 255])  # H, S, V max
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Plage de couleur pour le bleu (en HSV) 
    lower_blue = np.array([100, 100, 100]) 
    upper_blue = np.array([140, 255, 255]) 
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Plage élargie pour le rouge / rose foncé (#bb5f6e)
    lower_red = np.array([160, 50, 50])
    upper_red = np.array([180, 255, 255])
    red_mask = cv2.inRange(hsv, lower_red, upper_red)

    lower_marron = np.array([115, 5, 70]) # H, S, V min 
    upper_marron = np.array([150, 40, 150]) # H, S, V max 
    marron_mask = cv2.inRange(hsv, lower_marron, upper_marron)


    mask = blue_mask
    #mask = cv2.bitwise_or(yellow_mask, marron_mask)
    #mask = cv2.bitwise_or(mask, red_mask)
    #mask = cv2.bitwise_or(mask, blue_mask)

    # Appliquer le masque à l'image originale
    result = cv2.bitwise_and(img, img, mask=mask)

    # Calcul du centroïde
    M = cv2.moments(mask)  # moments géométriques
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        print(f"Centroïde : ({cX}, {cY})")
        height, width = img.shape[:2]
        centre = width/2
        angle = math.atan2(cX-centre,height-cY)
    else:
        print("Aucun pixel détecté pour calculer le centroïde.")
        angle = None
    
    
    #print(angle*180/math.pi)
    return angle 

# Chargement de l'image
#img = cv2.imread("./photoscotch2.jpg")
#angle = detect_lines_by_color(img)