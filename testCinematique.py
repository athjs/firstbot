import numpy as np
import cv2 as cv
import time
import pypot.dynamixel
import sys
import tty
import termios
import math
from camera import detect_lines_by_color
from camera import detect_lines_brown
from camera import Color
from kynematic import *

# codes couleur pour l'affichage
ANSI_CODE = {
    Color.Yellow: "\033[33m",
    Color.Blue:   "\033[34m",
    Color.Red:    "\033[31m",
}
RESET = "\033[0m"

# initialisation caméra 
def open_usb_cam(dev="/dev/video0", w=640, h=480, fps=30):
    cap = cv.VideoCapture(dev, cv.CAP_V4L2)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv.CAP_PROP_FPS, fps)
    return cap

# ouverture de la cam
cap = open_usb_cam()
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# initialisation Dynamixel 
ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No Dynamixel port found')
dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])   # active wheel mode sur les deux

# init des variables de la boucle
last_angle = 0
color_choice = Color.Yellow
color_code = ANSI_CODE[color_choice]
try:
    last_brown_detection_time = time.time() 
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        #détection du marron pour changement de couleur 
        print(color_code, color_choice, RESET)
        # Si t'as pas detecté de marron depuis + de 20s, teste si y'a du marron sur la frame. 
        if time.time() - last_brown_detection_time > 10:
            if detect_lines_brown(img=frame, seuil=0.10):
                brown_detection_time = time.time()

                color_choice = Color(color_choice.value + 1)    
                color_code = ANSI_CODE[color_choice]
                print("\033[38;2;255;165;0mTexte orange\033[0m","Marron détecté à", brown_detection_time, color_code, " \nPassage au : ", color_choice, )

                last_brown_detection_time = brown_detection_time
            else :
                print ("Pas de Marron detecté")

        # détection ligne -> [dX, dY, angle]
        result = detect_lines_by_color(frame, color_choice, last_angle)
        if result is None:
            continue
        dX, dY, angle = result
        if angle is None:
            continue
        
        if color_choice == Color.Red :
            v_base = 0.1
            ka = 1.2
        elif color_choice == Color.Blue :
            v_base = 0.19
            ka = 0.95
        else: 
            v_base = 0.22
            ka = 0.8

        # génération commande (v, w) à partir de l’angle
        v, w = point_direction(angle, v_base, ka)

        # cinématique inverse -> vitesses roues (rad/s)
        Vd, Vg = inverse_kinematics(v, w)

        # cinématique directe -> estimation (debug)
        v_est, w_est = direct_kinematics(Vd, Vg)

        # conversion Dynamixel
        speed_d = rad_s_to_dxl_speed(Vd)
        speed_g = rad_s_to_dxl_speed(Vg)

        #print(f"alpha={angle:.3f} rad | cmd: v={v:.2f}, w={w:.2f} "
         #     f"| roues: Vd={Vd:.2f}, Vg={Vg:.2f} rad/s "
          #    f"| estimé: v={v_est:.2f}, w={w_est:.2f}")

        # appliquer aux moteurs
        dxl_io.set_moving_speed({
            1: -speed_d,  # moteur gauche
            2:  speed_g  # moteur droit
        })

        last_angle=angle

        #time.sleep(0.01)

except KeyboardInterrupt:
    print("Ctrl+C détecté, arrêt…")

except Exception as e:
    print(f"Erreur: {e}")

finally:
    cap.release()
    cv.destroyAllWindows()
    dxl_io.set_moving_speed({1: 0, 2: 0})
