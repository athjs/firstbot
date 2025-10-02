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

# --- paramètres robot ---
WHEEL_RADIUS = 0.025   # m (rayon roue = 2.5 cm)
WHEEL_BASE   = 0.185   # m (distance entre roues = 18.5 cm)
MAX_V = 0.25           # m/s (avance max)
MAX_W = 2.0            # rad/s (rotation max)


# --- initialisation caméra ---
def open_usb_cam(dev="/dev/video0", w=640, h=480, fps=45):
    cap = cv.VideoCapture(dev, cv.CAP_V4L2)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv.CAP_PROP_FPS, fps)
    return cap

cap = open_usb_cam()
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# --- initialisation Dynamixel ---
ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No Dynamixel port found')
dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])   # active wheel mode sur les deux

last_angle = 0
color_choice = Color.Yellow

try:
    last_brown_detection_time = 0 
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        #détection du marron pour changement de couleur 
        brown_presence = detect_lines_brown(frame)
        print(color_choice)
        #print(brown_presence)
        if brown_presence :
            brown_detection_time = time.time()
            
            if brown_detection_time - last_brown_detection_time > 10: 
                color_choice = Color(color_choice.value + 1)    
                print("Marron détecté à", brown_detection_time, ", Passage au : ", color_choice)
            
            last_brown_detection_time = brown_detection_time

        # détection ligne -> [dX, dY, angle]
        result = detect_lines_by_color(frame, color_choice, last_angle)
        if result is None:
            continue
        dX, dY, angle = result
        if angle is None:
            continue

        # génération commande (v, w) à partir de l’angle
        v, w = point_direction(angle, v_base=0.20, ka=0.8)

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

        time.sleep(0.01)

except KeyboardInterrupt:
    print("Ctrl+C détecté, arrêt…")

except Exception as e:
    print(f"Erreur: {e}")

finally:
    cap.release()
    cv.destroyAllWindows()
    dxl_io.set_moving_speed({1: 0, 2: 0})
