import numpy as np
import cv2 as cv
import time
import pypot.dynamixel
import sys
import tty
import termios
import math
from camera import detect_lines_by_color
from camera import Color

# --- paramètres robot ---
WHEEL_RADIUS = 0.025   # m (rayon roue = 2.5 cm)
WHEEL_BASE   = 0.185   # m (distance entre roues = 18.5 cm)
MAX_V = 0.25           # m/s (avance max)
MAX_W = 2.0            # rad/s (rotation max)

# --- Fonctions cinématiques ---
def direct_kinematics(Vd, Vg, R=WHEEL_RADIUS, W=WHEEL_BASE):
    """
    Vd, Vg : vitesses roues (rad/s)
    Retourne (v, w) vitesse robot
    """
    v_r = Vd * R
    v_l = Vg * R
    v = (v_r + v_l) / 2.0
    w = (v_r - v_l) / W
    return v, w

def inverse_kinematics(v, w, R=WHEEL_RADIUS, W=WHEEL_BASE):
    """
    v, w : vitesse linéaire (m/s), vitesse angulaire (rad/s)
    Retourne (Vd, Vg) en rad/s
    """
    v_r = v + (w * W) / 2.0
    v_l = v - (w * W) / 2.0
    Vd = v_r / R
    Vg = v_l / R
    return Vd, Vg

def point_direction(alpha, v_base=0.15, ka=0.8, max_v=MAX_V, max_w=MAX_W):
    """
    alpha : angle (rad) entre -pi/2 et pi/2
    v_base : vitesse de base en ligne droite (m/s)
    ka : gain de correction angulaire
    """
    v = v_base
    w = -ka * alpha   # correction proportionnelle à l'angle
    return v, w

# --- conversion Dynamixel ---
def rad_s_to_dxl_speed(rad_s, wheel_radius=WHEEL_RADIUS, max_lin_speed=0.7, max_dxl=1023):
    """
    Conversion rad/s -> unité Dynamixel (approximation linéaire).
    max_lin_speed : vitesse linéaire roue max en m/s
    """
    max_rad_s = max_lin_speed / wheel_radius
    return int((rad_s / max_rad_s) * max_dxl)

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

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # détection ligne -> [dX, dY, angle]
        result = detect_lines_by_color(frame, Color.Blue, last_angle)
        if result is None:
            continue
        dX, dY, angle = result
        if angle is None:
            continue

        # génération commande (v, w) à partir de l’angle
        v, w = point_direction(angle, v_base=0.15, ka=0.8)

        # cinématique inverse -> vitesses roues (rad/s)
        Vd, Vg = inverse_kinematics(v, w)

        # cinématique directe -> estimation (debug)
        v_est, w_est = direct_kinematics(Vd, Vg)

        # conversion Dynamixel
        speed_d = rad_s_to_dxl_speed(Vd)
        speed_g = rad_s_to_dxl_speed(Vg)

        print(f"alpha={angle:.3f} rad | cmd: v={v:.2f}, w={w:.2f} "
              f"| roues: Vd={Vd:.2f}, Vg={Vg:.2f} rad/s "
              f"| estimé: v={v_est:.2f}, w={w_est:.2f}")

        # appliquer aux moteurs
        dxl_io.set_moving_speed({
            1: -speed_g,  # moteur gauche
            2:  speed_d   # moteur droit
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
