import math
import numpy as np
import pypot.dynamixel
import kynematic as ky   # tes fonctions cinématiques
import time as time

# -------------------------------
# Paramètres robot
# -------------------------------
WHEEL_RADIUS = 0.025   # m (rayon roue = 2.5 cm)
WHEEL_BASE   = 0.185   # m (distance entre roues = 18.5 cm)
MAX_V = 0.25           # m/s (avance max)
MAX_W = 2.0            # rad/s (rotation max)


# -------------------------------
# Conversion vitesses Dynamixel
# -------------------------------
def dxl_speed_to_rad_s(value):
    """
    Convertit la valeur renvoyée par Dynamixel (get_present_speed)
    en rad/s. (positif = anti-horaire, négatif = horaire)
    """
    direction = 1
    if value > 1023:  # bit de signe
        value = value - 1024
        direction = -1
    
    rpm = value * 0.111            # en tours/min
    rad_s = (rpm * 2 * math.pi) / 60.0
    return direction * rad_s


def rad_s_to_dxl_speed(rad_s, max_lin_speed=0.7, wheel_radius=WHEEL_RADIUS, max_dxl=1023):
    """
    Conversion rad/s -> unité Dynamixel (approximation linéaire).
    """
    max_rad_s = max_lin_speed / wheel_radius
    value = int((abs(rad_s) / max_rad_s) * max_dxl)
    if value > max_dxl:
        value = max_dxl
    
    # encode direction : 0-1023 sens horaire, 1024-2047 sens antihoraire
    if rad_s >= 0:
        return value
    else:
        return 1024 + value



# -------------------------------
# Odométrie
# -------------------------------
def odom(v, w, dt):
    if abs(w) < 1e-6:  # Cas rectiligne
        dx = v * dt
        dy = 0.0
        dtheta = 0.0
    else:  # Cas circulaire
        dx = (v / w) * math.sin(w * dt)
        dy = (v / w) * (1 - math.cos(w * dt))
        dtheta = w * dt
    return dx, dy, dtheta


def tick_odom(x, y, theta, v, w, dt):
    dx_r, dy_r, dtheta = odom(v, w, dt)
    dx_g = dx_r * math.cos(theta) - dy_r * math.sin(theta)
    dy_g = dx_r * math.sin(theta) + dy_r * math.cos(theta)
    return x + dx_g, y + dy_g, theta + dtheta


# -------------------------------
# Fonctions cinématiques
# -------------------------------
def direct_kinematics(Vd, Vg, R=WHEEL_RADIUS, W=WHEEL_BASE):
    v_r = Vd * R
    v_l = Vg * R
    v = (v_r + v_l) / 2.0
    w = (v_r - v_l) / W
    return v, w


def inverse_kinematics(v, w, R=WHEEL_RADIUS, W=WHEEL_BASE):
    v_r = v + (w * W) / 2.0
    v_l = v - (w * W) / 2.0
    Vd = v_r / R
    Vg = v_l / R
    return Vd, Vg


def point_direction(alpha, v_base=0.15, ka=0.8):
    v = v_base
    w = -ka * alpha   # correction proportionnelle à l'angle
    return v, w


# -------------------------------
# Fonction go_to principale
# -------------------------------
def go_to(x_target, y_target, theta_target,
          x=0.0, y=0.0, theta=0.0,
          dt=0.1, tol_pos=0.01, tol_theta=0.05):
    """
    Fait aller le robot de (x,y,theta) vers (x_target, y_target, theta_target)
    Les coordonnées (x,y) sont exprimées dans le repère monde :
      - x : vers l'avant du robot
      - y : vers la gauche du robot
    """

    path = []

    # --- initialisation Dynamixel ---
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        exit('No Dynamixel port found')
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1, 2])

    # orientation finale absolue (relative à theta initial)
    theta_goal = theta + theta_target

    while True:
        # vecteur vers la cible
        x_rest = x_target - x
        y_rest = y_target - y
        dist_rest = math.hypot(x_rest, y_rest)

        # angle relatif vers la cible
        alpha = math.atan2(y_rest, x_rest) - theta
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        # erreur d'orientation
        dtheta = (theta_goal - theta + math.pi) % (2 * math.pi) - math.pi

        # condition d’arrêt
        if dist_rest < tol_pos and abs(dtheta) < tol_theta:
            break

        # commande vitesse (v, w)
        v, w = point_direction(alpha)

        # ralentissement quand proche
        if dist_rest < 0.2:
            v *= dist_rest / 0.2

        # cinématique inverse
        Vd, Vg = inverse_kinematics(v, w)

        # conversion Dynamixel
        speed_d = rad_s_to_dxl_speed(Vd)
        speed_g = rad_s_to_dxl_speed(Vg)

        # appliquer aux moteurs
        dxl_io.set_moving_speed({
            1: -speed_d,   # roue droite
            2: speed_g    # roue gauche
        })

        # vitesses réelles
        Vd_real , Vg_real = dxl_io.get_present_speed({1,2})

        # mise à jour odométrie
        v_real, w_real = direct_kinematics(Vd_real, Vg_real)
        x, y, theta = tick_odom(x, y, theta, v_real, w_real, dt)

        path.append((x, y, theta))

    # arrêt moteur
    dxl_io.set_moving_speed({1: 0, 2: 0})

    return x, y, theta, path


def odometry(x=0.0, y=0.0, theta=0.0, dt=0.1, duration=10.0):
    """
    Mesure la position finale du robot en utilisant l’odométrie,
    en le déplaçant à la main (moteurs en roue libre).
    
    - (x, y, theta) : position initiale
    - dt : pas de temps pour mise à jour
    - duration : temps total d’échantillonnage (s)
    
    Retourne : (x, y, theta, path)
    """
    path = []

    # --- initialisation Dynamixel ---
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        exit('No Dynamixel port found')
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1, 2])   # mode roue (free wheeling si pas de commande)

    t = 0.0
    while t < duration:
        # lire vitesses roues en rad/s
        Vd_real, Vg_real = dxl_io.get_present_speed(dxl_io, [1, 2])

        # convertir en (v, w)
        v_real, w_real = direct_kinematics(Vd_real, Vg_real)

        # mise à jour odométrie
        x, y, theta = tick_odom(x, y, theta, v_real, w_real, dt)

        path.append((x, y, theta))

        time.sleep(dt)
        t += dt

    return x, y, theta, path
