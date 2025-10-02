import math
import time
import numpy as np
import pypot.dynamixel
# import kynematic as ky  # si tu veux réutiliser, mais fonctions ci-dessous sont suffisantes

# Paramètres
WHEEL_RADIUS = 0.025
WHEEL_BASE = 0.185

# mapping moteur -> sens physique (ajuste expérimentalement)
# si lors du test le robot tourne dans le mauvais sens, inversez -1 <-> 1
MOTOR_SIGN = {1: -1, 2: 1}  # ex: 1 = roue droite (doit être -1 pour ton montage), 2 = roue gauche

# conversion DXL <-> rad/s (doc: 0.916 rpm par unité)
def dxl_speed_to_rad_s(value):
    if value == 0 or value == 1024:
        return 0.0
    if value < 1024:  # CCW (on considère positif)
        rpm = value * 0.916
        return (rpm * 2 * math.pi) / 60.0
    else:  # CW -> négatif
        mag = value - 1024
        rpm = mag * 0.916
        return - (rpm * 2 * math.pi) / 60.0

def rad_s_to_dxl_speed(rad_s, max_dxl=1023):
    rpm = abs(rad_s) * 60.0 / (2 * math.pi)
    value = int(rpm / 0.916)
    if value > max_dxl:
        value = max_dxl
    return value if rad_s >= 0 else 1024 + value

# cinématique
def inverse_kinematics(v, w, R=WHEEL_RADIUS, W=WHEEL_BASE):
    v_r = v + (w * W) / 2.0
    v_l = v - (w * W) / 2.0
    Vd = v_r / R
    Vg = v_l / R
    return Vd, Vg

def direct_kinematics(Vd, Vg, R=WHEEL_RADIUS, W=WHEEL_BASE):
    v_r = Vd * R
    v_l = Vg * R
    v = (v_r + v_l) / 2.0
    w = (v_r - v_l) / W
    return v, w

# go_to corrigé : rotate -> translate -> rotate
def go_to(x_target, y_target, theta_target, v_base=0.15, w_base=0.8):
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        raise RuntimeError("No Dynamixel port found")
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1, 2])

    try:
        # 1) rotation initiale pour viser la cible (alpha)
        x_rest = x_target
        y_rest = y_target
        dist_rest = math.hypot(x_rest, y_rest)
        alpha = math.atan2(y_rest, x_rest)   # angle cible relatif (rad)

        if abs(alpha) > 1e-3:
            # sens de rotation: si alpha > 0 => tourner à gauche (CCW)
            w_cmd = w_base if alpha > 0 else -w_base
            Vd, Vg = inverse_kinematics(0.0, w_cmd)
            raw_d = rad_s_to_dxl_speed(Vd)
            raw_g = rad_s_to_dxl_speed(Vg)

            # appliquer la correction de signe physique par moteur
            cmd = {
                1: MOTOR_SIGN[1] * raw_d,
                2: MOTOR_SIGN[2] * raw_g
            }
            dxl_io.set_moving_speed(cmd)
            time.sleep(abs(alpha) / abs(w_cmd))
            dxl_io.set_moving_speed({1: 0, 2: 0})

        # 2) translation
        if dist_rest > 1e-4:
            v_cmd = v_base
            Vd, Vg = inverse_kinematics(v_cmd, 0.0)
            raw_d = rad_s_to_dxl_speed(Vd)
            raw_g = rad_s_to_dxl_speed(Vg)
            cmd = {1: MOTOR_SIGN[1] * raw_d, 2: MOTOR_SIGN[2] * raw_g}
            dxl_io.set_moving_speed(cmd)
            time.sleep(dist_rest / v_cmd)
            dxl_io.set_moving_speed({1: 0, 2: 0})

        # 3) rotation finale pour atteindre orientation voulue (relative)
        # note: theta_target here is final orientation relative to start.
        # after step1 we have turned by alpha, so remaining rotation = theta_target - alpha
        dtheta = theta_target - alpha
        if abs(dtheta) > 1e-3:
            w_cmd = w_base if dtheta > 0 else -w_base
            Vd, Vg = inverse_kinematics(0.0, w_cmd)
            raw_d = rad_s_to_dxl_speed(Vd)
            raw_g = rad_s_to_dxl_speed(Vg)
            cmd = {1: MOTOR_SIGN[1] * raw_d, 2: MOTOR_SIGN[2] * raw_g}
            dxl_io.set_moving_speed(cmd)
            time.sleep(abs(dtheta) / abs(w_cmd))
            dxl_io.set_moving_speed({1: 0, 2: 0})

    finally:
        dxl_io.set_moving_speed({1: 0, 2: 0})
go_to(-1,-1,np.pi/4)
