import math
import time
import numpy as np
import pypot.dynamixel

# ===============================
# Paramètres robot
# ===============================
WHEEL_RADIUS = 0.025   # m
WHEEL_BASE   = 0.185   # m

# 1 = roue droite, 2 = roue gauche
# Moteur 1 inversé mécaniquement
WHEEL_SIGN = {1: -1, 2: +1}

# Vitesses de commande (plus rapide qu'avant)
V_LIN  = 1.5   # m/s  (translation)
W_TURN = 2 * np.pi   # rad/s (rotation sur place)

# ===============================
# Conversions Dynamixel
# ===============================
def dxl_speed_to_rad_s(value):
    rpm = value * 0.916
    return (rpm * 2.0 * math.pi) / 60.0

def rad_s_to_dxl_ticks(rad_s, max_ticks=1023):
    rpm = abs(rad_s) * 60.0 / (2.0 * math.pi)
    ticks = int(rpm / 0.916)
    return max(0, min(max_ticks, ticks))

# ===============================
# Cinématique diff
# ===============================
def direct_kinematics(omega_r, omega_l, R=WHEEL_RADIUS, W=WHEEL_BASE):
    v_r = omega_r * R
    v_l = omega_l * R
    v = (v_r + v_l) / 2.0
    w = (v_r - v_l) / W
    return v, w

def inverse_kinematics(v, w, R=WHEEL_RADIUS, W=WHEEL_BASE):
    v_r = v + (w * W) / 2.0
    v_l = v - (w * W) / 2.0
    return v_r / R, v_l / R

# ===============================
# Helpers
# ===============================
def sgn(x): return -1.0 if x < 0 else (1.0 if x > 0 else 0.0)
def wrap_pi(a): return (a + math.pi) % (2.0 * math.pi) - math.pi

def wheel_rad_s_to_signed_ticks(motor_id, wheel_rad_s):
    amp = rad_s_to_dxl_ticks(wheel_rad_s)
    motor_sign = -1 if wheel_rad_s < 0 else (1 if wheel_rad_s > 0 else 0)
    motor_sign *= WHEEL_SIGN[motor_id]
    return amp * motor_sign

def set_wheel_speeds_ticks(dxl_io, omega_r, omega_l):
    sd = wheel_rad_s_to_signed_ticks(1, omega_r)
    sg = wheel_rad_s_to_signed_ticks(2, omega_l)
    dxl_io.set_moving_speed({1: sd, 2: sg})

def unwrap_deg(prev_deg, curr_deg, period=300.0):
    # remet l'écart dans [-period/2, +period/2[
    return (curr_deg - prev_deg + period/2.0) % period - period/2.0

# ===============================
# Primitives fermées
# ===============================
def rotate_by(dxl_io, dtheta_target, w_cmd=W_TURN, dt=0.02, eps=0.01):
    """Rotation sur place de dtheta_target (rad), fermée avec les positions."""
    if abs(dtheta_target) < eps:
        return
    # commande roue pour rotation pure
    w = w_cmd * sgn(dtheta_target)
    omega_r_cmd, omega_l_cmd = inverse_kinematics(0.0, w)
    set_wheel_speeds_ticks(dxl_io, omega_r_cmd, omega_l_cmd)

    prev_r_deg, prev_l_deg = dxl_io.get_present_position([1, 2])
    theta_acc = 0.0
    while abs(theta_acc) < abs(dtheta_target) - eps:
        time.sleep(dt)
        curr_r_deg, curr_l_deg = dxl_io.get_present_position([1, 2])
        d_r = math.radians(unwrap_deg(prev_r_deg, curr_r_deg)) * WHEEL_SIGN[1]
        d_l = math.radians(unwrap_deg(prev_l_deg, curr_l_deg)) * WHEEL_SIGN[2]
        prev_r_deg, prev_l_deg = curr_r_deg, curr_l_deg

        dtheta_est = WHEEL_RADIUS * (d_r - d_l) / WHEEL_BASE
        theta_acc += dtheta_est

    set_wheel_speeds_ticks(dxl_io, 0.0, 0.0)

def translate_by(dxl_io, dist_target, v_cmd=V_LIN, dt=0.02, k_heading=0.8):
    """Translation pure de dist_target (m), fermée avec les positions + petit asservissement cap."""
    if dist_target <= 1e-4:
        return
    omega_r_cmd, omega_l_cmd = inverse_kinematics(v_cmd * sgn(dist_target), 0.0)
    set_wheel_speeds_ticks(dxl_io, omega_r_cmd, omega_l_cmd)

    prev_r_deg, prev_l_deg = dxl_io.get_present_position([1, 2])
    s_acc = 0.0
    while s_acc < abs(dist_target):
        time.sleep(dt)
        curr_r_deg, curr_l_deg = dxl_io.get_present_position([1, 2])
        d_r = math.radians(unwrap_deg(prev_r_deg, curr_r_deg)) * WHEEL_SIGN[1]
        d_l = math.radians(unwrap_deg(prev_l_deg, curr_l_deg)) * WHEEL_SIGN[2]
        prev_r_deg, prev_l_deg = curr_r_deg, curr_l_deg

        ds = WHEEL_RADIUS * 0.5 * (d_r + d_l)
        s_acc += abs(ds)

        # petit rappel cap pour rester droit
        dtheta_est = WHEEL_RADIUS * (d_r - d_l) / WHEEL_BASE
        w_corr = -k_heading * dtheta_est / max(dt, 1e-3)
        w_corr = max(-0.6, min(0.6, w_corr))

        omega_r, omega_l = inverse_kinematics(v_cmd * sgn(dist_target), w_corr)
        set_wheel_speeds_ticks(dxl_io, omega_r, omega_l)

    set_wheel_speeds_ticks(dxl_io, 0.0, 0.0)

# ===============================
# go_to (closed-loop rotations + translation)
# ===============================
def go_to(x_target, y_target, theta_target):
    """
    Repère voulu = main droite : x en avant (index) > 0, y à gauche (pouce) > 0.
    But : aller de (0,0,0) à (x_target, y_target, theta_target).
    """
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        raise RuntimeError("No Dynamixel port found")
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1, 2])

    try:
        # 1) orienter vers la cible (x,y)
        alpha = math.atan2(y_target, x_target)       # OK: y à gauche positif
        rotate_by(dxl_io, alpha)

        # 2) traduire de la distance jusqu'à la cible
        dist = math.hypot(x_target, y_target)
        translate_by(dxl_io, dist)

        # 3) rotation finale vers theta_target (relatif à l’orientation de départ)
        dtheta = wrap_pi(theta_target - alpha)
        rotate_by(dxl_io, dtheta)

    finally:
        dxl_io.set_moving_speed({1: 0, 2: 0})

# ===============================
# Exemple
# ===============================
if __name__ == "__main__":
    # Exemple : avancer 1 m, à gauche 0.5 m, orientation finale 45°
    go_to(-1.0, -1.0, math.radians(0))
