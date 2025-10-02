import numpy as np
import math
import kynematic as ky

def odom(v, w, dt):
    """
    Calcule le déplacement du robot dans son repère local (dx, dy, dtheta)
    pour une vitesse linéaire v (m/s), une vitesse angulaire w (rad/s)
    et un intervalle de temps dt (s).
    """
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
    """
    Met à jour la position (x, y, theta) dans le repère monde
    en utilisant odom() pour calculer le déplacement local.
    """
    dx_r, dy_r, dtheta = odom(v, w, dt)  # déplacement dans le repère robot

    # Transformation repère robot -> repère monde
    dx_g = dx_r * math.cos(theta) - dy_r * math.sin(theta)
    dy_g = dx_r * math.sin(theta) + dy_r * math.cos(theta)

    return x + dx_g, y + dy_g, theta + dtheta


import math

def go_to(x_target, y_target, theta_target,
          x=0.0, y=0.0, theta=0.0,
          dt=0.1, tol_pos=0.01, tol_theta=0.05):
    """
    Fait aller le robot de (x,y,theta) vers (x_target, y_target, alpha_target)
    dt : intervalle de temps pour mise à jour
    tol_pos : tolérance position (m)
    tol_theta : tolérance orientation (rad)
    """
    path = []  # pour stocker la trajectoire

    # --- initialisation Dynamixel ---
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        exit('No Dynamixel port found')
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1, 2])   # active wheel mode sur les deux
    
    while True:
        # vecteur vers cible
        x_rest = x_target - x
        y_rest = y_target - y
        dist_rest = math.hypot(x_rest, y_rest)

        # angle vers la cible
        alpha = math.atan2(y_rest, x_rest) - theta
        # normalisation entre -pi et pi
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        # arrêt si proche de la cible
        if dist_rest < tol_pos and abs(theta_target - theta) < tol_theta:
            break

        # calcul des vitesses linéaire et angulaire
        v, w = ky.point_direction(math.pi - alpha)

        # cinématique inverse -> vitesses roues (rad/s)
        Vd, Vg = ky.inverse_kinematics(v, w)

        # conversion Dynamixel
        speed_d = ky.rad_s_to_dxl_speed(Vd)
        speed_g = ky.rad_s_to_dxl_speed(Vg)

        #print(f"alpha={angle:.3f} rad | cmd: v={v:.2f}, w={w:.2f} "
         #     f"| roues: Vd={Vd:.2f}, Vg={Vg:.2f} rad/s "
          #    f"| estimé: v={v_est:.2f}, w={w_est:.2f}")

        # appliquer aux moteurs
        dxl_io.set_moving_speed({
            1: -speed_d,  # moteur gauche
            2:  speed_g  # moteur droit
        })

        Vd_real , Vg_real = get_present_speed({1,2})

        # saturation vitesse si on est proche de la cible
        if dist_rest < 0.2:
            v *= dist_rest / 0.2  # ralentir progressivement

        # vitesses roues
        v, w = ky.direct_kinematics(Vd_real, Vg_real)

        # ici on pourrait convertir en Dynamixel : rad_s_to_dxl_speed(Vd), rad_s_to_dxl_speed(Vg)
        # mais pour simulation, on applique directement odométrie
        x, y, theta = tick_odom(x, y, theta, v, w, dt)

        path.append((x, y, theta))

    return x, y, theta, path

