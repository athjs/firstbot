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

def go_to(x_target, y_target, alpha_target,
          x=0.0, y=0.0, theta=0.0,
          dt=0.1, tol_pos=0.01, tol_theta=0.05):
    """
    Fait aller le robot de (x,y,theta) vers (x_target, y_target, alpha_target)
    dt : intervalle de temps pour mise à jour
    tol_pos : tolérance position (m)
    tol_theta : tolérance orientation (rad)
    """
    path = []  # pour stocker la trajectoire
    
    while True:
        # vecteur vers cible
        dx = x_target - x
        dy = y_target - y
        dist = math.hypot(dx, dy)

        # angle vers la cible
        alpha = math.atan2(dy, dx) - theta
        # normalisation entre -pi et pi
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        # arrêt si proche de la cible
        if dist < tol_pos and abs(alpha_target - theta) < tol_theta:
            break

        # calcul des vitesses linéaire et angulaire
        v, w = ky.point_direction(alpha)

        # saturation vitesse si on est proche de la cible
        if dist < 0.2:
            v *= dist / 0.2  # ralentir progressivement

        # vitesses roues
        Vd, Vg = ky.inverse_kinematics(v, w)

        # ici on pourrait convertir en Dynamixel : rad_s_to_dxl_speed(Vd), rad_s_to_dxl_speed(Vg)
        # mais pour simulation, on applique directement odométrie
        x, y, theta = tick_odom(x, y, theta, v, w, dt)

        path.append((x, y, theta))

    return x, y, theta, path

# --- Exemple ---
x_final, y_final, theta_final, path = go_to(1.0, 1.0, 0.0)
print(f"x={x_final:.3f}, y={y_final:.3f}, theta={theta_final:.3f}")
