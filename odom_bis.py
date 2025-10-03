import math
import numpy as np
import pypot.dynamixel
import kynematic as ky  
import time as time
from matplotlib import pyplot as plt

# params physiques du robot
WHEEL_RADIUS = 0.025   # usi (m)
WHEEL_BASE   = 0.185   # usi (m)


# fcts qui convertissent les rad/s en unités dynamixel et vice-versa
def dxl_speed_to_rad_s(value):
    if value == 0 or value == 1024: 
        return 0.0

    if value < 1024:  # CCW
        rpm = value * 0.916
        rad_s = (rpm * 2 * math.pi) / 60.0
        return rad_s
    else:  # CW
        value = value - 1024
        rpm = value * 0.916
        rad_s = (rpm * 2 * math.pi) / 60.0
        return -rad_s


def rad_s_to_dxl_speed(rad_s, max_dxl=1023):
    rpm = abs(rad_s) * 60.0 / (2 * math.pi)
    value = int(rpm / 0.916)

    if value > max_dxl:
        value = max_dxl

    if rad_s >= 0:  # CCW
        return value
    else:  # CW
        return 1024 + value
    
# odométrie
def odom(v, w, dt):
    if abs(w) < 1e-3:  # Cas rectiligne
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


def inverse_kinematics(v, w ,R=WHEEL_RADIUS, W=WHEEL_BASE):
    v_r = v + (w * W) / 2.0
    v_l = v - (w * W) / 2.0
    Vd = v_r / R
    Vg = v_l / R
    return Vd, Vg


def point_direction(alpha, v_base=0.15, ka=0.8):
    v = v_base
    w = -ka * alpha  # correction proportionnelle à l'angle
    return v, w


# -------------------------------
# Fonction go_to principale
# -------------------------------
def go_to(
    x_target,
    y_target,
    theta_target,
):
    """
    Fait aller le robot de (x,y,theta) vers (x_target, y_target, theta_target)
    Les coordonnées (x,y) sont exprimées dans le repère monde :
      - x : vers l'avant du robot
      - y : vers la gauche du robot
    """

    path = []
    
    print( "x :", x_target, 
            "y :", y_target, 
            "z : ", theta_target)

    # --- initialisation Dynamixel ---
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        exit("No Dynamixel port found")
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1, 2])

    
    w = 0.2
    v = 0.2
    try:
        print("cesar")
        # vecteur vers la cible

        x_rest = x_target 
        y_rest = y_target 
        dist_rest = math.hypot(x_rest, y_rest)
        print("distance de l'hypothénuse :", dist_rest)

        # angle relatif vers la cible
        alpha = math.atan2(y_rest, x_rest)
        print("angle relatif vers la cible :", alpha)
        # erreur d'orientation
        # dtheta = theta_goal - theta
        # cinématique inverse
        Vd, Vg = inverse_kinematics(0,w)
        Vd = - Vd
        
        print("Vd :", Vd, "Vg :", Vg)
        rotation_duration = np.abs(alpha/Vd)
        
        print("Je dois tourner pendant: ", rotation_duration, "s")
        # conversion Dynamixel
        speed_d = rad_s_to_dxl_speed(Vd)
        speed_g = rad_s_to_dxl_speed(Vg)
        print("speed_d : ", speed_d, "speed_g :", speed_g)
        # appliquer aux moteurs
        init = time.time()

        # appliquer aux moteurs 
        dxl_io.set_moving_speed({1: -speed_d, 2: speed_g})  # roue droite  # roue gauche
        while(time.time() - init < rotation_duration):
            continue   
        dxl_io.set_moving_speed({1: 0, 2: 0})
        # path.append((x, y, theta))
        # conversion Dynamixel
        
        # calculs vitesses pour aller tout droit
        Vd, Vg = inverse_kinematics(v,0)
        print("Vitesses tout droit, Vd : ", Vd, " Vg :",Vg)
        dt = dist_rest/(Vg*WHEEL_RADIUS)
        
        print("temps qu'il faut avancer tout droit :", dt, "s")
        speed_d = rad_s_to_dxl_speed(Vd)
        speed_g = rad_s_to_dxl_speed(Vg)
        print("speed_d : ", speed_d, "speed_g :", speed_g)
        init = time.time()
        dxl_io.set_moving_speed({1: -speed_d, 2: speed_g})  # roue droite  # roue gauche
        while(time.time() - init < dt):
            continue
        dxl_io.set_moving_speed({1: 0, 2: 0})
                # path.append((x, y, theta))
        theta_target = theta_target-alpha
        print("angle restant pour se mettre dans l'orientation  cible : ", theta_target)
        # erreur d'orientation
        # dtheta = theta_goal - theta
        # cinématique inverse
        Vd, Vg = inverse_kinematics(0,w)
        Vd = - Vd
        print("Vitesses des roues pour tourner, Vd : ", Vd, " Vg :",Vg)
        rotation_duration = np.abs(theta_target/Vd)
        print("temps de rotation : ", rotation_duration)
        # conversion Dynamixel
        speed_d = rad_s_to_dxl_speed(Vd)
        speed_g = rad_s_to_dxl_speed(Vg)
        print("speed_d : ", speed_d, "speed_g :", speed_g)
        print("to radians d :", dxl_speed_to_rad_s(speed_d),"to radians g :",dxl_speed_to_rad_s(speed_g) )
        #if theta_target-alpha<0: 
            #w = -w
        # appliquer aux moteurs
        init = time.time()
        dxl_io.set_moving_speed({1: speed_d, 2: speed_g})  # roue droite  # roue gauche
        while(time.time() - init < rotation_duration):
            continue
        dxl_io.set_moving_speed({1: 0, 2: 0})
    finally: 
        dxl_io.set_moving_speed({1: 0, 2: 0})
    # arrêt moteur
    dxl_io.set_moving_speed({1: 0, 2: 0})

    return

def _wrap_to_pi(a):
    return (a + math.pi) % (2*math.pi) - math.pi

def odometry(x=0.0, y=0.0, theta=0.0, dt=0.01, duration=200.0):
    path = []

    # --- initialisation Dynamixel ---
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        exit("No Dynamixel port found")
    dxl_io = pypot.dynamixel.DxlIO(ports[0])
    dxl_io.set_wheel_mode([1, 2])  # mode roue libre

    # positions initiales (en radians) : référence pour l’unwrapping
    prev_pos_deg = dxl_io.get_present_position([1, 2])  # degrés
    prev_pos = [math.radians(p) for p in prev_pos_deg]  # rad

    # positions cumulées CONTINUES, initialisées à 0
    cum_pos = [0.0, 0.0]  # [droit, gauche] si ton ordre est [1,2]

    t = 0.0
    print("trajet démarré")

    while t < duration:
        # lire nouvelle position (rad, bornée [-pi, pi] après conversion)
        curr_pos_deg = dxl_io.get_present_position([1, 2])
        curr_pos = [math.radians(p) for p in curr_pos_deg]

        # delta corrigé du wrap pour chaque roue
        d_right = _wrap_to_pi(curr_pos[0] - prev_pos[0])
        d_left  = _wrap_to_pi(curr_pos[1] - prev_pos[1])

        # mettre à jour positions cumulées (continues, partant de 0)
        cum_pos[0] += d_right
        cum_pos[1] += d_left

        # vitesses angulaires (rad/s) à partir des deltas corrigés
        Vd_real = d_right / dt
        Vg_real = d_left  / dt

        # si ta cinématique attend Vd inversé, on garde ta inversion
        Vd_real = -Vd_real

        # mise à jour du "prev" pour la prochaine itération
        prev_pos = curr_pos

        # convertir en (v, w) via ta cinématique directe existante
        v_real, w_real = direct_kinematics(Vd_real, Vg_real)

        # intégrer l’odométrie
        x, y, theta = tick_odom(x, y, theta, v_real, w_real, dt)

        path.append((x, y, theta))

        # (optionnel) debug : imprimer la position cumulée continue
        # print(f"cum_right={cum_pos[0]:.3f}, cum_left={cum_pos[1]:.3f}")

        time.sleep(dt)
        t += dt

    return x, y, theta, path

x , y , theta, path = odometry()

def plot_path(path, filename="trajet.png", show_orientations=True, step=10):
    """
    Trace le trajet (x,y) issu d'une liste path = [(x, y, theta), ...]
    et enregistre la figure dans le fichier `filename` (dans le répertoire courant).

    - path : liste de tuples (x, y, theta)
    - filename : nom du fichier de sortie (ex: 'trajet.png')
    - show_orientations : affiche quelques flèches orientées selon theta
    - step : une flèche toutes les `step` poses
    """
    if not path:
        raise ValueError("Le path est vide.")

    xs = [p[0] for p in path]
    ys = [p[1] for p in path]
    thetas = [p[2] for p in path]

    fig, ax = plt.subplots(figsize=(6, 6))

    # Trajectoire
    ax.plot(xs, ys, linewidth=2, label="Trajet")

    # Départ / Arrivée
    ax.scatter(xs[0], ys[0], marker="o", s=60, label="Départ")
    ax.scatter(xs[-1], ys[-1], marker="x", s=80, label="Arrivée")

    # Flèches d'orientation
    if show_orientations and len(path) > 1:
        idx = np.arange(0, len(path), max(1, int(step)))
        u = np.cos(np.array(thetas)[idx])
        v = np.sin(np.array(thetas)[idx])
        ax.quiver(np.array(xs)[idx], np.array(ys)[idx], u, v,
                  angles='xy', scale_units='xy', scale=5, width=0.003, alpha=0.6)

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Trajet odométrie")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best")

    fig.tight_layout()
    fig.savefig(filename, dpi=200)
    plt.close(fig) 
    return filename

plot_path(path)
    
    
