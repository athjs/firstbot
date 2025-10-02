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
    #print("Vitesse moteur gauches et droit",Vd,Vg)
    return Vd,Vg
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
