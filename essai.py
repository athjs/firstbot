# script pour téléguider le robot avec le clavier
import numpy as np
import cv2 as cv
import time
import pypot.dynamixel
import sys
import select
import tty
import termios
from image_traitement import detect_lines_by_color 

# Trouve le(s) port(s) du moteur
ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit('No port')


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


AVG_SPEED = 180

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])   # active wheel mode sur les deux

# --- préparation lecture clavier pour détecter Ctrl+X / Q / D ---
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setcbreak(fd)

try:
    coef_left = 1.0
    coef_right = 1.0
    while True:
        # Capte une image par la webcam
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        
        # détection de l'angle entre centroide de la courbe et le bas de l'image. (entre -pi/2 et pi/2)
        angle = detect_lines_by_color(frame)
        if angle is None: 
            break
            
        
        # On normalise pour faire passer entre -1 et 1 
        angle_norm = angle / (np.pi / 2)
        
        print(angle_norm)
        
        # si cet angle est positif alors il faut tourner à droite, je ralentis la roue droite 
        # et accèlère la roue droite en fonction de la valeur de l'angle. 
        if angle_norm > 0 :
            coef_left += np.abs(angle_norm / 1.2)
            coef_right -= np.abs(angle_norm / 1.2)
        
        # s'il est négatif alors l'inverse. 
        if angle_norm < 0: 
            coef_left -= np.abs(angle_norm / 1.2)
            coef_right += np.abs(angle_norm / 1.2)


        # applique les vitesses coeffficientées ajustées avec l'angle. 
        dxl_io.set_moving_speed({
            1: -AVG_SPEED * coef_right,   # moteur gauche
            2:  AVG_SPEED * coef_left   # moteur droit
        })


        # ptite pause dans la boucle à chaque itération pour pas surcharger le rasp. 
        time.sleep(0.01)  
        coef_left = 1.0
        coef_right = 1.0

except KeyboardInterrupt:
    print("Ctrl+C détecté, arrêt…")

except Exception as e:
    print(f"Erreur: {e}")

finally:
    cap.release()
    cv.destroyAllWindows()
    dxl_io.set_moving_speed({1: 0, 2: 0})  # stop toujours les moteurs
    
