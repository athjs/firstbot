# script pour téléguider le robot avec le clavier
import numpy as np
import cv2 as cv
import time
import pypot.dynamixel
import sys
import select
import tty
import termios

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


AVG_SPEED = 360

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
        

        # lecture non bloquante du clavier
        r, _, _ = select.select([sys.stdin], [], [], 0)
        if r:
            ch = sys.stdin.read(1).lower()
            if ch == '\x18':   # Ctrl+X
                #print("Ctrl+X détecté")
                break
            elif ch == 'q':    # virage à gauche -> ralentir roue gauche
                coef_left = 1
                coef_right = 1.3
                #print("Virage à gauche")
            elif ch == 'd':    # virage à droite -> ralentir roue droite
                coef_left = 1.3
                coef_right = 1
                #print("Virage à droite")
            elif ch == 'z':    #tout droit on accelère. 
                coef_left = 2.0
                coef_right = 2.0
                #print("Avance tout droit")
            elif ch == 's':    #tout droit on accelère. 
                coef_left = -1.0
                coef_right = -1.0
                #print("Avance tout droit")
            elif ch == 'x':    #On s'arrete 
                coef_left = 0
                coef_right = 0
                #print("On s'arrete")

        # applique les vitesses (ajustées selon les touches)
        dxl_io.set_moving_speed({
            1: -AVG_SPEED * coef_right,   # moteur gauche
            2:  AVG_SPEED * coef_left   # moteur droit
        })


        # Capte une image par la webcam
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        # Our operations on the frame come here
        # Display the resulting frame
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        print(gray)
        #cv.imshow('frame',frame)
        #print(np.shape(frame))
        
        # ptite pause dans la boucle à chaque itération pour pas surcharger le rasp. 
        time.sleep(0.05)  

except KeyboardInterrupt:
    print("Ctrl+C détecté, arrêt…")

except Exception as e:
    print(f"Erreur: {e}")

finally:
    cap.release()
    cv.destroyAllWindows()
    dxl_io.set_moving_speed({1: 0, 2: 0})  # stop toujours les moteurs
    termios.tcsetatt
