# script pour téléguider le robot avec le clavier
import numpy as np
import time
import pypot.dynamixel
import sys
import select
import tty
import termios
import camera as cam
import kynematic as ky
import time as time
import datetime as datetime

ports = pypot.dynamixel.get_available_ports()
if not ports:
    exit("No port")


cap = cam.open_usb_cam()

if not cap.isOpened():
    print("Cannot open camera")
    exit()


AVG_SPEED = 180

dxl_io = pypot.dynamixel.DxlIO(ports[0])
dxl_io.set_wheel_mode([1, 2])  # active wheel mode sur les deux
while True:
    init_time = time.time()
    ret, frame = cap.read()
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    dX_ligne, dY_ligne, angle_ligne = cam.detect_lines_by_color(frame, cam.Color.Yellow)
    Vl, Vo = ky.point_direction(dX_ligne, 20, np.pi / 4)
    Vd, Vg = ky.inverse_kinematics(Vl, Vo)
    dxl_io.set_moving_speed({1: -Vd, 2: Vg})
    current_time = time.time()
    print(current_time - init_time)
    time.sleep(1 / 30 - (current_time - init_time))
# try:
#     coef_left = 1.0
#     coef_right = 1.0
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("Can't receive frame (stream end?). Exiting ...")
#             break
#         dX_ligne, dY_ligne, angle_ligne = detect_lines_by_color(frame, Color.Yellow)
#         if angle_ligne is None:
#             break
#
#
#         # On normalise pour faire passer entre -1 et 1
#         angle_ligne_norm = angle_ligne / (np.pi / 2)
#
#         print(angle_ligne_norm)
#
#         # si cet angle_ligne est positif alors il faut tourner à droite, je ralentis la roue droite
#         # et accèlère la roue droite en fonction de la valeur de l'angle_ligne.
#         if angle_ligne_norm > 0 :
#             coef_left += np.abs(angle_ligne_norm / 1.2)
#             coef_right -= np.abs(angle_ligne_norm / 1.2)
#
#         # s'il est négatif alors l'inverse.
#         if angle_ligne_norm < 0:
#             coef_left -= np.abs(angle_ligne_norm / 1.2)
#             coef_right += np.abs(angle_ligne_norm / 1.2)
#
#
#         # applique les vitesses coeffficientées ajustées avec l'angle_ligne.
#         dxl_io.set_moving_speed({
#             1: -AVG_SPEED * coef_right,   # moteur gauche
#             2:  AVG_SPEED * coef_left   # moteur droit
#         })
#
#
#         # ptite pause dans la boucle à chaque itération pour pas surcharger le rasp.
#         time.sleep(0.01)
#         coef_left = 1.0
#         coef_right = 1.0
#
# except KeyboardInterrupt:
#     print("Ctrl+C détecté, arrêt…")
#
# except Exception as e:
#     print(f"Erreur: {e}")
#
# finally:
#     cap.release()
#     cv.destroyAllWindows()
#     dxl_io.set_moving_speed({1: 0, 2: 0})  # stop toujours les moteurs
#
