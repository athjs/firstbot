import numpy as np
import cv2 as cv
import time
import pypot.dynamixel
import sys
import tty
import termios
import math
from camera import detect_lines_by_color
from camera import detect_lines_brown
from camera import Color
from kynematic import *
from time import sleep
from odometry import *


def test_inversekinematics(): 
    try: 
        # --- initialisation Dynamixel ---
        ports = pypot.dynamixel.get_available_ports()
        if not ports:
            exit('No Dynamixel port found')
        dxl_io = pypot.dynamixel.DxlIO(ports[0])
        dxl_io.set_wheel_mode([1, 2])   # active wheel mode sur les deux
        
        
        
        Vd, Vg = inverse_kinematics(0, 0.8)
        speed_d = rad_s_to_dxl_speed(Vd)
        speed_g = rad_s_to_dxl_speed(Vg)
        

        while True: 
            # appliquer aux moteurs
                dxl_io.set_moving_speed({
                    1: -speed_d,  # moteur gauche
                    2:  speed_g  # moteur droit
                })
            
    finally : 
        dxl_io.set_moving_speed({1: 0, 2: 0})
        


def test_directkinematics(): 
    try: 
        # --- initialisation Dynamixel ---
        ports = pypot.dynamixel.get_available_ports()
        if not ports:
            exit('No Dynamixel port found')
        dxl_io = pypot.dynamixel.DxlIO(ports[0])
        dxl_io.set_wheel_mode([1, 2])   # active wheel mode sur les deux
        
        
        v, w = direct_kinematics(-6, 6)
        Vd, Vg = inverse_kinematics(v, w)
        speed_d = rad_s_to_dxl_speed(Vd)
        speed_g = rad_s_to_dxl_speed(Vg)
        

        while True: 
            # appliquer aux moteurs
                dxl_io.set_moving_speed({
                    1: -speed_d,  # moteur gauche
                    2:  speed_g  # moteur droit
                })
            
    finally : 
        dxl_io.set_moving_speed({1: 0, 2: 0})
        


def test_go_to(): 
    x, y, theta, path = go_to(250, 0, 0)
    print(x,y, theta, path)

test_go_to()
