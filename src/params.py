import numpy as np


CAMERA_ID = 5
IP_ADDRESS_CAMERA = '192.168.2.108'
URL = f'http://{IP_ADDRESS_CAMERA}:81/stream'

# Kamera-Kalibrierungsparameter
# Werte aus MATLAB
fx, fy = 306.9462, 314.8131
cx, cy = 159.3294, 119.3385
k1, k2 = -0.0323, 0.0464
p1, p2 = 0.0, 0.0
k3 = 0.0 

CAMERA_MATRIX = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=np.float32)
DISTCOEFFS = np.array([k1, k2, p1, p2, k3], dtype=np.float32)
MARKERLENGTH = 0.2

TOPIC_1 = "EZS/beschtegruppe/1"
TOPIC_2 = "EZS/beschtegruppe/2"
TOPIC_3 = "EZS/beschtegruppe/3"
TOPIC_4 = "EZS/beschtegruppe/4"
TOPIC_5 = "EZS/beschtegruppe/5"
TOPIC_6 = "EZS/beschtegruppe/6"

BROKER = "test.mosquitto.org"
PORT = 1883

