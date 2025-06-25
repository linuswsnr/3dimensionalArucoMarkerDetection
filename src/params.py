"""
Authors: Linus Wasner, Lukas Bauer
Date: 2025-06-20
Project: 3dimensionalArucoMarkerDetection
Lekture: Echtzeitsysteme, Masterprogram advanced driver assistance systems, University of Applied Sciences Kempten

This module contains parameters for camera calibration, MQTT topics, and marker configurations.
"""

import numpy as np

# video stream
IP_ADDRESS_CAMERA = '192.168.0.102'
URL = f'http://{IP_ADDRESS_CAMERA}:81/stream'

# camera calibration and dimensions
# values from MATLAB calibration
#fx, fy = -306.9462, 314.8131
fx, fy = 306.9462, 314.8131
cx, cy = 159.3294, 119.3385
k1, k2 = -0.0323, 0.0464
p1, p2 = 0.0, 0.0
k3 = 0.0 

CAMERA_MATRIX = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=np.float32)
DISTCOEFFS = np.array([k1, k2, p1, p2, k3], dtype=np.float32)
MARKERLENGTH = 0.02

# MQTT
TOPIC_1 = "EZS/beschtegruppe/1"
TOPIC_2 = "EZS/beschtegruppe/2"
TOPIC_3 = "EZS/beschtegruppe/3"
TOPIC_4 = "EZS/beschtegruppe/4"
TOPIC_5 = "EZS/beschtegruppe/5"
TOPIC_6 = "EZS/beschtegruppe/6"

BROKER = "test.mosquitto.org"
#BROKER = "broker.hivemq.com"
#BROKER = "192.168.3.113"
PORT = 1883

# Marker IDs to map cameras
MARKER_TO_CAMERA = {
    10: 1, 11: 1, 12: 1, 13: 1,
    20: 2, 21: 2, 22: 2, 23: 2,
    30: 3, 31: 3, 32: 3, 33: 3,
    40: 4, 41: 4, 42: 4, 43: 4,
    50: 5, 51: 5, 52: 5, 53: 5,
    60: 6, 61: 6, 62: 6, 63: 6,
}

ANCHOR_MARKER_IDS = {0, 1, 2, 3}

# own camera ID
CAMERA_ID = 5

# hardcoded 4x4 matrices (global origin, orientation as defined)
ANCHOR_MARKER_WORLD_POSES = {
    0: np.array([
        [1.0, 0.0, 0.0,  0.0 ],
        [0.0, 1.0, 0.0,  0.0 ],
        [0.0, 0.0, 1.0,  0.0 ],
        [0.0, 0.0, 0.0,  1.0 ]
    ]),
    1: np.array([
        [ 0.0, 0.0, 1.0, 0.0 ],
        [ 0.0, 1.0, 0.0, 0.0 ],
        [-1.0, 0.0, 0.0, 0.0 ],
        [ 0.0, 0.0, 0.0, 1.0 ]
    ]),
    2: np.array([
        [-1.0, 0.0,  0.0, 0.0 ],
        [ 0.0, 1.0,  0.0, 0.0 ],
        [ 0.0, 0.0, -1.0, 0.0 ],
        [ 0.0, 0.0,  0.0, 1.0 ]
    ]),
    3: np.array([
        [ 0.0, 0.0, -1.0, 0.0 ],
        [ 0.0, 1.0,  0.0, 0.0 ],
        [ 1.0, 0.0,  0.0, 0.0 ],
        [ 0.0, 0.0,  0.0, 1.0 ]
    ])
}

# with offset to kamera cube
# hardcoded 4x4  transformation matrices (global origin, orientation as defined)
# ANCHOR_MARKER_WORLD_POSES = {
#     0: np.array([
#         [1.0, 0.0, 0.0,  MARKERLENGTH / 2],
#         [0.0, 1.0, 0.0,  0.0 ],
#         [0.0, 0.0, 1.0,  0.0 ],
#         [0.0, 0.0, 0.0,  1.0 ]
#     ]),
#     1: np.array([
#         [0.0, 0.0, 1.0,  0.0 ],
#         [0.0,  1.0, 0.0,  MARKERLENGTH / 2],
#         [-1.0,  0.0, 0.0,  0.0 ],
#         [0.0,  0.0, 0.0,  1.0 ]
#     ]),
#     2: np.array([
#         [-1.0,  0.0, 0.0, -MARKERLENGTH / 2],
#         [ 0.0, -1.0, 0.0,  0.0 ],
#         [ 0.0,  0.0, -1.0,  0.0 ],
#         [ 0.0,  0.0, 0.0,  1.0 ]
#     ]),
#     3: np.array([
#         [ 0.0,  0.0, -1.0,  0.0 ],
#         [0.0,  1.0, 0.0, -MARKERLENGTH / 2],
#         [ 1.0,  0.0, 0.0,  0.0 ],
#         [ 0.0,  0.0, 0.0,  1.0 ]
#     ])
# }