"""
Authors: Lukas Bauer, Linus Wasner
Lecture: Driver Assistance Systems, University of Applied Sciences Kempten
Student Project: ArUco Marker Detection

Maintask:
- script detects aruco markers in a 2 dimensional space using a video stream from an ESP32 camera.
- all detected positions were saved and continuously updated in a markerPositions.json.
"""

import json
import cv2
import cv2.aruco as aruco
import numpy as np
import traceback
from datetime import datetime
import matplotlib.pyplot as plt     
import matplotlib.animation as animation


from Visualisation import redraw_marker_camera_network

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

class ArucoMarker():
    """
    Class representing an ArUco marker with its ID, distance, and angle in addition to the position of the camera which takes the image.
    """
    def __init__(self, detected_id, rvecs, tvecs, timestamp):
        self.detected_id = int(detected_id)
        self.rvecs = rvecs
        self.tvecs = tvecs
        self.timestamp = timestamp

    def __repr__(self):
        return f"ArucoMarker(id={self.detected_id}, rvecs={self.rvecs}, tvecs={self.tvecs}, timestamp={self.timestamp})"

    def update_position(self):
        """
        Updates the position of the marker in marker_positions.json.
        """
        # Load existing marker positions from JSON file
        with open ('Lukas_Linus/marker_positions_rvecs_tvecs.json', 'r') as file:
            marker_positions = json.load(file)

        # check if camera dict contains the if of the camera
        found = False
        for camera_dict in marker_positions:
            if camera_dict['id'] == CAMERA_ID:
                found = True
                break
        if not found:
            print("Camera ID not found in marker_positions.json")

        # find dictionary with the camera id
        for camera_dict in marker_positions:   
            if camera_dict['id'] == CAMERA_ID:  
                # find detected_id if existing and update values                            
                for other in camera_dict['Others']:                 
                    if other['detected_id'] == self.detected_id:
                        other['Position'] = [{'rvecs': self.rvecs}, {'tvecs': self.tvecs}]
                        camera_dict["time"] = self.timestamp
                        with open('Lukas_Linus/marker_positions_rvecs_tvecs.json', 'w') as f:
                            json.dump(marker_positions, f, indent=4)
                        return
                # append new detected block if detected_id not found
                camera_dict['Others'].append({
                    'detected_id': self.detected_id,
                    'Position': [
                        {'rvecs': self.rvecs}, {'tvecs': self.tvecs}
                    ]
                })
                camera_dict["time"] = self.timestamp
                with open('Lukas_Linus/marker_positions_rvecs_tvecs.json', 'w') as f:
                    json.dump(marker_positions, f, indent=4)
                return
            else:
                print(f"Camera ID {CAMERA_ID} not found in marker_positions.json")
def get_aruco_markers(frame):
    """
    Detects ArUco markers in the given frame.
    Returns a list of detected markers with their IDs, distances, and angles.


    !!! muss erst getestet werden !!!

    """
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(frame)
    
    if ids is not None:
        ids = ids.flatten()
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKERLENGTH, CAMERA_MATRIX, DISTCOEFFS)
        rvecs = rvecs[0].tolist()
        tvecs = tvecs[0].tolist()
        return ids, rvecs, tvecs
    else:
        print("No markers detected")	
        return [], [], []

def get_frame(cap):
    """
    Captures a frame from the video stream.
    Returns the captured frame.
    """
    ret, frame = cap.read()
    timestamp = datetime.now().isoformat()
    if not ret:
        print("Failed to grab frame")
        return None
    return frame, timestamp

def get_marker_detections(frame, photo_timestamp):
    """
    Detects ArUco markers in the given frame and returns a list of ArucoMarker objects.
    """
    ids, rvecs, tvecs = get_aruco_markers(frame)
    markers = []
    for detected_id, rvec, tvec in zip(ids, rvecs, tvecs):
        marker = ArucoMarker(detected_id, rvec, tvec, photo_timestamp)
        markers.append(marker)
    return markers

def setup_camera_stream():
    """
    Sets up the video stream from the ESP32 camera.
    Returns a VideoCapture object.
    """
    cap = cv2.VideoCapture(URL)
    if not cap.isOpened():
        print("Error: Cannot open video stream")
        exit()
    else:
        print("Success: Starting video stream")
    return cap


# get the system ready

# Load predefined dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)
# start the camera stream
cap = setup_camera_stream()

markers = {}
prev_second = datetime.now().second

plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-0.1, 0.1)
ax.set_ylim(-0.1, 0.1)
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_title("Rekonstruiertes Marker-Kamera-Netzwerk")
ax.grid(True)

while True:
    now = datetime.now()

    frame, photo_timestamp = get_frame(cap)     
    cv2.imshow("ESP32 Cam Stream", frame)  # ← wichtig für waitKey
    detected_markers = get_marker_detections(frame, photo_timestamp)
    for marker in detected_markers:
        # marker erzeugen falls noch nicht vorhanden und position aktualisieren
        markers[f"marker_{marker.detected_id}"] = marker
        print(markers[f"marker_{marker.detected_id}"].__repr__())
        marker.update_position()
    #print(f"{detected_markers} \n")
    # check if a new full second has started
    if now.second != prev_second:
        camera_positions, marker_positions = redraw_marker_camera_network()
        prev_second = now.second

        ax.clear()
        ax.set_xlim(-0.5, 0.5)
        ax.set_ylim(-0.5, 0.5)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title("Rekonstruiertes Marker-Kamera-Netzwerk")
        ax.grid(True)

        for cam_id, pos in camera_positions.items():
            circle = plt.Circle((pos[0], pos[1]), 0.005, color='green', fill=True)
            ax.add_artist(circle)
            # Radius 0.005 m = 0.5 cm Durchmesser
            ax.plot(pos[0], pos[1], 'ro')
            ax.text(pos[0] - 0.01, pos[1] - 0.01, cam_id, color='green')

        for marker_id, pos in marker_positions.items():
            ax.plot(pos[0], pos[1], 'bs')
            ax.text(pos[0] + 0.01, pos[1] + 0.01, marker_id, color='blue')

        fig.canvas.draw()
        fig.canvas.flush_events()

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break




## manually create a marker for testing purposes
# Aruco1 = ArucoMarker(16, 60, 50, photo_timestamp)
# Aruco1.update_position()
# print(Aruco1.__repr__())