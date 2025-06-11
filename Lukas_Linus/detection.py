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

CAMERA_ID = 1
IP_ADDRESS_CAMERA = '172.20.10.7'
URL = f'http://{IP_ADDRESS_CAMERA}:81/stream'

class ArucoMarker():
    """
    Class representing an ArUco marker with its ID, distance, and angle in addition to the position of the camera which takes the image.
    """
    def __init__(self, detected_id, distance, angle, timestamp):
        self.detected_id = detected_id
        self.distance = distance
        self.angle = angle
        self.timestamp = timestamp

    def __repr__(self):
        return f"ArucoMarker(id={self.detected_id}, distance={self.distance}, angle={self.angle}, timestamp={self.timestamp})"

    def update_position(self):
        """
        Updates the position of the marker in marker_positions.json.
        """
        # Load existing marker positions from JSON file
        with open ('Lukas_Linus/marker_positions.json', 'r') as file:
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
                        other['Position'] = [{'Distance': self.distance, 'Angle': self.angle}]
                        camera_dict["time"] = self.timestamp
                        with open('Lukas_Linus/marker_positions.json', 'w') as f:
                            json.dump(marker_positions, f, indent=4)
                        return
                # append new detected block if detected_id not found
                camera_dict['Others'].append({
                    'detected_id': self.detected_id,
                    'Position': [
                        {'Distance': self.distance, 'Angle': self.angle}
                    ]
                })
                camera_dict["time"] = self.timestamp
                with open('Lukas_Linus/marker_positions.json', 'w') as f:
                    json.dump(marker_positions, f, indent=4)
                return

def get_aruco_markers(frame):
    """
    Detects ArUco markers in the given frame.
    Returns a list of detected markers with their IDs, distances, and angles.
    """
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, _ = detector.detectMarkers(frame)
    
    if ids is not None:
        ids = ids.flatten()
        positions = []
        for i in range(len(ids)):
            # Calculate distance and angle (dummy values for example)
            distance = np.random.uniform(30, 100)  # Replace with actual distance calculation
            angle = np.random.uniform(-180, 180)   # Replace with actual angle calculation
            positions.append((ids[i], distance, angle))
        return positions
    else:
        print("No markers detected")	
        return []

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
    positions = get_aruco_markers(frame)
    markers = []
    for detected_id, distance, angle in positions:
        marker = ArucoMarker(detected_id, distance, angle, photo_timestamp)
        markers.append(marker)
    return markers





# get the system ready

# Load predefined dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)
# start the camera stream
cap = setup_camera_stream()


# update all markers every second
prev_second = datetime.now().second
while True:
    now = datetime.now()
    # check if a new full second has started
    if now.second != prev_second:
        frame, photo_timestamp = get_frame(cap)
        detected_markers = get_marker_detections(frame, photo_timestamp)
        for marker in detected_markers:
            marker.update_position()
            print(marker.__repr__())
        prev_second = now.second



## manually create a marker for testing purposes
# Aruco1 = ArucoMarker(16, 60, 50, photo_timestamp)
# Aruco1.update_position()
# print(Aruco1.__repr__())