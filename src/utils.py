import json
import cv2
import cv2.aruco as aruco
import numpy as np
from datetime import datetime
import params as params


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
        return f"ArucoMarker(id={self.detected_id}, rvecs={self.rvecs}, tvecs={self.tvecs}, timestamp={str(self.timestamp)})"

    def delete_position(self):
        """
        Deletes the position of the marker in marker_positions_rvecs_tvecs.json.
        """
        # Load existing marker positions from JSON file
        with open ('src/marker_positions_rvecs_tvecs.json', 'r') as file:
            marker_positions = json.load(file)

        # check if camera dict contains the if of the camera
        found = False
        for camera_dict in marker_positions:
            if camera_dict['id'] == params.CAMERA_ID:
                found = True
                break
        if not found:
            print("delete: Camera ID not found in marker_positions_rvecs_tvecs.json")

        # find dictionary with the camera id
        for camera_dict in marker_positions:   
            if camera_dict['id'] == params.CAMERA_ID:  
                # find detected_id if existing and delete values                            
                for other in camera_dict['Others']:                 
                    if other['detected_id'] == self.detected_id:
                        camera_dict['Others'].remove(other)
                        with open('src/marker_positions_rvecs_tvecs.json', 'w') as f:
                            json.dump(marker_positions, f, indent=4)
                        return
                    
    def update_position(self):
        """
        Updates the position of the marker in marker_positions.json.
        """
        # Load existing marker positions from JSON file
        with open ('src/marker_positions_rvecs_tvecs.json', 'r') as file:
            marker_positions = json.load(file)

        # check if camera dict contains the if of the camera
        found = False
        for camera_dict in marker_positions:
            if camera_dict['id'] == params.CAMERA_ID:
                found = True
                break
        if not found:
            print("update1:Camera ID not found in marker_positions.json")

        # find dictionary with the camera id
        for camera_dict in marker_positions:   
            if camera_dict['id'] == params.CAMERA_ID:  
                # find detected_id if existing and update values                            
                for other in camera_dict['Others']:                 
                    if other['detected_id'] == self.detected_id:
                        other['Position'] = [{'rvecs': self.rvecs}, {'tvecs': self.tvecs}]
                        camera_dict["time"] = str(self.timestamp)
                        with open('src/marker_positions_rvecs_tvecs.json', 'w') as f:
                            json.dump(marker_positions, f, indent=4)
                        return
                # append new detected block if detected_id not found
                camera_dict['Others'].append({
                    'detected_id': self.detected_id,
                    'Position': [
                        {'rvecs': self.rvecs}, {'tvecs': self.tvecs}
                    ]
                })
                camera_dict["time"] = str(self.timestamp)
                with open('src/marker_positions_rvecs_tvecs.json', 'w') as f:
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
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, params.MARKERLENGTH, params.CAMERA_MATRIX, params.DISTCOEFFS)
        rvecs = rvecs[0].tolist() # check if there is only considered one marker
        tvecs = tvecs[0].tolist() # check if there is only considered one marker
        return ids, rvecs, tvecs
    else:	
        return [], [], []
    
def get_camera_dict(camera_id):
    """
    Returns a dictionary with the camera ID and an empty list for detected markers.
    """
    with open('src/marker_positions_rvecs_tvecs.json', 'r') as file:
        marker_positions = json.load(file)  
        for camera_dict in marker_positions:   
            if camera_dict['id'] == camera_id:  
                return camera_dict

def get_frame(cap):
    """
    Captures a frame from the video stream.
    Returns the captured frame.
    """
    ret, frame = cap.read()
    timestamp = datetime.now()
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
    cap = cv2.VideoCapture(params.URL)
    if not cap.isOpened():
        print("Error: Cannot open video stream")
        exit()
    else:
        print("Success: Starting video stream")
    return cap

