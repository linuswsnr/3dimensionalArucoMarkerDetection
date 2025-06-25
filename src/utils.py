"""
Authors: Linus Wasner, Lukas Bauer
Date: 2025-06-10
Project: 3dimensionalArucoMarkerDetection
Lekture: Echtzeitsysteme, Masterprogram advanced driver assistance systems, University of Applied Sciences Kempten

This module contains utility functions for ArUco marker detection, camera setup, and JSON handling.
"""

import json
import cv2
import cv2.aruco as aruco
import numpy as np
from datetime import datetime
import params as params

class ArucoMarker():
    """
    Class representing an ArUco marker with its ID, distance, and angle in addition to the position of the camera which takes the image.

    Attributes:
        detected_id (int): ID of the detected marker.
        rvecs (list): Rotation vectors of the marker.
        tvecs (list): Translation vectors of the marker.
        timestamp (datetime): Timestamp when the marker was detected.
    Methods:
        delete_position(): Deletes the marker's position from the JSON file.
        update_position(): Updates the marker's position in the JSON file.
    """
    def __init__(self, detected_id, rvecs, tvecs, timestamp):
        self.detected_id = int(detected_id)
        self.rvecs = rvecs
        self.tvecs = tvecs
        self.timestamp = timestamp

        self.timestamp_mqtt = self.timestamp.strftime('%Y-%m-%d %H:%M:%S')

    def __repr__(self):
        return f"ArucoMarker(id={self.detected_id}, rvecs={self.rvecs}, tvecs={self.tvecs}, timestamp={str(self.timestamp)})"

    def delete_position(self, marker_positions):
        """
        Deletes the position of the marker in marker_positions_rvecs_tvecs.json.
        """
        # Load existing marker positions from JSON file
        # with open ('src/marker_positions_rvecs_tvecs.json', 'r') as file:
        #     marker_positions = json.load(file)
        

        # check if camera dict contains the ID of the camera
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
                        return marker_positions
                    
    def update_position(self, marker_positions):
        """
        Updates the position of the marker in marker_positions.json.
        """
        # Load existing marker positions from JSON file
        # with open ('src/marker_positions_rvecs_tvecs.json', 'r') as file:
        #     marker_positions = json.load(file)

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
                        camera_dict["time"] = str(self.timestamp_mqtt)
                        # with open('src/marker_positions_rvecs_tvecs.json', 'w') as f:
                        #     json.dump(marker_positions, f, indent=4)
                        return marker_positions
                    
                # append new detected block if detected_id does not exist
                camera_dict['Others'].append({
                    'detected_id': self.detected_id,
                    'Position': [
                        {'rvecs': self.rvecs}, {'tvecs': self.tvecs}
                    ]
                })
                camera_dict["time"] = str(self.timestamp_mqtt)
                # with open('src/marker_positions_rvecs_tvecs.json', 'w') as f:
                #     json.dump(marker_positions, f, indent=4)
                return marker_positions

def get_aruco_markers(frame):
    """
    Detects ArUco markers in the given frame.
    Returns a list of detected markers with their IDs, distances, and angles.
    Args:
        frame (numpy.ndarray): The image frame in which to detect markers.
    Returns:
        ids (list): List of detected marker IDs.
        rvecs (list): List of lists with rotation vectors for each detected marker.
        tvecs (list): List of lists with translation vectors for each detected marker.
    """
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, _ = detector.detectMarkers(frame)
    
    if ids is not None:
        ids = ids.flatten()
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, params.MARKERLENGTH, params.CAMERA_MATRIX, params.DISTCOEFFS)
        rvecs = [rvec[0].tolist() for rvec in rvecs]
        tvecs = [tvec[0].tolist() for tvec in tvecs]        
        return ids, rvecs, tvecs
    else:	
        return [], [], []
    
def get_camera_dict(camera_id):
    """
    Returns a dictionary with the camera ID and an empty list for detected markers.
    Args:
        camera_id (int): The ID of the camera.
    Returns:
        camera_dict (dict): contains the camera ID and an empty list for detected markers.
    """
    with open('src/marker_positions_rvecs_tvecs.json', 'r') as file:
        marker_positions = json.load(file)  
        for camera_dict in marker_positions:   
            if camera_dict['id'] == camera_id:  
                return camera_dict

def get_frame(cap):
    """
    Captures a frame from the video stream.
    Args:
        cap (cv2.VideoCapture): The video capture object.
    Returns:
        frame (numpy.ndarray): The captured frame.
        timestamp (datetime): The timestamp when the frame was captured.
    """
    ret, frame = cap.read()
    timestamp = datetime.now()
    if not ret:
        cap.release()
        cap = cv2.VideoCapture(params.URL)
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            return None

    return frame, timestamp

def get_marker_detections(frame, photo_timestamp):
    """
    Detects ArUco markers in the given frame
    Args:
        frame (numpy.ndarray): The image frame in which to detect markers.
        photo_timestamp (datetime): The timestamp when the photo was taken.
    Returns:
        markers (list): List of ArucoMarker objects from class ArucoMarker."""
    ids, rvecs, tvecs = get_aruco_markers(frame)
    markers = []
    for detected_id, rvec, tvec in zip(ids, rvecs, tvecs):
        marker = ArucoMarker(detected_id, rvec, tvec, photo_timestamp)
        markers.append(marker)
    return markers

def setup_camera_stream():
    """
    Sets up the video stream from the ESP32 camera.
    Returns:
        cap (cv2.VideoCapture): The video capture object for the camera stream.
    """
    cap = cv2.VideoCapture(params.URL)
    if not cap.isOpened():
        print("Error: Cannot open video stream")
        exit()
    else:
        print("Success: Starting video stream")
    return cap

