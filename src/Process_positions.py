"""
Authors: Linus Wasner, Lukas Bauer
Date: 2025-06-20
Project: 3dimensionalArucoMarkerDetection
Lekture: Echtzeitsysteme, Masterprogram advanced driver assistance systems, University of Applied Sciences Kempten

This module processes camera positions and marker detections.
It loads marker data, finds anchor cameras, computes global poses, and visualizes camera positions and directions.
It is called from the main.py script.
"""

import matplotlib
matplotlib.use('TkAgg')

import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
import params
import traceback

# Set global print options for numpy arrays
np.set_printoptions(precision=10, suppress=True)

def build_camera_pose_dataframe(solved_cameras_dict):
    """
    Creates a DataFrame with the global position and orientation of each solved cameras.
    Args:
        solved_cameras (dict): camera_id: global 4x4-Transformationsmatrix
    Returns:
        data (pd.DataFrame): DataFrame with columns ['id', 'x', 'z', 'dir_x', 'dir_z', 'angle_rad', 'angle_deg']
    """
    data = []
    for cam_id, T in solved_cameras_dict.items():
        x = T[0, 3]
        z = T[2, 3]
        dir_x = T[0, 2]*-1
        dir_z = T[2, 2]
        #dir_x = T[0, 2]
        #dir_z = T[1, 2]
        #dir_x = T[1, 2]
        #dir_z = T[2, 2]
        #angle_rad = np.arctan2(dir_z, dir_x)
        angle_rad = np.arctan2(dir_x, dir_z)
        angle_deg = np.degrees(angle_rad)
        data.append({
            'id': cam_id,
            'x': x,
            'z': z,
            'dir_x': dir_x,
            'dir_z': dir_z,
            'angle_rad': angle_rad,
            'angle_deg': angle_deg
        })
    return pd.DataFrame(data)

def flip_y_axis_rotation_(Matrix):
    """
    currently not used, but can be used to flip the rotation around the Y-axis of a 4x4 transformation matrix.

    Flips the rotation around the Y-axis of a 4x4 transformation matrix.
    Args:
        Matrix (np.ndarray): 4x4 transformation matrix.
    Returns:
        Matrix_flipped (np.ndarray): Flipped 4x4 transformation matrix.
    """
    Matrix_flipped = Matrix.copy()
    Matrix_flipped[:3, 0] *= -1  # invertiere X-Achse
    Matrix_flipped[:3, 2] *= -1  # invertiere Z-Achse
    return Matrix_flipped

def load_valid_marker_data(marker_detections):
    """
    loads marker positions and rvecs/tvecs from a JSON file.
    Filters out invalid entries.
    Args:
        path (str): Path to the JSON file containing marker positions and rvecs/tvecs.
    Returns:
        camera_views (dict): Mapping of all spotted markers in addition to all camera perspectives.
    """
    # with open(path, 'r') as f:
    #     raw_data = json.load(f)

    camera_views = {}
    for entry in marker_detections:
        cam_id = entry['id']
        valid_detections = []
        for other in entry['Others']:
            try:
                detected_id = int(other['detected_id'])
                rvec = other['Position'][0]['rvecs']
                tvec = other['Position'][1]['tvecs']
                if len(rvec) == 3 and len(tvec) == 3:
                    valid_detections.append({
                        'detected_id': detected_id,
                        'rvec': rvec,
                        'tvec': tvec
                    })
            except (ValueError, KeyError, IndexError, TypeError):
                continue
        camera_views[cam_id] = valid_detections
    return camera_views

def find_anchor_camera(camera_views, camera_priority_id, anchor_ids={0, 1, 2, 3}): # test with params.ANCHOR_MARKER_IDS
    """
    Finds the anchor camera that sees a global origin marker.
    Prefers the camera with camera_priority_id if it sees an anchor.
    Args:
        camera_views (dict): Mapping of camera IDs to marker detections.
        camera_priority_id (int): Preferred camera ID.
        anchor_ids (set): Set of marker IDs of the global origin cube.
    Returns:
        tuple (camera_id (int), detection (dict)): ID of the anchor camera and its detection entry stored in marker_positions_rvecs_tvecs.json.
    """
    # 1. Check if the prioritized camera sees an anchor marker
    detections = camera_views.get(camera_priority_id, [])
    for det in detections:
        if det['detected_id'] in anchor_ids:
            return camera_priority_id, det
    # 2. If not, iterate over all cameras and find the first one that sees an anchor marker
    for cam_id, detections in camera_views.items():
        for det in detections:
            if det['detected_id'] in anchor_ids:
                return cam_id, det
    # 3. No anchor found
    return None, None

def rvec_tvec_to_matrix(rvec, tvec):
    """
    Builds a 4x4 transformation matrix from rotation vector (rvec) and translation vector (tvec).
    Args:
        rvec (list or np.ndarray): Rotation vector (3 elements).
        tvec (list or np.ndarray): Translation vector (3 elements).
    Returns:
        np.ndarray: 4x4 transformation matrix.
    """
    rvec = np.array(rvec, dtype=np.float64).reshape(3, 1)
    tvec = np.array(tvec, dtype=np.float64).reshape(3, 1)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.array(tvec).reshape(3)
    return T

def try_solve_cameras_from_solved(camera_views, solved_cameras):
    """ 
    Tries to solve cameras with known cameras.
    Args:   
        camera_views (dict): Mapping of all spotted markers in addition to all camera perspectives.
        solved_cameras (dict): camera_id: global 4x4-Transformationsmatrix
    Returns:
        solved_cameras (dict): Updated solved_cameras with new cameras.
    """
    for cam_id in list(solved_cameras.keys()):
        for detection in camera_views[cam_id]:
            detection_id = detection['detected_id']
            if detection_id // 10 in set(solved_cameras.keys()) or detection_id // 10 == 0:
                continue  # Marker belongs to a camera that is already solved
            else:
                Transformer_matrix_marker_to_cam = rvec_tvec_to_matrix(detection['rvec'], detection['tvec']) 
                Transformer_matrix_global_to_cam = solved_cameras[cam_id] @ Transformer_matrix_marker_to_cam @ params.ANCHOR_MARKER_WORLD_POSES[detection_id % 10]
                Transformer_matrix_global_to_cam[0, 3] *= -1

                solved_cameras[detection_id // 10] = Transformer_matrix_global_to_cam

    return solved_cameras

def try_solve_cameras_from_unsolved(camera_views, solved_cameras):
    """
    Tries to solve cameras that are not yet solved by themselves. 
    Unsolved Cameras looking for solved cameras.
    Args:
        camera_views (dict): Mapping of all spotted markers in addition to all camera perspectives.
        solved_cameras (dict): {camera_id: global 4x4-Transformationsmatrix}
    Returns:
        solved_cameras (dict): Updated solved_cameras with new cameras.
    """
    unsolved_cameras = set(camera_views.keys()) - set(solved_cameras.keys())
    for cam_id in list(unsolved_cameras):
        for detection in camera_views[cam_id]:
            detection_id = detection['detected_id']
            if detection_id // 10 in solved_cameras and detection_id // 10 != 0:
                try:
                    Transformer_matrix_marker_to_cam = rvec_tvec_to_matrix(detection['rvec'], detection['tvec'])
                    Transformer_matrix_marker_to_cam = np.linalg.inv(Transformer_matrix_marker_to_cam)
                    Transformer_matrix_global_to_cam = solved_cameras[detection_id // 10] @ Transformer_matrix_marker_to_cam @ params.ANCHOR_MARKER_WORLD_POSES[detection_id % 10]
                    Transformer_matrix_global_to_cam[0, 3] *= -1

                    solved_cameras[detection_id // 10] = Transformer_matrix_global_to_cam
                    unsolved_cameras.remove(cam_id)
                except:
                    pass
            else:
                continue
    return solved_cameras

#--------------------------------------------------------------------------------#
# Main program
#--------------------------------------------------------------------------------#

def process_positions(marker_detections):
    """
    Main function for processing camera positions and marker detections.
    Called from main.py.

    Performs the following steps:
    1. Loads camera data from marker_positions_rvecs_tvecs
    2. Finds the anchor camera that sees a zero-point marker
    3. Computes the global pose of the anchor camera and updates solved_cameras
    4. Iterates over all cameras and computes their global poses
    5. Creates a DataFrame with all camera positions and their viewing directions

    Returns:
        global_camera_poses_positions (pd.DataFrame): DataFrame with columns ['id', 'x', 'z', 'dir_x', 'dir_z', 'angle_rad', 'angle_deg']."""
    try:
        
        # 1. load data
        camera_views = load_valid_marker_data(marker_detections)
        #print("camera_views:\n", camera_views)

        # 2. get anchor camera, prefered camera is params.CAMERA_ID, else the first camera that sees an anchor marker
        anchor_cam_id, anchor_detection = find_anchor_camera(camera_views, params.CAMERA_ID, params.ANCHOR_MARKER_IDS)
        if anchor_cam_id is None:
            raise ValueError("origin cube not found in any camera view.")
            exit(1)

        # 3. process Transformation Matrix from global to camera, update solved cameras
        anchor_marker_id = anchor_detection['detected_id']

        Transformer_matrix_marker_to_cam = rvec_tvec_to_matrix(anchor_detection['rvec'], anchor_detection['tvec']) # Marker (an neuer Kamera) aus Sicht bekannter Kamera
        Transformer_matrix_global_to_cam = Transformer_matrix_marker_to_cam @ params.ANCHOR_MARKER_WORLD_POSES[anchor_marker_id]
        Transformer_matrix_global_to_cam = np.linalg.inv(Transformer_matrix_global_to_cam)

        Transformer_matrix_global_to_cam[0, 3] *= -1

        solved_cameras = {anchor_cam_id: Transformer_matrix_global_to_cam}

        # 4. iterate over all cameras and try to solve them
        for _ in range(6):
            newly = try_solve_cameras_from_solved(camera_views, solved_cameras)
            solved_cameras.update(newly)
            newly = try_solve_cameras_from_unsolved(camera_views, solved_cameras)
            solved_cameras.update(newly)

        # 5. build dataframe with all camera positions and their directions
        global_camera_poses_positions = build_camera_pose_dataframe(solved_cameras)

        return global_camera_poses_positions
    
    except Exception as e:
        #traceback.print_exc()
        print(f"Error processing camera positions: {e}")
        return
